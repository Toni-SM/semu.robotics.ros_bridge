import math

import omni
import carb
import omni.kit
from pxr import Usd, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import get_stage_units

import rospy
import rosgraph
import actionlib
import control_msgs.msg     # apt-get install ros-<distro>-control-msgs
from trajectory_msgs.msg import JointTrajectoryPoint

import omni.add_on.RosControlBridgeSchema as ROSControlSchema


def acquire_ros_control_bridge_interface(plugin_name=None, library_path=None):
    return RosControlBridge()

def release_ros_control_bridge_interface(bridge):
    bridge.shutdown()


class RosControlBridge:
    def __init__(self):
        self._components = []
        self._node_name = carb.settings.get_settings().get("/exts/omni.add_on.ros_control_bridge/nodeName")

        # omni objects and interfaces
        self._usd_context = omni.usd.get_context()
        self._timeline = omni.timeline.get_timeline_interface()
        self._physx_interface = omni.physx.acquire_physx_interface()
        self._dci = _dynamic_control.acquire_dynamic_control_interface()
        
        # events
        self._update_event = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_update_event)
        self._timeline_event = self._timeline.get_timeline_event_stream().create_subscription_to_pop(self._on_timeline_event)
        self._stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)
        self._physx_event = self._physx_interface.subscribe_physics_step_events(self._on_physics_event)

        # ROS node
        self._init_ros_node()

    def shutdown(self):
        self._update_event = None
        self._timeline_event = None
        self._stage_event = None

        self._stop_components()

    def _init_ros_node(self):
        # check ROS master
        try:
            rosgraph.Master("/rostopic").getPid()
        except:
            print("[WARNING] {}: ROS master is not running".format(self._node_name))
            return False
        # start ROS node
        try:
            rospy.init_node(self._node_name)
            print("[INFO] {} node started".format(self._node_name))
        except rospy.ROSException as e:
            print("[ERROR] {}:".format(self._node_name), e)
            return False
        return True

    def _get_ros_control_bridge_schemas(self):
        schemas = []
        stage = self._usd_context.get_stage()
        for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath("/")):
            if prim.GetTypeName() == "RosControlFollowJointTrajectory":
                schemas.append(ROSControlSchema.RosControlFollowJointTrajectory(prim))
            elif prim.GetTypeName() == "RosControlGripperCommand":
                schemas.append(ROSControlSchema.RosControlGripperCommand(prim))
        return schemas

    def _stop_components(self):
        for component in self._components:
            component.stop()

    def _reload_components(self):
        # stop components
        self._stop_components()
        # load components
        self._components = []
        self._skip_update_step = True
        for schema in self._get_ros_control_bridge_schemas():
            if schema.__class__.__name__ == "RosControlFollowJointTrajectory":
                self._components.append(RosControlFollowJointTrajectory(self._usd_context, schema, self._dci))
            elif schema.__class__.__name__ == "RosControlGripperCommand":
                self._components.append(RosControllerGripperCommand(self._usd_context, schema, self._dci))

    def _on_update_event(self, event):
        if self._timeline.is_playing():
            for component in self._components:
                if self._skip_update_step:
                    self._skip_update_step = False
                    return
                # start components
                if not component.started:
                    component.start()
                    return
                # step
                component.update_step(event.payload["dt"])
    
    def _on_timeline_event(self, event):
        # reload components
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            self._reload_components()
            print("[INFO] RosControlBridge: components reloaded")
        # stop components
        elif event.type == int(omni.timeline.TimelineEventType.STOP) or event.type == int(omni.timeline.TimelineEventType.PAUSE):
            self._stop_components()
            print("[INFO] RosControlBridge: components stopped")

    def _on_stage_event(self, event):
        pass

    def _on_physics_event(self, step):
        for component in self._components:
            component.physics_step(step)


class RosController():
    def __init__(self, usd_context, schema):
        self._usd_context = usd_context
        self._schema = schema
        
        self.started = False

    def start(self):
        raise NotImplementedError

    def stop(self):
        print("[INFO] RosController: stopping", self._schema.__class__.__name__)
        self.started = False

    def update_step(self, dt):
        raise NotImplementedError

    def physics_step(self, step):
        raise NotImplementedError


class RosControlFollowJointTrajectory(RosController):
    def __init__(self, usd_context, schema, dci):
        super(RosControlFollowJointTrajectory, self).__init__(usd_context, schema)
        
        self._dci = dci

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._dt = 0.05
        self._action_server = None
       
        # TODO: add it to the schema
        self._action_tolerances = {}
        self._action_default_tolerance = 0.05

        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_point_index = 1

        # feedback / result
        self._action_result_message = control_msgs.msg.FollowJointTrajectoryResult()
        self._action_feedback_message = control_msgs.msg.FollowJointTrajectoryFeedback()
    
    def start(self):
        print("[INFO] RosControlFollowJointTrajectory: starting", self._schema.__class__.__name__)

        # get attributes and relationships
        action_namespace = self._schema.GetActionNamespaceAttr().Get()
        controller_name = self._schema.GetControllerNameAttr().Get()

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if not len(relationships):
            print("[WARNING] RosControlFollowJointTrajectory: empty relationships")
            return

        # check for articulation API
        stage = self._usd_context.get_stage()
        path = relationships[0].GetPrimPath().pathString
        if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxArticulationAPI):
            print("[WARNING] RosControlFollowJointTrajectory: prim {} doesn't have PhysxArticulationAPI".format(path))
            return
        
        # start action server
        self._shutdown_action_server()
        self._action_server = actionlib.ActionServer(controller_name + action_namespace, 
                                                     control_msgs.msg.FollowJointTrajectoryAction,
                                                     goal_cb=self._on_goal,
                                                     cancel_cb=self._on_cancel,
                                                     auto_start=False)
        try:
            self._action_server.start()
        except ConnectionRefusedError:
            print("[ERROR] RosControlFollowJointTrajectory: action server {} not started".format(controller_name + action_namespace))
            return
        print("[INFO] RosControlFollowJointTrajectory: register action:", controller_name + action_namespace)

        self.started = True
    
    def stop(self):
        super(RosControlFollowJointTrajectory, self).stop()
        self._articulation = _dynamic_control.INVALID_HANDLE
        self._shutdown_action_server()
        self._action_goal_handle = None
        self._action_goal = None

    def _shutdown_action_server(self):
        # /opt/ros/melodic/lib/python2.7/dist-packages/actionlib/action_server.py
        print("[INFO] RosControlFollowJointTrajectory: destroy action server:", self._schema.GetPrim().GetPath())
        if self._action_server:
            if self._action_server.started:
                self._action_server.started = False

                self._action_server.status_pub.unregister()
                self._action_server.result_pub.unregister()
                self._action_server.feedback_pub.unregister()

                self._action_server.goal_sub.unregister()
                self._action_server.cancel_sub.unregister()

            del self._action_server
            self._action_server = None

    def _init_articulation(self):
        # get articulation
        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        path = relationships[0].GetPrimPath().pathString
        self._articulation = self._dci.get_articulation(path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[WARNING] RosControlFollowJointTrajectory: prim {} is not an articulation".format(path))
            return
        
        dof_props = self._dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self._dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self._dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                dof_name = self._dci.get_dof_name(dof_ptr)
                if dof_name not in self._joints:
                    _joint = self._dci.find_articulation_joint(self._articulation, dof_name)
                    self._joints[dof_name] = {"joint": _joint,
                                              "type": self._dci.get_joint_type(_joint),
                                              "dof": self._dci.find_articulation_dof(self._articulation, dof_name), 
                                              "lower": lower_limits[i], 
                                              "upper": upper_limits[i], 
                                              "has_limits": has_limits[i]}

        if not self._joints:
            print("[WARNING] RosControlFollowJointTrajectory: no joints found")
            self.started = False

    def _set_joint_position(self, name, target_position):
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self._dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name):
        return self._dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos

    def _on_goal(self, goal_handle):
        goal = goal_handle.get_goal()

        # reject if joints don't match
        for name in goal.trajectory.joint_names:
            if name not in self._joints:
                print("[ERROR] RosControlFollowJointTrajectory: Received a goal with incorrect joint names ({} not in {})".format(name, list(self._joints.keys())))
                self._action_result_message.error_code = self._action_result_message.INVALID_JOINTS
                goal_handle.set_rejected(self._action_result_message, "")
                return

        # reject if infinity or NaN
        for point in goal.trajectory.points:
            for position, velocity in zip(point.positions, point.velocities):
                if math.isinf(position) or math.isnan(position) or math.isinf(velocity) or math.isnan(velocity):
                    print("[ERROR] RosControlFollowJointTrajectory: Received a goal with infinites or NaNs")
                    self._action_result_message.error_code = self._action_result_message.INVALID_GOAL
                    goal_handle.set_rejected(self._action_result_message, "")
                    return

        # reject if joints are already controlled
        if self._action_goal is not None:
            print("[ERROR] RosControlFollowJointTrajectory: Cannot accept multiple goals")
            self._action_result_message.error_code = self._action_result_message.INVALID_GOAL
            goal_handle.set_rejected(self._action_result_message, "")
            return

        # set tolerances
        for tolerance in goal.goal_tolerance:
            self._action_tolerances[tolerance.name] = tolerance.position
        for name in goal.trajectory.joint_names:
            if name not in self._action_tolerances:
                self._action_tolerances[name] = self._action_default_tolerance

        # if the user forget the initial position
        if goal.trajectory.points[0].time_from_start.to_sec():
            initial_point = JointTrajectoryPoint(positions=[self._get_joint_position(name) for name in goal.trajectory.joint_names],
                                                 time_from_start=rospy.Duration())
            goal.trajectory.points.insert(0, initial_point)
        
        # store goal data
        self._action_goal = goal
        self._action_goal_handle = goal_handle
        self._action_point_index = 1
        self._action_start_time = rospy.get_time()
        self._action_feedback_message.joint_names = list(goal.trajectory.joint_names)

        goal_handle.set_accepted()

    def _on_cancel(self, goal_handle):
        print("[INFO] RosControlFollowJointTrajectory: cancel goal")
        if self._action_goal is not None:
            self._action_goal = None
            self._feedback = None
            self._result = None
            goal_handle.set_accepted()
            return
        goal_handle.set_rejected()

    def update_step(self, dt):
        pass

    def physics_step(self, dt):
        if not self.started:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        self._dt = dt
        if self._action_goal is not None:
            # end of trajectory
            if self._action_point_index >= len(self._action_goal.trajectory.points):
                self._action_goal = None
                self._action_result_message.error_code = self._action_result_message.SUCCESSFUL
                self._action_goal_handle.set_succeeded(self._action_result_message)
                return
            
            previous_point = self._action_goal.trajectory.points[self._action_point_index - 1]
            current_point = self._action_goal.trajectory.points[self._action_point_index]
            time_passed = rospy.get_time() - self._action_start_time

            if time_passed <= current_point.time_from_start.to_sec():
                # linear interpolation
                ratio = (time_passed - previous_point.time_from_start.to_sec()) \
                      / (current_point.time_from_start.to_sec() - previous_point.time_from_start.to_sec())
                self._dci.wake_up_articulation(self._articulation)
                for i, name in enumerate(self._action_goal.trajectory.joint_names):
                    side = -1 if current_point.positions[i] < previous_point.positions[i] else 1
                    target_position = previous_point.positions[i] \
                                    + side * ratio * abs(current_point.positions[i] - previous_point.positions[i])
                    self._set_joint_position(name, target_position)
            else:
                self._action_point_index += 1
                self._action_feedback_message.actual.positions = [self._get_joint_position(name) for name in self._action_goal.trajectory.joint_names]
                self._action_feedback_message.actual.time_from_start = rospy.Duration.from_sec(time_passed)
                self._action_goal_handle.publish_feedback(self._action_feedback_message)

        
    # def _goal_cb(self, goal_handle):
    #     goal_handle.set_accepted("")
    #     goal = goal_handle.get_goal()
    #     # TODO: see other properties: goal_tolerance, path_tolerance, goal_time_tolerance

    #     last_time_from_start = 0
    #     joint_names = goal.trajectory.joint_names

    #     # execute trajectories
    #     for trajectory in goal.trajectory.points:
    #         # compute dt
    #         dt = trajectory.time_from_start.to_sec() - last_time_from_start
    #         last_time_from_start = trajectory.time_from_start.to_sec()

    #         # set target
    #         # TODO: add lock
    #         for i in range(len(joint_names)):
    #             self._dof[joint_names[i]]["target"] = trajectory.positions[i]
    #             self._dof[joint_names[i]]["dt"] = dt
            
    #         time.sleep(dt)
    #         # t = time.time()
    #         # while time.time() - t < dt:
    #         #     time.sleep(0.01)    
    #         #     error = 0
    #         #     for k in self._dof:
    #         #         if self._dof[k]["target"] is not None:
    #         #             error += abs(self._dof[k]["target"] - self._dof[k]["current"])
    #         #     print(error)
    #         #     if error < 0.35:
    #         #         break
            
    #         # TODO: add error
    #         # build feedback
    #         self._feedback.actual = trajectory
    #         self._feedback.desired = goal.trajectory.points[-1]

    #         # publish feedback
    #         goal_handle.publish_feedback(self._feedback)

    #     # release dof's target
    #     for k in self._dof:
    #         self._dof[k]["target"] = None

    #     success = True
    #     if success:
    #         self._result.error_code = self._result.SUCCESSFUL
    #         goal_handle.set_succeeded(self._result, "")
    #     else:
    #         self._result.error_code = self._result.SUCCESSFUL
    #         goal_handle.set_aborted(self._result, "")

    # def _cancel_cb(self, goal_id):
    #     # TODO: cancel current goal
    #     pass


class RosControllerGripperCommand(RosController):
    def __init__(self, dci, usd_context, schema):
        super().__init__(dci, usd_context, schema)
        
        # feedback/result
        self._feedback = control_msgs.msg.GripperCommandFeedback()
        self._result = control_msgs.msg.GripperCommandResult()
    
    def start(self):
        self.started = True
        print("[INFO] RosControllerGripperCommand: starting", self._schema.__class__.__name__)

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if len(relationships) > 1:
            # check for articulation API
            stage = self._usd_context.get_stage()
            path = relationships[0].GetPrimPath().pathString
            if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxArticulationAPI):
                print("[WARNING] RosControllerGripperCommand: prim {} doesn't have PhysxArticulationAPI".format(path))
                return
            
            # get articulation
            self._ar = self._dci.get_articulation(path)
            if self._ar == _dynamic_control.INVALID_HANDLE:
                print("[WARNING] RosControllerGripperCommand: prim {}: invalid handle".format(path))
                return

            # get DOF
            self._dof = {}
            dof_paths = [relationship.GetPrimPath().pathString for relationship in relationships[1:]]
            for i in range(self._dci.get_articulation_dof_count(self._ar)):
                dof = self._dci.get_articulation_dof(self._ar, i)
                if dof != _dynamic_control.DofType.DOF_NONE:
                    if self._dci.get_dof_path(dof) in dof_paths:
                        joint_name = self._dci.get_joint_name(self._dci.get_dof_joint(dof))
                        self._dof[joint_name] = {"dof": dof, "target": None}
            
            # build action name
            _action_name = self._schema.GetRosNodePrefixAttr().Get() \
                         + self._schema.GetControllerNameAttr().Get() \
                         + self._schema.GetActionNamespaceAttr().Get()

            # start actionlib server
            self._start_server(_action_name, control_msgs.msg.GripperCommandAction)
    
    def _goal_cb(self, goal_handle):
        goal_handle.set_accepted("")
        goal = goal_handle.get_goal()
        # TODO: see other properties: command.max_effort

        for k in self._dof:
            self._dof[k]["target"] = goal.command.position

        time.sleep(0.05)
        
        # publish feedback
        self._feedback.reached_goal = False
        self._feedback.stalled = False
        self._feedback.position = goal.command.position
        self._feedback.effort = goal.command.max_effort
        goal_handle.publish_feedback(self._feedback)
    
        time.sleep(0.05)

        # # release dof's target
        # for k in self._dof:
        #     self._dof[k]["target"] = None

        success = True
        if success:
            self._result.reached_goal = True
            self._result.stalled = False
            goal_handle.set_succeeded(self._result, "")
        else:
            self._result.reached_goal = False
            self._result.stalled = True
            goal_handle.set_aborted(self._result, "")

    def _cancel_cb(self, goal_id):
        # TODO: cancel current goal
        pass
