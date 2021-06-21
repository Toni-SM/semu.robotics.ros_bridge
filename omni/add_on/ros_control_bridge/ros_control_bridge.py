from logging import error
from subprocess import check_call
from threading import currentThread
import time

import omni
import carb
from pxr import Usd, PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control

import rospy
import actionlib
import control_msgs.msg     # apt-get install ros-<distro>-control-msgs
import rosgraph

import omni.add_on.RosControlBridgeSchema as ROSControlSchema
from rospy.names import reload_mappings


def acquire_roscontrolbridge_interface(plugin_name=None, library_path=None):
    return RosControlBridge()

def release_roscontrolbridge_interface(bridge):
    bridge.shutdown()


class RosControlBridge:
    def __init__(self):
        self._components = []

        # omni objects
        self._usd_context = omni.usd.get_context()
        self._timeline = omni.timeline.get_timeline_interface()
        
        # isaac interfaces
        self._dci = _dynamic_control.acquire_dynamic_control_interface()

        # events
        self._event_editor_step = (omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(self._on_editor_step_event))
        self._event_timeline = self._timeline.get_timeline_event_stream().create_subscription_to_pop(self._on_timeline_event)
        self._stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)

        # ROS node
        self._ros_node()

    def shutdown(self):
        self._event_editor_step = None
        self._event_timeline = None
        self._stage_event = None
        self._stop_components()

    def _ros_node(self):
        node_name = carb.settings.get_settings().get("/exts/omni.isaac.ros_control_bridge/nodeName")
        # check ROS master
        try:
            rosgraph.Master("/rostopic").getPid()
        except:
            print("[WARNING] {}: ROS master is not running",format(node_name))
            return False
        # start ROS node
        try:
            rospy.init_node(node_name)
            print("[INFO] {} node started".format(node_name))
        except rospy.ROSException as e:
            print("[ERROR] {}:".format(node_name), e)
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
        self._skip_step = True
        for schema in self._get_ros_control_bridge_schemas():
            if schema.__class__.__name__ == "RosControlFollowJointTrajectory":
                self._components.append(RosControllerFollowJointTrajectory(self._dci, self._usd_context, schema))
            elif schema.__class__.__name__ == "RosControlGripperCommand":
                self._components.append(RosControllerGripperCommand(self._dci, self._usd_context, schema))

    def _on_editor_step_event(self, event):
        if self._timeline.is_playing():
            for component in self._components:
                if self._skip_step:
                    self._skip_step = False
                    return
                # start components
                if not component.started:
                    component.start()
                    return
                # step
                component.step(event.payload["dt"])
    
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
        if event.type == int(omni.usd.StageEventType.OPENED):
            pass


class RosController():
    def __init__(self, dci, usd_context, schema):
        self._dci = dci
        self._usd_context = usd_context
        self._schema = schema
        
        self.started = False

        self._ar = None
        self._dof = {}
        self._server = None
    
    def start(self):
        raise NotImplementedError

    def stop(self):
        print("[INFO] RosController: stopping", self._schema.__class__.__name__)
        self._shutdown_server()
        self.started = False

    def step(self, dt):
        if self._ar and self._dof:
            self._dci.wake_up_articulation(self._ar)
            for k in self._dof:
                # get current position/rotation
                self._dof[k]["current"] = self._dci.get_dof_position(self._dof[k]["dof"])
                # set target
                target = self._dof[k]["target"]
                if target is not None:
                    self._dci.set_dof_position_target(self._dof[k]["dof"], target)

    def _start_server(self, action_name, ActionSpec):
        self._shutdown_server()
        self._server = actionlib.ActionServer(action_name, 
                                              ActionSpec,
                                              goal_cb=self._goal_cb,
                                              cancel_cb=self._cancel_cb,
                                              auto_start=False)
        self._server.start()
        print("[INFO] RosController server started", self._schema.GetPrim().GetPath())

    def _shutdown_server(self):
        # /opt/ros/melodic/lib/python2.7/dist-packages/actionlib/action_server.py
        print("[INFO] RosController server shutdown", self._schema.GetPrim().GetPath())
        if self._server:
            if self._server.started:
                self._server.started = False

                self._server.status_pub.unregister()
                self._server.result_pub.unregister()
                self._server.feedback_pub.unregister()

                self._server.goal_sub.unregister()
                self._server.cancel_sub.unregister()

            del self._server
            self._server = None
    
    def _goal_cb(self, goal_handle):
        raise NotImplementedError
        
    def _cancel_cb(self, goal_id):
        raise NotImplementedError


class RosControllerFollowJointTrajectory(RosController):
    def __init__(self, dci, usd_context, schema):
        super().__init__(dci, usd_context, schema)
        
        # feedback/result
        self._feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
        self._result = control_msgs.msg.FollowJointTrajectoryResult()
    
    def start(self):
        self.started = True
        print("[INFO] RosControllerFollowJointTrajectory: starting", self._schema.__class__.__name__)

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if not len(relationships):
            print("[WARNING] RosControllerFollowJointTrajectory: empty relationships")
            return

        # check for articulation API
        stage = self._usd_context.get_stage()
        path = relationships[0].GetPrimPath().pathString
        if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxArticulationAPI):
            print("[WARNING] RosControllerFollowJointTrajectory: prim {} doesn't have PhysxArticulationAPI".format(path))
            return
        
        # get articulation
        self._ar = self._dci.get_articulation(path)
        if self._ar == _dynamic_control.INVALID_HANDLE:
            print("[WARNING] RosControllerFollowJointTrajectory: prim {}: invalid handle".format(path))
            return

        # get DOF
        self._dof = {}
        for i in range(self._dci.get_articulation_dof_count(self._ar)):
            dof = self._dci.get_articulation_dof(self._ar, i)
            if dof != _dynamic_control.DofType.DOF_NONE:
                joint_name = self._dci.get_joint_name(self._dci.get_dof_joint(dof))
                self._dof[joint_name] = {"dof": dof, "target": None, "current": None, "error": None, "dt": None}

        # build action name
        _action_name = self._schema.GetRosNodePrefixAttr().Get() \
                        + self._schema.GetControllerNameAttr().Get() \
                        + self._schema.GetActionNamespaceAttr().Get()

        # start actionlib server
        self._start_server(_action_name, control_msgs.msg.FollowJointTrajectoryAction)
    
    def _goal_cb(self, goal_handle):
        goal_handle.set_accepted("")
        goal = goal_handle.get_goal()
        # TODO: see other properties: goal_tolerance, path_tolerance, goal_time_tolerance

        last_time_from_start = 0
        joint_names = goal.trajectory.joint_names

        # execute trajectories
        for trajectory in goal.trajectory.points:
            # compute dt
            dt = trajectory.time_from_start.to_sec() - last_time_from_start
            last_time_from_start = trajectory.time_from_start.to_sec()

            # set target
            # TODO: add lock
            for i in range(len(joint_names)):
                self._dof[joint_names[i]]["target"] = trajectory.positions[i]
                self._dof[joint_names[i]]["dt"] = dt
            
            time.sleep(dt)
            # t = time.time()
            # while time.time() - t < dt:
            #     time.sleep(0.01)    
            #     error = 0
            #     for k in self._dof:
            #         if self._dof[k]["target"] is not None:
            #             error += abs(self._dof[k]["target"] - self._dof[k]["current"])
            #     print(error)
            #     if error < 0.35:
            #         break
            
            # TODO: add error
            # build feedback
            self._feedback.actual = trajectory
            self._feedback.desired = goal.trajectory.points[-1]

            # publish feedback
            goal_handle.publish_feedback(self._feedback)

        # release dof's target
        for k in self._dof:
            self._dof[k]["target"] = None

        success = True
        if success:
            self._result.error_code = self._result.SUCCESSFUL
            goal_handle.set_succeeded(self._result, "")
        else:
            self._result.error_code = self._result.SUCCESSFUL
            goal_handle.set_aborted(self._result, "")

    def _cancel_cb(self, goal_id):
        # TODO: cancel current goal
        pass


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
