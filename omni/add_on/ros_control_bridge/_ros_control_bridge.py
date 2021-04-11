import time

import omni
import omni.kit.editor
from pxr import Usd, PhysicsSchema
from omni.isaac.dynamic_control import _dynamic_control

try:
    import rospy
except:
    import pyros_setup
    pyros_setup.configurable_import().configure().activate() # mysetup.cfg from pyros-setup
    import rospy

import actionlib
import control_msgs.msg     # apt-get install ros-<distro>-control-msgs

import omni.add_on.RosControlBridgeSchema as ROSControlSchema


def acquire_roscontrolbridge_interface(plugin_name=None, library_path=None):
    return RosControlBridge() 

def release_roscontrolbridge_interface(bridge):
    bridge.shutdown()


class RosControlBridge:
    def __init__(self):
        self._components = []

        # omni objects
        self._usd_context = omni.usd.get_context()
        self._editor = omni.kit.editor.get_editor_interface()
        
        # isaac interfaces
        self._dci = _dynamic_control.acquire_dynamic_control_interface()

        # events
        self._event_editor_step = self._editor.subscribe_to_update_events(self._on_editor_step_event)
        self._stage_event = self._usd_context.get_stage_event_stream().create_subscription_to_pop(self._on_stage_event)

        # ROS node
        try:
            rospy.init_node('OmniIsaacRosControlBridge')
        except rospy.ROSException as e:
            print("[ERROR] OmniIsaacRosControlBridge:", e)

    def shutdown(self):
        self._event_editor_step = None
        for component in self._components:
            component.stop()

    def _get_ros_control_bridge_schema(self):
        schemas = []
        stage = self._usd_context.get_stage()
        for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath("/")):
            if prim.GetTypeName() == "RosControlFollowJointTrajectory":
                schemas.append(ROSControlSchema.RosControlFollowJointTrajectory(prim))
            elif prim.GetTypeName() == "RosControlGripperCommand":
                schemas.append(ROSControlSchema.RosControlGripperCommand(prim))
        return schemas

    def _reload_components(self):
        # stop components
        for component in self._components:
            component.stop()
        # load components
        self._components = []
        for schema in self._get_ros_control_bridge_schema():
            if schema.__class__.__name__ == "RosControlFollowJointTrajectory":
                self._components.append(RosControllerFollowJointTrajectory(self._dci, self._usd_context, schema))
            elif schema.__class__.__name__ == "RosControlGripperCommand":
                self._components.append(RosControllerGripperCommand(self._dci, self._usd_context, schema))

    def _on_editor_step_event(self, step):
        if self._editor.is_playing():
            for component in self._components:
                if not component.started:
                    component.start()
                    return
                component.step(step)
        else:
            for component in self._components:
                if component.started:
                    print("stop component")
                    component.stop()

    def _on_stage_event(self, event):
        print(event.type)
        if event.type == int(omni.usd.StageEventType.OPENED):
            self._reload_components()


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
        print("[INFO] OmniIsaacRosControlBridge: stopping", self._schema.__class__.__name__)
        self._shutdown_server()
        self.started = False

    def step(self, step):
        if self._ar and self._dof:
            self._dci.wake_up_articulation(self._ar)
            for k in self._dof:
                if self._dof[k]["target"] is not None:
                    self._dci.set_dof_position_target(self._dof[k]["dof"], self._dof[k]["target"])

    def _start_server(self, action_name, ActionSpec):
        self._shutdown_server()
        self._server = actionlib.ActionServer(action_name, 
                                              ActionSpec,
                                              goal_cb=self._goal_cb,
                                              cancel_cb=self._cancel_cb,
                                              auto_start=False)
        self._server.start()
        print("[INFO] OmniIsaacRosControlBridge started", self._schema.GetPrim().GetPath())

    def _shutdown_server(self):
        # /opt/ros/melodic/lib/python2.7/dist-packages/actionlib/action_server.py
        print("[INFO] OmniIsaacRosControlBridge shutdown", self._schema.GetPrim().GetPath())
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
        print("[INFO] OmniIsaacRosControlBridge: starting", self._schema.__class__.__name__)

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if relationships:
            # check for articulation API
            stage = self._usd_context.get_stage()
            path = relationships[0].GetPrimPath().pathString
            if not stage.GetPrimAtPath(path).HasAPI(PhysicsSchema.ArticulationAPI):
                print("[WARNING] OmniIsaacRosControlBridge: prim {} doesn't have ArticulationAPI".format(path))
                return
            
            # get articulation
            self._ar = self._dci.get_articulation(path)
            if self._ar == _dynamic_control.INVALID_HANDLE:
                print("[WARNING] OmniIsaacRosControlBridge: prim {}: invalid handle".format(path))
                return

            # get DOF
            self._dof = {}
            for i in range(self._dci.get_articulation_dof_count(self._ar)):
                dof = self._dci.get_articulation_dof(self._ar, i)
                if dof != _dynamic_control.DofType.DOF_NONE:
                    joint_name = self._dci.get_joint_name(self._dci.get_dof_joint(dof))
                    self._dof[joint_name] = {"dof": dof, "target": None}

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
            # set target
            # TODO: add lock
            for i in range(len(joint_names)):
                self._dof[joint_names[i]]["target"] = trajectory.positions[i]

            # compute dt
            dt = trajectory.time_from_start.to_sec() - last_time_from_start
            last_time_from_start = trajectory.time_from_start.to_sec()
            time.sleep(dt)
            
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
        print("[INFO] OmniIsaacRosControlBridge: starting", self._schema.__class__.__name__)

        relationships = self._schema.GetArticulationPrimRel().GetTargets()
        if relationships:
            # check for articulation API
            stage = self._usd_context.get_stage()
            path = relationships[0].GetPrimPath().pathString
            if not stage.GetPrimAtPath(path).HasAPI(PhysicsSchema.ArticulationAPI):
                print("[WARNING] OmniIsaacRosControlBridge: prim {} doesn't have ArticulationAPI".format(path))
                return
            
            # get articulation
            self._ar = self._dci.get_articulation(path)
            if self._ar == _dynamic_control.INVALID_HANDLE:
                print("[WARNING] OmniIsaacRosControlBridge: prim {}: invalid handle".format(path))
                return

            # # get DOF
            # self._dof = {}
            # for i in range(self._dci.get_articulation_dof_count(self._ar)):
            #     dof = self._dci.get_articulation_dof(self._ar, i)
            #     if dof != _dynamic_control.DofType.DOF_NONE:
            #         joint_name = self._dci.get_joint_name(self._dci.get_dof_joint(dof))
            #         self._dof[joint_name] = {"dof": dof, "target": None}

            # build action name
            _action_name = self._schema.GetRosNodePrefixAttr().Get() \
                         + self._schema.GetControllerNameAttr().Get() \
                         + self._schema.GetActionNamespaceAttr().Get()

            # start actionlib server
            self._start_server(_action_name, control_msgs.msg.GripperCommandAction)
    
    def _goal_cb(self, goal_handle):
        goal_handle.set_accepted("")
        goal = goal_handle.get_goal()

        print("")
        print("_goal_cb:", type(goal))
        print("_goal_cb:", goal)
        print("goal.command.position:", goal.command.position)
        print("goal.command.max_effort:", goal.command.max_effort)

        # # TODO: see other properties: goal_tolerance, path_tolerance, goal_time_tolerance

        # last_time_from_start = 0
        # joint_names = goal.trajectory.joint_names

        # # execute trajectories
        # for trajectory in goal.trajectory.points:
        #     # set target
        #     # TODO: add lock
        #     for i in range(len(joint_names)):
        #         self._dof[joint_names[i]]["target"] = trajectory.positions[i]

        #     # compute dt
        #     dt = trajectory.time_from_start.to_sec() - last_time_from_start
        #     last_time_from_start = trajectory.time_from_start.to_sec()
        #     time.sleep(dt)
            
        #     # TODO: add error
        #     # build feedback
        #     self._feedback.actual = trajectory
        #     self._feedback.desired = goal.trajectory.points[-1]

        #     # publish feedback
        #     goal_handle.publish_feedback(self._feedback)

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
