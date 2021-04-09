import omni
import omni.kit.editor
from pxr import Usd, PhysicsSchema
from omni.isaac.dynamic_control import _dynamic_control

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

        # ROS node
        try:
            rospy.init_node('OmniIsaacRosControlBridge')
        except rospy.ROSException as e:
            print("[ERROR] OmniIsaacRosControlBridge:", e)

        for schema in self._get_ros_control_bridge_schema():
            self._components.append(RosController(self._dci, self._usd_context, schema))

    def shutdown(self):
        for component in self._components:
            component.stop()

    def _get_ros_control_bridge_schema(self):
        schemas = []
        stage = self._usd_context.get_stage()
        for prim in Usd.PrimRange.AllPrims(stage.GetPrimAtPath("/")):
            if prim.GetTypeName() == "RosControlFollowJointTrajectory":
                schemas.append(ROSControlSchema.RosControlFollowJointTrajectory(prim))
        return schemas


class RosController():
    def __init__(self, dci, usd_context, schema):
        self._dci = dci
        self._usd_context = usd_context
        self._schema = schema
        
        self._ar = None
        self._server = None
        self._joint_names = []

        # feedback/result
        self._feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
        self._result = control_msgs.msg.FollowJointTrajectoryResult()

        self.start()
    
    def start(self):
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

            # get joint names
            self._joint_names = []
            for i in range(self._dci.get_articulation_dof_count(self._ar)):
                dof = self._dci.get_articulation_dof(self._ar, i)
                if dof != _dynamic_control.DofType.DOF_NONE:
                    self._joint_names.append(self._dci.get_joint_name(self._dci.get_dof_joint(dof)))

            # build action name
            _action_name = self._schema.GetRosNodePrefixAttr().Get() \
                         + self._schema.GetControllerNameAttr().Get() \
                         + self._schema.GetActionNamespaceAttr().Get()

            # start actionlib server
            self._shutdown_server()
            self._server = actionlib.ActionServer(_action_name,
                                                  control_msgs.msg.FollowJointTrajectoryAction,
                                                  goal_cb=self._goal_cb,
                                                  cancel_cb=self._cancel_cb,
                                                  auto_start=False)
            self._server.start()

    def stop(self):
        self._shutdown_server()

    def _shutdown_server(self):
        # /opt/ros/melodic/lib/python2.7/dist-packages/actionlib/action_server.py
        print("shutdown", self._schema.GetPrim().GetPath())
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
        pass
        goal_handle.set_accepted("")
        goal = goal_handle.get_goal()

        print("")
        print("_goal_cb:", type(goal))
        print("goal_tolerance:", goal.goal_tolerance)
        print("path_tolerance:", goal.path_tolerance)
        print("goal_time_tolerance:", goal.goal_time_tolerance)
        print("trajectory.joint_names:", goal.trajectory.joint_names)
        print("trajectory.points:", len(goal.trajectory.points))

        last_time_from_start = 0

        for trajectory in goal.trajectory.points:
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

        success = True
        if success:
            self._result.error_code = self._result.SUCCESSFUL
            goal_handle.set_succeeded(self._result, "")
            print("set_succeeded")
        else:
            self._result.error_code = self._result.SUCCESSFUL
            goal_handle.set_aborted(self._result, "")
            print("set_aborted")
        print("end")

    def _cancel_cb(self, goal_id):
        print("_cancel_cb:", type(goal_id), goal_id)

    # self._editor.is_playing()
