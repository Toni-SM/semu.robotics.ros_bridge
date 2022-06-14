import omni
from pxr import PhysxSchema
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.stage import get_stage_units

import rospy
import actionlib
import control_msgs.msg

try:
    from ... import _ros_bridge
except:
    from ... import ros_bridge as _ros_bridge


class InternalState:
    def __init__(self):
        """Internal state for the ROS1 GripperCommand node
        """
        self.initialized = False

        self.dci = None
        self.usd_context = None
        self.timeline_event = None

        self.action_server = None
        self.articulation_path = ""
        self.gripper_joints_paths = []

        self._articulation = _dynamic_control.INVALID_HANDLE
        self._joints = {}

        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        # TODO: add to schema?
        self._action_timeout = 10.0
        self._action_position_threshold = 0.001
        self._action_previous_position_sum = float("inf")

        # feedback / result
        self._action_result_message = control_msgs.msg.GripperCommandResult()
        self._action_feedback_message = control_msgs.msg.GripperCommandFeedback()

    def on_timeline_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the timeline event

        :param event: Event
        :type event: carb.events._events.IEvent
        """
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            pass
        elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
            pass
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self.initialized = False
            self.shutdown_action_server()
            
            self._articulation = _dynamic_control.INVALID_HANDLE
            self._joints = {}

            self._action_goal = None
            self._action_goal_handle = None
            self._action_start_time = None

    def shutdown_action_server(self) -> None:
        """Shutdown the action server
        """
        # omni.isaac.ros_bridge/noetic/lib/python3/dist-packages/actionlib/action_server.py
        print("[Info][semu.robotics.ros_bridge] ROS1 GripperCommand: destroying action server")
        if self.action_server:
            if self.action_server.started:
                self.action_server.started = False

                self.action_server.status_pub.unregister()
                self.action_server.result_pub.unregister()
                self.action_server.feedback_pub.unregister()

                self.action_server.goal_sub.unregister()
                self.action_server.cancel_sub.unregister()

            del self.action_server
            self.action_server = None

    def _init_articulation(self) -> None:
        """Initialize the articulation and register joints
        """
        # get articulation
        path = self.articulation_path
        self._articulation = self.dci.get_articulation(path)
        if self._articulation == _dynamic_control.INVALID_HANDLE:
            print("[Warning][semu.robotics.ros_bridge] ROS1 GripperCommand: {} is not an articulation".format(path))
            return
        
        dof_props = self.dci.get_articulation_dof_properties(self._articulation)
        if dof_props is None:
            return

        upper_limits = dof_props["upper"]
        lower_limits = dof_props["lower"]
        has_limits = dof_props["hasLimits"]

        # get joints
        for i in range(self.dci.get_articulation_dof_count(self._articulation)):
            dof_ptr = self.dci.get_articulation_dof(self._articulation, i)
            if dof_ptr != _dynamic_control.DofType.DOF_NONE:
                # add only required joints
                if self.dci.get_dof_path(dof_ptr) in self.gripper_joints_paths:
                    dof_name = self.dci.get_dof_name(dof_ptr)
                    if dof_name not in self._joints:
                        _joint = self.dci.find_articulation_joint(self._articulation, dof_name)
                        self._joints[dof_name] = {"joint": _joint,
                                                  "type": self.dci.get_joint_type(_joint),
                                                  "dof": self.dci.find_articulation_dof(self._articulation, dof_name), 
                                                  "lower": lower_limits[i], 
                                                  "upper": upper_limits[i], 
                                                  "has_limits": has_limits[i]}

        if not self._joints:
            print("[Warning][semu.robotics.ros_bridge] ROS1 GripperCommand: no joints found in {}".format(path))
            self.initialized = False
    
    def _set_joint_position(self, name: str, target_position: float) -> None:
        """Set the target position of a joint in the articulation

        :param name: The joint name
        :type name: str
        :param target_position: The target position
        :type target_position: float
        """
        # clip target position
        if self._joints[name]["has_limits"]:
            target_position = min(max(target_position, self._joints[name]["lower"]), self._joints[name]["upper"])
        # scale target position for prismatic joints
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            target_position /= get_stage_units()
        # set target position
        self.dci.set_dof_position_target(self._joints[name]["dof"], target_position)

    def _get_joint_position(self, name: str) -> float:
        """Get the current position of a joint in the articulation

        :param name: The joint name
        :type name: str

        :return: The current position of the joint
        :rtype: float
        """
        position = self.dci.get_dof_state(self._joints[name]["dof"], _dynamic_control.STATE_POS).pos
        if self._joints[name]["type"] == _dynamic_control.JOINT_PRISMATIC:
            return position * get_stage_units()
        return position
    
    def on_goal(self, goal_handle: 'actionlib.ServerGoalHandle') -> None:
        """Callback function for handling new goal requests

        :param goal_handle: The goal handle
        :type goal_handle: actionlib.ServerGoalHandle
        """
        goal = goal_handle.get_goal()

        # reject if there is an active goal
        if self._action_goal is not None:
            print("[Warning][semu.robotics.ros_bridge] ROS1 GripperCommand: multiple goals not supported")
            goal_handle.set_rejected()
            return

        # store goal data
        self._action_goal = goal
        self._action_goal_handle = goal_handle
        self._action_start_time = rospy.get_time()
        self._action_previous_position_sum = float("inf")

        goal_handle.set_accepted()

    def on_cancel(self, goal_handle: 'actionlib.ServerGoalHandle') -> None:
        """Callback function for handling cancel requests

        :param goal_handle: The goal handle
        :type goal_handle: actionlib.ServerGoalHandle
        """
        if self._action_goal is None:
            goal_handle.set_rejected()
            return
        self._action_goal = None
        self._action_goal_handle = None
        self._action_start_time = None
        self._action_previous_position_sum = float("inf")
        goal_handle.set_canceled()

    def step(self, dt: float) -> None:
        """Update step

        :param dt: Delta time
        :type dt: float
        """
        if not self.initialized:
            return
        # init articulation
        if not self._joints:
            self._init_articulation()
            return
        # update articulation
        if self._action_goal is not None and self._action_goal_handle is not None:
            target_position = self._action_goal.command.position
            # set target
            self.dci.wake_up_articulation(self._articulation)
            for name in self._joints:
                self._set_joint_position(name, target_position)
            # end (position reached)
            position = 0
            current_position_sum = 0
            position_reached = True
            for name in self._joints:
                position = self._get_joint_position(name)
                current_position_sum += position
                if abs(position - target_position) > self._action_position_threshold:
                    position_reached = False
                    break
            if position_reached:
                self._action_goal = None
                self._action_result_message.position = position
                self._action_result_message.stalled = False
                self._action_result_message.reached_goal = True
                if self._action_goal_handle is not None:
                    self._action_goal_handle.set_succeeded(self._action_result_message)
                    self._action_goal_handle = None
                return
            # end (stalled)
            if abs(current_position_sum - self._action_previous_position_sum) < 1e-6:
                self._action_goal = None
                self._action_result_message.position = position
                self._action_result_message.stalled = True
                self._action_result_message.reached_goal = False
                if self._action_goal_handle is not None:
                    self._action_goal_handle.set_succeeded(self._action_result_message)
                    self._action_goal_handle = None
                return
            self._action_previous_position_sum = current_position_sum
            # end (timeout)
            time_passed = rospy.get_time() - self._action_start_time
            if time_passed >= self._action_timeout:
                self._action_goal = None
                if self._action_goal_handle is not None:
                    self._action_goal_handle.set_aborted()
                    self._action_goal_handle = None
            # TODO: send feedback
            # self._action_goal_handle.publish_feedback(self._action_feedback_message)


class OgnROS1ActionGripperCommand:
    """This node provides the services to list, read and write prim's attributes
    """
    @staticmethod
    def initialize(graph_context, node):
        pass

    @staticmethod
    def internal_state() -> InternalState:
        return InternalState()

    @staticmethod
    def compute(db) -> bool:
        if not _ros_bridge.is_ros_node_running():
            return False
        if db.internal_state.initialized:
            db.internal_state.step(0)
        else:
            try:
                db.internal_state.usd_context = omni.usd.get_context()
                db.internal_state.dci = _dynamic_control.acquire_dynamic_control_interface()

                if db.internal_state.timeline_event is None:
                    timeline = omni.timeline.get_timeline_interface()
                    db.internal_state.timeline_event = timeline.get_timeline_event_stream() \
                        .create_subscription_to_pop(db.internal_state.on_timeline_event)

                def get_action_name(namespace, controller_name, action_namespace):
                    controller_name = controller_name if controller_name.startswith("/") else "/" + controller_name
                    action_namespace = action_namespace if action_namespace.startswith("/") else "/" + action_namespace
                    return namespace if namespace else "" + controller_name + action_namespace

                def get_relationships(node, usd_context, attribute):
                    stage = usd_context.get_stage()
                    prim = stage.GetPrimAtPath(node.get_prim_path())
                    return prim.GetRelationship(attribute).GetTargets()

                # check for articulation
                path = db.inputs.targetPrim.path
                if not len(path):
                    print("[Warning][semu.robotics.ros_bridge] ROS1 GripperCommand: targetPrim not set")
                    return

                # check for articulation API
                stage = db.internal_state.usd_context.get_stage()
                if not stage.GetPrimAtPath(path).HasAPI(PhysxSchema.PhysxArticulationAPI):
                    print("[Warning][semu.robotics.ros_bridge] ROS1 GripperCommand: {} doesn't have PhysxArticulationAPI".format(path))
                    return

                db.internal_state.articulation_path = path

                # check for gripper joints
                relationships = get_relationships(db.node, db.internal_state.usd_context, "inputs:targetGripperJoints")
                paths = [relationship.GetPrimPath().pathString for relationship in relationships]
                if not paths:
                    print("[Warning][semu.robotics.ros_bridge] ROS1 GripperCommand: targetGripperJoints not set")
                    return

                db.internal_state.gripper_joints_paths = paths

                # create action server
                db.internal_state.shutdown_action_server()

                action_name = get_action_name(db.inputs.nodeNamespace, db.inputs.controllerName, db.inputs.actionNamespace)
                db.internal_state.action_server = actionlib.ActionServer(action_name, 
                                                            control_msgs.msg.GripperCommandAction,
                                                            goal_cb=db.internal_state.on_goal,
                                                            cancel_cb=db.internal_state.on_cancel,
                                                            auto_start=False)
                db.internal_state.action_server.start()
                print("[Info][semu.robotics.ros_bridge] ROS1 GripperCommand: register action {}".format(action_name))

            except ConnectionRefusedError as error:
                print("[Error][semu.robotics.ros_bridge] ROS1 GripperCommand: action server {} not started".format(action_name))
                db.log_error(str(error))
                db.internal_state.initialized = False
                return False
            except Exception as error:
                print("[Error][semu.robotics.ros_bridge] ROS1 GripperCommand: error: {}".format(error))
                db.log_error(str(error))
                db.internal_state.initialized = False
                return False
            
            db.internal_state.initialized = True

        return True
