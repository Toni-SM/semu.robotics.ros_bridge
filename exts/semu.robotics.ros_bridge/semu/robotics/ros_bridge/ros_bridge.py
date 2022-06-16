import omni
import carb

import rospy
import rosgraph


_ROS_NODE_RUNNING = False

def acquire_ros_bridge_interface(ext_id: str = "") -> 'RosBridge':
    """Acquire the RosBridge interface

    :param ext_id: The extension id
    :type ext_id: str

    :returns: The RosBridge interface
    :rtype: RosBridge
    """
    return RosBridge()

def release_ros_bridge_interface(bridge: 'RosBridge') -> None:
    """Release the RosBridge interface
    
    :param bridge: The RosBridge interface
    :type bridge: RosBridge
    """
    bridge.shutdown()

def is_ros_master_running() -> bool:
    """Check if the ROS master is running

    :returns: True if the ROS master is running, False otherwise
    :rtype: bool
    """
     # check ROS master
    try:
        rosgraph.Master("/rostopic").getPid()
    except:
        return False
    return True

def is_ros_node_running() -> bool:
    """Check if the ROS node is running

    :returns: True if the ROS node is running, False otherwise
    :rtype: bool
    """
    return _ROS_NODE_RUNNING


class RosBridge:
    def __init__(self) -> None:
        """Initialize the RosBridge interface
        """
        self._node_name = carb.settings.get_settings().get("/exts/semu.robotics.ros_bridge/nodeName")

        # omni objects and interfaces
        self._timeline = omni.timeline.get_timeline_interface()
        
        # events
        self._timeline_event = self._timeline.get_timeline_event_stream().create_subscription_to_pop(self._on_timeline_event)

        # ROS node
        if is_ros_master_running():
            self._init_ros_node()

    def shutdown(self) -> None:
        """Shutdown the RosBridge interface
        """
        self._timeline_event = None
        rospy.signal_shutdown("semu.robotics.ros_bridge shutdown")

    def _init_ros_node(self) -> None:
        global _ROS_NODE_RUNNING
        """Initialize the ROS node
        """
        try:
            rospy.init_node(self._node_name, disable_signals=False)
            _ROS_NODE_RUNNING = True
            print("[Info][semu.robotics.ros_bridge] {} node started".format(self._node_name))
        except rospy.ROSException as e:
            print("[Error][semu.robotics.ros_bridge] {}: {}".format(self._node_name, e))

    def _on_timeline_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the timeline event

        :param event: Event
        :type event: carb.events._events.IEvent
        """
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if is_ros_master_running():
                self._init_ros_node()
        elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
            pass
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            pass
