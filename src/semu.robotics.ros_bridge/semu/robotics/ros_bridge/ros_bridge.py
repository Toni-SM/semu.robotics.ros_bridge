import omni
import carb

import rospy
import rosgraph


def acquire_ros_bridge_interface(ext_id: str = "") -> 'RosBridge':
    """
    Acquire the RosBridge interface

    :param ext_id: The extension id
    :type ext_id: str

    :returns: The RosBridge interface
    :rtype: RosBridge
    """
    return RosBridge()

def release_ros_bridge_interface(bridge: 'RosBridge') -> None:
    """
    Release the RosBridge interface
    
    :param bridge: The RosBridge interface
    :type bridge: RosBridge
    """
    bridge.shutdown()


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
        self._init_ros_node()

    def shutdown(self) -> None:
        """Shutdown the RosBridge interface
        """
        self._timeline_event = None

    def _init_ros_node(self) -> None:
        """Initialize the ROS node
        """
        # check ROS master
        try:
            rosgraph.Master("/rostopic").getPid()
        except:
            print("[Warning][semu.robotics.ros_bridge] {}: ROS master is not running".format(self._node_name))
            return False
        # start ROS node
        try:
            rospy.init_node(self._node_name)
            print("[Info][semu.robotics.ros_bridge] {} node started".format(self._node_name))
        except rospy.ROSException as e:
            print("[Error][semu.robotics.ros_bridge] {}: {}".format(self._node_name, e))
            return False
        return True

    def _on_timeline_event(self, event: 'carb.events._events.IEvent') -> None:
        """Handle the timeline event

        :param event: Event
        :type event: carb.events._events.IEvent
        """
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            pass
        elif event.type == int(omni.timeline.TimelineEventType.PAUSE):
            pass
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            pass
