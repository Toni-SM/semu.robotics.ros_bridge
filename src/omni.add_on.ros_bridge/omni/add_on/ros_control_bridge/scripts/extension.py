import omni.ext
import carb
try:
    from .. import _ros_control_bridge
except:
    print(">>>> [DEVELOPMENT] import ros_control_bridge")
    from .. import ros_control_bridge as _ros_control_bridge

EXTENSION_NAME = "ROS Control Bridge"


class Extension(omni.ext.IExt):
    def on_startup(self):
        self._roscontrolbridge = None
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        if ext_manager.is_extension_enabled("omni.isaac.ros2_bridge"):
            carb.log_error("ROS Control Bridge extension cannot be enabled if ROS 2 Bridge is enabled")
            ext_manager.set_extension_enabled("omni.add_on.ros_control_bridge", False)
            return

        self._roscontrolbridge = _ros_control_bridge.acquire_ros_control_bridge_interface()
        
    def on_shutdown(self):
        if self._roscontrolbridge is not None:
            _ros_control_bridge.release_ros_control_bridge_interface(self._roscontrolbridge)
        