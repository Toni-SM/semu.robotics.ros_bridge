import omni.ext
import carb
try:
    from .. import _ros_bridge
except:
    print(">>>> [DEVELOPMENT] import ros_bridge")
    from .. import ros_bridge as _ros_bridge


class Extension(omni.ext.IExt):
    def on_startup(self):
        self._rosbridge = None
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        if ext_manager.is_extension_enabled("omni.isaac.ros2_bridge"):
            carb.log_error("ROS Bridge (add-on) extension cannot be enabled if ROS 2 Bridge is enabled")
            ext_manager.set_extension_enabled("omni.add_on.ros_bridge", False)
            return

        self._rosbridge = _ros_bridge.acquire_ros_bridge_interface()
        
    def on_shutdown(self):
        if self._rosbridge is not None:
            _ros_bridge.release_ros_bridge_interface(self._rosbridge)
        