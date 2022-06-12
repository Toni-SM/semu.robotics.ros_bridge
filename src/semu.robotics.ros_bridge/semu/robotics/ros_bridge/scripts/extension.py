import os
import sys
import carb
import omni.ext
import omni.graph.core as og
try:
    from .. import _ros_bridge
except:
    print(">>>> [DEVELOPMENT] import ros_bridge")
    from .. import ros_bridge as _ros_bridge


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self._rosbridge = None
        self._extension_path = None
        ext_manager = omni.kit.app.get_app().get_extension_manager()
        if ext_manager.is_extension_enabled("omni.isaac.ros2_bridge"):
            carb.log_error("ROS Bridge external extension cannot be enabled if ROS 2 Bridge is enabled")
            ext_manager.set_extension_enabled("semu.robotics.ros_bridge", False)
            return

        self._extension_path = ext_manager.get_extension_path(ext_id)
        sys.path.append(os.path.join(self._extension_path, "semu", "robotics", "ros_bridge", "packages"))

        self._rosbridge = _ros_bridge.acquire_ros_bridge_interface(ext_id)

        og.register_ogn_nodes(__file__, "semu.robotics.ros_bridge")
        
    def on_shutdown(self):
        if self._extension_path is not None:
            sys.path.remove(os.path.join(self._extension_path, "semu", "robotics", "ros_bridge", "packages"))
            self._extension_path = None
        if self._rosbridge is not None:
            _ros_bridge.release_ros_bridge_interface(self._rosbridge)
            self._rosbridge = None
