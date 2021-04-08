import omni.ext

from .. import _ros_control_bridge
from .menu import RosControlBridgeMenu

EXTENSION_NAME = "ROS Control Bridge"


class Extension(omni.ext.IExt):
    def on_startup(self):
        self._roscontrolbridge = _ros_control_bridge.acquire_roscontrolbridge_interface()
        self._menu = RosControlBridgeMenu()

    def on_shutdown(self):
        _ros_control_bridge.release_roscontrolbridge_interface(self._roscontrolbridge)
        self._menu.shutdown()
        self._menu = None
