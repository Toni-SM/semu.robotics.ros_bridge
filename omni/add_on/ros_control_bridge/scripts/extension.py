import omni.ext

from .menu import RosControlBridgeMenu

EXTENSION_NAME = "ROS Control Bridge"


class Extension(omni.ext.IExt):
    def on_startup(self):
        self._menu = RosControlBridgeMenu()

    def on_shutdown(self):
        self._menu.shutdown()
        self._menu = None
