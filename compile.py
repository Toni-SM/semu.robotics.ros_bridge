from distutils.core import setup
from distutils.extension import Extension

from Cython.Distutils import build_ext


ext_modules = [
    Extension("_ros_control_bridge",
             ["omni/add_on/ros_control_bridge/ros_control_bridge.py"]),
    Extension("_rosControlBridgeSchema",  
             ["omni/add_on/RosControlBridgeSchema/rosControlBridgeSchema.py"]),
]

setup(
    name = 'omni.add_on.ros_control_bridge',
    cmdclass = {'build_ext': build_ext},
    ext_modules = ext_modules
)
