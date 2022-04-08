import os
import sys
from distutils.core import setup
from distutils.extension import Extension

from Cython.Distutils import build_ext

# OV python (kit\python\include)
if sys.platform == 'win32':
    raise Exception('Windows is not supported')
    exit(1)
elif sys.platform == 'linux':
    python_library_dir = os.path.join(os.path.dirname(sys.executable), "..", "include")

if not os.path.exists(python_library_dir):
    raise Exception("OV Python library directory not found: {}".format(python_library_dir))

ext_modules = [
    Extension("_ros_bridge",
              [os.path.join("omni", "add_on", "ros_bridge", "ros_bridge.py")],
              library_dirs=[python_library_dir]),
]

setup(
    name = 'omni.add_on.ros_bridge',
    cmdclass = {'build_ext': build_ext},
    ext_modules = ext_modules
)
