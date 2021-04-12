# Run to compile the code
# 
# rm _ros_control_bridge.cpython-36m-x86_64-linux-gnu.so
# /isaac-sim/_build/target-deps/kit_sdk_release/_build/target-deps/python/bin/python3 compile.py build_ext --inplace


from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

ext_modules = [
    Extension("_ros_control_bridge",  ["ros_control_bridge.py"]),
]

setup(
    name = '_ros_control_bridge',
    cmdclass = {'build_ext': build_ext},
    ext_modules = ext_modules
)

