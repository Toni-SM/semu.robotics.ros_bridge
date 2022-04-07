export LIBRARY_PATH=/isaac-sim/kit/python/include

# delete old data
rm -r build
rm _ros*.so
rm omni/add_on/ros_bridge/*.so
rm omni/add_on/ros_bridge/*.c

# compile code
/isaac-sim/kit/python/bin/python3 compile.py build_ext --inplace

# move compiled file
mv _ros_bridge.cpython-37m-x86_64-linux-gnu.so omni/add_on/ros_bridge/

# delete temporal data
rm -r build
rm omni/add_on/ros_bridge/*.c

