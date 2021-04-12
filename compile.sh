# delete old data
rm -r build
rm _ros*.so
rm omni/add_on/ros_control_bridge/*.so
rm omni/add_on/ros_control_bridge/*.c
rm omni/add_on/RosControlBridgeSchema/*.so
rm omni/add_on/RosControlBridgeSchema/*.c

# compile code
/isaac-sim/_build/target-deps/kit_sdk_release/_build/target-deps/python/bin/python3 compile.py build_ext --inplace

# move compiled file
mv _ros_control_bridge.cpython-36m-x86_64-linux-gnu.so omni/add_on/ros_control_bridge/
mv _rosControlBridgeSchema.cpython-36m-x86_64-linux-gnu.so omni/add_on/RosControlBridgeSchema/_rosControlBridgeSchema.so

# delete temporal data
rm -r build
rm omni/add_on/ros_control_bridge/*.c
rm omni/add_on/RosControlBridgeSchema/*.c