#!/bin/bash

extension_dir=../../exts/omni.add_on.ros_bridge
extension_tree=omni/add_on/ros_bridge

# delete old files
rm -r $extension_dir

mkdir -p $extension_dir/$extension_tree
mkdir -p $extension_dir/$extension_tree/scripts
mkdir -p $extension_dir/$extension_tree/tests
mkdir -p $extension_dir/$extension_tree/packages

cp -r config $extension_dir
cp -r data $extension_dir
cp -r docs $extension_dir

# scripts folder
cp $extension_tree/scripts/extension.py $extension_dir/$extension_tree/scripts

# tests folder
cp $extension_tree/tests/__init__.py $extension_dir/$extension_tree/tests
cp $extension_tree/tests/test_ros_bridge.py $extension_dir/$extension_tree/tests

# packages folder
cp -r $extension_tree/packages/add_on_msgs $extension_dir/$extension_tree/packages

# single files
cp $extension_tree/__init__.py $extension_dir/$extension_tree/
cp $extension_tree/*.so $extension_dir/$extension_tree/

# remove cache
find $extension_dir -name '__pycache__' -exec rm -r {} \;
