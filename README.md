## ROS Control Bridge for NVIDIA Omniverse Isaac Sim

> This extension enables the **ROS action interfaces used for controlling robots**. Particularly those used by [MoveIt](https://moveit.ros.org/) to talk with the controllers on the robot

<br>

### Table of Contents

- [Add the extension to NVIDIA Omniverse Issac Sim and enable it](#extension)
- [Add Follow Joint Trajectory action](#follow_joint_trajectory)
- [Add Gripper Command action](#gripper_command)
- [Python API](#python-api)

<br>

<a name="extension"></a>
### Add the extension to NVIDIA Omniverse Issac Sim and enable it

**Prerequisites:**

All Isaac Sim's [ROS Bridge extension prerequisites](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ros_bridge.html#prerequisites) must be fulfilled before running this extension. Also, the [control_msgs](https://wiki.ros.org/control_msgs) package must be installed for the configured ROS distribution

```bash
apt-get install ros-<distro>-control-msgs
```

[pyros-setup](https://pypi.org/project/pyros-setup/) and [rospkg](https://pypi.org/project/rospkg/) python packages must be installed using Isaac Sim's python environment

```bash
cd /isaac-sim/_build/target-deps/kit_sdk_release/_build/target-deps/python

bin/python3 -m pip install pyros-setup rospkg
```

**Setup the extension:**

1. Copy or clone this repository (keeping its name ```omni.add_on.ros_control_bridge```) to the next path: ```/isaac-sim/_build/linux-x86_64/release/exts```

    ```
    cd /isaac-sim/_build/linux-x86_64/release/exts
    git clone https://github.com/Toni-SM/omni.add_on.ros_control_bridge.git omni.add_on.ros_control_bridge
    ```

2. Enable the extension in the menu *Window > Extension Manager* under the same name. When the ros_control_bridge extension is loaded, the ROS Control actions will automatically show up under *Create > Isaac Sim > ROS Control* menu

<br>

<a name="follow_joint_trajectory"></a>
### Add Follow Joint Trajectory action

<br>

<a name="gripper_command"></a>
### Add Gripper Command action

<br>

<a name="python-api"></a>
### Python API

ROS Control action can also be added using the Python API

**Add a Follow Joint Trajectory action**

This code can also be ran inside Omniverse Isaac Sim's Script Editor after the Franka Panda robot is already loaded onto the stage

```python
    from pxr import PhysicsSchema, Sdf
    import omni.usd
    import omni.kit
    import omni.add_on.RosControlBridgeSchema as ROSControlSchema

    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath("/panda")

    # check the robot has articulation
    assert robot_prim.HasAPI(PhysicsSchema.ArticulationAPI)

    # setup ROS action to control the robot
    fjt_prim = ROSControlSchema.RosControlFollowJointTrajectory.Define(stage, Sdf.Path("/ROSControl_FollowJointTrajectory"))

    # adding prefix to the action topics if needed
    fjt_prim.CreateRosNodePrefixAttr("")
    fjt_prim.CreateEnabledAttr(True)

    # adding controller namespaces
    fjt_prim.CreateControllerNameAttr("/robot_controller")
    fjt_prim.CreateActionNamespaceAttr("/follow_joint_trajectory")

    fjt_prim.CreateArticulationPrimRel()
    
    # the follow_joint_trajectory action must be connected to the root of the robot's articulation in order to control it
    ROSControl_prim = stage.GetPrimAtPath("/ROSControl_FollowJointTrajectory")
    ROSControl_prim.GetRelationship("articulationPrim").SetTargets(["/panda"])


    # editor must be playing for control the robot
    editor = omni.kit.editor.get_editor_interface()
    if not editor.is_playing():
        editor.play()
```
**Add a Gripper Command action**

```python
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath("/panda")

    # check the robot has articulation
    assert robot_prim.HasAPI(PhysicsSchema.ArticulationAPI)

    # setup ROS action to control the robot
    gc_prim = ROSControlSchema.RosControlGripperCommand.Define(stage, Sdf.Path("/ROSControl_GripperCommand"))

    # adding prefix to the action topics if needed
    gc_prim.CreateRosNodePrefixAttr("")
    gc_prim.CreateEnabledAttr(True)

    # adding controller namespaces
    gc_prim.CreateControllerNameAttr("/gripper_controller")
    gc_prim.CreateActionNamespaceAttr("/gripper_command")

    gc_prim.CreateArticulationPrimRel()
    
    # the follow_joint_trajectory action must be connected to the root of the robot's articulation in order to control it
    ROSControl_prim = stage.GetPrimAtPath("/ROSControl_GripperCommand")
    ROSControl_prim.GetRelationship("articulationPrim").SetTargets(["/panda", "/panda/panda_hand/panda_finger_joint1", "/panda/panda_hand/panda_finger_joint2"])


    # editor must be playing for control the robot
    editor = omni.kit.editor.get_editor_interface()
    if not editor.is_playing():
        editor.play()
```
