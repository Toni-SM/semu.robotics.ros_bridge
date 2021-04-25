## ROS Control Bridge for NVIDIA Omniverse Isaac Sim

> This extension enables the **ROS action interfaces used for controlling robots**. Particularly those used by [MoveIt](https://moveit.ros.org/) to talk with the controllers on the robot. You can combine Isaac Sim, ROS Control, and MoveIt for a powerful robotics development platform.

<br>

### Table of Contents

- [Setup the extension in NVIDIA Omniverse Isaac Sim and enable it](#extension)
- [Control your robot using MoveIt](#control)
- [Add Follow Joint Trajectory action](#follow_joint_trajectory)
- [Add Gripper Command action](#gripper_command)
- [Python API](#python-api)

<br>

<a name="extension"></a>
### Setup the extension in NVIDIA Omniverse Isaac Sim and enable it

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

<a name="control"></a>
### Control your robot using MoveIt

#### Concepts

MoveIt's *move_group* talks to the robot through ROS topics and actions. Three interfaces are required to control the robot. Go to the [MoveIt](https://moveit.ros.org/) website to read more about the [concepts](https://moveit.ros.org/documentation/concepts/) behind Moveit

- **Joint State information:** the ```/joint_states``` topic (publisher) is used to determining the current state information. Go to [Add Joint State publisher](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ros_bridge.html#add-joint-state-publisher) to add a publisher and setup it

- **Transform information:** the ROS TF library is used to monitor the transform information. Go to [Add TF topics](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ros_bridge.html#add-tf-topics) to add a tranform tree topic and setup it

- **Controller Interface:** *move_group* talks to the controllers on the robot using the *FollowJointTrajectory* action interface. However, it is possible to use other controller interfaces to control the robot

#### Configuration

**Isaac Sim side**

1. Import your robot from an URDF file using the [URDF Importer](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/urdf.html) extension. Go to [ROS Wiki: Using Xacro](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File#Using_Xacro) to convert your robot description form ```.xacro``` to ```.urdf``` 

2. Add and setup a [Joint State topic](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ros_bridge.html#add-joint-state-topics) using the [ROS Bridge](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ros_bridge.html) extension

3. Add and setup a [TF topic](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ros_bridge.html#add-tf-topics) topic using the [ROS Bridge](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ros_bridge.html) extension

4. Add the corresponding ROS control actions implemented by this extension (**see sections below**) according to your robot application requirements 

    **Note:** At that point, it is recommendable to setup the MoveIt YAML configuration file first to know the controllers' names and the action namespaces

5. Play the editor to activate the respective topics and actions. Remember the editor must be playing before launching the *move_group*

**MoveIt side**

1. Launch the [MoveIt Setup Assistant](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) and generate a new configuration package loading the same ```.xacro``` or ```.urdf``` used in Isaac Sim. Also, select the same ROS controller interfaces inside the step *ROS Control*
  
    ```bash
    roslaunch moveit_setup_assistant setup_assistant.launch
    ```
2. Launch the *move_group* using the configured controllers

    ```bash
    roslaunch moveit_config sample.launch
    ```

    **```sample.launch``` file:** replace ```moveit_config``` with the generated folder's name and ```/panda/joint_state``` with the Isaac Sim joint state topic
    
    ```xml
    <launch>
        <!-- Load the URDF, SRDF and other .yaml configuration files on the param server -->
        <include file="$(find moveit_config)/launch/planning_context.launch">
            <arg name="load_robot_description" value="true"/>
        </include>

        <!-- Publish joint states -->
        <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
            <rosparam param="/source_list">["/panda/joint_state"]</rosparam>
        </node>
        <node name="joint_state_desired_publisher" pkg="topic_tools" type="relay" args="joint_states joint_states_desired"/>

        <!-- Given the published joint states, publish tf for the robot links -->
        <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" />

        <!-- Run the main MoveIt executable -->
        <include file="$(find moveit_config)/launch/move_group.launch">
            <arg name="allow_trajectory_execution" value="true"/>
            <arg name="info" value="true"/>
            <arg name="debug" value="false"/>
        </include>

        <!-- Run Rviz (optional) -->
        <include file="$(find moveit_config)/launch/moveit_rviz.launch">
            <arg name="debug" value="false"/>
        </include>
    </launch>
    ```

    Also, you can modify the ```demo.launch``` file and disable the fake controller execution inside the main MoveIt executable

    ```xml
    <arg name="fake_execution" value="false"/>
    ```

3. Use the Move Group [C++](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_interface/move_group_interface_tutorial.html) or [Python](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html) interfaces to control the robot

<br>

<a name="follow_joint_trajectory"></a>
### Add Follow Joint Trajectory action

<br>

<a name="gripper_command"></a>
### Add Gripper Command action

<br>

<a name="python-api"></a>
### Python API

ROS Control actions can be added using the Python API

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
    
    # the gripper_command action must be connected to the root of the robot joint and the end effector joints, following this strict order, to control them
    ROSControl_prim = stage.GetPrimAtPath("/ROSControl_GripperCommand")
    ROSControl_prim.GetRelationship("articulationPrim").SetTargets(["/panda", "/panda/panda_hand/panda_finger_joint1", "/panda/panda_hand/panda_finger_joint2"])


    # editor must be playing for control the end-effector
    editor = omni.kit.editor.get_editor_interface()
    if not editor.is_playing():
        editor.play()
```
