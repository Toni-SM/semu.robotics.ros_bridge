## ROS Control Bridge (add-on) for NVIDIA Omniverse Isaac Sim

> This extension enables the **ROS action interfaces used for controlling robots**. Particularly those used by [MoveIt](https://moveit.ros.org/) to talk with the controllers on the robot (**FollowJointTrajectory** and **GripperCommand**)

<br>

### Table of Contents

- [Prerequisites](#prerequisites)
- [Add the extension to an NVIDIA Omniverse app and enable it](#extension)
- [Control your robot using MoveIt](#control)
- [Configure a FollowJointTrajectory action](#follow_joint_trajectory)
- [Configure a GripperCommand action](#gripper_command)

<br>

<a name="prerequisites"></a>
### Prerequisites


All prerequisites described in [ROS & ROS2 Bridge](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html) must be fulfilled before running this extension. In addition, this extension requires the following extensions to be present in the Isaac Sim:

- [omni.usd.schema.add_on](https://github.com/Toni-SM/omni.usd.schema.add_on): USD add-on schemas

<br>

<a name="extension"></a>
### Add the extension to an NVIDIA Omniverse app and enable it

1. Add the the extension by following the steps described in [Extension Search Paths](https://docs.omniverse.nvidia.com/py/kit/docs/guide/extensions.html#extension-search-paths) or simply download and unzip the latest [release](https://github.com/Toni-SM/omni.usd.schema.add_on/releases) in one of the extension folders such as ```PATH_TO_OMNIVERSE_APP/exts```

    Git url (git+https) as extension search path: 
    
    ```
    git+https://github.com/Toni-SM/omni.add_on.ros_control_bridge.git?branch=main&dir=exts
    ```

2. Enable the extension by following the steps described in [Extension Enabling/Disabling](https://docs.omniverse.nvidia.com/py/kit/docs/guide/extensions.html#extension-enabling-disabling)

<br>

<a name="control"></a>
### Control your robot using MoveIt

#### Concepts

MoveIt's *move_group* talks to the robot through ROS topics and actions. Three interfaces are required to control the robot. Go to the [MoveIt](https://moveit.ros.org/) website to read more about the [concepts](https://moveit.ros.org/documentation/concepts/) behind it

- **Joint State information:** the ```/joint_states``` topic (publisher) is used to determining the current state information

- **Transform information:** the ROS TF library is used to monitor the transform information

- **Controller Interface:** *move_group* talks to the controllers on the robot using the *FollowJointTrajectory* action interface. However, it is possible to use other controller interfaces to control the robot

#### Configuration

**Isaac Sim side**

1. Import your robot from an URDF file using the [URDF Importer](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_urdf.html) extension. Go to [ROS Wiki: Using Xacro](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File#Using_Xacro) to convert your robot description form ```.xacro``` to ```.urdf``` 

2. Add a *Joint State* topic using the menu (*Create > Isaac > ROS > Joint State*) according to the [ROS Bridge](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html) extension

3. Add a *TF topic* using the menu (*Create > Isaac > ROS > Pose Tree*) according to the [ROS Bridge](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html) extension

4. Add the corresponding ROS control actions implemented by this extension (**see the sections below**) according to your robotic application's requirements 

    **Note:** At that point, it is recommendable to setup the MoveIt YAML configuration file first to know the controllers' names and the action namespaces

5. Play the editor to activate the respective topics and actions. Remember the editor must be playing before launching the *move_group*

**MoveIt side**

1. Launch the [MoveIt Setup Assistant](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html) and generate a new configuration package loading the same ```.xacro``` or ```.urdf``` used in Isaac Sim. Also, select the same ROS controller interfaces inside the step *ROS Control*
  
    ```bash
    roslaunch moveit_setup_assistant setup_assistant.launch
    ```
2. Launch the *move_group* using the configured controllers

    For example, you can use the next launch file (sample.launch) which is configured to support the *FollowJointTrajectory* and *GripperCommand* controllers (panda_gripper_controllers.yaml) by replacing the ```panda_moveit_config``` with the generated folder's name and ```/panda/joint_state``` with the Isaac Sim joint state topic name

    ```bash
    roslaunch moveit_config sample.launch
    ```
    
    panda_gripper_controllers.yaml
    ```yaml
    controller_list:
    - name: panda_arm_controller
        action_ns: follow_joint_trajectory
        type: FollowJointTrajectory
        default: true
        joints:
        - panda_joint1
        - panda_joint2
        - panda_joint3
        - panda_joint4
        - panda_joint5
        - panda_joint6
        - panda_joint7
    - name: panda_gripper
        action_ns: gripper_command
        type: GripperCommand
        default: true
        joints:
        - panda_finger_joint1
        - panda_finger_joint2
    ```

    sample.launch
    ```xml
    <launch>
        <!-- Load the URDF, SRDF and other .yaml configuration files on the param server -->
        <include file="$(find panda_moveit_config)/launch/planning_context.launch">
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
        <include file="$(find panda_moveit_config)/launch/move_group.launch">
            <arg name="allow_trajectory_execution" value="true"/>
            <arg name="info" value="true"/>
            <arg name="debug" value="false"/>
        </include>

        <!-- Run Rviz (optional) -->
        <include file="$(find panda_moveit_config)/launch/moveit_rviz.launch">
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
### Configure a FollowJointTrajectory action

To add a ```FollowJointTrajectory``` action go to menu *Create > Isaac > ROS Control* and select *Follow Joint Trajectory*

To connect the schema with the robot of your choice, select the root of the articulation tree by pressing the *Add Target(s)* button under the ```articulationPrim``` field. Press the play button in the editor, and the topics related to this action will be activated for use

The communication shall take place in the namespace defined by the following fields:

```python
rosNodePrefix + controllerName + actionNamespace
```

<br>

The following figure shows a *FollowJointTrajectory* schema configured to match the ```panda_gripper_controllers.yaml``` described above

![followjointtrajectory](https://user-images.githubusercontent.com/22400377/123482177-22aee580-d605-11eb-83da-f360310ecd14.png)

<br>

<a name="gripper_command"></a>
### Configure a GripperCommand action


To add a ```GripperCommand``` action go to menu *Create > Isaac > ROS Control* and select *Gripper Command*

To connect the schematic to the end-effector of your choice, first select the root of the articulation tree and then select each of the end-effector's articulations to control following this strict order. This can be done by pressing the *Add Target(s)* button repeatedly under the ```articulationPrim``` field. Press the play button in the editor, and the topics related to this action will be activated for use

The communication shall take place in the namespace defined by the following fields:

```python
rosNodePrefix + controllerName + actionNamespace
```

The ```GripperCommand``` action definition doesn't specify which joints will be controlled. The value manage by this action will affect all the specified joints equally

<br>

The following figure shows a *GripperCommand* schema configured to match the ```panda_gripper_controllers.yaml``` described above

![grippercommand](https://user-images.githubusercontent.com/22400377/123482246-39553c80-d605-11eb-840b-5834a5e192b2.png)
