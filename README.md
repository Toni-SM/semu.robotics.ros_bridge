## ROS Bridge (add-on) for NVIDIA Omniverse Isaac Sim

> This extension allows publishing additional topics and enabling services for agile prototyping of robotic applications in [ROS](https://www.ros.org/)

<br>

### Table of Contents

- [Prerequisites](#prerequisites)
- [Add the extension to NVIDIA Omniverse Isaac Sim and enable it](#extension)
- [Supported components](#components)

<br>

<a name="prerequisites"></a>
### Prerequisites

All Isaac Sim's [ROS Bridge prerequisites](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html#prerequisites) must be fulfilled before running this extension. In addition, this extension requires the following extensions to be present in the Isaac Sim's extension path:

* [omni.usd.schema.add_on](https://github.com/Toni-SM/omni.usd.schema.add_on): USD add-on schemas

* [omni.add_on.ros_bridge_ui](https://github.com/Toni-SM/omni.add_on.ros_bridge_ui): ROS Bridge add-on UI

<br>

<a name="extension"></a>
### Add the extension to NVIDIA Omniverse Isaac Sim and enable it

1. Add the the extension by following the steps described in [Extension Search Paths](https://docs.omniverse.nvidia.com/py/kit/docs/guide/extensions.html#extension-search-paths) or simply download and unzip the latest [release](https://github.com/Toni-SM/omni.add_on.ros_bridge/releases) in one of the extension folders such as ```PATH_TO_OMNIVERSE_APP/exts```

    Git url (git+https) as extension search path: 
    
    ```
    git+https://github.com/Toni-SM/omni.add_on.ros_bridge.git?branch=main&dir=exts
    ```

2. Enable the extension by following the steps described in [Extension Enabling/Disabling](https://docs.omniverse.nvidia.com/py/kit/docs/guide/extensions.html#extension-enabling-disabling)

<br>

<a name="components"></a>
### Supported components

The following components are supported:

* **Compressed Camera (ROS publisher):** publish the topics enabled by each RosCompressedCamera's prim in the stage
    
    The images are published as [sensor_msgs/CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html) (encoded to ```jpeg```)

* **Attribute (ROS service):** enables services for getting and setting the attributes of a prim under the following services. The ROS package [add_on_msgs](https://github.com/Toni-SM/omni.add_on.ros_bridge/releases) contains the definition of the messages (download it and add it to a catkin workspace). A sample code of a [python client application](https://github.com/Toni-SM/omni.add_on.ros_bridge/releases) is also provided

  Prim attributes are retrieved and modified as JSON (applied directly to the data, without keys). Arrays, vectors, matrixes and other numeric classes (```pxr.Gf.Vec3f```, ```pxr.Gf.Matrix4d```, ```pxr.Gf.Quatf```, ```pxr.Vt.Vec2fArray```, etc.) are interpreted as a list of numbers (row first)

  - **add_on_msgs.srv.GetPrims**: Get all prim path under the specified path

    ```yaml
    string path             # get prims at path
    ---
    string[] paths          # list of prim paths
    string[] types          # prim type names
    bool success            # indicate a successful execution of the service
    string message          # informational, e.g. for error messages
    ```
  
  - **add_on_msgs.srv.GetPrimAttributes**: Get prim attribute names and their types
    
    ```yaml
    string path             # prim path
    ---
    string[] names          # list of attribute base names (name used to Get or Set an attribute)
    string[] displays       # list of attribute display names (name displayed in Property tab)
    string[] types          # list of attribute data types
    bool success            # indicate a successful execution of the service
    string message          # informational, e.g. for error messages
    ```
  
  - **add_on_msgs.srv.GetPrimAttribute**: Get prim attribute
    
    ```yaml
    string path             # prim path
    string attribute        # attribute name
    ---
    string value            # attribute value (as JSON)
    string type             # attribute type
    bool success            # indicate a successful execution of the service
    string message          # informational, e.g. for error messages
    ```
  
  - **add_on_msgs.srv.SetPrimAttribute**: Set prim attribute
    
    ```yaml
    string path             # prim path
    string attribute        # attribute name
    string value            # attribute value (as JSON)
    ---
    bool success            # indicate a successful execution of the service
    string message          # informational, e.g. for error messages
    ```
