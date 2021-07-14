# omni.add_on.ros_bridge
ROS Bridge (add-on) for NVIDIA Omniverse Isaac Sim

> This extension enables **publishing additional rostopics** useful for robotic applications using [ROS](https://www.ros.org/).

<br>

### Table of Contents

- [Prerequisites](#prerequisites)
- [Add the extension to NVIDIA Omniverse Isaac Sim and enable it](#extension)
- [Supported components](#components)

<br>

<a name="prerequisites"></a>
### Prerequisites

All Isaac Sim's [ROS Bridge extension prerequisites](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html#prerequisites) must be fulfilled before running this extension. In addition, this extension requires the following extensions to be present in the Isaac Sim's extension path:

* [omni.usd.schema.add_on](https://github.com/Toni-SM/omni.usd.schema.add_on): USD add-on schemas
* [omni.add_on.ros_bridge_ui](https://github.com/Toni-SM/omni.add_on.ros_bridge_ui): ROS Bridge add-on UI

<br>

<a name="extension"></a>
### Add the extension to NVIDIA Omniverse Isaac Sim and enable it

1. Download the latest [release](https://github.com/Toni-SM/omni.add_on.ros_bridge/releases), or any release according to your Isaac Sim version, and unzip it into the Isaac Sim's extension path (```/isaac-sim/exts``` for containers or ```~/.local/share/ov/pkg/isaac_sim-2021.1.0/exts``` for native workstations)
2. Enable the extension in the menu *Window > Extensions* under the same name

<br>

<a name="components"></a>
### Supported components

The following components are supported:

* **Compressed Camera:** publish the topics enabled by each RosCompressedCamera's prim in the stage.
    
    The images are published encoded to ```jpeg``` as [sensor_msgs/CompressedImage](https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html)
