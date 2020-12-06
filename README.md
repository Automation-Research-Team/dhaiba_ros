dhaiba_ros: connecting ROS and DhaibaWorks
===
## Introduction

This package provides a set of software for making communications between ROS and [DhaibaWorks](https://www.dhaibaworks.com/).

## Installation
Firstly, `DhaibaConnect` plugin available from [portal site of DhaibaWorks](https://dhaibaweb.azurewebsites.net/start.php?id=EFB14EBA21CB85B13CAAA817E7CDA7C3257BB412E1CE6589BDF735780B1DFCCA7B91B2B2DD6151CA0C4001F11540899BA2ABA8476D77540139F4D402DBCF5E4D528096C7740D8CBA) should be installed to your `DhaibaWorks` environment on macOS or Windows.

Secondly, `DhaibaConnectN` should be installed on the Linux environment following the instructions described in [the overview of DhaibaConnect](doc/DhaibaConnect.pdf).

Finally, you should install `dhaiba_ros` package;

```bash
$ sudo pip install pybind11 pybind11-cmake
$ cd catkin_ws/src
$ git clone https://gitlab.com/art-aist-private/dhaiba_ros
$ catkin build dhaiba_ros
```

## urdf_publisher
The `urdf_publisher` publishes entire description of ROS world stored in a ROS parameter typically named `/robot_description` with [URDF](http://wiki.ros.org/urdf) format. In addition, it also dynamically updates the geometrical state of the world according to the [TF](http://wiki.ros.org/tf) tree. The former is published as a set of DhaibaWorks topics of `ShapeBox` and `ShapeSphere` types for boxes and spheres respectively, or `BinaryFile` type for meshes. The latter is published as a topic of `Armature` type.

The `urdf_publisher` is launched by the following command;

```bash
$ roslaunch dhaiba_ros publish_urdf.launch [...]
```
where
- **description_param**: Name of ROS parameter containing robot description. (default: `/robot_description`)
- **root_frame**: Frame ID of the root of TF (sub)tree to be published to DhaibaWorks. (default: `world`)
- **rate**: Rate of publishing in Hz (default: `10`)
- **publish_armature**: If true, publish TF (sub)tree as an armature of DhaibaWorks. (default: `true`)
- **publish_elements**: If true, publish geometrical elements such as boxes, spheres and meshes. (default: `true`)

## dhaiba_ros_bridge
The `dhaiba_ros_bridge` provides functions for interchanging any type of messages between `ROS` and `DhabaWorks`. Messages are encoded with `YAML` format and transferred as `Note` of `DhaibaConnect` . The following three functions are supported.

### Subscribe ROS topic and then publish it to DhaibaWorks
Any ROS topic of arbitrary type can be transferred to DhaibaWorks;

```bash
$ roslaunch dhaiba_ros run.launch [name:=<ROS node name>] op:=pub msg:=<ROS topic name> dw_msg:=<DhaibaWorks topic name> 
```
where
- **name**: specifies ROS node name. This parameter also gives participant's name appeared in DhaibaWorks (default: `dhaiba_ros_bridge`)
- **msg**: specifies ROS topic name to be transferred to DhaibaWorks, e.g. `/tf`.
- **dw_msg**: specifies DhaibaWorks element name for the transferred ROS topic, e.g. `tf`. The incoming topic appears as `<name>/<dw_msg>` on the DhaibaWorks side. 

### Subscribe DhaibaWorks topic and then publish it to ROS
Any DhaibaWoks topic can be transferred to ROS if its message type is known to ROS;
```bash
$ roslaunch dhaiba_ros run.launch [name:=<ROS node name>] op:=sub msg:=<ROS topic name> dw_msg:=<DhaibaWorks topic name> type:=<ROS message type>
```
where
- **name**: specifies ROS node name. (default: `dhaiba_ros_bridge`)
- **msg**: specifies ROS topic name, e.g. `/my_tf`.
- **dw_msg**: specifies DhaibaWorks topic name to be transferred to ROS.Note that the name must include the participant name of the DhaibaConnect publisher, e.g. `dhaiba_ros_publisher/tf`.
- **type**: specifies ROS message type to be transferred from DhaibaWorks, e.g. `tf2_msgs/TFMessage`.

### Transfer commands from DhaibaWorks to ROS nodes as ROS service requests and then return the responses from the nodes back to DW


## Testing
Three commands are available for testing `dahiba_ros_bridge`.

### Transferring robot descriptions
The following command load the specified URDF file into a ROS parameter named `/robot_description` and diplays in `RViz`. It also launches `urdf_publisher` which transfers the description to DhaibaWorks.
```bash
$ roslaunch dhaiba_ros test_urdf.launch [model:=<urdf file>]
```
The default model is `dhaiba_ros/urdf/test_arm.urdf`.

### Transferring topics
The following command launches two ROS nodes `dhaiba_ros_publisher` and `dhaiba_ros_subscriber`. The former subscribes ROS topic `/tf` and publishes it as DhaibaWorks Note topic named `dhaiba_ros_publisher/tf`. The latter subscribes the topic published by the former and publishes it as ROS topic named `/my_tf`. Therefore, this test can be performed without using DhaibaWorks.
```bash
$ roslaunch dhaiba_ros test_topic.launch
``` 

### Interpretation as ROS services
response の中身はない（空）だが、視覚的に turtle が動く。

```bash
$ python scripts/dhaiba_connect_topic.py srv DhaibaConectNoteService Request.Note Response.Note /turtle1/teleport_absolute turtlesim/TeleportAbsolute
$ python test/sub.py DhaibaConectNoteService Response.Note
$ python test/pub.py DhaibaConectNoteService Request.Note t
```

response の中身はあるが、視覚的に turtle が動く等はない。

```bash
$ python scripts/dhaiba_connect_topic.py srv DhaibaConectNoteService Request.Note Response.Note /turtle_pointer/tf2_frames tf2_msgs/FrameGraph
$ python test/sub.py DhaibaConectNoteService Response.Note
$ python test/pub.py DhaibaConectNoteService Request.Note f
```

