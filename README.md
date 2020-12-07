dhaiba_ros: connecting ROS and DhaibaWorks
===
## Introduction

This package provides a set of software for making communications between ROS and [DhaibaWorks](https://www.dhaibaworks.com/). The following two bridging nodes are available;
- **urdf_publisher**: Publish a robot description loaded from the specified ROS parameter as well as its dynamical state obtained from TF to DhaibaWorks.
- **dhaiba_ros_bridge**: Transfer any type of messages between ROS and DhaibaWorks as ROS topics, services or actions.

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

### Subscribe ROS topic and then publish it toward DhaibaWorks
Any ROS topic of arbitrary type can be transferred to DhaibaWorks;

```bash
$ roslaunch dhaiba_ros run.launch  op:=pub participant:=<participant name> element:=<element name>  name:=<topic name>
```
where
- **participant**: specifies participant name of this node as a DhaibaConnect publisher (default: `dhaiba_ros_bridge`)
- **element**: specifies DhaibaWorks element name of the transferred ROS topic, e.g. `tf`. The incoming topic appears as `<participant>/<element>` on the DhaibaWorks side.
- **name**: specifies ROS topic name to be transferred to DhaibaWorks, e.g. `/tf`.

### Subscribe DhaibaWorks topic and then publish it toward ROS
Any DhaibaWoks topic can be transferred to ROS if its message type is known to ROS;
```bash
$ roslaunch dhaiba_ros run.launch op:=sub name:=<topic name> participant:=<participant name> element:=<element name> type:=<message type>
```
where
- **participant**: specifies participant name of the **DhaibaWorks host** with which this node communicates.
- **element**: specifies DhaibaWorks element name to be transferred to ROS.The subscribed topic appears as `<participant>/<element>`.
- **name**: specifies ROS topic name, e.g. `/my_tf`.
- **type**: specifies ROS message type to be transferred from DhaibaWorks, e.g. `tf2_msgs/TFMessage`.

### Send commands from DhaibaWorks to a ROS node as ROS service requests and return the received responses back to DW

```bash
$ roslaunch dhaiba_ros run.launch op:=srv participant:=<participant name> name:=<service name> type:=<service type>
```
where
- **participant**: specifies participant name of the **DhaibaWorks host** with which this node communicates.
- **name**: specifies ROS service name, e.g. `/tutle1/teleport_absolute`.
- **type**: specifies ROS service type to be transferred from DhaibaWorks, e.g. `turtlesim/TeleportAbsolute`.

### Send commands from DhaibaWorks to a ROS node as ROS action goals and returns received feedbacks and results back to DW

```bash
$ roslaunch dhaiba_ros run.launch op:=act participant:=<participant name> name:=<action name> type:=<action type>
```
where
- **participant**: specifies participant name of the **DhaibaWorks host** with which this node communicates.
- **name**: specifies ROS action name, e.g. `/fibonacci`.
- **type**: specifies ROS action type to be transferred from DhaibaWorks, e.g. `actionlib_tutorials/FibonacciAction`.

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

### Communicating over ROS services
response の中身はない（空）だが、視覚的に turtle が動く。
```bash
$ roslaunch dhaiba_ros test_srv.launch
$ rosrun dhaiba_ros pub.py dhaiba_ros_bridge Request.Note t
```

response の中身はあるが、視覚的に turtle が動く等はない。
```bash
$ roslaunch dhaiba_ros test_srv.launch teleport:=false
$ rosrun dhaiba_ros pub.py dhaiba_ros_bridge Request.Note f
```

### Communicating over ROS actions
```bash
$ roslaunch dhaiba_ros test_act.launch
$ rosrun dhaiba_ros pub.py dhaiba_ros_bridge Goal.Note a
```

