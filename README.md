dhaiba_ros
===

## Sending robot_description from ROS to DhaibaWorks

```bash
$ roslaunch dhaiba_ros run.launch [...]
```
The following parameters can be specified:
- **description_param**: Name of ROS parameter containing robot description. (default: `/robot_description`)
- **root_frame**: Frame ID of the root of TF (sub)tree to be published to DhaibaWorks. (default: `world`)
- **rate**: Rate of publishing in Hz (default: `10`)
- **publish_armature**: If true, publish TF (sub)tree as an armature of DhaibaWorks. (default: `true`)
- **publish_elements**: If true, publish geometrical elements as boxes, spheres or meshes. (default: `true`)

## Relaying topics between ROS and DhaibaWorks

```bash
$ python scripts/dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note /trial tf2_msgs/TFMessage
$ python scripts/dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note
```

```bash
$ rosrun dhaiba_ros dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note /trial tf2_msgs/TFMessage
$ rosrun dhaiba_ros dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note
```

```bash
$ rosrun dhaiba_ros dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note /trial tf2_msgs/TFMessage __ns:="abc1" __name:="efg2"
$ rosrun dhaiba_ros dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note __ns:="hij3" __name:="klm4"
```

## Transfer commands from DhaibaWorks as ROS service requests and sending back the responses

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

