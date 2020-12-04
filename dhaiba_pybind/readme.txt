[ publish & subscribe ]

$ python scripts/dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note /trial tf2_msgs/TFMessage
$ python scripts/dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note

$ rosrun dhaiba_ros dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note /trial tf2_msgs/TFMessage
$ rosrun dhaiba_ros dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note

$ rosrun dhaiba_ros dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note /trial tf2_msgs/TFMessage __ns:="abc1" __name:="efg2"
$ rosrun dhaiba_ros dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note __ns:="hij3" __name:="klm4"

[ service ]

response の中身はない（空）だが、視覚的に turtle が動く。

$ python scripts/dhaiba_connect_topic.py srv DhaibaConectNoteService Request.Note Response.Note /turtle1/teleport_absolute turtlesim/TeleportAbsolute
$ python test/sub.py DhaibaConectNoteService Response.Note
$ python test/pub.py DhaibaConectNoteService Request.Note t

response の中身はあるが、視覚的に turtle が動く等はない。

$ python scripts/dhaiba_connect_topic.py srv DhaibaConectNoteService Request.Note Response.Note /turtle_pointer/tf2_frames tf2_msgs/FrameGraph
$ python test/sub.py DhaibaConectNoteService Response.Note
$ python test/pub.py DhaibaConectNoteService Request.Note f

