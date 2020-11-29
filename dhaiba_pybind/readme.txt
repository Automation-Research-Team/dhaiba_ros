$ python ../scripts/dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note /trial tf2_msgs/TFMessage
$ python ./scripts/dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note

$ rosrun dhaiba_ros dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note /trial tf2_msgs/TFMessage
$ rosrun dhaiba_ros dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note

$ rosrun dhaiba_ros dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note /trial tf2_msgs/TFMessage __ns:="abc1" __name:="efg2"
$ rosrun dhaiba_ros dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note __ns:="hij3" __name:="klm4"

