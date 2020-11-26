$ python ./scripts/dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note
$ python ./scripts/dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note

$ rosrun dhaiba_ros dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note
$ rosrun dhaiba_ros dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note

$ rosrun dhaiba_ros dhaiba_connect_topic.py sub DhaibaConectNoteSub DhaibaConectNotePub/Trial.Note __ns:="abc1" __name:="efg2"
$ rosrun dhaiba_ros dhaiba_connect_topic.py pub /tf DhaibaConectNotePub Trial.Note __ns:="hij3" __name:="klm4"

