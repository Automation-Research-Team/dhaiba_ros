#!/usr/bin/env python

import sys
import rospy
import dhaiba_ros

participant = "DhaibaConectNotePub"
topic = "SampleNote.Note"

if len(sys.argv) >= 3:
    participant = sys.argv[1]
    topic = sys.argv[2]

data_TeleportAbsolute = """
x: 0
y: 0
theta: 0
---"""

data_FrameGraph=""

data_FibonacciGoal="""
order: 10
"""

data = ''

if len(sys.argv) >= 4:
    if sys.argv[3] == 't':
        data = data_TeleportAbsolute
    elif sys.argv[3] == 'f':
        data = data_FrameGraph
    elif sys.argv[3] == 'a':
        data = data_FibonacciGoal

print ">>>>> note_pub <<<<< %s %s" % (participant, topic)
obj = dhaiba_ros.note_publisher(participant, topic)

for i in range(2):
    obj.write(data)
    rospy.rostime.wallsleep(0.5)
