#!/usr/bin/env python

import sys
import dhaiba_ros

def callback(data):
    print('callback:', data)

participant = "DhaibaConectNoteSub"
topic = "DhaibaConectNotePub/SampleNote.Note"

if len(sys.argv) >= 3:
    participant = sys.argv[1]
    topic = sys.argv[2]

print ">>>>> note_sub <<<<< %s %s" % (participant, topic)
obj = dhaiba_ros.note_subscriber(participant, participant+'/'+topic, callback)
