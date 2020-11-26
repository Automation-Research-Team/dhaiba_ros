#!/usr/bin/env python

from __future__ import division, print_function

NAME='dhaiba_connect_topic'

import os
import sys
import socket
import time
import traceback
from operator import itemgetter
import genpy

import roslib.message
import rosgraph
import rospy

import dhaiba_pybind

class DhaibaConnectTopicException(Exception):
    pass

def _check_master():
    try:
        rosgraph.Master('/'+NAME).getPid()
    except socket.error as e:
        sys.stderr.write("ERROR(check_master): %s\n" % str(e))
        raise DhaibaConnectTopicException("Unable to communicate with master!")
    
def _master_get_topic_types(master):
    try:
        val = master.getTopicTypes()
    except Fault:
        sys.stderr.write("WARNING(master_get_topic_types): getTopicTypes is Fault\n")
        val = master.getPublishedTopics('/')
    return val

def _sleep(duration):
    rospy.rostime.wallsleep(duration)

def _get_nested_attribute(msg, nested_attributes):
    value = msg
    for attr in nested_attributes.split('/'):
        value = getattr(value, attr)
    return value

def _get_topic_type(topic):
    try:
        val = _master_get_topic_types(rosgraph.Master('/'+NAME))
    except socket.error as e:
        sys.stderr.write("ERROR(_get_topic_type): %s\n" % str(e))
        raise DhaibaConnectTopicException("Unable to communicate with master!")

    matches = [(t, t_type) for t, t_type in val if t == topic]
    if not matches:
        matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
        matches.sort(key=itemgetter(0), reverse=True)
        while matches:
            t, t_type = matches[0]
            msg_class = roslib.message.get_message_class(t_type)
            if not msg_class:
                break
            msg = msg_class()
            nested_attributes = topic[len(t) + 1:].rstrip('/')
            nested_attributes = nested_attributes.split('[')[0]
            if nested_attributes == '':
                break
            try:
                _get_nested_attribute(msg, nested_attributes)
            except AttributeError:
                matches.pop(0)
                continue
            matches = [(t, t_type)]
            break
    if matches:
        t, t_type = matches[0]
        if t_type == rosgraph.names.ANYTYPE:
            return None, None, None
        return t_type, t, None
    else:
        return None, None, None

def get_topic_type(topic, blocking=False):
    topic_type, real_topic, msg_eval = _get_topic_type(topic)
    if topic_type:
        return topic_type, real_topic, msg_eval
    elif blocking:
        sys.stderr.write("WARNING: [%s] does not appear to be published yet\n"%topic)
        while not rospy.is_shutdown():
            topic_type, real_topic, msg_eval = _get_topic_type(topic)
            if topic_type:
                return topic_type, real_topic, msg_eval
            else:
                _sleep(0.1)
    return None, None, None

def get_topic_class(topic, blocking=False):
    topic_type, real_topic, msg_eval = get_topic_type(topic, blocking=blocking)
    if topic_type is None:
        return None, None, None
    msg_class = roslib.message.get_message_class(topic_type)
    if not msg_class:
        raise DhaibaConnectTopicException("Cannot load message class for [%s]. Are your messages built?" % topic_type)
    return msg_class, real_topic, msg_eval

class CallbackPublisher(object):
    def __init__(self, topic, dhaiba):
        if topic and topic[-1] == '/':
            topic = topic[:-1]
        self.topic = topic
        self.dhaiba = dhaiba

        self.msg_eval = None

        self.prefix = ''
        self.suffix = '\n---'
        
        self.done = False
        self.str_fn = self.custom_strify_message
        self.first = True

        self.last_topic = None
        self.last_msg_eval = None

    def custom_strify_message(self, val, indent='', type_information=None):
        if type_information and type_information.startswith('uint8['):
            val = [ord(x) for x in val]
        return genpy.message.strify_message(val)

    def callback(self, data, callback_args):
        topic = callback_args['topic']
        type_information = callback_args.get('type_information', None)
        try:
            msg_eval = self.msg_eval
            if topic == self.topic:
                pass
            elif self.topic.startswith(topic + '/'):
                if topic == self.last_topic:
                    msg_eval = self.last_msg_eval
                else:
                    self.last_msg_eval = msg_eval
                    self.last_topic = topic
            else:
                return
            if msg_eval is not None:
                data = msg_eval(data)
            if data is not None:
                cur_msg = self.str_fn(data, type_information=type_information)
                sys.stdout.write(self.prefix + cur_msg + self.suffix + '\n')
                sys.stdout.flush()
                self.dhaiba.write(cur_msg)
        except IOError:
            self.done = True
        except:
            self.done = True
            traceback.print_exc()

def _topic_pub(topic, cb):
    _check_master()
    # rospy.init_node(NAME, anonymous=True)
    msg_class, real_topic, msg_eval = get_topic_class(topic, blocking=True)
    if msg_class is None:
        return
    cb.msg_eval = msg_eval
    type_information = None
    if len(topic) > len(real_topic):
        subtopic = topic[len(real_topic):]
        subtopic = subtopic.strip('/')
        if subtopic:
            fields = subtopic.split('/')
            submsg_class = msg_class
            while fields:
                field = fields[0].split('[')[0]
                del fields[0]
                index = submsg_class.__slots__.index(field)
                type_information = submsg_class._slot_types[index]
                if fields:
                    submsg_class = roslib.message.get_message_class(
                                        type_information.split('[', 1)[0])
                    if not submsg_class:
                        raise DhaibaConnectTopicException("Cannot load message class for [%s]. Are your messages built?" % type_information)

    sub = rospy.Subscriber(real_topic, msg_class, cb.callback,
                {'topic': topic, 'type_information': type_information})

    while not rospy.is_shutdown() and not cb.done:
        _sleep(0.1)

def _cmd_pub(argv):
    from optparse import OptionParser

    args = argv[2:]
    parser = OptionParser(usage="usage: %prog pub /topic fastrtps_participant fastrtps_topic", prog=argv[0])
    if len(args) == 0:
        parser.error("topic must be specified")
    topic = rosgraph.names.script_resolve_name(NAME+'Pub', args[0])
    fastrtps_participant = args[1]
    fastrtps_topic = args[2]

    rospy.init_node(NAME, anonymous=True)

    dhaiba = dhaiba_pybind.note_publisher(fastrtps_participant, fastrtps_topic)

    cb = CallbackPublisher(topic, dhaiba)
    try:
        _topic_pub(topic, cb)
    except socket.error:
        sys.stderr.write("ERROR(_cmd_pub): Network communication failed.\n")

def _cmd_sub(argv):
    from optparse import OptionParser

    args = argv[2:]
    parser = OptionParser(usage="usage: %prog sub fastrtps_participant fastrtps_topic", prog=argv[0])
    if len(args) == 0:
        parser.error("topic must be specified")
    fastrtps_participant = args[0]
    fastrtps_topic = args[1]

    rospy.init_node(NAME, anonymous=True)
    dhaiba = dhaiba_pybind.note_subscriber(fastrtps_participant, fastrtps_topic)

def _usage(cmd):
    print("""usage:
\t%s pub TopicName\t\tpublish
\t%s sub TopicName\t\tsubscribe
""" % (cmd, cmd))
    sys.exit(getattr(os, 'EX_USAGE', 1))

def main(argv=None):
    if argv is None:
        argv=sys.argv
    argv = rospy.myargv(argv)
    
    if len(argv) == 1:
        _fullusage(argv[0])
    try:
        command = argv[1]
        if command == 'pub':
            _cmd_pub(argv)
        elif command == 'sub':
            _cmd_sub(argv)
        else:
            _usage(argv[0])
    except KeyboardInterrupt: pass
    except Exception as e:
        sys.stderr.write("ERROR: %s\n"%str(e))
        sys.exit(1)

main()

