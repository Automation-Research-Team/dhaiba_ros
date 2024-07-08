#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2023, National Institute of Advanced Industrial Science and Technology (AIST)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Toshio Ueshiba (t.ueshiba@aist.go.jp)
#
import rospy, rosgraph, rostopic, rosservice
import threading

from dhaiba_ros     import note_publisher, note_subscriber
from roslib.message import get_message_class
from rospy_message_converter.json_message_converter \
import convert_ros_message_to_json, convert_json_to_ros_message

#########################################################################
#  class DhaibaBridge                                                   #
#########################################################################
class DhaibaBridge(object):
    def __init__(self):
        super().__init__()

        self._participant = rospy.get_name().replace('/', '')
        print('### ' + self._participant)
        self._ros_to_dhaiba_threads = {}
        for ros_to_dhaiba_info in rospy.get_param('~ros_to_dhaiba', []):
            th = self._create_ros_to_dhaiba_thread(ros_to_dhaiba_info)
            if th:
                ros_topic_name = ros_to_dhaiba_info['from']
                self._ros_to_dhaiba_threads[ros_topic_name] = th

        self._dhaiba_to_ros_threads = {}
        for dhaiba_to_ros_info in rospy.get_param('~dhaiba_to_ros', []):
            th = self._create_dhaiba_to_ros_thread(dhaiba_to_ros_info)
            if th:
                dhaiba_element_name = dhaiba_to_ros_info['from']
                self._dhaiba_to_ros_threads[dhaiba_element_name] = th

        self._service_client_threads = {}
        for service_client_info in rospy.get_param('~service_clients', []):
            th = self._create_service_client_thread(service_client_info)
            if th:
                dhaiba_request_name = service_client_info['request']
                self._service_client_threads[dhaiba_request_name] = th

        self._service_server_threads = {}
        for service_server_info in rospy.get_param('~service_servers', []):
            th = self._create_service_server_thread(service_server_info)
            if th:
                dhaiba_request_name = service_server_info['request']
                self._service_server_threads[dhaiba_request_name] = th

        rospy.loginfo('(DhaibaBridge) started')

    # ROS ==> DHAIBA stuffs
    def _create_ros_to_dhaiba_thread(self, ros_to_dhaiba_info):
        ros_topic_name = ros_to_dhaiba_info['from']
        master         = rosgraph.Master('/rostopic')
        timeout_time   = rospy.get_rostime() + rospy.Duration(5.0)

        while rospy.get_rostime() < timeout_time:
            topics = master.getSystemState()[0]
            topic  = next(filter(lambda topic: topic[0] == ros_topic_name,
                                 topics),
                          None)
            # l = filter(lambda topic: topic[0] == ros_topic_name, topics)
            # topic = None if len(l) == 0 else l[0]
            if topic is not None:
                break
            rospy.sleep(0.5)
        if topic is None:
            rospy.logerr('(DhaibaBridge) unknown ROS topic[%s]',
                         ros_topic_name)
            return None

        dhaiba_element_name  = ros_to_dhaiba_info['to']
        th = threading.Thread(target=self._ros_to_dhaiba_thread,
                              args=(ros_topic_name, dhaiba_element_name))
        th.daemon = True
        th.start()
        rospy.loginfo('(DhaibaBridge) ROS[%s] ==> DHAIBA[%s]',
                      ros_topic_name, dhaiba_element_name)
        return th

    def _ros_to_dhaiba_thread(self, ros_topic_name, dhaiba_element_name):
        ros_topic_type  = rostopic.get_topic_type(ros_topic_name)[0]
        dhaiba_pub      = note_publisher(self._participant,
                                         dhaiba_element_name)
        ros_topic_class = rostopic.get_topic_class(ros_topic_name)[0]
        ros_sub         = rospy.Subscriber(ros_topic_name, ros_topic_class,
                                           self._ros_to_dhaiba_cb,
                                           dhaiba_pub)
        rospy.spin()

    def _ros_to_dhaiba_cb(self, msg, dhaiba_pub):
        dhaiba_pub.write(convert_ros_message_to_json(msg, True))

    # ROS <== DHAIBA stuffs
    def _create_dhaiba_to_ros_thread(self, dhaiba_to_ros_info):
        ros_topic_type  = dhaiba_to_ros_info['type']
        ros_topic_class = get_message_class(ros_topic_type)
        if ros_topic_class is None:
            rospy.logerr('(DhaibaBridge) unknown ROS topic type[%s]',
                         ros_topic_type)
            return None

        dhaiba_element_name = dhaiba_to_ros_info['from']
        ros_topic_name      = dhaiba_to_ros_info['to']
        th = threading.Thread(target=self._dhaiba_to_ros_thread,
                              args=(dhaiba_element_name,
                                    ros_topic_name, ros_topic_type))
        th.daemon = True
        th.start()
        rospy.loginfo('(DhaibaBridge) ROS[%s] <== DHAIBA[%s]',
                      ros_topic_name, dhaiba_element_name)
        return th

    def _dhaiba_to_ros_thread(self, dhaiba_element_name,
                              ros_topic_name, ros_topic_type):
        dhaiba_sub = note_subscriber(self._participant, dhaiba_element_name)
        ros_pub    = rospy.Publisher(ros_topic_name,
                                     get_message_class(ros_topic_type),
                                     queue_size=100)
        dhaiba_sub.register_callback(lambda json:
                                     self._dhaiba_to_ros_cb(json,
                                                            ros_topic_type,
                                                            ros_pub))
        rospy.spin()

    def _dhaiba_to_ros_cb(self, json, ros_topic_type, ros_pub):
        ros_pub.publish(convert_json_to_ros_message(ros_topic_type, json))

    # DHAIBA ==> ROS ==> DHAIBA service stuffs
    def _create_service_client_thread(self, service_client_info):
        service_name = service_info['name']
        service_type = rosservice.get_service_type(service_name)
        if service_type is None:
            rospy.logerr('(DhaibaBridge) unknown ROS service[%s]',
                         service_name)
            return None

        dhaiba_request_name  = service_client_info['request']
        dhaiba_response_name = service_client_info['response']
        th = threading.Thread(target=self._service_client_thread,
                              args=(service_name,
                                    dhaiba_request_name, dhaiba_response_name))
        th.daemon = True
        th.start()
        rospy.loginfo('(DhaibaBridge) DHAIBA[%s] ==> ROS[%s(%s)] ==> DHAIBA[%s]',
                      dhaiba_request_name, service_name, service_type,
                      dhaiba_response_name)
        return th

    def _service_client_thread(self, service_name,
                               dhaiba_request_name, dhaiba_response_name):
        dhaiba_pub    = note_publisher(self._participant, dhaiba_response_name)
        service_type  = rosservice.get_service_type(service_name)
        service_class = rosservice.get_service_class_by_name(service_name)
        service_call  = rospy.ServiceProxy(service_name, service_class)
        dhaiba_sub    = note_subscriber(self._participant, dhaiba_request_name)
        dhaiba_sub.register_callback(lambda req_json:
                                     self._service_client_cb(req_json,
                                                             service_type,
                                                             service_call,
                                                             dhaiba_pub))
        rospy.spin()

    def _service_client_cb(self, req_json,
                           service_type, service_call, dhaiba_pub):
        dhaiba_pub.write(
            convert_ros_message_to_json(
                service_call(
                    convert_json_to_ros_message(service_type, req_json)),
                True))

    # ROS ==> DHAIBA ==> ROS service stuffs
    class DhaibaResponse():
        def __init__():
            self._condition = threading.Condition()
            self._response  = None

        def wait():
            loop_period = rospy.Duration(0.1)
            with self._condition:
                while self._response is None:
                    self._condition.wait(loop_preriod)

        def set_response(res):
            self._response = res
            with self._condition:
                self._confition.notifyAll()

        def get_response():
            return self._response

    def _create_service_server_thread(self, service_server_info):
        service_name = service_info['name']
        service_type = rosservice.get_service_type(service_name)
        if service_type is None:
            rospy.logerr('(DhaibaBridge) unknown ROS service[%s]',
                         service_name)
            return None

        dhaiba_request_name  = service_server_info['request']
        dhaiba_response_name = service_server_info['response']
        th = threading.Thread(target=self._service_server_thread,
                              args=(service_name,
                                    dhaiba_request_name, dhaiba_response_name))
        th.daemon = True
        th.start()
        rospy.loginfo('(DhaibaBridge) %s:%s ROS ==> DHAIBA[%s] ==> DHAIBA[%s] ==> ROS',
                      service_name, service_type,
                      dhaiba_request_name, dhaiba_response_name)
        return th

    def _service_server_thread(self, service_name,
                               dhaiba_request_name, dhaiba_response_name):
        dhaiba_pub = note_publisher(self._participant, dhaiba_request_name)
        dhaiba_sub = note_subscriber(self._participant, dhaiba_response_name)
        dhaiba_response = DhaibaResponse()
        dhaiba_sub.register_callback(lambda res :
                                     self._dhaiba_response_cb(res,
                                                              dhaiba_response))
        service_type  = rosservice.get_service_type(service_name)
        service_class = rosservice.get_service_class_by_name(service_name)
        service_srver = rospy.Service(service_name, service_class,
                                      lambda req :
                                      self._service_server_cb(req,
                                                              service_type,
                                                              dhaiba_response))
        rospy.spin()

    def _service_server_cb(self, req, service_type, dhaiba_response):
        dhaiba_pub.write(convert_ros_message_to_json(req, True))
        dhaiba_response.wait()
        return convert_json_to_ros_message(service_type,
                                           dhaiba_response.get_reponse())

    def _dhaiba_response_cb(self, res, dhaiba_response):
        dhaiba_response.set_response(res)

#########################################################################
#  entry point                                                          #
#########################################################################
if __name__ == '__main__':
    rospy.init_node('dhaiba_bridge')

    bridge = DhaibaBridge()
    rospy.spin()
