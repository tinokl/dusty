#!/usr/bin/env python

import roslib
roslib.load_manifest('lta_server_update')
import requests
import rospy
import sys

from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from marathon_reporter.msg import MarathonSession


class LtaServerUpdateNode():
    
    def __init__(self):

        self.server = rospy.get_param('~server', 'ais.ist.tugraz.at')
        self.topic_log = rospy.get_param('~topic_log', '')
        self.topic_events = rospy.get_param('~topic_events', '')
        self.topic_stats = rospy.get_param('~topic_stats', '')
        self.sleep_time = rospy.get_param('~ros_sleep_time', '')

        rospy.Subscriber(self.topic_log, Log, self.log_callback)
        rospy.Subscriber(self.topic_events, Log, self.event_callback)
        rospy.Subscriber(self.topic_stats, MarathonSession, self.statistic_callback)

        rospy.spin()

        while not rospy.is_shutdown():
            rospy.sleep(self.sleep_time)

    def statistic_callback(self, data):
        try:
            url = self.server + '/put_statistics.php'
            t = data.duration
            seconds = t.to_sec()
            payload = {'distance': float(data.distance), 'duration': float(seconds)}
            r = requests.post(url, data=payload)
        except:
            print " Error: unable to post statistic data!"

    def event_callback(self, data):
        try:
            url = self.server + '/put_event.php'
            payload = {'msg': str(data.msg), 'public': 1, 'visual': int(data.level)}
            r = requests.post(url, data=payload)
        except:
            print " Error: unable to post event data!"

    def log_callback(self, data):
        try:
            url = self.server + '/put_event.php'
            payload = {'msg': str(data.msg), 'public': 0, 'visual': int(data.level)}
            r = requests.post(url, data=payload)
        except:
            print " Error: unable to post log data!"


if __name__ == '__main__':
    rospy.init_node('lta_server_update')
    try:
        ne = LtaServerUpdateNode()
    except rospy.ROSInterruptException:
        pass
