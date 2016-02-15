#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com

import roslib
import rospy

from sensor_msgs.msg import LaserScan
from kobuki_msgs.msg import BumperEvent


class LaserBumper:

    def __init__(self):

        rospy.Subscriber("bumper_scan", LaserScan, self.scan_cb)
        self.pub = rospy.Publisher("bumper", BumperEvent, queue_size = 10)

        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    def scan_cb(self, data):
        event = BumperEvent()
        event.bumper = event.CENTER
        event.state = event.RELEASED
        for i in range(0,len(data.ranges), 1):
            if data.ranges[i] < 0.4:
                # rospy.logerr("data: " + str(data.ranges))
                event.state = event.PRESSED

        self.pub.publish(event)


if __name__ == '__main__':
    rospy.init_node('LaserBumper')
    try:
        LaserBumper()
    except rospy.ROSInterruptException:
        pass

