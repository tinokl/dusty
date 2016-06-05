#!/usr/bin/env python

import roslib
import rospy
import sys
import math
import pigpio

from spider_msgs.msg import BumperEvent


class SpiderEventNode:
    
    def __init__(self):
        self.pin_interrupt = rospy.get_param('~pin_interrupt', 5)
        self.frequency = rospy.get_param('~frequency', 30)
        self.rate = 1.0/self.frequency

        self.debug_mode = rospy.get_param('~debug_mode', False)

        try:
            if not self.debug_mode:
                self.pi = pigpio.pi()

                self.pi.set_mode(self.pin_interrupt, pigpio.INPUT)
        except:
            rospy.logerr("SpiderEventNode::Error: unable to set mode of pin!")
            return

        self.last_msg = None

        self.pub = rospy.Publisher('bumper', BumperEvent, queue_size=10)

        while not rospy.is_shutdown():
            rospy.sleep(self.rate)
            self.check_state()

    def check_state(self):
        state = 1
        try:
            state = self.pi.read(self.pin_interrupt)
        except:
            rospy.logerr("SpiderEventNode::Error: unable to read pin!")

        event = BumperEvent()
        # normal high, pressed low
        if state == 0:
            event.state = BumperEvent.PRESSED
        else:
            event.state = BumperEvent.RELEASED
        self.pub.publish(event)



if __name__ == '__main__':
    rospy.init_node('spider_event_node')
    try:
        ne = SpiderEventNode()
    except rospy.ROSInterruptException:
        pass
