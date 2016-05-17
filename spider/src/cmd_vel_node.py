#!/usr/bin/env python

import roslib
import rospy
import sys
import math
from RPi import GPIO

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SpiderCMDVelNode:
    
    def __init__(self):
        self.pin_motor_left_front = rospy.get_param('~pin_motor_left_front', 23)
        self.pin_motor_right_front = rospy.get_param('~pin_motor_right_front', 21)
        self.pin_motor_left_back = rospy.get_param('~pin_motor_left_back', 8)
        self.pin_motor_right_back = rospy.get_param('~pin_motor_right_back', 12)

        self.wheel_dist = rospy.get_param('~wheel_dist', 22.3)

        try:
            GPIO.setmode(GPIO.BOARD)

            # switch them off for now
            GPIO.output(self.pin_motor_left_front, False)
            GPIO.output(self.pin_motor_right_front, False)
            GPIO.output(self.pin_motor_left_back, False)
            GPIO.output(self.pin_motor_right_back, False)
        except:
            print " Error: unable to set cmd vel!"
            #return

        self.speed_right = 0.0
        self.speed_left = 0.0

        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)

        while not rospy.is_shutdown():
            rospy.sleep(0.02)  # 50Hz
            self.set_motors()

    def cmd_cb(self, msg):
        speed = msg.linear.x
        angle = msg.angular.z
        self.speed_right = angle * math.pi * self.wheel_dist / 2.0 + speed
        self.speed_left = speed * 2 - self.speed_right
        rospy.loginfo("speed right: " + str(self.speed_right) + " speed left: " + str(self.speed_left))

    def set_motors(self):
        try:
            if self.speed_left > 0:
                GPIO.output(self.pin_motor_left_back, False)
                GPIO.output(self.pin_motor_left_front, True)
            elif self.speed_left < 0:
                GPIO.output(self.pin_motor_left_front, False)
                GPIO.output(self.pin_motor_left_back, True)
            else:
                GPIO.output(self.pin_motor_left_front, False)
                GPIO.output(self.pin_motor_left_back, False)

            if self.speed_right > 0:
                GPIO.output(self.pin_motor_right_back, False)
                GPIO.output(self.pin_motor_right_front, True)
            elif self.speed_right < 0:
                GPIO.output(self.pin_motor_right_front, False)
                GPIO.output(self.pin_motor_right_back, True)
            else:
                GPIO.output(self.pin_motor_right_back, False)
                GPIO.output(self.pin_motor_right_front, False)
        except:
            print " Error: unable to set cmd vel!"


if __name__ == '__main__':
    rospy.init_node('spider_cmd_vel_node')
    try:
        ne = SpiderCMDVelNode()
    except rospy.ROSInterruptException:
        pass
