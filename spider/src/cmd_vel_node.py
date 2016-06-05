#!/usr/bin/env python

import roslib
import rospy
import sys
import math
import pigpio

from std_msgs.msg import String
from geometry_msgs.msg import Twist


class SpiderCMDVelNode:
    
    def __init__(self):
        self.pin_motor_left_front = rospy.get_param('~pin_motor_left_front', 11)
        self.pin_motor_right_front = rospy.get_param('~pin_motor_right_front', 9)
        self.pin_motor_left_back = rospy.get_param('~pin_motor_left_back', 14)
        self.pin_motor_right_back = rospy.get_param('~pin_motor_right_back', 18)

        self.wheel_dist = rospy.get_param('~wheel_dist', 22.3)
        self.velocity_corr = rospy.get_param('~velocity_correction', 1.0)
        self.rotation_corr = rospy.get_param('~rotation_correction', 1.0)

        self.debug_mode = True

        try:
            if not self.debug_mode:
                self.pi = pigpio.pi()

                # switch them off for now
                pi.write(self.pin_motor_left_front, 0)
                pi.write(self.pin_motor_right_front, 0)
                pi.write(self.pin_motor_left_back, 0)
                pi.write(self.pin_motor_right_back, 0)
        except:
            print " Error: unable to set cmd vel!"
            # return

        self.vel_right = 0.0
        self.vel_left = 0.0

        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)

        while not rospy.is_shutdown():
            rospy.sleep(0.02)  # 50Hz
            # TODO: publish odometry

    def cmd_cb(self, msg):
        trans = msg.linear.x
        rot = msg.angular.z

        #self.speed_right = angle * math.pi * self.wheel_dist / 2.0 + speed
        #self.speed_left = speed * 2 - self.speed_right
        #rospy.loginfo("speed right: " + str(self.speed_right) + " speed left: " + str(self.speed_left))

        # TODO CHECK MAX VALUES ANGLE AND SPEED

        rot = rot * self.rotation_corr
        vel_left = trans - 1.0 * rot * self.wheel_dist
        vel_right = trans + 1.0 * rot * self.wheel_dist

        self.vel_left = vel_left * self.velocity_corr
        self.vel_right = vel_right * self.velocity_corr

        if self.debug_mode:
            rospy.loginfo("vel right: " + str(self.vel_right) + " vel left: " + str(self.vel_left))
        else:
            self.set_motors()

    def set_motors(self):
        try:
            # ## left motor
            if self.vel_left > 0:
                pi.write(self.pin_motor_left_back, 0)
                # pi.write(self.pin_motor_left_front, 1)
                pi.set_PWM_dutycycle(self.pin_motor_left_front, abs(self.vel_left))
            elif self.vel_left < 0:
                pi.write(self.pin_motor_left_front, 0)
                # pi.write(self.pin_motor_left_back, 1)
                pi.set_PWM_dutycycle(self.pin_motor_left_back, abs(self.vel_left))
            else:
                pi.write(self.pin_motor_left_front, 0)
                pi.write(self.pin_motor_left_back, 0)

            # ## right motor
            if self.vel_right > 0:
                pi.write(self.pin_motor_right_back, 0)
                # pi.write(self.pin_motor_right_front, 1)
                pi.set_PWM_dutycycle(self.pin_motor_right_front, abs(self.vel_right))
            elif self.vel_right < 0:
                pi.write(self.pin_motor_right_front, 0)
                # pi.write(self.pin_motor_right_back, 1)
                pi.set_PWM_dutycycle(self.pin_motor_right_back, abs(self.vel_right))
            else:
                pi.write(self.pin_motor_right_back, 0)
                pi.write(self.pin_motor_right_front, 0)
        except:
            print " Error: unable to set cmd vel!"


if __name__ == '__main__':
    rospy.init_node('spider_cmd_vel_node')
    try:
        ne = SpiderCMDVelNode()
    except rospy.ROSInterruptException:
        pass
