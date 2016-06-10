#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com

import roslib
import rospy
import random

from geometry_msgs.msg import Twist
from spider_msgs.msg import BumperEvent
from std_srvs.srv import *


class CleaningActionServer:

    def __init__(self):

        self.hit = False
        self.rotating = False
        self.straight = True
        self.angular = 1.0
        self.time = 0

        self.enable_random = False
        self.enable_circle = False
        self.enable_wall_follower = False

        rospy.Subscriber("/bumper", BumperEvent, self.bumper_cb)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.r_service = rospy.Service('trigger_movement_random', SetBool, self.trigger_movement_random)
        self.c_service = rospy.Service('trigger_movement_circle', SetBool, self.trigger_movement_circle)
        self.w_service = rospy.Service('trigger_movement_wall_follower', SetBool, self.trigger_movement_wall_follower)

        while not rospy.is_shutdown():
            if self.enable_circle:
                self.movement_circle()
            if self.enable_random:
                self.movement_random()
            if self.enable_wall_follower:
                self.movement_wall_follower()
            rospy.sleep(0.1)

    def disable_all(self):
        self.reset()
        self.enable_random = False
        self.enable_circle = False
        self.enable_wall_follower = False

    def trigger_movement_random(self, req):
        resp = SetBoolResponse()
        resp.success = True

        if req.data:
            self.disable_all()
            self.enable_random = True
            resp.message = "trigger_movement_random ON"
        else:
            self.enable_random = False
            resp.message = "trigger_movement_random OFF"
        return resp

    def trigger_movement_wall_follower(self, req):
        resp = SetBoolResponse()
        resp.success = True

        if req.data:
            self.disable_all()
            self.enable_wall_follower = True
            resp.message = "trigger_movement_wall_follower ON"
        else:
            self.enable_wall_follower = False
            resp.message = "trigger_movement_wall_follower OFF"
        return resp

    def trigger_movement_circle(self, req):
        resp = SetBoolResponse()
        resp.success = True

        if req.data:
            self.disable_all()
            self.enable_circle = True
            resp.message = "trigger_movement_circle ON"
        else:
            self.enable_circle = False
            resp.message = "trigger_movement_circle OFF"
        return resp

    def bumper_cb(self, data):
        if data.state == BumperEvent.PRESSED:
            self.hit = True
        else:
            self.hit = False

    def reset(self):
        self.rotating = False
        self.straight = True
        self.time = 0
        self.angular = 1.5

    def movement_random(self):
        move = Twist()

        if self.hit:
            self.rotating = True
            self.time = random.randint(5, 20)

        if self.time > 0:
            self.time -= 1
            move.angular.z = 1.0
        else:
            self.rotating = False
            move.linear.x = 1.0

        self.pub.publish(move)

    def movement_wall_follower(self):
        move = Twist()

        if self.straight:
            self.time = 2

        if self.hit:
            self.straight = False
            move.angular.z = 2.0
            self.time = 2
        else:
            move.linear.x = 1.0
            self.time -= 1

            if self.time < 0:
                move.angular.z = -2.0
                move.linear.x = 0.8

        self.pub.publish(move)

    def movement_circle(self):
        move = Twist()

        if self.hit:
            return

        self.angular -= 0.001
        move.linear.x = 0.5 + 0.5 - self.angular/2.0
        move.angular.z = self.angular

        self.pub.publish(move)


if __name__ == '__main__':
    rospy.init_node('CleaningActionServer')
    try:
        CleaningActionServer()
    except rospy.ROSInterruptException:
        pass

