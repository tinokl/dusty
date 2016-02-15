#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com

import roslib
import rospy
import random

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent


class CleaningActionServer:

    def __init__(self):

        self.hit = False
        self.rotating = False
        self.straight = True
        self.angular = 1.0
        self.time = 0

        rospy.Subscriber("bumper", BumperEvent, self.bumper_cb)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size = 10)

        while not rospy.is_shutdown():
            # self.random_movement()
            self.movement_circle()
            rospy.sleep(0.1)

    def bumper_cb(self, data):
        if data.state == BumperEvent.PRESSED:
            self.hit = True
        else:
            self.hit = False

    def reset(self):
        self.rotating = False
        self.straight = True
        self.time = 0
        self.angular = 1.0

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
            move.angular.z = 1.0
            self.time = 2
        else:
            move.linear.x = 1.0
            self.time -= 1

            if self.time < 0:
                move.angular.z = -1.0
                move.linear.x = 0.5

        self.pub.publish(move)

    def movement_circle(self):
        move = Twist()

        if self.hit:
            return

        self.angular -= 0.005
        move.linear.x = 0.5 + 0.5 - self.angular/2.0
        move.angular.z = self.angular

        self.pub.publish(move)


if __name__ == '__main__':
    rospy.init_node('CleaningActionServer')
    try:
        CleaningActionServer()
    except rospy.ROSInterruptException:
        pass


