#!/usr/bin/env python

import roslib
import rospy
import sys
import math
import tf
import pigpio

from geometry_msgs.msg import Twist
from nav_msgs.msgs import Odometry
from geometry_msgs.msgs import TransformStamped


class SpiderCMDVelNode:
    
    def __init__(self):
        self.pin_motor_left_front = rospy.get_param('~pin_motor_left_front', 11)
        self.pin_motor_right_front = rospy.get_param('~pin_motor_right_front', 9)
        self.pin_motor_left_back = rospy.get_param('~pin_motor_left_back', 14)
        self.pin_motor_right_back = rospy.get_param('~pin_motor_right_back', 18)

        # in meter
        self.wheel_dist = rospy.get_param('~wheel_dist', 0.223)

        self.max_vel = rospy.get_param('~max_velocity', 255)
        self.min_vel = rospy.get_param('~min_velocity', 100)

        # in sec
        self.timeout = rospy.get_param('~timeout', 1)

        self.velocity_corr = rospy.get_param('~velocity_correction', 1.0)
        self.rotation_corr = rospy.get_param('~rotation_correction', 1.0)

        self.debug_mode = rospy.get_param('~debug_mode', False)

        try:
            if not self.debug_mode:
                self.pi = pigpio.pi()

                self.pi.set_mode(self.pin_motor_left_front, pigpio.OUTPUT)
                self.pi.set_mode(self.pin_motor_right_front, pigpio.OUTPUT)
                self.pi.set_mode(self.pin_motor_left_back, pigpio.OUTPUT)
                self.pi.set_mode(self.pin_motor_right_back, pigpio.OUTPUT)

                # switch them off for now
                self.pi.write(self.pin_motor_left_front, 0)
                self.pi.write(self.pin_motor_right_front, 0)
                self.pi.write(self.pin_motor_left_back, 0)
                self.pi.write(self.pin_motor_right_back, 0)
        except:
            print " Error: unable to set cmd vel!"
            # return

        self.vel_right = 0.0
        self.vel_left = 0.0
        self.last_msg = None
        self.enabled = False

        self.x = 0
        self.y = 0
        self.vx = 0
        self.vy = 0
        self.yaw = 0
        self.vyaw = 0
        self.last_time = None

        rospy.Subscriber("cmd_vel", Twist, self.cmd_cb)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.tfbr = tf.TransformBroadcaster()

        while not rospy.is_shutdown():
            rospy.sleep(0.02)  # 50Hz
            self.check_timeout()
            self.publish_odometry()

    def check_timeout(self):
        if not self.enabled:
            return
        if not self.last_msg:
            self.vel_right = 0
            self.vel_left = 0
        else:
            now = rospy.Time.now()
            diff = now - self.last_msg
            if diff.to_sec() > self.timeout:
                self.enabled = False
                self.vel_right = 0
                self.vel_left = 0
                self.set_motors()

    def publish_odometry(self):
        quat = tf.transformations.quaternion_from_euler(0, 0, self.yaw)

        if self.last_time is None:
            self.last_time = rospy.Time.now()

        vx = self.vx #(msg->twist.twist.linear.x) *trans_mul_;
        vy = self.vy #msg->twist.twist.linear.y;
        vth = self.vyaw #(msg->twist.twist.angular.z) *rot_mul_;

        current_time = rospy.Time.now()

        dt = (current_time - self.last_time).toSec()
        delta_x = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        delta_y = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        delta_th = vth * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_th

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth

        self.odom_pub.publish(odom)

        transform_stamped = TransformStamped()
        transform_stamped.header = odom.header

        transform_stamped.transform.translation.x = self.x
        transform_stamped.transform.translation.y = self.y
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation = quat

        self.tfbr.sendTransform(transform_stamped)
        self.last_time = current_time

    def cmd_cb(self, msg):
        self.enabled = True
        self.last_msg = rospy.Time.now()
        trans = msg.linear.x
        rot = msg.angular.z

        self.vx = trans
        self.vyaw = rot

        rot = rot * self.rotation_corr
        vel_left = trans - 1.0 * rot * self.wheel_dist
        vel_right = trans + 1.0 * rot * self.wheel_dist

        raw_vel_left = vel_left * self.velocity_corr
        raw_vel_right = vel_right * self.velocity_corr

        # normalize input
        if raw_vel_left > 1.0:
            raw_vel_left = 1.0
        if raw_vel_right > 1.0:
            raw_vel_right = 1.0
        if raw_vel_left < -1.0:
            raw_vel_left = -1.0
        if raw_vel_right < -1.0:
            raw_vel_right = -1.0

        # now its between 0 and 2
        raw_vel_left += 1
        raw_vel_right += 1

        # X between A and B, Y to fall between C and D
        # Y = (X-A)/(B-A) * (D-C) + C
        # X between 0 and 2, Y to fall between min_vel and max_vel
        # Y = (X-0)/(2-0) * (255+255) - 255
        self.vel_left = (raw_vel_left)/(2.0) * (255+255) - 255
        self.vel_right = (raw_vel_right)/(2.0) * (255+255) - 255

        if self.vel_left != 0:
            if abs(self.vel_left) > self.max_vel:
                self.vel_left = math.copysign(self.max_vel, self.vel_left)
            if abs(self.vel_left) < self.min_vel:
                self.vel_left = math.copysign(self.min_vel, self.vel_left)

        if self.vel_right != 0:
            if abs(self.vel_right) > self.max_vel:
                self.vel_right = math.copysign(self.max_vel, self.vel_right)
            if abs(self.vel_right) < self.min_vel:
                self.vel_right = math.copysign(self.min_vel, self.vel_right)

        if self.debug_mode:
            rospy.loginfo("vel right: " + str(self.vel_right) + " vel left: " + str(self.vel_left))
        else:
            self.set_motors()

    def set_motors(self):
        try:
            # ## left motor
            if self.vel_left > 0:
                self.pi.write(self.pin_motor_left_back, 0)
                # pi.write(self.pin_motor_left_front, 1)
                self.pi.set_PWM_dutycycle(self.pin_motor_left_front, abs(self.vel_left))
            elif self.vel_left < 0:
                self.pi.write(self.pin_motor_left_front, 0)
                # pi.write(self.pin_motor_left_back, 1)
                self.pi.set_PWM_dutycycle(self.pin_motor_left_back, abs(self.vel_left))
            else:
                self.pi.write(self.pin_motor_left_front, 0)
                self.pi.write(self.pin_motor_left_back, 0)

            # ## right motor
            if self.vel_right > 0:
                self.pi.write(self.pin_motor_right_back, 0)
                # pi.write(self.pin_motor_right_front, 1)
                self.pi.set_PWM_dutycycle(self.pin_motor_right_front, abs(self.vel_right))
            elif self.vel_right < 0:
                self.pi.write(self.pin_motor_right_front, 0)
                # pi.write(self.pin_motor_right_back, 1)
                self.pi.set_PWM_dutycycle(self.pin_motor_right_back, abs(self.vel_right))
            else:
                self.pi.write(self.pin_motor_right_back, 0)
                self.pi.write(self.pin_motor_right_front, 0)
        except:
            print " Error: unable to set cmd vel!"


if __name__ == '__main__':
    rospy.init_node('spider_cmd_vel_node')
    try:
        ne = SpiderCMDVelNode()
    except rospy.ROSInterruptException:
        pass