#!/usr/bin/env python

import roslib
import rospy
import sys
import math
import pigpio

from std_srvs.srv import *


class SpiderServiceNode:
    
    def __init__(self):
        self.pin_vakuum = rospy.get_param('~pin_vakuum', 15)
        self.pin_power = rospy.get_param('~pin_vakuum', 21)
        self.debug_mode = rospy.get_param('~debug_mode', False)

        # LED = 22
        # beep = 6

        self.v_service = rospy.Service('trigger_vakuum', SetBool, self.vakuum_service)
        self.p_service = rospy.Service('trigger_power', SetBool, self.power_service)        

        try:
            if not self.debug_mode:
                self.pi = pigpio.pi()
                self.pi.set_mode(self.pin_vakuum, pigpio.OUTPUT)
        except:
            rospy.logerr("SpiderServicesNode::Error: unable to set mode of pin!")
            return

        while not rospy.is_shutdown():
            rospy.spin()

    def buzzer(self):
        pi.set_PWM_dutycycle(beep, 5)
        time.sleep(0.5)
        pi.write(beep, 0)
        time.sleep(0.5)

    def vakuum_service(self, req):
        resp = SetBoolResponse()
        resp.success = True

        try:
            if req.data:
                if not self.debug_mode:
                    self.pi.write(self.pin_vakuum, 1)
                resp.message = "Switched Vakuum ON"
            else:
                if not self.debug_mode:
                    self.pi.write(self.pin_vakuum, 0)
                resp.message = "Switched Vakuum OFF"
        except:
            rospy.logerr("SpiderServiceNode::Error: unable to read vakuum pin!")
            resp.message = "Error: unable to read vakuum pin!"
            resp.success = False

        return resp

    def power_service(self, req):
        resp = SetBoolResponse()
        resp.success = True

        try:
            if req.data:
                if not self.debug_mode:
                    self.pi.write(self.pin_power, 1)
                resp.message = "Switched Power ON"
            else:
                if not self.debug_mode:
                    self.pi.write(self.pin_power, 0)
                resp.message = "Switched Power OFF"
        except:
            rospy.logerr("SpiderServiceNode::Error: unable to read power pin!")
            resp.message = "Error: unable to read power pin!"
            resp.success = False

        return resp

if __name__ == '__main__':
    rospy.init_node('spider_service_node')
    try:
        ne = SpiderServiceNode()
    except rospy.ROSInterruptException:
        pass
