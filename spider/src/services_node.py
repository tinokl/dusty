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
        self.debug_mode = rospy.get_param('~debug_mode', False)

        # LED = 22
        # beep = 6

        self.v_service = rospy.Service('trigger_vakuum', SetBool, self.vakuum_service)

        try:
            if not self.debug_mode:
                self.pi = pigpio.pi()
                self.pi.set_mode(self.pin_vakuum, pigpio.OUTPUT)
        except:
            rospy.logerr("SpiderServicesNode::Error: unable to set mode of pin!")
            return

        while not rospy.is_shutdown():
            rospy.spin()

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
            rospy.logerr("SpiderServiceNode::Error: unable to read pin!")
            resp.message = "Error: unable to read pin!"
            resp.success = False

        return resp

if __name__ == '__main__':
    rospy.init_node('spider_service_node')
    try:
        ne = SpiderServiceNode()
    except rospy.ROSInterruptException:
        pass
