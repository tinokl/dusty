#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com

import roslib
import rospy

from sensor_msgs.msg import Joy
from std_srvs.srv import *

ON = True
OFF = False


class HighLevelControl:

    def __init__(self):

        self.vakuum_state = OFF

        self.pressed = 1
        self.unpressed = 0

        # xbox one controller mapping
        self.button_B = 1

        rospy.Subscriber("/joy", Joy, self.joy_cb)

        while not rospy.is_shutdown():
            rospy.sleep(1.0)

    def joy_cb(self, data):
        if data.buttons[self.button_B] == self.pressed:
          self.call_vakuum_service()

    def call_vakuum_service(self):
      rospy.wait_for_service('/trigger_vakuum')
      try:
          trigger_vakkum_service = rospy.ServiceProxy('/trigger_vakuum', SetBool)
          resp = None
          if self.vakuum_state == OFF:
            resp = trigger_vakkum_service(ON)
            if resp:
              self.vakuum_state = ON
          else:
            resp = trigger_vakkum_service(OFF)
            if resp:
              self.vakuum_state = OFF
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node('HighLevelControl')
    try:
        HighLevelControl()
    except rospy.ROSInterruptException:
        pass

