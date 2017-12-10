#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com

import roslib
import rospy
import subprocess

from datetime import datetime
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import Joy
from std_srvs.srv import *

ON = True
OFF = False


class HighLevelControl:

    def __init__(self):

        self.vakuum_state = OFF
        self.battery_power_state = OFF
        # switch battery off, at least after this
        self.battery_start_time = datetime.now()
        self.battery_max_temperature = 30
        self.battery_max_loading = 60*60*3

        self.pressed = 1
        self.unpressed = 0

        # xbox one controller mapping
        self.button_B = 1

        self.pub_event = rospy.Publisher("/events", Log, queue_size = 10)
        rospy.Subscriber("/joy", Joy, self.joy_cb)

        while not rospy.is_shutdown():
          self.battery_power_control()
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

    def call_battery_service(self, state):
      rospy.wait_for_service('/trigger_power')
      try:
          trigger_power_service = rospy.ServiceProxy('/trigger_power', SetBool)
          resp = None
          if state == ON:
            resp = trigger_power_service(ON)
            if resp:
              self.battery_power_state = ON
          else:
            resp = trigger_power_service(OFF)
            if resp:
              self.battery_power_state = OFF
      except rospy.ServiceException, e:
          print "Service call failed: %s"%e

    def read_temperature(self):
      path = "/sys/bus/w1/devices/10-000800db2069/w1_slave"
      temp = 1000 # make it damn hot if something fails
      try:
        output = subprocess.check_output("cat " + path, shell=True)
        temp = int(output[-5:])/1000
      except OSError as e:
        print "Temperature could not be read: %s"%e
      return temp

    def battery_power_control(self):
      temp = self.read_temperature()

      now = datetime.now()
      power_online = now - self.battery_start_time
      batt_time = False
      batt_temp = False

      if power_online.total_seconds() > self.battery_max_loading:
        batt_time = True
      if temp > self.battery_max_temperature:
        batt_temp = True

      if batt_time or batt_temp:
        self.call_battery_service(OFF)

        log = Log()
        msg = "Switch OFF battery power because of"
        if batt_temp:
          msg += " Temperature: " + str(temp)
        if batt_time:
          msg += " Time: " + str(power_online)

        log.msg = msg
        log.level = log.INFO
        self.pub_event.publish(log)


if __name__ == '__main__':
    rospy.init_node('HighLevelControl')
    try:
        HighLevelControl()
    except rospy.ROSInterruptException:
        pass
