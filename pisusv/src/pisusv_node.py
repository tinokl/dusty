#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com

import roslib
import rospy
import subprocess

from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import BatteryState


class PISUSV:

    def __init__(self):

        self.pub = rospy.Publisher("/susv", BatteryState, queue_size = 10)
        self.pub_event = rospy.Publisher("events", Log, queue_size = 10)
        self.design_capacity = 3000
        self.state_charging = 0
        self.percentage = 0
        self.voltage = 0
        self.power_battery = 0
        self.charging_current = 0

        while not rospy.is_shutdown():
            self.check_usv()
            self.publish_state()
            rospy.sleep(1)

    def check_usv(self):
        output = subprocess.check_output("sudo /opt/susvd/susv -status", shell=True)

        for item in output.split("\n"):
          if "Charging circuit: ONLINE" in item:
            self.state_charging = BatteryState.POWER_SUPPLY_STATUS_CHARGING
          elif "Charging circuit: OFFLINE" in item:
            self.state_charging = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

          if "Charging current" in item:
            string = item.strip()
            string = string.replace('*', '')
            string = " ".join(string.split())
            e = string.replace("Charging current: ", '')
            charging_current = float(e.replace("mA", ''))
            self.charging_current = (charging_current / 1000.0)

          if "Power Battery" in item:
            string = item.strip()
            string = string.replace('*', '')
            string = " ".join(string.split())
            e = string.replace("Power Battery: ", '')
            power_battery = float(e.replace("mA", ''))
            self.power_battery = (power_battery / 1000.0)
            if self.power_battery > 0:
                self.power_battery *= -1.0

          if "Battery voltage" in item:
            string = item.strip()
            string = string.replace('*', '')
            string = " ".join(string.split())
            e = string.replace("Battery voltage: ", '')
            voltage = float(e.replace("V", ''))
            self.voltage = voltage

          if "Battery capacity" in item:
            string = item.strip()
            string = string.replace('*', '')
            string = " ".join(string.split())
            e = string.replace("Battery capacity: ", '')
            percentage = float(e.replace("%",''))

            # send event msg if something changed
            if abs(percentage - self.percentage) > 5:
              log = Log()
              log.msg = str(string)
              log.level = log.INFO
              self.pub_event.publish(log)
              self.percentage = percentage

    def publish_state(self):
        state = BatteryState()
        state.header.frame_id = "usv"
        state.header.stamp = rospy.Time.now()

        state.voltage = self.voltage
        state.current = self.power_battery
        state.charge = self.charging_current
        #state.capacity = self.design_capacity * (self.percentage / 100.0)
        state.design_capacity = self.design_capacity
        state.percentage = (self.percentage/100.0)
        state.power_supply_status = self.state_charging
        state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        state.present = True

        state.cell_voltage = [self.voltage]
        state.location = "Slot 1"
        state.serial_number = "SUSV LIPO 3000mAH"

        self.pub.publish(state)


if __name__ == '__main__':
    rospy.init_node('pisusv')
    try:
        PISUSV()
    except rospy.ROSInterruptException:
        pass



