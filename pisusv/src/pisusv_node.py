#!/usr/bin/env python

# AUTOR: Konstantin Lassnig
# konstantin.lassnig@gmail.com

import roslib
import rospy
import subprocess

from std_msgs.msg import String


class PISUSV:

    def __init__(self):

        self.pub = rospy.Publisher("susv", String, queue_size = 10)

        while not rospy.is_shutdown():
            self.check_usv()
            rospy.sleep(0.1)

    def check_usv(self):
        rospy.logerr("calling command")
        #os.system("sudo ./opt/susvd/susv -status")

        output = subprocess.check_output("sudo ./opt/susvd/susv -status", shell=True)
        rospy.logerr("output: " + str(output))
        #move = Twist()
        #self.pub.publish(move)


if __name__ == '__main__':
    rospy.init_node('pisusv')
    try:
        PISUSV()
    except rospy.ROSInterruptException:
        pass


