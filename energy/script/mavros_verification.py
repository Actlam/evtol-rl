#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import *
# from mavros_msgs.msg import *

def callback(data):
    print(data)
    print('--------------------------------------')

def home_pos():
    rospy.Subscriber('/mavros/global_position/global', NavSatFix, callback)

if __name__ == '__main__':
    try:
        rospy.init_node('listener')
        while not rospy.is_shutdown():
            home_pos() # Change function here to what you are interested in testing
            rospy.spin()
    except rospy.ROSInterruptException:
        pass