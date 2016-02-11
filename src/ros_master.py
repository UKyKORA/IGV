#! /usr/bin/env python
# Master Control Node

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu

def callback(data):
    print data
    rospy.loginfo(rospy.get_caller_id() + " data = %s", data)

def main():
    print "---- CONTROL ----"
    rospy.init_node('control')
    rospy.Subscriber('imu', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    main()
