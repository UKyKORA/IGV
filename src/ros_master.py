#! /usr/bin/env python
# Master Control Node

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import message_filters

def callback(imu, fix):
    print imu.orientation, fix.latitude, fix.longitude

def main():
    print "---- CONTROL ----"
    rospy.init_node('control')
    imu_sub = message_filters.Subscriber('imu', Imu)
    gps_sub = message_filters.Subscriber('location', NavSatFix)
    ts = message_filters.ApproximateTimeSynchronizer([imu_sub, gps_sub], 10, 0.1)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    main()
