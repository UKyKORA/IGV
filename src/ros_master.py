#! /usr/bin/env python
# Master Control Node

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import message_filters
from tf.transformations import euler_from_quaternion
import math

def callback(imu, fix):
    heading, _, _ = euler_from_quaternion([imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z])
    heading = heading / math.pi * 180.0
    print heading, fix.latitude, fix.longitude

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
