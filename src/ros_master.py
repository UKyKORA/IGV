#! /usr/bin/env python
# Master Control Node

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import message_filters
from tf.transformations import euler_from_quaternion
import math
from igvc.msg import motor_speeds

dest = (38.037402, -84.504964)
P = -2.0 / math.pi
BASE_SPEED = 2.0
motor_pub = rospy.Publisher('motor_speeds', motor_speeds, queue_size=10)

def callback(imu, fix):
    heading, _, _ = euler_from_quaternion([imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z])
    dlat = dest[0] - fix.latitude
    dlon = dest[1] - fix.longitude
    leftspeed = 0.0
    rightspeed = 0.0
    if abs(dlat) > 0.001 or abs(dlon) > 0.001:
        # do nav
        desired_heading = math.atan2(dlat, dlon)
        dspeed = P*(desired_heading - heading)
        leftspeed = BASE_SPEED - dspeed
        rightspeed = BASE_SPEED + dspeed
        msg = motor_speeds()
        msg.left_motor_speed = leftspeed
        msg.right_motor_speed = rightspeed
        motor_pub.publish(msg)
        print msg

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
