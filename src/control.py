#! /usr/bin/env python
# Master Control Node

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
import message_filters
from tf.transformations import euler_from_quaternion
import math
from igvc.msg import motor_speeds
from config_loader import load_yaml_config

class ControlNode(object):
    ''' Main control algorithm. '''
    def __init__(self):
        rospy.init_node('control_node')
        cfg = load_yaml_config('/home/ubuntu/catkin_ws/src/igvc/config.yml')
        if cfg == None:
            rospy.loginfo(rospy.get_caller_id() + ': Unable to load config. Halting.')
            return
        self.cfg = cfg[rospy.get_caller_id()[1:]]
        self.dest = (self.cfg['dest']['lat'], self.cfg['dest']['lon'])
        self.base_speed_rps = self.cfg['base_speed_rps']
        self.gain = -self.base_speed_rps / math.pi
        self.motor_pub = rospy.Publisher('motor_speeds', motor_speeds, queue_size=10)
        imu_sub = message_filters.Subscriber('imu', Imu)
        gps_sub = message_filters.Subscriber('location', NavSatFix)
        self.ts = message_filters.ApproximateTimeSynchronizer([imu_sub, gps_sub], 10, 0.1)
        self.ts.registerCallback(self.sensor_callback)

    def sensor_callback(self, imu, fix):
        heading, _, _ = euler_from_quaternion([imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z])
        dlat = self.dest[0] - fix.latitude
        dlon = self.dest[1] - fix.longitude
        leftspeed = 0.0
        rightspeed = 0.0
        rospy.loginfo(rospy.get_caller_id() + ": heading = %f, lat = %f, lon = %f", heading / math.pi * 180, fix.latitude, fix.longitude)
        if abs(dlat) > 0.0001 or abs(dlon) > 0.0001:
            # do nav
            desired_heading = -math.atan2(dlat, dlon) + math.pi/2
            rospy.loginfo(rospy.get_caller_id() + ": desired_heading = %f", desired_heading / math.pi * 180.0)
            dspeed = self.gain*(desired_heading - heading)
            leftspeed = self.base_speed_rps - dspeed
            rightspeed = self.base_speed_rps + dspeed
            msg = motor_speeds()
            msg.left_motor_speed = leftspeed
            msg.right_motor_speed = rightspeed
            self.motor_pub.publish(msg)

def main():
    ControlNode()
    rospy.spin()

if __name__ == '__main__':
    main()
