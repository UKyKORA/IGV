#! /usr/bin/env python

import rospy
from std_msgs.msg import motor_speeds
import serial
import time
import sys

DURATION = 5
SPEED = -1023
PORT = '/dev/ttyACM0'
ser = serial.Serial(PORT)

def set_speeds(ser, left, right):
	#print 'Setting speeds ({0}, {1})'.format(int(left), int(right))
	ser.write(str.encode('M1: {0}\r\nM2: {1}\r\n'.format(int(left), -int(right))))

def motor_speed_callback(motor_speeds):
	rospy.loginfo(rospy.get_caller_id() + "Try to set left motor to %s rad/s, right motor to %s rad/s", motor_speeds.left_motor_speed, motor_speeds.right_motor_speed)
	set_speeds(ser, motor_speeds.left_motor_speed, motor_speeds.right_motor_speed);

def main():
	rospy.init_node('motor_ctrl_node')
	rospy.Subscriber("motor_speeds", motor_speeds, motor_speed_callback)

	rospy.spin() # Keeps python from exiting while node is running

if __name__ == '__main__':
	# kick it off
	main()
