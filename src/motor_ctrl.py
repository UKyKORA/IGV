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

def rps_to_motor_input(motor_speeds):
	max_velocity_kph = 6.44
	wheel_diameter_cm = 25.4
	cm_per_km = 100000
	sec_per_hr = 3600
	max_input = 2048

	for index, angular_velocity in enumerate(motor_speeds):
		input_val[index] = (angular_velocity * (wheel_diameter_cm/2) * sec_per_hr * max_input) / (max_velocity_kph * cm_per_km)
	return input_val

def motor_speed_callback(motor_speeds):
	rospy.loginfo(rospy.get_caller_id() + "Try to set left motor to %s rad/s, right motor to %s rad/s", motor_speeds.left_motor_speed, motor_speeds.right_motor_speed)
	input_val = rps_to_motor_input(motor_speeds)
	rospy.loginfo(rospy.get_caller_id() + "Motor inputs - Left: %s, Right: %s", input_val[0], input_val[1])
	set_speeds(ser, input_val[0], input_val[1]);

def main():
	rospy.init_node('motor_ctrl_node')
	rospy.Subscriber("motor_speeds", motor_speeds, motor_speed_callback)

	rospy.spin() # Keeps python from exiting while node is running

if __name__ == '__main__':
	# kick it off
	main()
