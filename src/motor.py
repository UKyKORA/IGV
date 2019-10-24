#! /usr/bin/env python

import rospy
from igvc.msg import motor_speeds
import serial
from config_loader import load_yaml_config

class MotorNode(object):
	''' A ROS node to interface with the Sabertooth Motor Driver. '''
	def __init__(self):
		rospy.init_node('motor_node')
		cfg = load_yaml_config('config.yml')
		if cfg == None:
			rospy.loginfo(rospy.get_caller_id() + ': Unable to load config. Halting.')
			return
		self.cfg = cfg[rospy.get_caller_id()]
		if self.cfg['active']:
			self.ser = serial.Serial(self.cfg['port'])
		rospy.Subscriber("motor_speeds", motor_speeds, self.motor_speed_callback)

	def set_speeds(self, left, right):
		if self.cfg['active']:
			self.ser.write(str.encode('M1: {0}\r\nM2: {1}\r\n'.format(int(left), -int(right))))

	def rps_to_motor_input(self, motor_speeds):
		max_velocity_kph = self.cfg['max_velocity_kph']
		wheel_diameter_cm = self.cfg['wheel_diameter_cm']
		cm_per_km = 100000
		sec_per_hr = 3600
		max_input = self.cfg['max_driver_input']
		motor_speeds_list = [motor_speeds.left_motor_speed, motor_speeds.right_motor_speed]
		input_val = []
		for angular_velocity in motor_speeds_list:
			input_val.append(int(angular_velocity * (wheel_diameter_cm/2) * sec_per_hr * max_input) \
				/ (max_velocity_kph * cm_per_km))
		return input_val

	def motor_speed_callback(self, motor_speeds):
		rospy.loginfo(rospy.get_caller_id() + ": Recvd left motor = %1.2f rad/s, right motor = %1.2f rad/s", \
			motor_speeds.left_motor_speed, motor_speeds.right_motor_speed)
		input_val = self.rps_to_motor_input(motor_speeds)
		rospy.loginfo(rospy.get_caller_id() + ": To Driver - Left: %d, Right: %d", input_val[0], input_val[1])
		self.set_speeds(input_val[0], input_val[1])

def main():
	MotorNode()
	rospy.spin() # Keeps python from exiting while node is running

if __name__ == '__main__':
	# kick it off
	main()
