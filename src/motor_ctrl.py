#! /usr/bin/env python

import rospy
import serial
import time

DURATION = 5
SPEED = -1023

def set_speeds(ser, left, right):
	#print 'Setting speeds ({0}, {1})'.format(int(left), int(right))
	ser.write(str.encode('M1: {0}\r\nM2: {1}\r\n'.format(int(left), -int(right))))

def ramp(ser, initial, final):
	STEPS = 8
	DT = 0.25
	DS_LEFT = float(final[0] - initial[0]) / STEPS
	DS_RIGHT = float(final[1] - initial[1]) / STEPS
	for i in range(STEPS):
		set_speeds(ser, initial[0] + DS_LEFT * i, initial[1] + DS_RIGHT * i)
		time.sleep(DT)
	set_speeds(ser, final[0], final[1])

def main():
	rospy.init_node('motor_ctrl_node')
	print "\n\nSabertooth 2x32 Driver\n\n"
	ser = serial.Serial('/dev/ttyACM0')
	ramp(ser, (0,0), (SPEED, SPEED))
	time.sleep(DURATION)
	ramp(ser, (SPEED, SPEED), (0, 0))
	time.sleep(5)
	ramp(ser, (0,0), (-SPEED, -SPEED))
	time.sleep(DURATION)
	ramp(ser, (-SPEED, -SPEED), (0,0))
	rospy.spin()

# kick it off
main()
