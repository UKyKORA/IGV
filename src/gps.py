#! /usr/bin/env python

#import rospy
import serial
import time
import sys

PORT = '/dev/ttyACM0'

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

def parse_gpgga(line):
	parts = line.split(',')
	lat = parts[2]
	lat_sign = (parts[3] == 'S') * -2 + 1
	decimal_lat = float(lat[:2]) + float(lat[2:])/60
	lon = parts[4]
	lon_sign = (parts[5] == 'W') * -2 + 1
	decimal_lon = float(lon[:3]) + float(lon[3:])/60
	return (decimal_lat * lat_sign, decimal_lon * lon_sign)

def main():
#	rospy.init_node('gps_node')
	ser = serial.Serial(PORT)
	while True:
		line = ser.readline()
		if line.startswith('$GPGGA'):
			print parse_gpgga(line)
#	rospy.spin()

# kick it off
main()
