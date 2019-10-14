#! /usr/bin/env python

#import rospy
import serial
import time
import sys

PORT = '/dev/ttyACM0'

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
