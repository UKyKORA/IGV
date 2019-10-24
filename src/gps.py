#! /usr/bin/env python
import rospy
import serial
import time
import sys
from sensor_msgs.msg import NavSatFix

# NOTE: This will need changed depending on the actual GPS tty
PORT = '/dev/ttyACM1'

def parse_gpgga(line):
	rospy.loginfo(rospy.get_caller_id() + ": %s", line)
	parts = line.split(',')
	lat = parts[2]
	decimal_lat = float(lat[:2]) + float(lat[2:])/60
	if parts[3] == 'S':
		decimal_lat = -decimal_lat
	lon = parts[4]
	decimal_lon = float(lon[:3]) + float(lon[3:])/60
	if parts[5] == 'W':
		decimal_lon = -decimal_lon
	msg = NavSatFix()
	msg.latitude = decimal_lat
	msg.longitude = decimal_lon
	return msg

def main():
	pub = rospy.Publisher('location', NavSatFix, queue_size=10)
	rospy.init_node('gps_node')
	rate = rospy.Rate(10) # 10Hz
	ser = serial.Serial(PORT)
	while not rospy.is_shutdown():
		line = ser.readline()
		if line.startswith('$GPGGA'):
			try:
				fix = parse_gpgga(line)
				pub.publish(fix)
			except:
#				print "Error in GPS processing"
				rospy.loginfo(rospy.get_caller_id() + ": FAILED TO PROCESS GPS DATA")
				#print e
				fix = NavSatFix()
				pub.publish(fix)
		rate.sleep()

# kick it off
if __name__ == "__main__":
	main()
