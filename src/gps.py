#! /usr/bin/env python
import rospy
import serial
from sensor_msgs.msg import NavSatFix
from config_loader import load_yaml_config

def parse_gpgga(line):
	''' Parses a GPGGA line of GPS data into an Imu message with latitude and longitude. '''
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
	rospy.init_node('gps_node')
	cfg = load_yaml_config('/home/ubuntu/catkin_ws/src/igvc/config.yml')
	if cfg == None:
		rospy.loginfo(rospy.get_caller_id() + ': Unable to load config file. Stopping.')
		return
	node_cfg = cfg[rospy.get_caller_id()[1:]]

	pub = rospy.Publisher('location', NavSatFix, queue_size=10)
	rate = rospy.Rate(node_cfg['rate'])
	fix = NavSatFix()
	if node_cfg['active']:
		ser = serial.Serial(node_cfg['port'])
	while not rospy.is_shutdown():
		if node_cfg['active']:
			line = ser.readline()
			if line.startswith('$GPGGA'):
				try:
					fix = parse_gpgga(line)
				except:
					rospy.loginfo(rospy.get_caller_id() + ": FAILED TO PROCESS GPS DATA")
		else:
			fix.latitude = node_cfg['sim_data']['lat']
			fix.longitude = node_cfg['sim_data']['lon']
		pub.publish(fix)
		rate.sleep()

# kick it off
if __name__ == "__main__":
	main()
