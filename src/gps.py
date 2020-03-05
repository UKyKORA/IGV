#! /usr/bin/env python
import rospy
import serial
from sensor_msgs.msg import NavSatFix
from config_loader import load_yaml_config

class GPSNode(object):
	''' A ROS node to interface with a serial NMEA GPS. '''
	def __init__(self):
		rospy.init_node('gps_node')
		cfg = load_yaml_config('/home/ubuntu/catkin_ws/src/igvc/config.yml')
		if cfg == None:
			rospy.loginfo(rospy.get_caller_id() + ': Unable to load config file. Stopping.')
			return
			
		self.node_cfg = cfg[rospy.get_caller_id()[1:]]

		self.pub = rospy.Publisher('location', NavSatFix, queue_size=10)
		self.rate = rospy.Rate(self.node_cfg['rate'])
		self.fix = NavSatFix()
		if self.node_cfg['active']:
			self.ser = serial.Serial(self.node_cfg['port'])

	def run(self):	
		while not rospy.is_shutdown():
			if self.node_cfg['active']:
				line = self.ser.readline()
				if line.startswith('$GPGGA'):
					try:
						self.fix = self.parse_gpgga(line)
					except:
						rospy.loginfo(rospy.get_caller_id() + ": FAILED TO PROCESS GPS DATA")
			else:
				self.fix.latitude = self.node_cfg['sim_data']['lat']
				self.fix.longitude = self.node_cfg['sim_data']['lon']
			self.pub.publish(fix)
			self.rate.sleep()

	def parse_gpgga(self, line):
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

# kick it off
if __name__ == "__main__":
	node = GPSNode()
	node.run()
