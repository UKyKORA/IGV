#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from Adafruit_BNO055 import BNO055
from config_loader import load_yaml_config
from tf.transformations import quaternion_from_euler
import math

class IMUNode(object):
	''' A ROS node for reading data from a BNO055 IMU over serial. '''
	def __init__(self):
		rospy.init_node('imu_node')
		cfg = load_yaml_config('/home/ubuntu/catkin_ws/src/igvc/config.yml')
		if cfg == None:
			rospy.loginfo(rospy.get_caller_id() + ': Unable to load conifg. Halting.')
			rospy.spin()

		self.node_cfg = cfg[rospy.get_caller_id()[1:]]

		self.pub = rospy.Publisher('imu', Imu, queue_size=10)
		self.rate = rospy.Rate(self.node_cfg['rate'])

		# initialize the BNO055 sensor
		if self.node_cfg['active']:
			self.bno = BNO055.BNO055(serial_port=self.node_cfg['port'], rst=22)
			if not self.bno.begin():
				raise RuntimeException('Unable to find BNO055!')	
			
	def run(self):
		while not rospy.is_shutdown():
			imu = Imu()
			if self.node_cfg['active']:
				x,y,z,w = self.bno.read_quaternion()
				imu.orientation.x = x
				imu.orientation.y = y
				imu.orientation.z = z
				imu.orientation.w = w
			else:
				quat = quaternion_from_euler(self.node_cfg['sim_data']['heading'] / 180.0 * math.pi, 0.0, 0.0)
				imu.orientation.w = quat[0]
				imu.orientation.x = quat[1]
				imu.orientation.y = quat[2]
				imu.orientation.z = quat[3]
			self.pub.publish(imu)
			self.rate.sleep()

# kick it off
if __name__ == "__main__":
	node = IMUNode()
	node.run()
