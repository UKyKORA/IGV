#! /usr/bin/env python

import rospy
from sensor_msgs.msgs import Imu
from geometry_msgs.msgs import Quaternion
from Adafruit_BNO055 import BNO055

PORT = '/dev/serial0'

def main():
	pub = rospy.Publisher('imu', Imu, queue_size=10)
	rospy.init_node('imu_node')
	rate = rospy.Rate(40) # 40 Hz

	# initialize the BNO055 sensor
	bno = BNO055.BNO055(serial_port=PORT, rst=22)
	if not bno.begin():
		raise RuntimeException('Unable to find BNO055!')

	while not rospy.is_shutdown():
		x,y,z,w = bno.read_quaternion()
		imu = Imu()
		quat = Quaternion()
		quat.x = x
		quat.y = y
		quat.z = z
		quat.w = w
		imu.orientation = quat
		pub.publish(imu)
		rate.sleep()

# kick it off
main()
