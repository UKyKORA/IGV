# Master Control Node

import rospy
from sensor_msgs.msg import NavSatFix

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " data = %s", data.data)

def master():
    rospy.init_node('master')
    rospy.Subscriber('location', NavSatFix, callback)

    rospy.spin()

if __name__ == '__main__':
    master()