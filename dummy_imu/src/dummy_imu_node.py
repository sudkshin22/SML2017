#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import Imu

def dummy_imu():
	rospy.init_node("dummy_imu_node")

	pub = rospy.Publisher('dummy_imu_topic', Imu, queue_size=1)
	imu = Imu()

	rate = rospy.Rate(10)	
	while not rospy.is_shutdown():
		imu.orientation.x = np.random.rand()
		imu.orientation.y = np.random.rand()
		imu.orientation.z = np.random.rand()

		pub.publish(imu)
		rate.sleep()

if __name__=='__main__':
	dummy_imu()

