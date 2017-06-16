#! /usr/bin/env python

import rospy
import numpy as np
import serial
from get_gps.msg import gps_reach

def receiver():
	port = rospy.get_param('~port', '/dev/ttyACM0')
	ser = serial.Serial(port=port, baudrate=115200, timeout=1)
	
	rospy.init_node('gps_receiver', anonymous=True)
	pub = rospy.Publisher('gps_stream', gps_reach, queue_size=10)
	
	solution_types = ['fixed', 'float', 'reserved', 'DGPS', 'single']
	
	gps = gps_reach()
	while not rospy.is_shutdown():
		line = ser.readline()
		words = line.split()

		if len(words) == 15:
			gps.date = words[0]
			gps.time = words[1]
			
			gps.latitude = float(words[2])
			gps.longitude = float(words[3])
			gps.height = float(words[4])
			
			gps.solution = solution_types[int(words[5])-1] 
			gps.num_satelites = int(words[6])
			
			gps.stn = float(words[7])
			gps.ste = float(words[8])
			gps.stu = float(words[9])
			gps.stne = float(words[9])
			gps.steu = float(words[10])
			gps.stun = float(words[11])
			
			gps.age = float(words[12])
			gps.ratio = float(words[13])

		pub.publish(gps)

receiver()
