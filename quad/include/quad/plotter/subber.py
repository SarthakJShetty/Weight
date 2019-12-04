#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import datetime
import time

def position_writer_1(data):
	data_file = 'uav1_pos.txt'
	data_file_writer = open(data_file, 'a')
	data_file_writer.write(data)
	data_file_writer.write('\n')
	data_file_writer.close()

def position_writer_2(data):
	data_file = 'uav2_pos.txt'
	data_file_writer = open(data_file, 'a')
	data_file_writer.write(data)
	data_file_writer.write('\n')
	data_file_writer.close()

def pos1(data):
	position_writer_1(str(data.pose.pose.position.x))
	position_writer_1(str(data.pose.pose.position.y))
	position_writer_1(str(data.pose.pose.position.z))
	position_writer_1(("SECONDS:"+str(datetime.datetime.now().time().minute)+str(datetime.datetime.now().time().second)))

def pos2(data):
	position_writer_2(str(data.pose.position.x))
	position_writer_2(str(data.pose.position.y))
	position_writer_2(str(data.pose.position.z))
	position_writer_2(("SECONDS:"+str(datetime.datetime.now().time().minute)+str(datetime.datetime.now().time().second)))

def plot():
	rospy.init_node('plott', anonymous =True)
	while not rospy.is_shutdown():
		rospy.Subscriber("/uav0/mavros/global_position/local", Odometry, pos1)
		rospy.Subscriber("/uav0/survivor_position", PoseStamped, pos2)
		time.sleep(1)
		rospy.spin()

if __name__ == '__main__':
	try:
		plot()
	except rospy.ROSInterruptException:
		pass