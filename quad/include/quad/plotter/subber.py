#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import datetime
import time


def position_writer_1(data):
    '''Writing 1st position, i.e. UAV 1's position data.'''
    data_file = 'uav1_pos.txt'
    data_file_writer = open(data_file, 'a')
    data_file_writer.write(data)
    data_file_writer.write('\n')
    data_file_writer.close()


def position_writer_2(data):
    '''Writing 1st position, i.e. UAV 1's position data.'''
    data_file = 'uav2_pos.txt'
    data_file_writer = open(data_file, 'a')
    data_file_writer.write(data)
    data_file_writer.write('\n')
    data_file_writer.close()


def position_writer_3(data):
    '''Writing survivor's data to the disc.'''
    data_file = 'survivor_pos.txt'
    data_file_writer = open(data_file, 'a')
    data_file_writer.write(data)
    data_file_writer.write('\n')
    data_file_writer.close()


def pos1(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to UAV 1 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.pose.position.y), str(data.pose.pose.position.x), (str(data.pose.pose.position.z))))
    position_writer_1(str(data.pose.pose.position.x))
    position_writer_1(str(data.pose.pose.position.y))
    position_writer_1(str(data.pose.pose.position.z))
    position_writer_1(("SECONDS:"+str(datetime.datetime.now().time().minute) +
                       str(datetime.datetime.now().time().second)))


def pos2(data):
    '''Subscribes to the 1st position, i.e. UAV 2's position data.'''
    print("Subscribing to UAV 2 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.pose.position.y), str(data.pose.pose.position.x), (str(data.pose.pose.position.z))))
    position_writer_2(str(data.pose.pose.position.x))
    position_writer_2(str(data.pose.pose.position.y))
    position_writer_2(str(data.pose.pose.position.z))
    position_writer_2(("SECONDS:"+str(datetime.datetime.now().time().minute) +
                       str(datetime.datetime.now().time().second)))


def pos3(data):
    '''Subscribes to the survivor's position data.'''
    print("Subscribing to Survivor\'s Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (
        str(data.pose.position.y), str(data.pose.position.x), (str(data.pose.position.z))))
    position_writer_3(str(data.pose.position.x))
    position_writer_3(str(data.pose.position.y))
    position_writer_3(str(data.pose.position.z))
    position_writer_3(("SECONDS:"+str(datetime.datetime.now().time().minute) +
                       str(datetime.datetime.now().time().second)))


def plot():
    '''Plotting all the data here'''
    rospy.init_node('plott', anonymous=True)
    while not rospy.is_shutdown():
        '''More subscribers can be added here'''
        rospy.Subscriber("/uav0/mavros/global_position/local", Odometry, pos1)
        rospy.Subscriber("/uav1/mavros/global_position/local", Odometry, pos2)
        rospy.Subscriber("/uav0/survivor_position", PoseStamped, pos3)
        time.sleep(1)
        rospy.spin()


if __name__ == '__main__':
    try:
        plot()
    except rospy.ROSInterruptException:
        pass
