#!/usr/bin/env python

'''This script subscribes to the position of the individual agents and also the survivor.
The script is split into two bits, one set that saves the subscribed position and the subscribers. The data is dumped as a .txt file,
and is then plotted by the grapher.py code that presents temporal variations in the position.

-Sarthak
(02/02/2020)'''

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import datetime
import time
import os

def archive_previous_data():
    '''This function archives all the .txt and .csv files belonging to the previous run to a timestamped log folder'''
    if os.path.isfile('data/uav0_pos.txt'):
        '''We use this to check if there's at least some data from the previous run, if not don't create the LOG folder for no reason'''
        archive_folder = "data/LOG_" + str(datetime.datetime.now().year) + "_" + str(datetime.datetime.now().month) + "_" + str(datetime.datetime.now().day) + "_" + str(datetime.datetime.now().time().hour) + "_" + str(datetime.datetime.now().time().minute) + "_" + str(datetime.datetime.now().time().second)
        '''We first create the timestamped folder where the data will be stored'''
        command_to_rm_function = "mkdir " + archive_folder
        '''Making the timestamped folder'''
        os.system(command_to_rm_function)
        '''We then create the data sub-folder inside the timestamped folder so that we can run the grapher/densityPlotter/mapPlotter/splitPlotter scripts from inside the timestamped folder'''
        archive_folder = archive_folder + '/data'
        command_to_rm_function = "mkdir " + archive_folder
        os.system(command_to_rm_function)
        command_to_rm_function = 'mv data/*.csv ' + archive_folder
        '''Moving all the .csv files generated'''
        os.system(command_to_rm_function)
        command_to_rm_function = 'mv data/*.txt ' + archive_folder
        '''Moving all the .txt files generated'''
        os.system(command_to_rm_function)

def global_position_writer(data, data_file):
    '''Writing survivor's data to the disc.'''
    data_file_writer = open(data_file, 'a')
    data_file_writer.write("START: " + data)
    data_file_writer.write('\n')
    data_file_writer.close()

def position_writer(data, data_file):
    '''Writing survivor's data to the disc.'''
    data_file_writer = open(data_file, 'a')
    data_file_writer.write(data)
    data_file_writer.close()

def uav_pos1(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to UAV 0 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.pose.position.y), str(data.pose.pose.position.x), (str(data.pose.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav0_pos.txt'
    position_writer(str(data.pose.pose.position.x)+'\n'+str(data.pose.pose.position.y)+'\n'+str(data.pose.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def uav_pos2(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to UAV 1 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.pose.position.y), str(data.pose.pose.position.x), (str(data.pose.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav1_pos.txt'
    position_writer(str(data.pose.pose.position.x)+'\n'+str(data.pose.pose.position.y)+'\n'+str(data.pose.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def uav_pos3(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to UAV 2 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.pose.position.y), str(data.pose.pose.position.x), (str(data.pose.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav2_pos.txt'
    position_writer(str(data.pose.pose.position.x)+'\n'+str(data.pose.pose.position.y)+'\n'+str(data.pose.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def uav_pos4(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to UAV 3 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.pose.position.y), str(data.pose.pose.position.x), (str(data.pose.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav3_pos.txt'
    position_writer(str(data.pose.pose.position.x)+'\n'+str(data.pose.pose.position.y)+'\n'+str(data.pose.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def uav_pos5(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to UAV 4 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.pose.position.y), str(data.pose.pose.position.x), (str(data.pose.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav4_pos.txt'
    position_writer(str(data.pose.pose.position.x)+'\n'+str(data.pose.pose.position.y)+'\n'+str(data.pose.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def survivor_pos1(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to Survivor 0 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.position.y), str(data.pose.position.x), (str(data.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/survivor0_pos.txt'
    position_writer(str(data.pose.position.x)+'\n'+str(data.pose.position.y)+'\n'+str(data.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def survivor_pos2(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to UAV 1 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.position.y), str(data.pose.position.x), (str(data.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/survivor1_pos.txt'
    position_writer(str(data.pose.position.x)+'\n'+str(data.pose.position.y)+'\n'+str(data.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def survivor_pos3(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to Survivor 3 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.position.y), str(data.pose.position.x), (str(data.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/survivor2_pos.txt'
    position_writer(str(data.pose.position.x)+'\n'+str(data.pose.position.y)+'\n'+str(data.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def survivor_pos4(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to Survivor 4 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.position.y), str(data.pose.position.x), (str(data.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/survivor3_pos.txt'
    position_writer(str(data.pose.position.x)+'\n'+str(data.pose.position.y)+'\n'+str(data.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def survivor_pos5(data):
    '''Subscribes to the 1st position, i.e. UAV 1's position data.'''
    print("Subscribing to Survivor 5 Data:\nX_Position: %s\nY_Position: %s\nZ_Position: %s\n" % (str(
        data.pose.position.y), str(data.pose.position.x), (str(data.pose.position.z))))
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/survivor4_pos.txt'
    position_writer(str(data.pose.position.x)+'\n'+str(data.pose.position.y)+'\n'+str(data.pose.position.z)+'\n'+("SECONDS:" + str(datetime.datetime.now().time().hour) + "." + str(datetime.datetime.now().time().minute) + "." +
                       str(datetime.datetime.now().time().second)+'\n'), data_file)

def global_pos1(data):
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav0_pos.txt'
    global_position_writer(str(data.pose.position.x) + " " + str(data.pose.position.y), data_file)

def global_pos2(data):
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav1_pos.txt'
    global_position_writer(str(data.pose.position.x) + " " + str(data.pose.position.y), data_file)

def global_pos3(data):
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav2_pos.txt'
    global_position_writer(str(data.pose.position.x) + " " + str(data.pose.position.y), data_file)

def global_pos4(data):
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav3_pos.txt'
    global_position_writer(str(data.pose.position.x) + " " + str(data.pose.position.y), data_file)

def global_pos5(data):
    data_file = '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav4_pos.txt'
    global_position_writer(str(data.pose.position.x) + " " + str(data.pose.position.y), data_file)

def plot():
    '''Plotting all the data here'''
    rospy.init_node('plott', anonymous=True)
    while not rospy.is_shutdown():
        '''Relative global position of the UAV is being subscribed to here'''
        rospy.Subscriber("/uav0/global_position", PoseStamped, global_pos1)
        rospy.Subscriber("/uav1/global_position", PoseStamped, global_pos2)
        rospy.Subscriber("/uav2/global_position", PoseStamped, global_pos3)
        rospy.Subscriber("/uav3/global_position", PoseStamped, global_pos4)
        rospy.Subscriber("/uav4/global_position", PoseStamped, global_pos5)

        '''Local position of the UAV is being subscribed to here'''
        rospy.Subscriber("/uav0/mavros/global_position/local", Odometry, uav_pos1)
        rospy.Subscriber("/uav1/mavros/global_position/local", Odometry, uav_pos2)
        rospy.Subscriber("/uav2/mavros/global_position/local", Odometry, uav_pos3)
        rospy.Subscriber("/uav3/mavros/global_position/local", Odometry, uav_pos4)
        rospy.Subscriber("/uav4/mavros/global_position/local", Odometry, uav_pos5)

        '''Global position of the survivors is being subscribed to here'''
        rospy.Subscriber("/uav0/survivor_position", PoseStamped, survivor_pos1)
        rospy.Subscriber("/uav1/survivor_position", PoseStamped, survivor_pos2)
        rospy.Subscriber("/uav2/survivor_position", PoseStamped, survivor_pos3)
        rospy.Subscriber("/uav3/survivor_position", PoseStamped, survivor_pos4)
        rospy.Subscriber("/uav4/survivor_position", PoseStamped, survivor_pos5)

        time.sleep(1)
        rospy.spin()

if __name__ == '__main__':
    try:
        archive_previous_data()
        plot()
    except rospy.ROSInterruptException:
        pass