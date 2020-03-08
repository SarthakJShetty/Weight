'''In this script we trigger the terminals required to run the MAVROS node and associated topics including: 
1. roscore 
2. mavros_node 
3. /mavros/state'''

#CD'ing to the catkin workspace from where the code is invoked
cd ~/catkin_ws/

#Building the environment here before running the code on the UAV
catkin build

#Souring the work environment here
source devel/setup.bash

#Initializing the roscore here for the MAVROS node to connect to
gnome-terminal -e "bash -c \"roscore; exec bash\""

#Sleeps the code for 10 seconds, mainly when hardware testing on laptop.
sleep 10

#Triggering the mavros_node here, connecting to the USB port where the transmitter here
gnome-terminal -e "bash -c \"rosrun mavros mavros_node _fcu_url:=/dev/ttyUSB0; exec bash\""

#The code sleeps for a bit before publishing the state of the mavros connection. The mavros_node takes a bit of time to connect to the master
sleep 10

#Subscribing to the mavros_node state
gnome-terminal -e "bash -c \"rostopic echo /mavros/state; exec bash\""