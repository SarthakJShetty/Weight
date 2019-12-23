cd ~/catkin_ws/
catkin build
cd ~/src/Firmware
rm /home/sarthak/src/Firmware/launch/multi_uav_mavros_sitl.launch
cp /home/sarthak/catkin_ws/src/config/multi_uav_mavros_sitl.launch /home/sarthak/src/Firmware/launch/
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 multi_uav_mavros_sitl.launch