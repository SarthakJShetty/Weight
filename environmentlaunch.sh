cd ~/src/Firmware
source ~/catkin_ws/devel/setup.bash
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
gnome-terminal -e "bash -c \"roslaunch px4 multi_uav_mavros_sitl.launch; exec bash\""
sleep 10
gnome-terminal -e "bash -c \"rosrun quad quad_node; exec bash\""