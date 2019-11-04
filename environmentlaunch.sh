cd ~/catkin_ws/
source devel/setup.bash
gnome-terminal -e "bash -c \"roscore; exec bash\""
gnome-terminal -e "bash -c \"rosrun mavros mavros_node _fcu_url:=/dev/ttyUSB0; exec bash\""
sleep 5
gnome-terminal -e "bash -c \"rostopic echo /mavros/state; exec bash\""