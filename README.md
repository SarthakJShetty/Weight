# Weight
Probabilistic path planning for heterogenous UAV swarms.

*Note:* This branch exclusively peratains to the hardware implementation of the models developed in this repository.

[Here](https://github.com/SarthakJShetty/Weight) is the main repository!

:warning: **Code is buggy** :warning:

This repository contains files related to the "Multi-UAV Swarm for Search & Rescue" project, undertaken at the [Mobile Robotics Laboratory](http://aero.iisc.ac.in/people/debasish-ghose/) in the [Indian Institute of Science, Bengaluru](https://iisc.ac.in).

## Steps:

1. Trigger ```ROSCORE``` so that the ```MAVROS_NODE``` can connect to the ```GCS``` running the control code.

        sarthak@Cipher:roscore

2. Trigger the ```MAVROS_NODE``` which communicates with the ```MAVROS``` node running on the PX4 on-board the quad.

        sarthak@Cipher:rosrun mavros mavros_node _fcu_url:=/dev/ttyUSB0

3. Run the node where the experiments are designed. ```quad_node``` in this case:

        sarthak@Cipher:~/catkin_ws/src$ rosrun  quad quad_node
        