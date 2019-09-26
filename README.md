# Weight
Probabilistic path planning for heterogenous UAV swarms.

:warning: Code is buggy :warning:

This repository contains files related to the "Multi-UAV Swarm for Search & Rescue" project, undertaken at the [Mobile Robotics Laboratory](http://aero.iisc.ac.in/people/debasish-ghose/) in the [Indian Institute of Science, Bengaluru](https://iisc.ac.in).

## Steps:

1. Change the permissions to run ```startscript.sh``` & ```mavroslaunch.sh```:
    
        sarthak@Cipher:~/catkin_ws/src$ chmod +x environmentlaunch.sh
        sarthak@Cipher:~/catkin_ws/src$ chmod +x mavroslaunch.sh

2. Trigger ```MAVROS```:

        sarthak@Cipher:~/catkin_ws/src$ ./mavroslaunch.sh

3. Trigger the ```ENVIRONMENT``` in a different terminal:

        sarthak@Cipher:~/catkin_ws/src$ ./environmentlaunch.sh

4. Run the node where the experiments are designed. ```quad_node``` in this case:

        sarthak@Cipher:~/catkin_ws/src$ rosrun  quad quad_node
        