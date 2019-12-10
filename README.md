# Weight

Probabilistic path planning for heterogenous UAV swarms.

:warning: **Code is buggy** :warning:

## 1.0 About

+ This repository contains files related to the "Multi-UAV Swarm for Search & Rescue" project, undertaken at the [Mobile Robotics Laboratory](http://aero.iisc.ac.in/people/debasish-ghose/) in the [Indian Institute of Science, Bengaluru](https://iisc.ac.in).

+ The coordination model being designed is based on the novel "Weight Based Probabilistic" path planning algorithms ¹.


## 2.0 Simulations:

2.1. Trigger the ```ENVIRONMENT```. This launches ```mavros```, ```mavlink``` & the prototyping environment:

        sarthak@Cipher:~/catkin_ws/src$ ./environmentlaunch.sh

This triggers the ```mavros``` node to control the UAV, the ```quad_node``` which enables ```OFFBOARD``` control for autonomous navigation, and also ```sources``` the ```catkin``` environment.

2.2. Trigger the survivor model by invoking the ```quad_observer_node``` . 

        sarthak@Cipher:~/catkin_ws/src$ rosrun quad quad_observer_node
`
Currently, the velocity of the survivor has been set in [variables.hpp](https://github.com/SarthakJShetty/Weight/blob/master/quad/include/quad/variables/variables.hpp) to 0.3 m/s. 

2.3. A [```subber.py```](https://github.com/SarthakJShetty/Weight/blob/master/quad/include/quad/plotter/subber.py) script has been created to subscribe to the UAV's and the survivor's position, which are described as ROS topics.

        sarthak@Cipher:~/catkin_ws/src/quad/include/quad/plotter$ python subber.py

Right now, the script subscribes to 2 topics for plotting, ```/uavX/mavros/global_position/local``` & ```/uavX/survivor_position```.

***Note:*** X referes to a variable number of UAVs that can be deployed using the model.

## 3.0 Results:

***Note:*** *These are preliminary results being posted here. We expect more to be added here soon.*

### 3.1 Lawn-mower search pattern:

Here, the UAV attempts to find the survivor using just the novel lawn-mower algorithm that we designed.

![alt text](https://raw.githubusercontent.com/SarthakJShetty/Weight/master/assets/Lawn_Mower_0.3.png "Lawn Mower Pattern")

### 3.2 Probabilistic search + lawn-mower:

The UAV starts out following the lawn-mower pattern, then follows the probabilistic pattern upon receiving a stream of messages from the ```quad_observer_node```, which broadcasts the survivor's last known coordinates and heading.

![alt text](https://raw.githubusercontent.com/SarthakJShetty/Weight/master/assets/Probabilistic_Lawn_Mower_0.3.png "Probabilistic + Lawn Mower")

***Note:*** *We'll be adding more results here, including with variable velocities and different headings with varying trajectories.*