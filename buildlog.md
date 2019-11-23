# Build-Log

This is a build-log about the day-to-day developments with the [Weight](https://github.com/SarthakJShetty/Weight) project. For a more detailed LOG about the build process, check out the commits; each is accompanied  by very detailed (I try) commit messages.

There are 3 parts here:

1. Stuff that was completed

2. Stuff that remains

3. Stuff to work on tomorrow (_Not anymore - 23/11/2019_)

## 3.0 Date: 23/11/2019

**Note:** _Step function rather of an update._

### 3.1 Completed:
+ Designed a lawn-mower planning algorithm from [scratch](https://github.com/SarthakJShetty/Weight/blob/master/quad/include/quad/lawnmower/src/lawnmower.cpp) ! It works really well! Agnostic to the dimensions of the space and symmetry of the encironment.

+ Integrated the weight-based search pattern and the lawn-mower model! It works, will be uploaded screen grabs of each implementation.

+ We now have a survivor [model](https://github.com/SarthakJShetty/Weight/tree/master/quad/include/quad/survivor) as well. Just like the lawn-mower code, it's extremely robust to testing in a number of environments.

+ We've integrated the entire system into one, with ```rosrun quad quad_observer_node``` triggering the survivor and making it dynamic with a constant velocity.

+ Hardware testing of the individual models is done!

### 3.2 Remaining:
+ There's a bug in parsing the final waypoint in the survivor's model which has to be fixed.

+ Hardware testing of the entire, integrated model. Currently backlogged due to controller issues.

+ _(Still prototyping)_ Sharing of a global coordinate system between multiple UAVs.

### 3.3 Tomorrow:

**Note:** Scrapping this section from here on out.

## 2.0 Date: 01/10/2019

### 2.1 Completed:

+ Multiple UAV control using MAVROS has been established

+ Code can be adopted for "N" UAVs as well.

### 2.2 Remaining:

+ UAVs are not localized in the same coordinate system. Need to look into this.

+ Add exploration parameter to the weighted map structure.

### 2.3 Tomorrow:

+ Fix the common coordinate issue for the UAVs.

+ Readup papers and design a dynamic area allocation algorithm, given N UAVs.

+ Will deploy Voronoi Partitioning as a final option.

## 1.0 Date: 27/09/2019

### 1.1 Completed:

+ Multiple UAVs deployed in the environement.

+ Solved the coordinate mixup issue.

### 1.2 Remaining:

+ Control both UAVs from ROS code.

+ Avoid hardcoding the UAV publisher/subscriber. Use loops with variables from 0 -> N_UAV.

### 1.3 Tomorrow:

+ Partitioning of enviroment into "N" partitions, using Voronoi partitioning.

+ Generate waypoints in individual partitions.