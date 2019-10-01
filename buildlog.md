# Build-Log

This is a build-log about the day-to-day developments with the [Weight](https://github.com/SarthakJShetty/Weight) project.

There are 3 parts here:

1. Stuff that was completed
2. Stuff that remains
3. Stuff to work on tomorrow

## 2.0 Date: 01/10/2019

### 2.1 Completed:

+ Multiple UAV control using MAVROS has been established
+ Code can be adopted for "N" UAVs as well.

### 2.2 Remaining:

+ UAVs are not localized in the same coordinate system. Need to look into this.
+ Add exploration parameter to the weighted map structure.

### 1.3 Tomorrow:

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