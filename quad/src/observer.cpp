/*This code is part of the larger "Weight" project, where we are trying to develop probabilisitc path-planning algorithms
for multi-UAV systems.
Here, a constant streams of 1's are published when the code is triggered to resemble an observer detecting a survivor
in it's vicinity.

-Sarthak
(19/11/2019)
*/

#include "ros/ros.h"
#include <std_msgs/Int32MultiArray.h>

using namespace std;

/*Changes to be implemented here:
1. Move this to independent file within the quad filespace.
2. Trigger the node using a temrinal based launch file rather than a .cpp file trigger.
3. How can we decouple the generality of the file from the control of the individual UAVs?
*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "observer");
    ros::NodeHandle n;

    //Declaring the number of UAVs here
    int N_UAV = 2;

    //Declaring the publisher here. Only a single topic to control the switch nodes of all UAVs at their respective indices.
    ros::Publisher switch_publisher_1 = n.advertise<std_msgs::Int32MultiArray>("/uav/switch_node", 1000);

    //Lowering the rate here and seems like the erraticity of the model has been reduced. Need to debug this further.
    ros::Rate loop_rate(1);

    //Here, we declare the array that gets published over the ROS Comms to the quad_node.
    std_msgs::Int32MultiArray msg_1;

    //Specific UAV that has to be triggered by the observer node.
    int uav_to_trigger = 0;

    //This variable makes sure that the cout statement prints only 10 times once the node has been triggered.
    int print_counter = 0;
    while (ros::ok())
    {
        for (int i = 0; i < N_UAV; i++)
        {
            if(uav_to_trigger == i)
            {
            //If the specifc UAVs index is encountered in the array, then 1 it.
            msg_1.data.push_back(1);
            }
            else
            {
            //If the specifc UAVs index is not encountered in the array, then 0 it.
            msg_1.data.push_back(0);
            }
        }
        //Publish the data to the publisher to the ROS Comms.
        switch_publisher_1.publish(msg_1);
        ros::spinOnce();
        loop_rate.sleep();
        //We clear the previous data before repopulating it for the next loop. Otherwise, the array becomes multi-dimensoned.
        msg_1.data.clear();
    }
    return 0;
}