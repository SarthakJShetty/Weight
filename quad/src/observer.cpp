/*This code is part of the larger "Weight" project, where we are trying to develop probabilisitc path-planning algorithms
for multi-UAV systems.
Here, a constant streams of 1's are published when the code is triggered to resemble an observer detecting a survivor
in it's vicinity.
This is a temporary code, will replace it with a more robust solution for multiple UAV control.

-Sarthak
(19/11/2019)
*/

#include "ros/ros.h"
#include "std_msgs/Int8.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "observer");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int8>("/uav0/switch_node", 1000);
    ros::Rate loop_rate(10);
    int count = 0;

    while (ros::ok())
    {
        std_msgs::Int8 msg;
        msg.data = 1;
        chatter_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}