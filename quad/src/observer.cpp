/*This code is part of the larger "Weight" project, where we are trying to develop probabilisitc path-planning algorithms
for multi-UAV systems.
Here, a constant streams of 1's are published when the code is triggered to resemble an observer detecting a survivor
in it's vicinity.
This is a temporary code, will replace it with a more robust solution for multiple UAV control.

-Sarthak
(19/11/2019)
*/

#include "ros/ros.h"
#include <std_msgs/Int8.h>

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
    ros::Publisher chatter_pub = n.advertise<std_msgs::Int8>("/uav0/switch_node", 1000);
    ros::Rate loop_rate(10);
    std_msgs::Int8 msg;

    int count = 0;

    while (ros::ok())
    {
        while (count < 10)
        {
            cout << "The observer has been triggered!" << endl;
            msg.data = 1;
            chatter_pub.publish(msg);
            count += 1;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}