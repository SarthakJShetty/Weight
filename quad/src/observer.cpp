/*This code is part of the larger "Weight" project, where we are trying to develop probabilisitc path-planning algorithms
for multi-UAV systems.
Here, a constant streams of 1's are published when the code is triggered to resemble an observer detecting a survivor
in it's vicinity.
This is a temporary code, will replace it with a more robust solution for multiple UAV control.

-Sarthak
(19/11/2019)
*/

#include "ros/ros.h"
#include <std_msgs/Int32.h>

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

    ros::Publisher switch_publisher_1 = n.advertise<std_msgs::Int32>("switch_node", 1000);

    //Lowering the rate here and seems like the erraticity of the model has been reduced. Need to debug this further.
    ros::Rate loop_rate(0.5);
    std_msgs::Int32 msg_1;

    //This variable makes sure that the cout statement prints only 10 times once the node has been triggered.
    int print_counter = 0;
    while (ros::ok())
    {
        while (print_counter < 5)
        {
            print_counter += 1;
            cout << "The observer has been triggered!" << endl;

            msg_1.data = 1;
            switch_publisher_1.publish(msg_1);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}