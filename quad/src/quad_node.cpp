/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>
#include "weight.hpp"
#include "variables.hpp"

mavros_msgs::State current_state;
//current_position is used to check whether or not the drone can take the next set of waypoints or not
float current_position_x;
float current_position_y;
float current_position_z;

//This keeps track of the waypoint currently conveyed to the UAV. If = 100 stops
int counter = -1;

extern vector<int> maximum_value_x_indices;
extern vector<int> maximum_value_y_indices;

//Distance keeps track of initial position and desired positon
float dist;

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void pose_sub(const geometry_msgs::PoseStamped msg)
{
    //Subscribing to the current position of the UAV
    current_position_x = msg.pose.position.x;
    current_position_y = msg.pose.position.y;
    current_position_z = msg.pose.position.z;
}

int main(int argc, char **argv)
{
    //Generates the waypoints for the UAV to followw
    weight_generator_function(uav_x_position, uav_y_position, survivor_direction, x_corner_coordinate_1, x_corner_coordinate_2, x_corner_coordinate_3, x_corner_coordinate_4, y_corner_coordinate_1, y_corner_coordinate_2, y_corner_coordinate_3, y_corner_coordinate_4, maximum_value, map_priority, element_cycler, list_maximum_value_x_indices, list_maximum_value_y_indices);
    
    //Initializing the node to handle all the process associated with the code
    ros::init(argc, argv, "offb_node");

    ros::NodeHandle nh;

    //Subscribes to the current position of the UAV
    ros::Subscriber position_subscriber = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_sub);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    //Topic to publish to, to change the UAVs position
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    //Topic to check whether the UAV is armed or not
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    //Topic to check whether control is ONBOARD or OFFBOARD
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped pose;

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {

        if (counter == -1)
        {
            //Initial set of coordinates being published to the position topic of the UAV
            pose.pose.position.x = list_maximum_value_x_indices[counter];
            pose.pose.position.y = list_maximum_value_y_indices[counter];
            pose.pose.position.z = 2;
            counter += 1;
        }
        else
        {
            //Difference between the current position and the next waypoint (x, y)
            dist = sqrt(pow((pose.pose.position.x - current_position_x), 2) + pow((pose.pose.position.y - current_position_y), 2));

            cout << "Current X Position" << current_position_x << endl;
            cout << "Current Y Position" << current_position_y << endl;
            cout << "Current X Waypoint" << pose.pose.position.x << endl;
            cout << "Current Y Waypoint" << pose.pose.position.y << endl;

            if (dist < 0.5)
            {
                cout << "Distance <0.5" << endl;
                if (counter < 100)
                {
                    cout << "Counter" << counter << endl;
                    cout << "Maximum_Value_X_Indices: " << counter <<" "<<list_maximum_value_x_indices[counter] << endl;
                    cout << "Maximum_Value_Y_Indices: " << counter <<" "<<list_maximum_value_y_indices[counter] << endl;
                    pose.pose.position.x = list_maximum_value_x_indices[counter];
                    pose.pose.position.y = list_maximum_value_y_indices[counter];
                    pose.pose.position.z = 2;
                    counter += 1;
                }
                else
                {
                    cout << "RTL" << endl;
                    pose.pose.position.x = 2;
                    pose.pose.position.y = 2;
                    pose.pose.position.z = 2;
                }
            }
        }
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}