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

mavros_msgs::State current_state[N_UAV];

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    cout << "STATE SUB INITIALIZER" << endl;
    current_state[hello_there] = *msg;
}

void pose_sub(const geometry_msgs::PoseStamped msg)
{
    cout << "PRE PUB SUB INITIALIZER: " << hello_there << endl;
    //Subscribing to the current position of the UAV
    current_position_x[hello_there] = msg.pose.position.x;
    current_position_y[hello_there] = msg.pose.position.y;
    current_position_z[hello_there] = msg.pose.position.z;
}

int main(int argc, char **argv)
{
    //Generates the waypoints for the UAV to followw
    weight_generator_function(uav_x_position, uav_y_position, survivor_direction, x_corner_coordinate_1, x_corner_coordinate_2, x_corner_coordinate_3, x_corner_coordinate_4, y_corner_coordinate_1, y_corner_coordinate_2, y_corner_coordinate_3, y_corner_coordinate_4, maximum_value, map_priority, element_cycler, list_maximum_value_x_indices, list_maximum_value_y_indices);

    //Initializing the node to handle all the process associated with the code
    ros::init(argc, argv, "offb_node");

    ros::NodeHandle nh;

    /*
    What are we trying to incorporate here?
    1. A for loop from 0 -> N_UAV
    2. Get the string that goes into the subscriber/publisher
    3. Declare N_UAVs for each subscriber/publisher
    4. Store each UAVs information in the corresponding publisher/subscriber
    */

    ros::Subscriber position_subscriber[N_UAV];
    ros::Subscriber state_sub[N_UAV];
    ros::Publisher local_pos_pub[N_UAV];
    ros::ServiceClient arming_client[N_UAV];
    ros::ServiceClient set_mode_client[N_UAV];

    for (int pre_pub_sub_initializer = 0; pre_pub_sub_initializer < N_UAV; pre_pub_sub_initializer++)
    {
        hello_there = pre_pub_sub_initializer;
        stringstream pub_sub_initializer;
        pub_sub_initializer << pre_pub_sub_initializer;

        string position_subscriber_string;
        position_subscriber_string = "/uav" + pub_sub_initializer.str() + "/mavros/local_position/pose";
        cout << "position_subscriber_string: " << position_subscriber_string << endl;
        position_subscriber[pre_pub_sub_initializer] = nh.subscribe<geometry_msgs::PoseStamped>(position_subscriber_string, 10, pose_sub);

        string state_sub_string;
        state_sub_string = "/uav" + pub_sub_initializer.str() + "/mavros/state";
        cout << "state_sub_string: " << state_sub_string << endl;
        state_sub[pre_pub_sub_initializer] = nh.subscribe<mavros_msgs::State>(state_sub_string, 10, state_cb);

        string local_pos_pub_string;
        local_pos_pub_string = "/uav" + pub_sub_initializer.str() + "/mavros/setpoint_position/local";
        cout << "local_pos_pub_string: " << local_pos_pub_string << endl;
        local_pos_pub[pre_pub_sub_initializer] = nh.advertise<geometry_msgs::PoseStamped>(local_pos_pub_string, 10);

        string arming_client_string;
        arming_client_string = "/uav" + pub_sub_initializer.str() + "/mavros/cmd/arming";
        cout << "arming_client_string: " << arming_client_string << endl;
        arming_client[pre_pub_sub_initializer] = nh.serviceClient<mavros_msgs::CommandBool>(arming_client_string);

        string set_mode_client_string;
        set_mode_client_string = "/uav" + pub_sub_initializer.str() + "/mavros/set_mode";
        cout << "set_mode_client_string: " << set_mode_client_string << endl;
        set_mode_client[pre_pub_sub_initializer] = nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_string);

        cout << endl;
    }

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped pose[N_UAV];

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        hello_there = UAV_COUNTER;
        counter[UAV_COUNTER] = -1;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        hello_there = UAV_COUNTER;
        cout << "PUSHING TO POSE OF UAV: " << UAV_COUNTER << endl;
        pose[UAV_COUNTER].pose.position.x = 0;
        pose[UAV_COUNTER].pose.position.y = 0;
        pose[UAV_COUNTER].pose.position.z = 2;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        hello_there = UAV_COUNTER;
        cout << "PRESENTING POSE OF UAV: " << UAV_COUNTER << endl;
        cout << "X: " << pose[UAV_COUNTER].pose.position.x << endl;
        cout << "Y: " << pose[UAV_COUNTER].pose.position.y << endl;
        cout << "Z: " << pose[UAV_COUNTER].pose.position.z << endl;
    }

    int connected_state_counter;

    for (int i = 0; i < N_UAV; i++)
    {
        hello_there = i;
        if (!current_state[i].connected)
        {
            cout << "CURRENT_STATE" << endl;
            connected_state_counter += 1;
        }
    }

    // wait for FCU connection
    while (ros::ok() && (connected_state_counter == N_UAV))
    {
        cout << "CURRENT_STATE FCU NOT TRIGGERED" << endl;
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        hello_there = UAV_COUNTER;
        for (int i = 100; ros::ok() && i > 0; --i)
        {
            cout << "INITAL PUBLISHING: " << i << endl;
            cout << "UAV_COUNTER: " << UAV_COUNTER << endl;
            local_pos_pub[UAV_COUNTER].publish(pose[UAV_COUNTER]);
            cout << "HERE" << endl;
            ros::spinOnce();
            rate.sleep();
            cout << endl;
        }
    }
    mavros_msgs::SetMode offb_set_mode[N_UAV];

    for (int i = 0; i < N_UAV; i++)
    {
        hello_there = i;
        cout << "OFFBOARD TRIGGER" << endl;
        offb_set_mode[i].request.custom_mode = "OFFBOARD";
    }

    mavros_msgs::CommandBool arm_cmd[N_UAV];
    for (int i = 0; i < N_UAV; i++)
    {
        hello_there = i;
        cout << "ARMING" << endl;
        arm_cmd[i].request.value = true;
    }

    ros::Time last_request[N_UAV];
    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        hello_there = UAV_COUNTER;
        cout << "LAST REQUEST" << endl;
        last_request[UAV_COUNTER] = ros::Time::now();
    }

    while (ros::ok())
    {
        for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
        {
            hello_there = UAV_COUNTER;
            if (counter[UAV_COUNTER] == -1)
            {
                counter[UAV_COUNTER] += 1;
                //Initial set of coordinates being published to the position topic of the UAV
                pose[UAV_COUNTER].pose.position.x = list_maximum_value_y_indices[counter[UAV_COUNTER]];
                pose[UAV_COUNTER].pose.position.y = list_maximum_value_x_indices[counter[UAV_COUNTER]];
                pose[UAV_COUNTER].pose.position.z = 2;
            }
            else
            {
                //Difference between the current position and the next waypoint (x, y)
                dist = sqrt(pow((pose[UAV_COUNTER].pose.position.x - current_position_x[UAV_COUNTER]), 2) + pow((pose[UAV_COUNTER].pose.position.y - current_position_y[UAV_COUNTER]), 2));

                //From hereon out, we switch the X and Y coordinates to match that of the map

                //Printing the current position of the UAV
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current X Position: " << current_position_y[UAV_COUNTER] << endl;
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current Y Position: " << current_position_x[UAV_COUNTER] << endl;

                //Printing the waypoint that the UAV has to reach
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current X Waypoint: " << pose[UAV_COUNTER].pose.position.y << endl;
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current Y Waypoint: " << pose[UAV_COUNTER].pose.position.x << endl;

                if (dist < 0.5)
                {
                    cout << "Distance < 0.5" << endl;
                    if (counter[UAV_COUNTER] < (y_max * x_max))
                    {
                        cout << "Counter" << counter[UAV_COUNTER] << endl;
                        cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                             << "Maximum_Value_X_Indices: " << counter[UAV_COUNTER] << " " << list_maximum_value_x_indices[counter[UAV_COUNTER]] << endl;
                        cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                             << "Maximum_Value_Y_Indices: " << counter[UAV_COUNTER] << " " << list_maximum_value_y_indices[counter[UAV_COUNTER]] << endl;
                        pose[UAV_COUNTER].pose.position.x = list_maximum_value_y_indices[counter[UAV_COUNTER]];
                        pose[UAV_COUNTER].pose.position.y = list_maximum_value_x_indices[counter[UAV_COUNTER]];
                        pose[UAV_COUNTER].pose.position.z = 2;
                        counter[UAV_COUNTER] += 1;
                    }
                    else
                    {
                        cout << "UAV COUNTER: " << UAV_COUNTER << " "
                             << "RTL" << endl;
                        pose[UAV_COUNTER].pose.position.x = 1;
                        pose[UAV_COUNTER].pose.position.y = 1;
                        pose[UAV_COUNTER].pose.position.z = 1;
                    }
                }
            }
            if (current_state[UAV_COUNTER].mode != "OFFBOARD" &&
                (ros::Time::now() - last_request[UAV_COUNTER] > ros::Duration(5.0)))
            {
                if (set_mode_client[UAV_COUNTER].call(offb_set_mode[UAV_COUNTER]) &&
                    offb_set_mode[UAV_COUNTER].response.mode_sent)
                {
                    cout << "UAV COUNTER: " << UAV_COUNTER << " "
                         << "Offboard enabled" << endl;
                }
                last_request[UAV_COUNTER] = ros::Time::now();
            }
            else
            {
                if (!current_state[UAV_COUNTER].armed &&
                    (ros::Time::now() - last_request[UAV_COUNTER] > ros::Duration(5.0)))
                {
                    if (arming_client[UAV_COUNTER].call(arm_cmd[UAV_COUNTER]) &&
                        arm_cmd[UAV_COUNTER].response.success)
                    {
                        cout << "UAV COUNTER: " << UAV_COUNTER << " "
                             << "Vehicle armed" << endl;
                    }
                    last_request[UAV_COUNTER] = ros::Time::now();
                }
            }

            local_pos_pub[UAV_COUNTER].publish(pose[UAV_COUNTER]);

            ros::spinOnce();
            rate.sleep();
        }
    }
    return 0;
}