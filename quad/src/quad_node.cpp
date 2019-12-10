#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include "weight.hpp"
#include "lawnmower.hpp"
#include "survivor.hpp"
#include "variables.hpp"

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state[global_pointer] = *msg;
}

void cv_sub(const std_msgs::Int8 msg)
{
    //This callback function subscribes to the computer-vision code node and checks if a survivor has been found in the field-of-view of the UAVs camera
    cv_msgs[global_pointer] = msg;
}

void switch_sub(const std_msgs::Int8 msg)
{
    //This callback function interacts with the observer node. If a non-zero value is received the topic triggers a switch to the weight-based trajectory planning
    switch_msgs[global_pointer] = msg;
}

void pose_sub(const nav_msgs::Odometry msg)
{
    //Subscribing to the current position of the UAV
    current_position_x[global_pointer] = msg.pose.pose.position.x;
    current_position_y[global_pointer] = msg.pose.pose.position.y;
    current_position_z[global_pointer] = msg.pose.pose.position.z;
}

int main(int argc, char **argv)
{
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

    for (int pre_pub_sub_initializer = 0; pre_pub_sub_initializer < N_UAV; pre_pub_sub_initializer++)
    {
        global_pointer = pre_pub_sub_initializer;
        stringstream pub_sub_initializer;
        pub_sub_initializer << pre_pub_sub_initializer;

        //Subscribes declared here
        string cv_node_subsciber_string;
        cv_node_subsciber_string = "/uav" + pub_sub_initializer.str() + "/cv_node";
        cout << "cv_node_subscriber_string: " << cv_node_subsciber_string << endl;
        cv_node[pre_pub_sub_initializer] = nh.subscribe<std_msgs::Int8>(cv_node_subsciber_string, 10, cv_sub);

        string switch_node_subsciber_string;
        switch_node_subsciber_string = "/uav" + pub_sub_initializer.str() + "/switch_node";
        cout << "switch_node_subscriber_string: " << switch_node_subsciber_string << endl;
        switch_node[pre_pub_sub_initializer] = nh.subscribe<std_msgs::Int8>(switch_node_subsciber_string, 10, switch_sub);

        string position_subscriber_string;
        position_subscriber_string = "/uav" + pub_sub_initializer.str() + "/mavros/global_position/local";
        cout << "position_subscriber_string: " << position_subscriber_string << endl;
        position_subscriber[pre_pub_sub_initializer] = nh.subscribe<nav_msgs::Odometry>(position_subscriber_string, 10, pose_sub);

        string state_sub_string;
        state_sub_string = "/uav" + pub_sub_initializer.str() + "/mavros/state";
        cout << "state_sub_string: " << state_sub_string << endl;
        state_sub[pre_pub_sub_initializer] = nh.subscribe<mavros_msgs::State>(state_sub_string, 10, state_cb);

        //Publishers declared here
        string local_pos_pub_string;
        local_pos_pub_string = "/uav" + pub_sub_initializer.str() + "/mavros/setpoint_position/local";
        cout << "local_pos_pub_string: " << local_pos_pub_string << endl;
        local_pos_pub[pre_pub_sub_initializer] = nh.advertise<geometry_msgs::PoseStamped>(local_pos_pub_string, 10);

        string survivor_position_pub_string;
        survivor_position_pub_string = "/uav" + pub_sub_initializer.str() + "/survivor_position";
        cout << "survivor_position_pub_string: " << survivor_position_pub_string << endl;
        survivor_position_pub[pre_pub_sub_initializer] = nh.advertise<geometry_msgs::PoseStamped>(survivor_position_pub_string, 10);

        //Service clients to trigger modes
        string arming_client_string;
        arming_client_string = "/uav" + pub_sub_initializer.str() + "/mavros/cmd/arming";
        cout << "arming_client_string: " << arming_client_string << endl;
        arming_client[pre_pub_sub_initializer] = nh.serviceClient<mavros_msgs::CommandBool>(arming_client_string);

        string set_mode_client_string;
        set_mode_client_string = "/uav" + pub_sub_initializer.str() + "/mavros/set_mode";
        cout << "set_mode_client_string: " << set_mode_client_string << endl;
        set_mode_client[pre_pub_sub_initializer] = nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_string);

        string take_off_string;
        take_off_string = "/uav" + pub_sub_initializer.str() + "/mavros/cmd/takeoff";
        takeoff_client[pre_pub_sub_initializer] = nh.serviceClient<mavros_msgs::CommandTOL>(take_off_string);

        cout << endl;
    }

    ros::Rate rate(20.0);

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        counter[UAV_COUNTER] = -1;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        cout << "PRESENTING POSE OF UAV: " << UAV_COUNTER << endl;
        cout << "X: " << pose[UAV_COUNTER].pose.position.x << endl;
        cout << "Y: " << pose[UAV_COUNTER].pose.position.y << endl;
        cout << "Z: " << pose[UAV_COUNTER].pose.position.z << endl;
    }

    int connected_state_counter;

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        if (!current_state[UAV_COUNTER].connected)
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

    //Send a few setpoints before starting
    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        for (int i = 100; ros::ok() && i > 0; --i)
        {
            cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                 << "INITAL PUBLISHING: " << i << endl;
            local_pos_pub[UAV_COUNTER].publish(pose[UAV_COUNTER]);
            ros::spinOnce();
            rate.sleep();
            cout << endl;
        }
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        cout << "OFFBOARD TRIGGER 1" << endl;
        offb_set_mode[UAV_COUNTER].request.custom_mode = "OFFBOARD";
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        cout << "ARMING 1" << endl;
        arm_cmd[UAV_COUNTER].request.value = true;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        last_request[UAV_COUNTER] = ros::Time::now();
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        lawn_mower_trigger_check[UAV_COUNTER] = 0;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        weight_trigger_check[UAV_COUNTER] = 0;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        survivor_detection_check[UAV_COUNTER] = 0;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = UAV_COUNTER;
        cv_msgs[UAV_COUNTER].data = 0;
    }

    while (ros::ok())
    {
        for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
        {
            global_pointer = UAV_COUNTER;
            if (weight_trigger_check[UAV_COUNTER] == 1)
            {
                /*What needs to be implemented here?
                1. A variable type that can be transacted across varibales, quad_node and survivor
                2. Implement a pointer here which points to the memory location of survivor_x_coordinate & survivor_y_coordinate.
                3. The memory location is then shared across the files as oppossed to passing solely by reference.*/

                if (survivor_detection_check[UAV_COUNTER] != 1)
                {
                    //This condition makes sure that the survivor's model updates the coordinates in the heading reported only if the survivor has not been encountered by the UAV.
                    survivor_model(x_max, y_max, survivor_direction, current_second, previous_second, survivor_x_coordinate, survivor_y_coordinate, velocity, time_step);
                }
                //Calculating the distance between the survivor's coordinates and the UAV's current position.
                survivor_dist = sqrt(pow((*survivor_x_coordinate - current_position_x[UAV_COUNTER]), 2) + pow((*survivor_y_coordinate - current_position_y[UAV_COUNTER]), 2));

                //Publishing the survivor's coordinates using the survivor_position_pub publisher for plotting and echoing.
                survivor_pose[UAV_COUNTER].pose.position.x = *survivor_x_coordinate;
                survivor_pose[UAV_COUNTER].pose.position.y = *survivor_y_coordinate;
                survivor_pose[UAV_COUNTER].pose.position.z = 0;
                survivor_position_pub[UAV_COUNTER].publish(survivor_pose[UAV_COUNTER]);

                cout << "UAV COUNTER: " << UAV_COUNTER << " "
                     << "Survivor X: " << setprecision(4) << *survivor_x_coordinate << ", "
                     << "Survivor Y: " << setprecision(4) << *survivor_y_coordinate << ", "
                     << "Survivor Distance: " << survivor_dist << endl;

                if (survivor_dist < survivor_dist_threshold)
                {
                    cout << "UAV COUNTER: " << UAV_COUNTER << " "
                         << "Distance < " << survivor_dist_threshold << endl;
                    if (counter[UAV_COUNTER] < (y_max * x_max))
                    {
                        cout << "UAV COUNTER: " << UAV_COUNTER << " "
                             << "Human Detected" << endl;
                        counter[UAV_COUNTER] = (y_max * x_max);
                        //This check makes sure that the survivor's model does not update the survivor's location after it has been detected.
                        survivor_detection_check[UAV_COUNTER] = 1;
                    }
                }
            }
        }
        for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
        {
            //In this loop we check if a survivor has been detected by an observer. If yes, the weight-based exploration is triggered.
            global_pointer = UAV_COUNTER;
            if (switch_msgs[UAV_COUNTER].data == 0)
            {
                if (lawn_mower_trigger_check[UAV_COUNTER] != 1)
                {
                    //This counter is to make sure that the lawn-mower coordinates are triggered only once in the entire program
                    lawn_mower_trigger_check[UAV_COUNTER] = 1;
                    lawn_mower_generator_function(y_max, x_max, uav_x_position, uav_y_position, list_maximum_value_x_indices, list_maximum_value_y_indices, pre_list_lawn_mower_x_indices, pre_list_lawn_mower_y_indices, lawn_mower_element_cycler, lawn_lawn_mower_element_cycler);
                }
            }
            else
            {
                if (weight_trigger_check[UAV_COUNTER] != 1)
                {
                    //This counter is refreshed so that the exploration can begin again.
                    counter[UAV_COUNTER] = 0;
                    //This counter is to make sure that the lawn-mower coordinates are triggered only once in the entire program
                    weight_trigger_check[UAV_COUNTER] = 1;
                    weight_generator_function(uav_x_position, uav_y_position, survivor_direction, x_corner_coordinate_1, x_corner_coordinate_2, x_corner_coordinate_3, x_corner_coordinate_4, y_corner_coordinate_1, y_corner_coordinate_2, y_corner_coordinate_3, y_corner_coordinate_4, maximum_value, map_priority, weight_element_cycler, list_maximum_value_x_indices, list_maximum_value_y_indices);
                }
            }
        }
        for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
        {
            global_pointer = UAV_COUNTER;
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
                waypoint_dist = sqrt(pow((pose[UAV_COUNTER].pose.position.x - current_position_x[UAV_COUNTER]), 2) + pow((pose[UAV_COUNTER].pose.position.y - current_position_y[UAV_COUNTER]), 2) + pow((pose[UAV_COUNTER].pose.position.z - current_position_z[UAV_COUNTER]), 2));

                //From hereon out, we switch the X and Y coordinates to match that of the map

                //Printing the current position of the UAV
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current X Position: " << current_position_y[UAV_COUNTER] << endl;
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current Y Position: " << current_position_x[UAV_COUNTER] << endl;
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current Z Position: " << current_position_z[UAV_COUNTER] << endl;

                //Printing the waypoint that the UAV has to reach
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current X Waypoint: " << pose[UAV_COUNTER].pose.position.y << endl;
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current Y Waypoint: " << pose[UAV_COUNTER].pose.position.x << endl;
                cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                     << "Current Z Waypoint: " << pose[UAV_COUNTER].pose.position.z << endl;

                if (waypoint_dist < waypoint_dist_threshold)
                {
                    cout << "Distance < " << waypoint_dist_threshold << endl;
                    if (counter[UAV_COUNTER] < (y_max * x_max))
                    {
                        if (cv_msgs[UAV_COUNTER].data == 1)
                        {
                            cout << "Human Detected by: " << UAV_COUNTER << " UAV" << endl;
                            cout << "RTL" << endl;
                            pose[UAV_COUNTER].pose.position.x = 1;
                            pose[UAV_COUNTER].pose.position.y = 1;
                            pose[UAV_COUNTER].pose.position.z = 2;
                            counter[UAV_COUNTER] = (y_max * x_max);
                        }
                        cout << "Counter: " << counter[UAV_COUNTER] << endl;
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
                        pose[UAV_COUNTER].pose.position.z = 2;
                    }
                }
            }
            if (current_state[UAV_COUNTER].mode != "OFFBOARD" && (ros::Time::now() - last_request[UAV_COUNTER] > ros::Duration(5.0)))
            {
                if (set_mode_client[UAV_COUNTER].call(offb_set_mode[UAV_COUNTER]) && offb_set_mode[UAV_COUNTER].response.mode_sent)
                {
                    cout << "UAV COUNTER: " << UAV_COUNTER << " Offboard enabled 2" << endl;
                }
                last_request[UAV_COUNTER] = ros::Time::now();
            }
            else
            {
                if (!current_state[UAV_COUNTER].armed && (ros::Time::now() - last_request[UAV_COUNTER] > ros::Duration(5.0)))
                {
                    if (arming_client[UAV_COUNTER].call(arm_cmd[UAV_COUNTER]) && arm_cmd[UAV_COUNTER].response.success)
                    {
                        cout << "UAV COUNTER: " << UAV_COUNTER << " Vehicle armed 2" << endl;
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