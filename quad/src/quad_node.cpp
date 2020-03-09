#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Int32.h>
#include <math.h>
#include "weight.hpp"
#include "lawnmower.hpp"
#include "survivor.hpp"
#include "variables.hpp"

void state_cb(const mavros_msgs::State::ConstPtr &state_msg)
{
    current_state[*global_pointer] = *state_msg;
}

void cv_sub(const std_msgs::Int32 cv_msg)
{
    //This callback function subscribes to the computer-vision code node and checks if a survivor has been found in the field-of-view of the UAVs camera
    cv_msgs[*global_pointer] = cv_msg;
}

void switch_sub(const std_msgs::Int32 switch_msg)
{
    //This callback function interacts with the observer node. If a non-zero value is received the topic triggers a switch to the weight-based trajectory planning
    switch_msgs[*global_pointer] = switch_msg;
}

void pose_sub(const nav_msgs::Odometry odom_msg)
{
    //Subscribing to the current position of the UAV
    current_position_x[*global_pointer] = odom_msg.pose.pose.position.x;
    current_position_y[*global_pointer] = odom_msg.pose.pose.position.y;
    current_position_z[*global_pointer] = odom_msg.pose.pose.position.z;
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
        global_pointer = &pre_pub_sub_initializer;
        stringstream pub_sub_initializer;
        pub_sub_initializer << pre_pub_sub_initializer;

        //Subscribes declared here
        string cv_node_subsciber_string;
        cv_node_subsciber_string = "/cv_node";
        cv_node[pre_pub_sub_initializer] = nh.subscribe<std_msgs::Int32>(cv_node_subsciber_string, 10, cv_sub);

        string switch_node_subsciber_string;
        switch_node_subsciber_string = "/switch_node";
        switch_node[pre_pub_sub_initializer] = nh.subscribe<std_msgs::Int32>(switch_node_subsciber_string, 10, switch_sub);

        string position_subscriber_string;
        position_subscriber_string = "/mavros/global_position/local";
        position_subscriber[pre_pub_sub_initializer] = nh.subscribe<nav_msgs::Odometry>(position_subscriber_string, 10, pose_sub);

        string state_sub_string;
        state_sub_string = "/mavros/state";
        state_sub[pre_pub_sub_initializer] = nh.subscribe<mavros_msgs::State>(state_sub_string, 10, state_cb);

        //Publishers declared here
        string local_pos_pub_string;
        local_pos_pub_string = "/mavros/setpoint_position/local";
        local_pos_pub[pre_pub_sub_initializer] = nh.advertise<geometry_msgs::PoseStamped>(local_pos_pub_string, 10);

        //The observer publishes to this topic when it notices a survivor in their vicinity
        string survivor_position_pub_string;
        survivor_position_pub_string = "/survivor_position";
        survivor_position_pub[pre_pub_sub_initializer] = nh.advertise<geometry_msgs::PoseStamped>(survivor_position_pub_string, 10);

        string counter_pub_string;
        counter_pub_string = "/waypoint_counter_element";
        counter_pub[pre_pub_sub_initializer] = nh.advertise<std_msgs::Int32>(counter_pub_string, 10);

        //Service clients to trigger modes
        string arming_client_string;
        arming_client_string = "/mavros/cmd/arming";
        arming_client[pre_pub_sub_initializer] = nh.serviceClient<mavros_msgs::CommandBool>(arming_client_string);

        string set_mode_client_string;
        set_mode_client_string = "/mavros/set_mode";
        set_mode_client[pre_pub_sub_initializer] = nh.serviceClient<mavros_msgs::SetMode>(set_mode_client_string);

        string take_off_string;
        take_off_string = "/mavros/cmd/takeoff";
        takeoff_client[pre_pub_sub_initializer] = nh.serviceClient<mavros_msgs::CommandTOL>(take_off_string);
    }

    ros::Rate rate(10.0);

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        cout << "PRESENTING POSE OF UAV: " << UAV_COUNTER << endl;
        cout << "X: " << pose[UAV_COUNTER].pose.position.x << endl;
        cout << "Y: " << pose[UAV_COUNTER].pose.position.y << endl;
        cout << "Z: " << pose[UAV_COUNTER].pose.position.z << endl;
    }

    int connected_state_counter;

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        if (!current_state[UAV_COUNTER].connected)
        {
            cout << "CURRENT_STATE" << endl;
            connected_state_counter += 1;
        }
        ros::spinOnce();
        rate.sleep();
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
        global_pointer = &UAV_COUNTER;
        for (int i = 100; ros::ok() && i > 0; --i)
        {
            cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                 << "INITAL PUBLISHING: " << i << endl;
            local_pos_pub[UAV_COUNTER].publish(pose[UAV_COUNTER]);
            ros::spinOnce();
            rate.sleep();
        }
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        cout << "OFFBOARD TRIGGER 1" << endl;
        offb_set_mode[UAV_COUNTER].request.custom_mode = "OFFBOARD";
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        cout << "ARMING 1" << endl;
        arm_cmd[UAV_COUNTER].request.value = true;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        last_request[UAV_COUNTER] = ros::Time::now();
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        lawn_mower_trigger_check[UAV_COUNTER] = 0;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        weight_trigger_check[UAV_COUNTER] = 0;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        survivor_detection_check[UAV_COUNTER] = 0;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        cv_msgs[UAV_COUNTER].data = 0;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        exploration_dump_check[UAV_COUNTER] = 0;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        global_pointer = &UAV_COUNTER;
        counter_msgs[UAV_COUNTER].data = 0;
    }

    for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
    {
        /*Here, we initialize the N-Dimensioned array to the default start point of the survivor.
        We do this to ensure that different survivor positions are ported to the respective publishers of the UAVs.*/
        global_pointer = &UAV_COUNTER;
        survivor_x_coordinate_array[UAV_COUNTER] = start_survivor_x_coordinate;
        survivor_y_coordinate_array[UAV_COUNTER] = start_survivor_y_coordinate;
    }

    while (ros::ok())
    {
        for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
        {
            //In this loop we check if a survivor has been detected by an observer. If yes, the weight-based exploration is triggered.
            global_pointer = &UAV_COUNTER;

            /*Here, the survivor coordinate's point to the respective element of the survivor coordinate array.
            This variable is refreshed each time we run the loop, and the value overwritten by the respective element of the coordinate array.*/
            *survivor_x_coordinate = survivor_x_coordinate_array[UAV_COUNTER];
            *survivor_y_coordinate = survivor_y_coordinate_array[UAV_COUNTER];
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
                //Calculating the distance between the survivor's coordinates and the UAV's current position, once the weight based search has been triggered.
                survivor_dist[UAV_COUNTER] = sqrt(pow((*survivor_x_coordinate - current_position_x[UAV_COUNTER]), 2) + pow((*survivor_y_coordinate - current_position_y[UAV_COUNTER]), 2));

                cout << "UAV COUNTER: " << UAV_COUNTER << " "
                     << "Survivor X: " << setprecision(4) << *survivor_x_coordinate << ", "
                     << "Survivor Y: " << setprecision(4) << *survivor_y_coordinate << ", "
                     << "Survivor Distance: " << survivor_dist[UAV_COUNTER] << endl;

                if (survivor_dist[UAV_COUNTER] < survivor_dist_threshold)
                {
                    //If survivor position is within the calculated threshold
                    cout << "UAV COUNTER: " << UAV_COUNTER << " "
                         << "Distance < " << survivor_dist_threshold << endl;
                    if (counter_msgs[UAV_COUNTER].data < (grid_points))
                    {
                        //Setting exploration parameter to 2 to indicate a survivor has been found
                        environment_map[list_maximum_value_y_indices[counter_msgs[UAV_COUNTER].data]][list_maximum_value_x_indices[counter_msgs[UAV_COUNTER].data]].exploration = 2;
                        //If survivor within the calulcated threshold and counter hasn't been set to (grid_points), send a detected message to the CL and set counter to (grid_points)
                        cout << "UAV COUNTER: " << UAV_COUNTER << " "
                             << "Human Detected" << endl;
                        counter_msgs[UAV_COUNTER].data = (grid_points);
                        //This check makes sure that the survivor's model does not update the survivor's location after it has been detected.
                        survivor_detection_check[UAV_COUNTER] = 1;
                    }
                }
            }

            //Refreshing the survivor coordinate arrays with the newly updated survivor coordinate pointers
            survivor_x_coordinate_array[UAV_COUNTER] = *survivor_x_coordinate;
            survivor_y_coordinate_array[UAV_COUNTER] = *survivor_y_coordinate;

            /* This else statement is opened to ensure that the survivor's coordinates are published regardless of whether or not the 
            observer node has been triggered or not*/
            survivor_pose[UAV_COUNTER].pose.position.x = survivor_x_coordinate_array[UAV_COUNTER];
            survivor_pose[UAV_COUNTER].pose.position.y = survivor_y_coordinate_array[UAV_COUNTER];
            survivor_pose[UAV_COUNTER].pose.position.z = 0;
            survivor_position_pub[UAV_COUNTER].publish(survivor_pose[UAV_COUNTER]);
            ros::spinOnce();
            rate.sleep();

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
                    counter_msgs[UAV_COUNTER].data = 0;
                    //This counter is to make sure that the lawn-mower coordinates are triggered only once in the entire program
                    weight_trigger_check[UAV_COUNTER] = 1;
                    weight_generator_function(uav_x_position, uav_y_position, X_1, X_2, X_3, X_4, X_5, n_x_difference, n_y_difference, n_set, survivor_direction, x_corner_coordinate_1, x_corner_coordinate_2, x_corner_coordinate_3, x_corner_coordinate_4, y_corner_coordinate_1, y_corner_coordinate_2, y_corner_coordinate_3, y_corner_coordinate_4, maximum_value, weight_element_cycler, list_maximum_value_x_indices, list_maximum_value_y_indices);
                }
            }
            //Difference between the current position and the next waypoint (x, y)
            waypoint_dist[UAV_COUNTER] = sqrt(pow((pose[UAV_COUNTER].pose.position.x - current_position_x[UAV_COUNTER]), 2) + pow((pose[UAV_COUNTER].pose.position.y - current_position_y[UAV_COUNTER]), 2) + pow((pose[UAV_COUNTER].pose.position.z - current_position_z[UAV_COUNTER]), 2));

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

            if (waypoint_dist[UAV_COUNTER] < waypoint_dist_threshold)
            {
                cout << "Distance < " << waypoint_dist_threshold << endl;
                if (counter_msgs[UAV_COUNTER].data < (y_max * x_max))
                {
                    if (cv_msgs[UAV_COUNTER].data == 1)
                    {
                        //Status 2 indicates that a survivor was found at that point.
                        environment_map[list_maximum_value_y_indices[counter_msgs[UAV_COUNTER].data]][list_maximum_value_x_indices[counter_msgs[UAV_COUNTER].data]].exploration = 2;
                        cout << "Human Detected by: " << UAV_COUNTER << " UAV" << endl;
                        cout << "RTL" << endl;
                        pose[UAV_COUNTER].pose.position.x = 1;
                        pose[UAV_COUNTER].pose.position.y = 1;
                        pose[UAV_COUNTER].pose.position.z = 1;
                        counter_msgs[UAV_COUNTER].data = (y_max * x_max);
                    }
                    else
                    {
                        //Status 1 for exploration states that the UAV traversed the given point and did not find any survivor at that spot.
                        environment_map[list_maximum_value_y_indices[counter_msgs[UAV_COUNTER].data]][list_maximum_value_x_indices[counter_msgs[UAV_COUNTER].data]].exploration = 1;
                        cout << "Counter: " << counter_msgs[UAV_COUNTER].data << endl;
                        cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                             << "Maximum_Value_X_Indices: " << counter_msgs[UAV_COUNTER].data << " " << list_maximum_value_x_indices[counter_msgs[UAV_COUNTER].data] << endl;
                        cout << "UAV_COUNTER: " << UAV_COUNTER << " "
                             << "Maximum_Value_Y_Indices: " << counter_msgs[UAV_COUNTER].data << " " << list_maximum_value_y_indices[counter_msgs[UAV_COUNTER].data] << endl;
                        pose[UAV_COUNTER].pose.position.x = list_maximum_value_y_indices[counter_msgs[UAV_COUNTER].data];
                        pose[UAV_COUNTER].pose.position.y = list_maximum_value_x_indices[counter_msgs[UAV_COUNTER].data];
                        pose[UAV_COUNTER].pose.position.z = 2;
                        counter_msgs[UAV_COUNTER].data += 1;
                    }
                }

                else if (counter_msgs[UAV_COUNTER].data == (grid_points) && survivor_detection_check[UAV_COUNTER] == 1)
                {
                    //This condition implies that the UAV has found the survivor, which has resulted in the counter being
                    //assigned to grid_points and survivor_detection_check equating to 1.
                    cout << "UAV COUNTER: " << UAV_COUNTER << " "
                         << "Survivor Status: " << survivor_detection_check[UAV_COUNTER]
                         << " "
                         << "RTL" << endl;
                    pose[UAV_COUNTER].pose.position.x = 0;
                    pose[UAV_COUNTER].pose.position.y = 0;
                    pose[UAV_COUNTER].pose.position.z = 1;
                    if ((exploration_dump_check[UAV_COUNTER]) == 0)
                    {
                        //This condition is to make sure that the exploration map gets dumped only once to the disc
                        exploration_dumper(environment_map, x_max, y_max, UAV_COUNTER);
                        exploration_dump_check[UAV_COUNTER] = 1;
                    }
                }
                else if (counter_msgs[UAV_COUNTER].data == (grid_points) && survivor_detection_check[UAV_COUNTER] == 0)
                {
                    //This condition implies that the UAV has not found the survivor and now needs to head back to the origin.
                    cout << "UAV COUNTER: " << UAV_COUNTER << " "
                         << "Survivor Status: " << survivor_detection_check[UAV_COUNTER]
                         << " "
                         << "RTL" << endl;
                    pose[UAV_COUNTER].pose.position.x = 0;
                    pose[UAV_COUNTER].pose.position.y = 0;
                    pose[UAV_COUNTER].pose.position.z = 1;
                    if ((exploration_dump_check[UAV_COUNTER]) == 0)
                    {
                        //This condition is to make sure that the exploration map gets dumped only once to the disc
                        exploration_dumper(environment_map, x_max, y_max, UAV_COUNTER);
                        exploration_dump_check[UAV_COUNTER] = 1;
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

            counter_pub[UAV_COUNTER].publish(counter_msgs[UAV_COUNTER]);
            ros::spinOnce();
            rate.sleep();
        }
    }
    return 0;
}