#include <vector>
#include <stdlib.h>
#include <iostream>

using namespace std;

//global_pointer points to the corresponding publisher/subscriber combination at the top of the ROS code while subscribing to the individual agent's parameters
int *global_pointer;

//Threshld to switch to next waypoint
float waypoint_dist_threshold = 0.5;

//This variable is a threshold to check whether or not the UAV has reached the survivor's position or not
//This value has been obtained by calculating the FOV assuming a 45° observer angle and a 2m height
//In the future, we'll design a model that dynamically adapts this value to the height of the UAV
float survivor_dist_threshold = 4.0;

//Declaring the number of UAVs being used here
const int N_UAV = 2;

//Boundaries of the flood environment
extern const int x_max;
extern const int y_max;
extern const int grid_points;
weighted_map environment_map[y_max][x_max];

//Subscribers and publishers declared here
ros::Subscriber position_subscriber[N_UAV];
ros::Subscriber state_sub[N_UAV];
ros::Publisher local_pos_pub[N_UAV];
ros::Publisher survivor_position_pub[N_UAV];
ros::Publisher counter_pub[N_UAV];

//Service clients to handle ROS services
ros::ServiceClient arming_client[N_UAV];
ros::ServiceClient set_mode_client[N_UAV];
ros::ServiceClient takeoff_client[N_UAV];

//Declaring the CV node here, which subscribes to the computer vision node subscribes to
ros::Subscriber cv_node[N_UAV];

//Declaring the switch node here which switches the agent from lawnmower exploration to weight-based exploration
ros::Subscriber switch_node[N_UAV];

//Generating an integer to store the binary value sent over by the CV node
std_msgs::Int32 cv_msgs[N_UAV];
//This variable keeps track of which search pattern is being triggered
std_msgs::Int32 switch_msgs[N_UAV];
//This counter is used to publish the counter value for diagnostics primarily
std_msgs::Int32 counter_msgs[N_UAV];

//Declaring the pose here for both UAVs
geometry_msgs::PoseStamped pose[N_UAV];

//Declaring the pose here for both UAVs
geometry_msgs::PoseStamped survivor_pose[N_UAV];

//State of the UAVs connected/non-connected
mavros_msgs::State current_state[N_UAV];

//Topic to set UAV to off-board mode
mavros_msgs::SetMode offb_set_mode[N_UAV];

//ARMING the UAV with this topic
mavros_msgs::CommandBool arm_cmd[N_UAV];

//Service client to trigger takeoff
mavros_msgs::CommandTOL srv_takeoff[N_UAV];

//Topic to check last request time
ros::Time last_request[N_UAV];

//current_position is used to check whether or not the drone can take the next set of waypoints or not
float current_position_x[N_UAV];
float current_position_y[N_UAV];
float current_position_z[N_UAV];

//Distance keeps track of initial position and desired positon
float waypoint_dist[N_UAV];

//This variable keeps track of distance between UAV's position and the survivor's position. Then compared to survivor_dist_threshold
float survivor_dist[N_UAV];

//This is the survivor's x, y coordinate & direction. We use pointers to ensure consistancy across the files
float start_survivor_x_coordinate = 10.0;
float start_survivor_y_coordinate = 10.0;

//Survivor coordinate is continuously updated from the start_survivor_coordinate
float *survivor_x_coordinate = &start_survivor_x_coordinate;
float *survivor_y_coordinate = &start_survivor_y_coordinate;

//Introducing survivor coordinates as arrays so that coordinates are truly seperate for UAV i and UAV j ∀ i, j ∈ {0, N}
float survivor_x_coordinate_array[N_UAV];
float survivor_y_coordinate_array[N_UAV];

//Velocity of the survivor
float velocity = 0.3;
//Time step to calculate the next set of survivor coordinates from the previous velocity
float time_step = 1.0;

//Incorporating time variables to keep track of how often the coordinates of the survivor are sent to quad_node
int current_second = 0;
//The start_previous_second is used to initialize the previous second pointer
int start_previous_second = 0;
int *previous_second = &start_previous_second;

//Survivor's direction 1-> ++, 2-> -+, 3-> --, 4-> +-
int survivor_direction = 4;

//UAVs current position
int uav_y_position = start_survivor_x_coordinate;
int uav_x_position = start_survivor_y_coordinate;

//Corner coordinates of each of the quadrants of exploration
int x_corner_coordinate_1 = x_max;
int y_corner_coordinate_1 = 0;

int x_corner_coordinate_2 = 0;
int y_corner_coordinate_2 = 0;

int x_corner_coordinate_3 = 0;
int y_corner_coordinate_3 = y_max;

int x_corner_coordinate_4 = x_max;
int y_corner_coordinate_4 = y_max;

//Initializing the weight pointers that will be used to assign weights to the different quadrants by the weight.xpp function
float X_1 = 1.0;
float X_2 = 1.0;
float X_3 = 1.0;
float X_4 = 1.0;
float X_5 = 1.0;

//These variables compute the number of iterations required along the X and Y axes to hit the boundaries of the environment
int n_x_difference;
int n_y_difference;
int n_set;

//Holds the maximum value that the code has come across in the map
int maximum_value = 0;

//Cycles through the elements of the weight map generated
int weight_element_cycler = 0;
//This element mowes through the pre-ordered list and interacts with the lawn_mower array
int lawn_mower_element_cycler = 0;
int lawn_lawn_mower_element_cycler = 0;

//Creating lists to iterate through waypoints. Hypothesis is that sharing of vectors is causing seg faults.
int list_maximum_value_x_indices[N_UAV][y_max * x_max];
int list_maximum_value_y_indices[N_UAV][y_max * x_max];

//Creating a list of waypointss for conveying the lawnmower waypoints
int pre_list_lawn_mower_x_indices[y_max * x_max];
int pre_list_lawn_mower_y_indices[y_max * x_max];

//This variable makes sure that each UAV generates the lawnmower code only once
int lawn_mower_trigger_check[N_UAV];

//This variable makes sure that each UAV generates the weighted map only once
int weight_trigger_check[N_UAV];

//This variable checks whether each of the UAVs have found the survivor or not. If triggered to one, implies that that particular UAV has found the survivor
int survivor_detection_check[N_UAV];

//This array makes sure whether or not the explorationDump has taken place or not for the respective UAV
int exploration_dump_check[N_UAV];