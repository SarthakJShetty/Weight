/*In this model we will obtain waypoints for the UAV to fly in according to the weight based method

-Sarthak
14/08/2019*/
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <fstream>
#include <string>
#include <sstream>
#include "vector"

using namespace std;

//Boundaries of the flood environment
const int x_max = 19;
const int y_max = 19;
const int grid_points = y_max * x_max;

//Creating a structure which contains the weights assigned to each coordinate and an exploration status
struct weighted_map
{
    ///Weight is the value added to each coordinate for the weight-based method
    int weight;
    //Including the priority of exploration of the specific waypoint as well
    int priority;
    //Exploration status for each cell 0-Unexplored; 1-Explored; 2-Survivor detected
    int exploration;
    //Split contains the UAV to which the given cell belongs to
    int split;
};

int weighting_function(int weight_uav_x_position, int weight_uav_y_position, float &X_1, float &X_2, float &X_3, float &X_4, float &X_5, int n_x_difference, int n_y_difference, int n_set, int x_max, int y_max);
int weight_initializer(weighted_map environment_map[y_max][x_max], int x_max, int y_max);
int exploration_initializer(weighted_map environment_map[y_max][x_max], int x_max, int y_max);
int split_initializer(weighted_map environment_map[y_max][x_max], int x_max, int y_max);
int locator(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int weight_uav_x_position, int weight_uav_y_position);
int weight_generator_function(int weight_uav_x_position, int weight_uav_y_position, float &X_1, float &X_2, float &X_3, float X_4, float &X_5, int n_x_difference, int n_y_difference, int n_set, int survivor_direction, int x_corner_coordinate_1, int x_corner_coordinate_2, int x_corner_coordinate_3, int x_corner_coordinate_4, int y_corner_coordinate_1, int y_corner_coordinate_2, int y_corner_coordinate_3, int y_corner_coordinate_4, int maximum_value, int weight_element_cycler, vector<int> &vector_list_maximum_value_x_indices, vector<int> &vector_list_maximum_value_y_indices, int list_maximum_value_x_indices[], int list_maximum_value_y_indices[], int start_uav_x_position[], int start_uav_y_position[], int N_UAV, int UAV_COUNTER);
int split_environment(weighted_map environment_map[y_max][x_max], int start_uav_x_position[], int start_uav_y_position[], int y_max, int x_max, int N_UAV, int UAV_COUNTER);
int weight_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int UAV_COUNTER);
int priority_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int UAV_COUNTER);
int exploration_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int UAV_COUNTER);
int split_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int UAV_COUNTER);