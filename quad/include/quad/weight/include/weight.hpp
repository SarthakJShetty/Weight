/*In this model we will obtain waypoints for the UAV to fly in according to the weight based method

-Sarthak
14/08/2019*/
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <algorithm>
#include <vector>

using namespace std;

//Boundaries of the flood environment
const int x_max = 10;
const int y_max = 10;

//Creating a structure which contains the weights assigned to each coordinate and an exploration status
struct weighted_map
{
    ///Weight is the value added to each coordinate for the weight-based method
    int weight;
    //Exploration status for each cell 0-Unexplored; 1-Explored
    int exploration;
};

int weight_generator(weighted_map map[y_max][x_max], int x_max, int y_max);
int exploration_generator(weighted_map map[y_max][x_max], int x_max, int y_max);
int locator(weighted_map map[y_max][x_max], int x_max, int y_max, int uav_x_position, int uav_y_position);
int weight_generator_function(int uav_x_position, int uav_y_position, int survivor_direction, int x_corner_coordinate_1, int x_corner_coordinate_2, int x_corner_coordinate_3, int x_corner_coordinate_4, int y_corner_coordinate_1, int y_corner_coordinate_2, int y_corner_coordinate_3, int y_corner_coordinate_4, int maximum_value, int map_priority[y_max][x_max], int weight_element_cycler, int list_maximum_value_x_indices[], int list_maximum_value_y_indices[]);