/*This script is part of the Weight project where we are trying to create a model for multi-UAV exploration.
This code aids in the creation of the watpoint list for carrying out lawn-mower search.

-Sarthak
(18/11/2019)*/

#include <iostream>
#include <stdlib.h>
#include "vector"

using namespace std;

//This is the only function accessed by quad_node. It generates a NxM map for exploration with priority based on the lawn-mower pattern
int lawn_mower_initializer_function(int y_max, int x_max, int pre_list_lawn_mower_x_indices[], int pre_list_lawn_mower_y_indices[], int lawn_mower_element_cycler);
int lawn_mower_generator_function(int y_max, int x_max, vector <int> &vector_list_maximum_value_x_indices, vector <int> &vector_list_maximum_value_y_indices, int list_lawn_mower_x_indices[], int list_lawn_mower_y_indices[], int pre_list_lawn_mower_x_indices[], int pre_list_lawn_mower_y_indices[], int lawn_mower_element_cycler, int lawn_lawn_mower_element_cycler, int start_uav_x_position[], int start_uav_y_position[], int N_UAV, int UAV_COUNTER);