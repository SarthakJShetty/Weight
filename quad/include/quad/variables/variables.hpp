#include <vector>
#include <stdlib.h>
#include <iostream>

using namespace std;

//Boundaries of the flood environment
extern const int x_max;
extern const int y_max;

//This is the survivor's x, y coordinate & direction
int survivor_x_coordinate = 2;
int survivor_y_coordinate = 2;

//Survivor's direction 1-> ++, 2-> -+, 3-> --, 4-> +-
int survivor_direction = 4;

//UAVs current position
int uav_y_position = 10;
int uav_x_position = 10;

//Corner coordinates of each of the quadrants of exploration
int x_corner_coordinate_1 = x_max;
int y_corner_coordinate_1 = 0;

int x_corner_coordinate_2 = 0;
int y_corner_coordinate_2 = 0;

int x_corner_coordinate_3 = 0;
int y_corner_coordinate_3 = y_max;

int x_corner_coordinate_4 = x_max;
int y_corner_coordinate_4 = y_max;

int maximum_value = 0;

int element_cycler = 0;

int map_priority[y_max][x_max];

//Creating lists to iterate through waypoints. Hypothesis is that sharing of vectors is causing seg faults.
int list_maximum_value_x_indices[y_max*x_max];
int list_maximum_value_y_indices[y_max*x_max];