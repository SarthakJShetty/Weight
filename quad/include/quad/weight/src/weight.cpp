#include "weight.hpp"

int weight_initializer(weighted_map map[y_max][x_max], int x_max, int y_max)
{
	//Initializing a map with 0 weights
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			map[j][i].weight = 0;
		}
	}
	return 0;
}

int exploration_initializer(weighted_map map[y_max][x_max], int x_max, int y_max)
{
	//Initializing the exploration component of the map structure
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			map[j][i].exploration = 0;
		}
	}
	return 0;
}

int locator(weighted_map map[y_max][x_max], int x_max, int y_max, int uav_x_position, int uav_y_position)
{
	//Locating the UAV in the map generated
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			if (i == (uav_x_position) && j == (uav_y_position))
			{
				std::cout << "*\t";
			}
			else
			{
				std::cout << map[j][i].weight << "\t";
			}
		}
		std::cout << endl;
	}
	return 0;
}

int weight_dumper(weighted_map map[y_max][x_max], int x_max, int y_max)
{
	//This function takes the weightage map, runs two for loops and dumps the corresponding weight map to the disc, as a csv file.
	//We then plot these weights as a density map, to check the weightage of the waypoints even if we cannot plot them using the grapher.py function.

	//The CSV of the map generated is located at within the plotter package so that all entitities to be plotted are located in the same directory.
	ofstream weightMapCSV("/home/sarthak/catkin_ws/src/quad/include/quad/plotter/weightMap.csv");
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			weightMapCSV << map[j][i].weight << "\t";
		}
		weightMapCSV << "\n";
	}
	weightMapCSV.close();
}

int priority_dumper(int map_priority[y_max][x_max], int x_max, int y_max)
{
	//This function takes the weightage map, runs two for loops and dumps the corresponding weight map to the disc, as a csv file.
	//We then plot the priority of the weights as a density map, to check the priority of the waypoints even if we cannot plot them using the grapher.py function from the locations of the UAV
	// during the simulations.

	//The CSV of the map generated is located at within the plotter package so that all entitities to be plotted are located in the same directory.
	ofstream priorityMapCSV("/home/sarthak/catkin_ws/src/quad/include/quad/plotter/priorityMap.csv");
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			priorityMapCSV << map_priority[j][i] << "\t";
		}
		priorityMapCSV << "\n";
	}
	priorityMapCSV.close();
}

int weighting_function(int uav_x_position, int uav_y_position, float *X_1, float *X_2, float *X_3, float X_4, float *X_5, int n_x_difference, int n_y_difference, int n_set, int x_max, int y_max)
{
	/*Experimental!
	We're creating a new function to decide the weights to be assigned to each of the quadrants while generating weights. 
	This function solves 2 problems:
	1. No need to guess and chalk out the weights through trial and error.
	2. Agnostic to the dimensions of the prototyping environment.*/

	/*What needs to be implemented here?
	1. A calulcation of N(x, y), which is the maximum number of iterations that each weight will be subjected to.
	2. Utilization of N to get X_1, X_2 and X_3, which are the weights in each quadrant.
	3. Assumption that X_4 is always 1.
	4. X_5 is the weight assigned to map[uav_y_position][uav_x_position].weight.*/

	//uav_x/y_position is being subtracted from x/y_max since the boundary is the largest possible value of n_x/y_difference.
	n_x_difference = x_max - uav_x_position;
	n_y_difference = y_max - uav_y_position;

	if (n_x_difference > n_y_difference)
	{
		n_set = n_x_difference;
	}
	else
	{
		n_set = n_y_difference;
	}

	X_4 = 1;
	
	float calc_X_1 = ((X_4 * pow(n_set, 3)) + pow(n_set, 1) + pow(n_set, 2) + pow(n_set, 3));
	float calc_X_3 = ((calc_X_1 - pow(n_set, 1)) - pow(n_set, 2)) / pow(n_set, 2);
	float calc_X_2 = (calc_X_1 - pow(n_set, 1)) / pow(n_set, 1);
	float calc_X_5 = (calc_X_1 * n_set * 10);

	X_3 = &calc_X_3;
	X_2 = &calc_X_2;
	X_1 = &calc_X_1;
	X_5 = &calc_X_5;

	for (int i = 0; i < 10000; i++)
	{
		cout << *X_1 << endl;
		cout << *X_2 << endl;
		cout << *X_3 << endl;
		cout << X_4 << endl;
		cout << *X_5 << endl;
	}
}

int weight_generator_function(int uav_x_position, int uav_y_position, float *X_1, float *X_2, float *X_3, float X_4, float *X_5, int n_x_difference, int n_y_difference, int n_set, int survivor_direction, int x_corner_coordinate_1, int x_corner_coordinate_2, int x_corner_coordinate_3, int x_corner_coordinate_4, int y_corner_coordinate_1, int y_corner_coordinate_2, int y_corner_coordinate_3, int y_corner_coordinate_4, int maximum_value, int map_priority[y_max][x_max], int element_cycler, int list_maximum_value_x_indices[], int list_maximum_value_y_indices[])
{
	/*NOTE: These weights are manually set after brief hand-calculations of the maximum possible weights in either quadrant. Depending on your environment
	you have to set these values accordingly.
	Next steps would be define the constraints as a Linear Programming Problem and assign weights for each quadrant.*/

	weighted_map map[y_max][x_max];
	weight_initializer(map, x_max, y_max);
	exploration_initializer(map, x_max, y_max);

	/*NOTE: To the user who will be modifying this: As your environment gets bigger you need to manually increase this value to a larger value, other wise with larger iterations
	the maximum value will be undermined by greater number of iterations.*/

	weighting_function(uav_x_position, uav_y_position, X_1, X_2, X_3, X_4, X_5, n_x_difference, n_y_difference, n_set, x_max, y_max);

	for (int i = 0; i < 1000000; i++)
	{
		cout <<"YEEET" << *X_2 << endl;
		cout <<"YEEET" << *X_1 << endl;
		cout <<"YEEET" << *X_3 << endl;
		cout <<"YEEET" << X_4 << endl;
		cout <<"YEEET" << *X_5 << endl;
	}

	map[uav_y_position][uav_x_position].weight = *X_5;

	//Visualizing the for loops as vectors helps. Y direction as vertical movement, X as horizontal movement
	while ((x_corner_coordinate_1 != uav_x_position))
	{
		for (int j = (uav_y_position - 1); j >= y_corner_coordinate_1; j--)
		{
			for (int i = (uav_x_position); i < x_corner_coordinate_1; i++)
			{
				if (survivor_direction == 1)
				{
					map[j][i].weight += *X_1;
				}
				else if (survivor_direction == 2)
				{
					map[j][i].weight += *X_2;
				}
				else if (survivor_direction == 3)
				{
					map[j][i].weight += X_4;
				}
				else if (survivor_direction == 4)
				{
					map[j][i].weight += *X_3;
				}
				//locator(map, x_max, y_max, uav_x_position, uav_y_position);
				//std::cout << endl;
			}
		}
		x_corner_coordinate_1 -= 1;
		y_corner_coordinate_1 += 1;
	}

	while ((x_corner_coordinate_2 != uav_x_position))
	{
		for (int j = (uav_y_position); j >= y_corner_coordinate_2; j--)
		{
			for (int i = (uav_x_position - 1); i >= x_corner_coordinate_2; i--)
			{
				if (survivor_direction == 1)
				{
					map[j][i].weight += *X_3;
				}
				else if (survivor_direction == 2)
				{
					map[j][i].weight += *X_1;
				}
				else if (survivor_direction == 3)
				{
					map[j][i].weight += *X_2;
				}
				else if (survivor_direction == 4)
				{
					map[j][i].weight += X_4;
				}
				//locator(map, x_max, y_max, uav_x_position, uav_y_position);
				//std::cout << endl;
			}
		}
		x_corner_coordinate_2 += 1;
		y_corner_coordinate_2 += 1;
	}

	while ((x_corner_coordinate_3 != uav_x_position))
	{
		for (int j = (uav_y_position + 1); j < y_corner_coordinate_3; j++)
		{
			for (int i = (uav_x_position); i >= x_corner_coordinate_3; i--)
			{
				if (survivor_direction == 1)
				{
					map[j][i].weight += X_4;
				}
				else if (survivor_direction == 2)
				{
					map[j][i].weight += *X_3;
				}
				else if (survivor_direction == 3)
				{
					map[j][i].weight += *X_1;
				}
				else if (survivor_direction == 4)
				{
					map[j][i].weight += *X_2;
				}
				//locator(map, x_max, y_max, uav_x_position, uav_y_position);
				//std::cout << endl;
			}
		}
		x_corner_coordinate_3 += 1;
		y_corner_coordinate_3 -= 1;
	}

	while ((x_corner_coordinate_4 != uav_x_position))
	{
		for (int j = (uav_y_position); j < y_corner_coordinate_4; j++)
		{
			for (int i = (uav_x_position + 1); i < x_corner_coordinate_4; i++)
			{
				if (survivor_direction == 1)
				{
					map[j][i].weight += *X_3;
				}
				else if (survivor_direction == 2)
				{
					map[j][i].weight += X_4;
				}
				else if (survivor_direction == 3)
				{
					map[j][i].weight += *X_2;
				}
				else if (survivor_direction == 4)
				{
					map[j][i].weight += *X_1;
				}
				//locator(map, x_max, y_max, uav_x_position, uav_y_position);
				//std::cout << endl;
			}
		}
		x_corner_coordinate_4 -= 1;
		y_corner_coordinate_4 -= 1;
	}

	//Dumping the entire weightage map to a .csv file to plot it
	//We are not including it at the end of the functions since we nullify all weights after the map population, to ensure
	//that all weights have been prioritized and added to the list of exploration waypoints.
	weight_dumper(map, x_max, y_max);

	/*Finding the maximum element in the map
    1. Cycle through each row and find maximum
    2. If you find maximum store it as the maximum value
    3. If you encounter >= current maximum then append index (x, y) 
    4. Check the maximum value and remove it each row
    5. Iterate through map again*/

	while (element_cycler <= (y_max * x_max))
	{
		for (int row_iterator = 0; row_iterator < y_max; row_iterator++)
		{
			for (int column_iterator = 0; column_iterator < x_max; column_iterator++)
			{
				if (map[row_iterator][column_iterator].weight > maximum_value)
				{
					maximum_value = map[row_iterator][column_iterator].weight;
				}
			}
		}

		for (int row_iterator_2 = 0; row_iterator_2 < y_max; row_iterator_2++)
		{
			for (int column_iterator_2 = 0; column_iterator_2 < x_max; column_iterator_2++)
			{
				if ((map[row_iterator_2][column_iterator_2].weight == maximum_value))
				{
					map[row_iterator_2][column_iterator_2].weight = 0;
					element_cycler += 1;

					if (maximum_value != 0)
					{
						map_priority[row_iterator_2][column_iterator_2] = element_cycler;
						list_maximum_value_x_indices[element_cycler - 1] = (column_iterator_2);
						list_maximum_value_y_indices[element_cycler - 1] = (row_iterator_2);
					}
				}
			}
		}
		maximum_value = 0;
		//locator(map, x_max, y_max, uav_x_position, uav_y_position);
		//std::cout << endl;
	}

	for (int i = 0; i < (y_max * x_max); i++)
	{
		std::cout << "Maximum Value X Index:" << list_maximum_value_x_indices[i] << endl;
		std::cout << "Maximum Value Y Index:" << list_maximum_value_y_indices[i] << endl;
	}

	for (int i = 0; i < y_max; i++)
	{
		for (int j = 0; j < x_max; j++)
		{
			std::cout << map_priority[i][j] << "\t";
		}
		std::cout << endl;
	}
	priority_dumper(map_priority, x_max, y_max);
	return 0;
}