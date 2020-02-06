#include "weight.hpp"

int weight_initializer(weighted_map environment_map[y_max][x_max], int x_max, int y_max)
{
	//Initializing a environment_map with 0 weights
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			environment_map[j][i].weight = 0;
		}
	}
	return 0;
}

int exploration_initializer(weighted_map environment_map[y_max][x_max], int x_max, int y_max)
{
	//Initializing the exploration component of the environment_map structure
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			environment_map[j][i].exploration = 0;
		}
	}
	return 0;
}

int locator(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int uav_x_position, int uav_y_position)
{
	//Locating the UAV in the environment_map generated
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
				std::cout << environment_map[j][i].weight << "\t";
			}
		}
		std::cout << endl;
	}
	return 0;
}

int weight_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max)
{
	//This function takes the weightage environment_map, runs two for loops and dumps the corresponding weight environment_map to the disc, as a csv file.
	//We then plot these weights as a density environment_map, to check the weightage of the waypoints even if we cannot plot them using the grapher.py function.

	//The CSV of the environment_map generated is located at within the plotter package so that all entitities to be plotted are located in the same directory.
	ofstream weightMapCSV("/home/sarthak/catkin_ws/src/quad/include/quad/plotter/weightMap.csv");
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			weightMapCSV << environment_map[j][i].weight << "\t";
		}
		weightMapCSV << "\n";
	}
	weightMapCSV.close();
}

int priority_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max)
{
	//This function takes the weightage environment_map, runs two for loops and dumps the corresponding weight environment_map to the disc, as a csv file.
	//We then plot the priority of the weights as a density environment_map, to check the priority of the waypoints even if we cannot plot them using the grapher.py function from the locations of the UAV
	// during the simulations.

	//The CSV of the environment_map generated is located at within the plotter package so that all entitities to be plotted are located in the same directory.
	ofstream priorityMapCSV("/home/sarthak/catkin_ws/src/quad/include/quad/plotter/priorityMap.csv");
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			priorityMapCSV << environment_map[j][i].priority << "\t";
		}
		priorityMapCSV << "\n";
	}
	priorityMapCSV.close();
}

int exploration_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int UAV_COUNTER)
{
	//This function takes the weightage environment_map, runs two for loops and dumps the corresponding exploration status to the disc, as a csv file.
	//We then plot the exploration status of each cell in the environment.

	//The CSV of the environment_map generated is located at within the plotter package so that all entitities to be plotted are located in the same directory.
	string initial_filename = "/home/sarthak/catkin_ws/src/quad/include/quad/plotter/explorationMap_";
	stringstream str_UAV_COUNTER;
	str_UAV_COUNTER << UAV_COUNTER;
	string extension = ".csv";
	string exploration_filename = initial_filename + str_UAV_COUNTER.str() + extension;

	ofstream explorationMapCSV(exploration_filename.c_str());
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			explorationMapCSV << environment_map[j][i].exploration << "\t";
		}
		explorationMapCSV << "\n";
	}
	explorationMapCSV.close();
}

int weighting_function(int uav_x_position, int uav_y_position, float &X_1, float &X_2, float &X_3, float &X_4, float &X_5, int n_x_difference, int n_y_difference, int n_set, int x_max, int y_max)
{
	/*We've creating a new function to decide the weights to be assigned to each of the quadrants while generating weights. 
	This function solves 2 problems:
	1. No need to guess and chalk out the weights through trial and error.
	2. Agnostic to the dimensions of the prototyping environment.*/

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

	X_1 = ((X_4 * pow(n_set, 3)) + pow(n_set, 1) + pow(n_set, 2) + pow(n_set, 3));
	X_3 = ((X_1 - pow(n_set, 1)) - pow(n_set, 2)) / pow(n_set, 2);
	X_2 = (X_1 - pow(n_set, 1)) / pow(n_set, 1);
	X_5 = (X_1 * n_set * 10);
}

int weight_generator_function(int uav_x_position, int uav_y_position, float &X_1, float &X_2, float &X_3, float X_4, float &X_5, int n_x_difference, int n_y_difference, int n_set, int survivor_direction, int x_corner_coordinate_1, int x_corner_coordinate_2, int x_corner_coordinate_3, int x_corner_coordinate_4, int y_corner_coordinate_1, int y_corner_coordinate_2, int y_corner_coordinate_3, int y_corner_coordinate_4, int maximum_value, int weight_element_cycler, int list_maximum_value_x_indices[], int list_maximum_value_y_indices[])
{
	weighted_map environment_map[y_max][x_max];

	//Initializes the default weights in each of the coordinates, since unitialized maps tend to be filled with garbage values.
	weight_initializer(environment_map, x_max, y_max);
	//Initializes the default exploration status of each, changing the default gibberish.
	exploration_initializer(environment_map, x_max, y_max);

	//This function generates the weights that have to be assigned to each quadrant of the environment.
	weighting_function(uav_x_position, uav_y_position, X_1, X_2, X_3, X_4, X_5, n_x_difference, n_y_difference, n_set, x_max, y_max);

	environment_map[uav_y_position][uav_x_position].weight = X_5;

	//Visualizing the for loops as vectors helps. Y direction as vertical movement, X as horizontal movement
	while ((x_corner_coordinate_1 != uav_x_position))
	{
		for (int j = (uav_y_position - 1); j >= y_corner_coordinate_1; j--)
		{
			for (int i = (uav_x_position); i < x_corner_coordinate_1; i++)
			{
				if (survivor_direction == 1)
				{
					environment_map[j][i].weight += X_1;
				}
				else if (survivor_direction == 2)
				{
					environment_map[j][i].weight += X_2;
				}
				else if (survivor_direction == 3)
				{
					environment_map[j][i].weight += X_4;
				}
				else if (survivor_direction == 4)
				{
					environment_map[j][i].weight += X_3;
				}
				//locator(environment_map, x_max, y_max, uav_x_position, uav_y_position);
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
					environment_map[j][i].weight += X_3;
				}
				else if (survivor_direction == 2)
				{
					environment_map[j][i].weight += X_1;
				}
				else if (survivor_direction == 3)
				{
					environment_map[j][i].weight += X_2;
				}
				else if (survivor_direction == 4)
				{
					environment_map[j][i].weight += X_4;
				}
				//locator(environment_map, x_max, y_max, uav_x_position, uav_y_position);
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
					environment_map[j][i].weight += X_4;
				}
				else if (survivor_direction == 2)
				{
					environment_map[j][i].weight += X_3;
				}
				else if (survivor_direction == 3)
				{
					environment_map[j][i].weight += X_1;
				}
				else if (survivor_direction == 4)
				{
					environment_map[j][i].weight += X_2;
				}
				//locator(environment_map, x_max, y_max, uav_x_position, uav_y_position);
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
					environment_map[j][i].weight += X_3;
				}
				else if (survivor_direction == 2)
				{
					environment_map[j][i].weight += X_4;
				}
				else if (survivor_direction == 3)
				{
					environment_map[j][i].weight += X_2;
				}
				else if (survivor_direction == 4)
				{
					environment_map[j][i].weight += X_1;
				}
				//locator(environment_map, x_max, y_max, uav_x_position, uav_y_position);
				//std::cout << endl;
			}
		}
		x_corner_coordinate_4 -= 1;
		y_corner_coordinate_4 -= 1;
	}

	//Dumping the entire weightage environment_map to a .csv file to plot it
	//We are not including it at the end of the functions since we nullify all weights after the environment_map population, to ensure
	//that all weights have been prioritized and added to the list of exploration waypoints.
	weight_dumper(environment_map, x_max, y_max);

	/*Finding the maximum element in the environment_map
    1. Cycle through each row and find maximum
    2. If you find maximum store it as the maximum value
    3. If you encounter >= current maximum then append index (x, y) 
    4. Check the maximum value and remove it each row
    5. Iterate through environment_map again*/

	while (weight_element_cycler <= (y_max * x_max))
	{
		for (int row_iterator = 0; row_iterator < y_max; row_iterator++)
		{
			for (int column_iterator = 0; column_iterator < x_max; column_iterator++)
			{
				if (environment_map[row_iterator][column_iterator].weight > maximum_value)
				{
					maximum_value = environment_map[row_iterator][column_iterator].weight;
				}
			}
		}

		for (int row_iterator_2 = 0; row_iterator_2 < y_max; row_iterator_2++)
		{
			for (int column_iterator_2 = 0; column_iterator_2 < x_max; column_iterator_2++)
			{
				if ((environment_map[row_iterator_2][column_iterator_2].weight == maximum_value))
				{
					environment_map[row_iterator_2][column_iterator_2].weight = 0;
					weight_element_cycler += 1;

					if (maximum_value != 0)
					{
						environment_map[row_iterator_2][column_iterator_2].priority = weight_element_cycler;
						list_maximum_value_x_indices[weight_element_cycler - 1] = (column_iterator_2);
						list_maximum_value_y_indices[weight_element_cycler - 1] = (row_iterator_2);
					}
				}
			}
		}
		maximum_value = 0;
		//locator(environment_map, x_max, y_max, uav_x_position, uav_y_position);
		//std::cout << endl;
	}

	for (int i = 0; i < (y_max * x_max); i++)
	{
		std::cout << "Maximum Value X Index:" << list_maximum_value_x_indices[i] << endl;
		std::cout << "Maximum Value Y Index:" << list_maximum_value_y_indices[i] << endl;
	}

	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			std::cout << environment_map[j][i].priority << "\t";
		}
		std::cout << endl;
	}
	priority_dumper(environment_map, x_max, y_max);
	return 0;
}