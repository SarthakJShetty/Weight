#include "weight.hpp"
#include "split.hpp"

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

int split_initializer(weighted_map environment_map[y_max][x_max], int x_max, int y_max)
{
	//Initializing the split component of the environment_map structure
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			environment_map[j][i].split = -1;
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

int locator(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int weight_uav_x_position, int weight_uav_y_position)
{
	//Locating the UAV in the environment_map generated
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			if (i == (weight_uav_x_position) && j == (weight_uav_y_position))
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

int weight_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int UAV_COUNTER)
{
	//This function takes the weightage environment_map, runs two for loops and dumps the corresponding weight environment_map to the disc, as a csv file.
	//We then plot these weights as a density environment_map, to check the weightage of the waypoints even if we cannot plot them using the grapher.py function.

	//The CSV of the environment_map generated is located at within the plotter package so that all entitities to be plotted are located in the same directory.

	string initial_filename = "/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/weightMap_";
	stringstream str_UAV_COUNTER;
	str_UAV_COUNTER << UAV_COUNTER;
	string extension = ".csv";
	string weights_filename = initial_filename + str_UAV_COUNTER.str() + extension;

	ofstream weightMapCSV(weights_filename.c_str());

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

int priority_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int UAV_COUNTER)
{
	//This function takes the weightage environment_map, runs two for loops and dumps the corresponding priotity of the coordinates to the disc, as a csv file.
	//We then plot the priority of the weights as a priority environment_map, to check the priority of the waypoints even if we cannot plot them using the grapher.py function from the locations of the UAV
	// during the simulations.

	//The CSV of the environment_map generated is located at within the plotter package so that all entitities to be plotted are located in the same directory.

	string initial_filename = "/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/priorityMap_";
	stringstream str_UAV_COUNTER;
	str_UAV_COUNTER << UAV_COUNTER;
	string extension = ".csv";
	string priority_filename = initial_filename + str_UAV_COUNTER.str() + extension;

	ofstream priorityMapCSV(priority_filename.c_str());

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

int split_environment(weighted_map environment_map[y_max][x_max], int start_uav_x_position[], int start_uav_y_position[], int y_max, int x_max, int N_UAV, int UAV_COUNTER)
{
	//This function splits the environment amongst the different UAV, by assigning the closest UAV to each coordinate.
	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			//We calculate the distance to UAV from each of the coordinates
			int distance_to_uav[N_UAV];
			//We keep track of which UAV yields the smallest distance to the given coordinate using this variable
			int least_distance_uav_index = 0;
			for (int UAV_COUNTER = 0; UAV_COUNTER < N_UAV; UAV_COUNTER++)
			{
				distance_to_uav[UAV_COUNTER] = sqrt(pow((j - start_uav_y_position[UAV_COUNTER]), 2) + pow((i - start_uav_x_position[UAV_COUNTER]), 2));
				if (distance_to_uav[UAV_COUNTER] < distance_to_uav[least_distance_uav_index])
				{
					//If the given distance is smaller than previously recorded smallest distance, then replace the least_distance_uav_index with that counter
					least_distance_uav_index = UAV_COUNTER;
				}
			}
			//Assigning the closest UAV to that coordinate
			environment_map[j][i].split = least_distance_uav_index;
		}
	}
	//Dumping the closest UAV information to the disc, to be plotted
	split_dumper(environment_map, x_max, y_max, UAV_COUNTER);
	return 0;
}

int split_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int UAV_COUNTER)
{
	//This function takes the weightage environment_map, runs two for loops and dumps the split coordinates of the weight environment_map to the disc, as a csv file.
	//We then plot the UAV assigned to each coordinate as a heatmap.

	//The CSV of the environment_map generated is located at within the plotter package so that all entitities to be plotted are located in the same directory.

	string initial_filename = "/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/splitMap_";
	stringstream str_UAV_COUNTER;
	str_UAV_COUNTER << UAV_COUNTER;
	string extension = ".csv";
	string split_filename = initial_filename + str_UAV_COUNTER.str() + extension;

	ofstream splitMapCSV(split_filename.c_str());

	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			splitMapCSV << environment_map[j][i].split << "\t";
		}
		splitMapCSV << "\n";
	}
	splitMapCSV.close();
}

int exploration_dumper(weighted_map environment_map[y_max][x_max], int x_max, int y_max, int UAV_COUNTER)
{
	//This function takes the weightage environment_map, runs two for loops and dumps the corresponding exploration status to the disc, as a csv file.
	//We then plot the exploration status of each cell in the environment.

	//The CSV of the environment_map generated is located at within the plotter package so that all entitities to be plotted are located in the same directory.
	string initial_filename = "/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/explorationMap_";
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

int weighting_function(int weight_uav_x_position, int weight_uav_y_position, float &X_1, float &X_2, float &X_3, float &X_4, float &X_5, int n_x_difference, int n_y_difference, int n_set, int x_max, int y_max)
{
	/*We've creating a new function to decide the weights to be assigned to each of the quadrants while generating weights. 
	This function solves 2 problems:
	1. No need to guess and chalk out the weights through trial and error.
	2. Agnostic to the dimensions of the prototyping environment.*/

	//uav_x/y_position is being subtracted from x/y_max since the boundary is the largest possible value of n_x/y_difference.

	n_x_difference = x_max - weight_uav_x_position;
	n_y_difference = y_max - weight_uav_y_position;

	if (n_x_difference > n_y_difference)
	{
		n_set = n_x_difference;
	}
	else
	{
		n_set = n_y_difference;
	}

	X_1 = ((X_4 * pow(n_set, 3)) + pow(n_set, 2) + pow(n_set, 1)) + 1;
	X_3 = ((X_1 - pow(n_set, 1)) - pow(n_set, 1)) / pow(n_set, 2);
	X_2 = (X_1 - pow(n_set, 0)) / pow(n_set, 1);
	X_5 = ((X_1 * n_set) + 1);
}

int weight_generator_function(int weight_uav_x_position, int weight_uav_y_position, float &X_1, float &X_2, float &X_3, float X_4, float &X_5, int n_x_difference, int n_y_difference, int n_set, int survivor_direction, int x_corner_coordinate_1, int x_corner_coordinate_2, int x_corner_coordinate_3, int x_corner_coordinate_4, int y_corner_coordinate_1, int y_corner_coordinate_2, int y_corner_coordinate_3, int y_corner_coordinate_4, int maximum_value, int weight_element_cycler, vector<int> &vector_list_maximum_value_x_indices, vector<int> &vector_list_maximum_value_y_indices, int list_maximum_value_x_indices[], int list_maximum_value_y_indices[], int start_uav_x_position[], int start_uav_y_position[], int N_UAV, int UAV_COUNTER)
{
	weighted_map environment_map[y_max][x_max];

	//Initializes the default weights in each of the coordinates, since unitialized maps tend to be filled with garbage values.
	weight_initializer(environment_map, x_max, y_max);
	//Initializes the default exploration status of each, changing the default gibberish.
	exploration_initializer(environment_map, x_max, y_max);
	//Initializes the default split status of each, to prevent garbage being initialized.
	split_initializer(environment_map, x_max, y_max);

	//This function generates the weights that have to be assigned to each quadrant of the environment.
	weighting_function(weight_uav_x_position, weight_uav_y_position, X_1, X_2, X_3, X_4, X_5, n_x_difference, n_y_difference, n_set, x_max, y_max);

	//Assigning the highest weight to the start coordinate from where the UAV begins exploration.
	environment_map[weight_uav_y_position][weight_uav_x_position].weight = X_5;

	/*Visualizing the for loops as vectors helps. Y direction as vertical movement, X as horizontal movement.
	We address each quadrant here. The top left corner is (0, 0) and the right hand axis is X, and the bottom, Y. We assign weights to the outer quadrants first,
	and then move inside.*/
	while ((x_corner_coordinate_1 != weight_uav_x_position))
	{
		for (int j = (weight_uav_y_position - 1); j >= y_corner_coordinate_1; j--)
		{
			for (int i = (weight_uav_x_position); i < x_corner_coordinate_1; i++)
			{
				if (survivor_direction == 1)
				{
					environment_map[j][i].weight += X_1;
				}
				else if (survivor_direction == 2)
				{
					environment_map[j][i].weight += X_3;
				}
				else if (survivor_direction == 3)
				{
					environment_map[j][i].weight += X_4;
				}
				else if (survivor_direction == 4)
				{
					environment_map[j][i].weight += X_2;
				}
				//locator(environment_map, x_max, y_max, weight_uav_x_position, weight_uav_y_position);
				//std::cout << endl;
			}
		}
		x_corner_coordinate_1 -= 1;
		y_corner_coordinate_1 += 1;
	}

	while ((x_corner_coordinate_2 != weight_uav_x_position))
	{
		for (int j = (weight_uav_y_position); j >= y_corner_coordinate_2; j--)
		{
			for (int i = (weight_uav_x_position - 1); i >= x_corner_coordinate_2; i--)
			{
				if (survivor_direction == 1)
				{
					environment_map[j][i].weight += X_2;
				}
				else if (survivor_direction == 2)
				{
					environment_map[j][i].weight += X_1;
				}
				else if (survivor_direction == 3)
				{
					environment_map[j][i].weight += X_3;
				}
				else if (survivor_direction == 4)
				{
					environment_map[j][i].weight += X_4;
				}
				//locator(environment_map, x_max, y_max, weight_uav_x_position, weight_uav_y_position);
				//std::cout << endl;
			}
		}
		x_corner_coordinate_2 += 1;
		y_corner_coordinate_2 += 1;
	}

	while ((x_corner_coordinate_3 != weight_uav_x_position))
	{
		for (int j = (weight_uav_y_position + 1); j < y_corner_coordinate_3; j++)
		{
			for (int i = (weight_uav_x_position); i >= x_corner_coordinate_3; i--)
			{
				if (survivor_direction == 1)
				{
					environment_map[j][i].weight += X_4;
				}
				else if (survivor_direction == 2)
				{
					environment_map[j][i].weight += X_2;
				}
				else if (survivor_direction == 3)
				{
					environment_map[j][i].weight += X_1;
				}
				else if (survivor_direction == 4)
				{
					environment_map[j][i].weight += X_3;
				}
				//locator(environment_map, x_max, y_max, weight_uav_x_position, weight_uav_y_position);
				//std::cout << endl;
			}
		}
		x_corner_coordinate_3 += 1;
		y_corner_coordinate_3 -= 1;
	}

	while ((x_corner_coordinate_4 != weight_uav_x_position))
	{
		for (int j = (weight_uav_y_position); j < y_corner_coordinate_4; j++)
		{
			for (int i = (weight_uav_x_position + 1); i < x_corner_coordinate_4; i++)
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
				//locator(environment_map, x_max, y_max, weight_uav_x_position, weight_uav_y_position);
				//std::cout << endl;
			}
		}
		x_corner_coordinate_4 -= 1;
		y_corner_coordinate_4 -= 1;
	}

	//Dumping the entire weightage environment_map to a .csv file to plot it
	//We are not including it at the end of the functions since we nullify all weights after the environment_map population, to ensure
	//that all weights have been prioritized and added to the list of exploration waypoints.
	weight_dumper(environment_map, x_max, y_max, UAV_COUNTER);

	/*Finding the maximum element in the environment_map
    1. Cycle through each row and find maximum
    2. If you find maximum store it as the maximum value
    3. If you encounter >= current maximum then append index (x, y) 
    4. Check the maximum value and remove it each row
    5. Iterate through environment_map again*/

	/*We use row_element to tabulate the switching schemes of the weight-based pattern.
	We use y/x_element to keep track of where the coordinate occurs so that the limits for the quadrant can be selected appropriately.*/
	int row_element, y_element, x_element;

	while (weight_element_cycler <= (y_max * x_max))
	{
		for (int row_iterator = 0; row_iterator < y_max; row_iterator++)
		{
			for (int column_iterator = 0; column_iterator < x_max; column_iterator++)
			{
				if (row_iterator <= weight_uav_y_position)
				{
					//If the maximum value element occurs above weight_uav_y_position, then take the first occuring maximum_value to tabulate the row_element
					if (environment_map[row_iterator][column_iterator].weight > maximum_value)
					{
						maximum_value = environment_map[row_iterator][column_iterator].weight;
						row_element = abs(weight_uav_y_position - row_iterator);
						y_element = row_iterator;
						x_element = column_iterator;
					}
				}
				else
				{
					//If the maximum value element occurs below weight_uav_y_position, then take the last occuring maximum_value to tabulate the row_element
					if (environment_map[row_iterator][column_iterator].weight >= maximum_value)
					{
						maximum_value = environment_map[row_iterator][column_iterator].weight;
						row_element = abs(weight_uav_y_position - row_iterator);
						y_element = row_iterator;
						x_element = column_iterator;
					}
				}
			}
		}

		/*What has been implemented here?
		1. A variable (row_element) to keep track of the Y-coordinate () of the maximum_value.
		2. If this variable is even, then turn the for loops in the next section are as follows:
			2.1 X: 0 -> X_MAX
			2.2 Y: Y_MAX -> 0
		3. If the variable is odd, the turn the for loops in the next section as follows:
			3.1 X: X_MAX -> 0
			3.2 Y: 0 -> Y_MAX*/

		if ((y_element >= weight_uav_y_position) && (x_element >= (weight_uav_x_position + 1)))
		{
			//If the maximum_value element is in the FOURTH quadrant then:
			if ((row_element) % 2 == 0)
			//If the Y axis of the weightiest element is even, then:
			{
				for (int row_iterator_2 = (y_max - 1); row_iterator_2 >= 0; row_iterator_2--)
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
			}
			else
			//If the Y axis of the weightiest element is odd, then:
			{
				for (int row_iterator_2 = 0; row_iterator_2 <= (y_max - 1); row_iterator_2++)
				{
					for (int column_iterator_2 = (x_max - 1); column_iterator_2 >= 0; column_iterator_2--)
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
			}
		}
		else if ((y_element >= (weight_uav_y_position + 1)) && (x_element <= (weight_uav_x_position)))
		{
			//If the maximum_value element is in the THIRD quadrant then:
			if ((row_element) % 2 == 0)
			//If the Y axis of the weightiest element is even, then:
			{
				for (int row_iterator_2 = (y_max - 1); row_iterator_2 >= 0; row_iterator_2--)
				{
					for (int column_iterator_2 = (x_max - 1); column_iterator_2 >= 0; column_iterator_2--)
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
			}
			else
			//If the Y axis of the weightiest element is odd, then:
			{
				for (int row_iterator_2 = 0; row_iterator_2 <= (y_max - 1); row_iterator_2++)
				{
					for (int column_iterator_2 = 0; column_iterator_2 <= (x_max - 1); column_iterator_2++)
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
			}
		}
		else if ((y_element <= (weight_uav_y_position)) && (x_element <= (weight_uav_x_position - 1)))
		{
			//If the maximum_value element is in the SECOND quadrant then:
			if ((row_element) % 2 == 0)
			//If the Y axis of the weightiest element is even, then:
			{
				for (int row_iterator_2 = 0; row_iterator_2 <= (y_max - 1); row_iterator_2++)
				{
					for (int column_iterator_2 = (x_max - 1); column_iterator_2 >= 0; column_iterator_2--)
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
			}
			else
			//If the Y axis of the weightiest element is odd, then:
			{
				for (int row_iterator_2 = (y_max - 1); row_iterator_2 >= 0; row_iterator_2--)
				{
					for (int column_iterator_2 = 0; column_iterator_2 <= (x_max - 1); column_iterator_2++)
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
			}
		}
		else if ((y_element <= (weight_uav_y_position - 1)) && (x_element >= (weight_uav_x_position)))
		{
			//If the maximum_value element is in the FIRST quadrant then:
			if ((row_element) % 2 == 0)
			//If the Y axis of the weightiest element is even, then:
			{
				for (int row_iterator_2 = 0; row_iterator_2 <= (y_max - 1); row_iterator_2++)
				{
					for (int column_iterator_2 = 0; column_iterator_2 <= (x_max - 1); column_iterator_2++)
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
			}
			else
			//If the Y axis of the weightiest element is odd, then:
			{
				for (int row_iterator_2 = (y_max - 1); row_iterator_2 >= 0; row_iterator_2--)
				{
					for (int column_iterator_2 = (x_max - 1); column_iterator_2 >= 0; column_iterator_2--)
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
			}
		}
		else if ((x_element == weight_uav_x_position) && (y_element == weight_uav_y_position))
		{
			//If the maximum_value element is uav_x/y_position, it isn't indexed by any of the previous loops, hence:
			environment_map[weight_uav_y_position][weight_uav_x_position].weight = 0;
			weight_element_cycler += 1;
			environment_map[weight_uav_y_position][weight_uav_x_position].priority = weight_element_cycler;
			list_maximum_value_x_indices[weight_element_cycler - 1] = x_element;
			list_maximum_value_y_indices[weight_element_cycler - 1] = y_element;
		}

		maximum_value = 0;
		//locator(environment_map, x_max, y_max, weight_uav_x_position, weight_uav_y_position);
		//std::cout << endl;
	}

	vector_list_maximum_value_x_indices.resize(0);
	vector_list_maximum_value_y_indices.resize(0);

	for (int i = 0; i < (y_max * x_max); i++)
	{
		std::cout << "Maximum Value X Index:" << list_maximum_value_x_indices[i] << endl;
		std::cout << "Maximum Value Y Index:" << list_maximum_value_y_indices[i] << endl;
		if (UAV_COUNTER == split_environment(list_maximum_value_x_indices[i], list_maximum_value_y_indices[i], start_uav_x_position, start_uav_y_position, N_UAV))
		{
			vector_list_maximum_value_x_indices.push_back(list_maximum_value_x_indices[i]);
			vector_list_maximum_value_y_indices.push_back(list_maximum_value_y_indices[i]);
		}
	}

	for (int j = 0; j < y_max; j++)
	{
		for (int i = 0; i < x_max; i++)
		{
			std::cout << environment_map[j][i].priority << "\t";
		}
		std::cout << endl;
	}
	priority_dumper(environment_map, x_max, y_max, UAV_COUNTER);
	return 0;
}