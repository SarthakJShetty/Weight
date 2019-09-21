#include "weight.hpp"

//This is the survivor's x, y coordinate & direction
int survivor_x_coordinate = 2;
int survivor_y_coordinate = 2;
//Survivor's direction 1-> ++, 2-> -+, 3-> --, 4-> +-
int survivor_direction = 4;

//UAVs current position
int uav_y_position = 2;
int uav_x_position = 2;

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

    //These vectors hold the X and the Y
vector<int> maximum_value_x_indices;
vector<int> maximum_value_y_indices;

int map_priority[y_max][x_max];

int weight_generator(weighted_map map[y_max][x_max], int x_max, int y_max)
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

int exploration_generator(weighted_map map[y_max][x_max], int x_max, int y_max)
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
				cout << "*\t";
			}
			else
			{
				cout << map[j][i].weight << "\t";
			}
		}
		cout << endl;
	}
	return 0;
}

int weight_generator_function(int maximum_value, vector<int> maximum_value_x_indices, vector<int> maximum_value_y_indices, int map_priority[y_max][x_max], int element_cycler)
{
	weighted_map map[y_max][x_max];
	weight_generator(map, x_max, y_max);
	exploration_generator(map, x_max, y_max);

	map[uav_y_position][uav_x_position].weight = 1000;

    //Visualizing the for loops as vectors helps. Y direction as vertical movement, X as horizontal movement
	while ((x_corner_coordinate_1 != uav_x_position))
	{
		for (int j = uav_y_position; j >= y_corner_coordinate_1; j--)
		{
			for (int i = (uav_x_position + 1); i < x_corner_coordinate_1; i++)
			{

				if (survivor_direction == 1)
				{
					map[j][i].weight += 100;
				}
				else if (survivor_direction == 2)
				{
					map[j][i].weight += 2;
				}
				else if (survivor_direction == 3)
				{
					map[j][i].weight += 1;
				}
				else if (survivor_direction == 4)
				{
					map[j][i].weight += 2;
				}
				locator(map, x_max, y_max, uav_x_position, uav_y_position);
				cout << endl;
			}
		}
		x_corner_coordinate_1 -= 1;
		y_corner_coordinate_1 += 1;
	}

	while ((x_corner_coordinate_2 != uav_x_position))
	{
		for (int j = (uav_y_position - 1); j >= y_corner_coordinate_2; j--)
		{
			for (int i = (uav_x_position); i >= x_corner_coordinate_2; i--)
			{
				if (survivor_direction == 1)
				{
					map[j][i].weight += 2;
				}
				else if (survivor_direction == 2)
				{
					map[j][i].weight += 100;
				}
				else if (survivor_direction == 3)
				{
					map[j][i].weight += 2;
				}
				else if (survivor_direction == 4)
				{
					map[j][i].weight += 1;
				}
				locator(map, x_max, y_max, uav_x_position, uav_y_position);
				cout << endl;
			}
		}
		x_corner_coordinate_2 += 1;
		y_corner_coordinate_2 += 1;
	}

	while ((x_corner_coordinate_3 != uav_x_position))
	{
		for (int j = uav_y_position; j < y_corner_coordinate_3; j++)
		{
			for (int i = (uav_x_position - 1); i >= x_corner_coordinate_3; i--)
			{
				if (survivor_direction == 1)
				{
					map[j][i].weight += 1;
				}
				else if (survivor_direction == 2)
				{
					map[j][i].weight += 2;
				}
				else if (survivor_direction == 3)
				{
					map[j][i].weight += 100;
				}
				else if (survivor_direction == 4)
				{
					map[j][i].weight += 2;
				}
				locator(map, x_max, y_max, uav_x_position, uav_y_position);
				cout << endl;
			}
		}
		x_corner_coordinate_3 += 1;
		y_corner_coordinate_3 -= 1;
	}

	while ((x_corner_coordinate_4 != uav_x_position))
	{
		for (int j = (uav_y_position + 1); j < y_corner_coordinate_4; j++)
		{
			for (int i = (uav_x_position); i < x_corner_coordinate_4; i++)
			{
				if (survivor_direction == 1)
				{
					map[j][i].weight += 2;
				}
				else if (survivor_direction == 2)
				{
					map[j][i].weight += 1;
				}
				else if (survivor_direction == 3)
				{
					map[j][i].weight += 2;
				}
				else if (survivor_direction == 4)
				{
					map[j][i].weight += 100;
				}
				locator(map, x_max, y_max, uav_x_position, uav_y_position);
				cout << endl;
			}
		}
		x_corner_coordinate_4 -= 1;
		y_corner_coordinate_4 -= 1;
	}

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
						maximum_value_x_indices.push_back(column_iterator_2);
						maximum_value_y_indices.push_back(row_iterator_2);
					}
				}
			}
		}
		maximum_value = 0;
		locator(map, x_max, y_max, uav_x_position, uav_y_position);
		cout << endl;
	}

	locator(map, x_max, y_max, uav_x_position, uav_y_position);

	for (int i = 0; i < (maximum_value_x_indices.size()); i++)
	{
		cout << "Maximum Value X Index:" << maximum_value_x_indices[i] << endl;
		cout << "Maximum Value Y Index:" << maximum_value_y_indices[i] << endl;
	}

	for (int i = 0; i < y_max; i++)
	{
		for (int j = 0; j < x_max; j++)
		{
			cout << map_priority[i][j] << "\t";
		}
		cout << endl;
	}
	cout << maximum_value_x_indices.size() << endl;
	return 0;
}