/*These functions serve two main purposes:
1. Generate the list of points to be searched by lawn-mower.
2. Generate the prioritized list of points in lawn-mower pattern.*/

#include "lawnmower.hpp"
#include "split.hpp"

int lawn_mower_initializer_function(int y_max, int x_max, int pre_list_lawn_mower_x_indices[], int pre_list_lawn_mower_y_indices[], int lawn_mower_element_cycler)
{
    /*This function generates the set of the points, which will be manipulated to give the lawn-mower pattern in a new list(s) called the lawn_mower_indices*/

    //Element cycler generates a linear set of points for further manipulation
    while (lawn_mower_element_cycler < (x_max * y_max))
    {
        for (int i = 0; i < x_max; i++)
        {
            for (int j = 0; j < y_max; j++)
            {
                pre_list_lawn_mower_x_indices[lawn_mower_element_cycler] = i;
                pre_list_lawn_mower_y_indices[lawn_mower_element_cycler] = j;
                lawn_mower_element_cycler += 1;
            }
        }
    }
}

int lawn_mower_generator_function(int y_max, int x_max, vector<int> &vector_list_maximum_value_x_indices, vector<int> &vector_list_maximum_value_y_indices, int list_lawn_mower_x_indices[], int list_lawn_mower_y_indices[], int pre_list_lawn_mower_x_indices[], int pre_list_lawn_mower_y_indices[], int lawn_mower_element_cycler, int lawn_lawn_mower_element_cycler, int start_uav_x_position[], int start_uav_y_position[], int N_UAV, int UAV_COUNTER)
{
    lawn_mower_initializer_function(y_max, x_max, pre_list_lawn_mower_x_indices, pre_list_lawn_mower_y_indices, lawn_mower_element_cycler);
    /*This function manipulates the pre_list to give the distinct lawn-mower pattern*/

    //This variable keeps track of which row is currently being accessed by the function written below
    lawn_mower_element_cycler = 0;
    while (lawn_mower_element_cycler < (x_max * y_max))
    {
        if (lawn_mower_element_cycler != ((y_max * x_max) - 1))
        {
            /* cout << "Entering the loop: "
                  << "Element Cycler: " << lawn_mower_element_cycler << endl;*/
            if (pre_list_lawn_mower_y_indices[lawn_mower_element_cycler] == (y_max - 1))
            {
                //cout << "Encoutered the edge of a row" << endl;
                if ((pre_list_lawn_mower_x_indices[lawn_mower_element_cycler] % 2) == 0)
                {
                    //cout << "Edge of the row which is at a turning point as well" << endl;
                    list_lawn_mower_x_indices[lawn_mower_element_cycler] = pre_list_lawn_mower_x_indices[lawn_mower_element_cycler];
                    list_lawn_mower_y_indices[lawn_mower_element_cycler] = pre_list_lawn_mower_y_indices[lawn_mower_element_cycler];
                    /*cout << "Here 1: " << list_lawn_mower_x_indices[lawn_mower_element_cycler] << ", " << list_lawn_mower_y_indices[lawn_mower_element_cycler] << ", NEXT:" << pre_list_lawn_mower_x_indices[lawn_mower_element_cycler] << ", " << pre_list_lawn_mower_y_indices[lawn_mower_element_cycler] << endl;*/
                    lawn_mower_element_cycler += 1;
                    lawn_lawn_mower_element_cycler = lawn_mower_element_cycler;
                    for (int row_iterator = 0; row_iterator < y_max; row_iterator++)
                    {
                        //cout << "Looping through the mini-array that we are trying to reverse" << endl;
                        list_lawn_mower_x_indices[lawn_lawn_mower_element_cycler] = pre_list_lawn_mower_x_indices[lawn_mower_element_cycler + (y_max - 1) - row_iterator];
                        list_lawn_mower_y_indices[lawn_lawn_mower_element_cycler] = pre_list_lawn_mower_y_indices[lawn_mower_element_cycler + (y_max - 1) - row_iterator];
                        /*cout << "Here 2: " << list_lawn_mower_x_indices[lawn_lawn_mower_element_cycler] << ", " << list_lawn_mower_y_indices[lawn_lawn_mower_element_cycler] << endl;*/
                        lawn_lawn_mower_element_cycler += 1;
                    }
                    lawn_mower_element_cycler += y_max;
                }
                else
                {
                    list_lawn_mower_x_indices[lawn_mower_element_cycler] = pre_list_lawn_mower_x_indices[lawn_mower_element_cycler];
                    list_lawn_mower_y_indices[lawn_mower_element_cycler] = pre_list_lawn_mower_y_indices[lawn_mower_element_cycler];
                    /*cout << "Here 3: " << list_lawn_mower_x_indices[lawn_mower_element_cycler] << ", " << list_lawn_mower_y_indices[lawn_mower_element_cycler] << ", NEXT:" << pre_list_lawn_mower_x_indices[lawn_mower_element_cycler] << ", " << pre_list_lawn_mower_y_indices[lawn_mower_element_cycler] << endl;*/
                    lawn_mower_element_cycler += 1;
                }
            }
            else
            {
                list_lawn_mower_x_indices[lawn_mower_element_cycler] = pre_list_lawn_mower_x_indices[lawn_mower_element_cycler];
                list_lawn_mower_y_indices[lawn_mower_element_cycler] = pre_list_lawn_mower_y_indices[lawn_mower_element_cycler];
                /*cout << "Here 4: " << list_lawn_mower_x_indices[lawn_mower_element_cycler] << ", " << list_lawn_mower_y_indices[lawn_mower_element_cycler] << ", NEXT:" << pre_list_lawn_mower_x_indices[lawn_mower_element_cycler] << ", " << pre_list_lawn_mower_y_indices[lawn_mower_element_cycler] << endl; */
                lawn_mower_element_cycler += 1;
            }
        }
        else
        {
            list_lawn_mower_x_indices[lawn_mower_element_cycler] = pre_list_lawn_mower_x_indices[lawn_mower_element_cycler];
            list_lawn_mower_y_indices[lawn_mower_element_cycler] = pre_list_lawn_mower_y_indices[lawn_mower_element_cycler];
            lawn_mower_element_cycler += 1;
        }
    }

    lawn_mower_element_cycler = 0;
    std::cout << "NON-PRIORITIZED COORDINATES" << endl;

    while (lawn_mower_element_cycler < (x_max * y_max))
    {
        std::cout << pre_list_lawn_mower_x_indices[lawn_mower_element_cycler] << ", " << pre_list_lawn_mower_y_indices[lawn_mower_element_cycler];
        std::cout << endl;
        lawn_mower_element_cycler += 1;
    }

    lawn_mower_element_cycler = 0;

    /*We resize the vector here before populating it because, if the UAV has decided to revert to the lawnmower appraoch after running the weighted model, then there will be pre-existing coordinates
    in both the lists. Therefore, we have to resize the vector before each of those model calls from the quad_node.*/
	vector_list_maximum_value_x_indices.resize(0);
	vector_list_maximum_value_y_indices.resize(0);

    std::cout << "LAWN MOWER COORDINATES" << endl;
    while (lawn_mower_element_cycler < (x_max * y_max))
    {
        if (UAV_COUNTER == split_environment(list_lawn_mower_x_indices[lawn_mower_element_cycler], list_lawn_mower_y_indices[lawn_mower_element_cycler], start_uav_x_position, start_uav_y_position, N_UAV))
        {
            vector_list_maximum_value_x_indices.push_back(list_lawn_mower_x_indices[lawn_mower_element_cycler]);
            vector_list_maximum_value_y_indices.push_back(list_lawn_mower_y_indices[lawn_mower_element_cycler]);
        }
        std::cout << list_lawn_mower_x_indices[lawn_mower_element_cycler] << ", " << list_lawn_mower_y_indices[lawn_mower_element_cycler];
        std::cout << endl;
        lawn_mower_element_cycler += 1;
    }
    lawn_mower_element_cycler = 0;
    std::cout << "VECTOR LAWN MOWER COORDINATES" << endl;
    while (lawn_mower_element_cycler < vector_list_maximum_value_x_indices.size())
    {
        std::cout << vector_list_maximum_value_x_indices[lawn_mower_element_cycler] << ", " << vector_list_maximum_value_y_indices[lawn_mower_element_cycler];
        std::cout << endl;
        lawn_mower_element_cycler += 1;
    }
}