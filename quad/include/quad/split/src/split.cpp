#include "split.hpp"

int split_environment(int i, int j, int start_uav_x_position[], int start_uav_y_position[], int N_UAV)
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
    //Dumping the closest UAV information to the disc, to be plotted
    return least_distance_uav_index;
}