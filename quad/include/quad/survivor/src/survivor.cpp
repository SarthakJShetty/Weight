/*This node creates a virtual survivor that continously updates the survivor_x_coordinate & survivor_y_coordinate variables.
At each coordinate in the X/Y frame of the drone, the current (X, Y) of the drone is checked for overlap with the 
survivors coordinates

-Sarthak
(19/11/2019)
*/
#include "survivor.hpp"

int survivor_model(int x_max, int y_max, int survivor_direction, int current_second, int *previous_second, float *survivor_x_coordinate, float *survivor_y_coordinate, float velocity, float time_step)
{
    //These are the time variables. I do not understand them. Do not touch them
    time_t t = time(0);
    tm *now = localtime(&t);
    current_second = now->tm_sec;
    if (current_second != *previous_second)
    {
        if (survivor_direction == 1)
        {
            if ((*survivor_y_coordinate >= y_max) || (*survivor_x_coordinate <= 0))
            {
                *survivor_x_coordinate = *survivor_x_coordinate;
                *survivor_y_coordinate = *survivor_y_coordinate;
            }
            else
            {
                *survivor_x_coordinate -= (velocity * time_step);
                *survivor_y_coordinate += (velocity * time_step);

                cout << "Survivor X: " << setprecision(4) << *survivor_x_coordinate << ", "
                     << "Survivor Y: " << setprecision(4) << *survivor_y_coordinate << endl;
            }
        }
        else if (survivor_direction == 2)
        {
            if ((*survivor_y_coordinate <= 0) || (*survivor_x_coordinate <= 0))
            {
                *survivor_x_coordinate = *survivor_y_coordinate;
                *survivor_y_coordinate = *survivor_x_coordinate;
            }
            else
            {
                *survivor_x_coordinate -= (velocity * time_step);
                *survivor_y_coordinate -= (velocity * time_step);
                cout << "Survivor X: " << setprecision(4) << *survivor_x_coordinate << ", "
                     << "Survivor Y: " << setprecision(4) << *survivor_y_coordinate << endl;
            }
        }
        else if (survivor_direction == 3)
        {
            if ((*survivor_y_coordinate <= 0) || (*survivor_x_coordinate >= x_max))
            {
                *survivor_x_coordinate = *survivor_x_coordinate;
                *survivor_y_coordinate = *survivor_y_coordinate;
            }
            else
            {
                *survivor_x_coordinate += (velocity * time_step);
                *survivor_y_coordinate -= (velocity * time_step);
                cout << "Survivor X: " << setprecision(4) << *survivor_x_coordinate << ", "
                     << "Survivor Y: " << setprecision(4) << *survivor_y_coordinate << endl;
            }
        }
        else if (survivor_direction == 4)
        {
            if ((*survivor_y_coordinate >= y_max) || (*survivor_x_coordinate >= x_max))
            {
                *survivor_x_coordinate = *survivor_x_coordinate;
                *survivor_y_coordinate = *survivor_y_coordinate;
            }
            else
            {
                *survivor_x_coordinate += (velocity * time_step);
                *survivor_y_coordinate += (velocity * time_step);
                cout << "Survivor X: " << setprecision(4) << *survivor_x_coordinate << ", "
                     << "Survivor Y: " << setprecision(4) << *survivor_y_coordinate << endl;
            }
        }
        *previous_second = current_second;
    }
}