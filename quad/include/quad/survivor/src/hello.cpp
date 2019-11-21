#include <iostream>
#include <ctime>
#include <iomanip>
using namespace std;

int main()
{
    //These are the time variables. I do not understand them. Do not touch them
    time_t t = time(0);
    tm *now = localtime(&t);
    int current_second = now->tm_sec;
    int previous_second;
    int survivor_direction = 4;
    float velocity = 0.3;
    float survivor_y_coordinate = 5;
    float survivor_x_coordinate = 5;
    int x_max = 10;
    int time_step = 1;
    int y_max = 10;
    int cycle = 0;
    while (cycle < 100)
    {
        time_t t = time(0);
        tm *now = localtime(&t);
        current_second = now->tm_sec;
        if (current_second != previous_second)
        {
            previous_second = current_second;
            if (survivor_direction == 1)
            {
                if ((survivor_x_coordinate >= x_max) || (survivor_y_coordinate <= 0))
                {
                    survivor_x_coordinate = survivor_x_coordinate;
                    survivor_y_coordinate = survivor_y_coordinate;
                }
                else
                {
                    survivor_x_coordinate += (velocity * time_step);
                    survivor_y_coordinate -= (velocity * time_step);

                    cout << "Survivor X: " << setprecision(4) << survivor_x_coordinate << ", "
                         << "Survivor Y: " << setprecision(4) << survivor_y_coordinate << endl;
                }
            }
            else if (survivor_direction == 2)
            {
                if ((survivor_x_coordinate <= 0) || (survivor_y_coordinate <= 0))
                {
                    survivor_x_coordinate = survivor_y_coordinate;
                    survivor_y_coordinate = survivor_x_coordinate;
                }
                else
                {
                    survivor_x_coordinate -= (velocity * time_step);
                    survivor_y_coordinate -= (velocity * time_step);
                    cout << "Survivor X: " << setprecision(4) << survivor_x_coordinate << ", "
                         << "Survivor Y: " << setprecision(4) << survivor_y_coordinate << endl;
                }
            }
            else if (survivor_direction == 3)
            {
                if ((survivor_x_coordinate <= 0) || (survivor_y_coordinate >= y_max))
                {
                    survivor_x_coordinate = survivor_x_coordinate;
                    survivor_y_coordinate = survivor_y_coordinate;
                }
                else
                {
                    survivor_x_coordinate -= (velocity * time_step);
                    survivor_y_coordinate += (velocity * time_step);
                    cout << "Survivor X: " << setprecision(4) << survivor_x_coordinate << ", "
                         << "Survivor Y: " << setprecision(4) << survivor_y_coordinate << endl;
                }
            }
            else if (survivor_direction == 4)
            {
                cout << "Here 1!" << endl;
                cout << "Current Seconds: " << current_second << " Previous Second" << previous_second << endl;
                cout << "Survivor X, Y:" << setprecision(4) << survivor_x_coordinate << ", " << setprecision(4) << survivor_y_coordinate << endl;
                if ((survivor_x_coordinate >= x_max) || (survivor_y_coordinate >= y_max))
                {
                    survivor_x_coordinate = survivor_x_coordinate;
                    survivor_y_coordinate = survivor_y_coordinate;
                }
                else
                {
                    cout << "Here 2!" << endl;
                    survivor_x_coordinate = survivor_x_coordinate + (velocity * time_step);
                    survivor_y_coordinate = survivor_y_coordinate + (velocity * time_step);
                    cout << "Survivor X: " << setprecision(4) << survivor_x_coordinate << ", "
                         << "Survivor Y: " << setprecision(4) << survivor_y_coordinate << endl;
                }
            }
        }
    }
}