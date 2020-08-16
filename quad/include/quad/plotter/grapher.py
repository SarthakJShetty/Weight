'''In this script, we plot the agent's position as well their trajectory from the .txt files that the subber.py script dumps during the subscribiction
process. We plot:
1. Position (X, Y, Z) vs Time
2. 3D Trajectory of all agents involved in the simulation

To-Do:
1. Dump CSVs instead of .txt files

-Sarthak
(03/02/2020)'''

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def second_converter(time_in_decimal):
    '''Here, we take the start and the end second in the decimal format as sent over by the subber,
    and calulate the time in pure second format.
    We compute the pure second format for both the start and the end second, then compute the difference to get the time elapsed.'''

    split_time = time_in_decimal.split(".")
    '''Converting the individual units to integers and then to seconds'''
    time_hours = int(split_time[0]) * 60 * 60 * 1
    time_minute = int(split_time[1]) * 60 * 1
    time_second = int(split_time[2]) * 1

    '''Returning the total number of seconds here'''
    return (time_hours + time_minute + time_second)


def file_reader(coordinates_file_txt, plotting_parameter):
    '''This function reads the coordinates_file_text and depending on the plotting_parameter, charts either the position, velocity or acceleration.'''
    position_file = open(coordinates_file_txt)
    '''Keeps track of the previous second, so that multiple datapoints belonging to the same second aren't plotted on the chart'''
    old_second = 0
    '''Keeps track of the current second noted in the data file. References it during plotting and compares it to the previous second to make sure
    multiple datapoints of the same second aren't plotted.'''
    current_second = 0
    '''Notes the first second recorded in the txt file, used to compute the time elapsed through the course of the simulation.'''
    start_second = 0
    '''Last recorded second in the given agents data file'''
    end_second = 0
    '''A variable to keep track of whether or not the first second has been recorded or not'''
    start_tracker = 0
    '''List to hold all the elements of the file, so that they can be accessed easily through the course of the program execution'''
    file_elements = []
    '''Lists to hold the X, Y and Z coordinates respectively'''
    x_points = []
    y_points = []
    z_points = []
    '''Reading the lines in the text files and holding it in the file_elements list'''
    for line in position_file:
        if line != '\n':
            file_elements.append(line)
    '''Traversing the elements of file_elements and checking if it contains the SECONDS keyword to grab the time of data collection'''
    for line in file_elements:
        if "SECONDS" in line:
            '''Noting the start_second here'''
            if(start_tracker == 0):
                start_second = (line.split(":")[1])
                '''Sending it over to the converter which takes the raw time and converts it into seconds'''
                start_second_decimal = second_converter(start_second)
                start_tracker = 1
            else:
                '''If the start_second has already been recorded continue to read the lines which have SECONDS in them'''
                current_second = line
        else:
            pass
        '''Adding recorded positions to the X, Y, Z point lists, either for position, velocity or acceleration plotting.'''
        if((current_second != old_second) and (plotting_parameter == 'pos')):
            x_points.append(
                round(float(file_elements[file_elements.index(line)+1][:-1]), 2))
            y_points.append(
                round(float(file_elements[file_elements.index(line)+2][:-1]), 2))
            z_points.append(
                round(float(file_elements[file_elements.index(line)+3][:-1]), 2))
            old_second = current_second
        elif((current_second != old_second) and ((plotting_parameter == 'vel') or (plotting_parameter == 'acc'))):
            x_points.append(
                abs(round(float(file_elements[file_elements.index(line)+1][:-1]), 2)))
            y_points.append(
                abs(round(float(file_elements[file_elements.index(line)+2][:-1]), 2)))
            z_points.append(
                abs(round(float(file_elements[file_elements.index(line)+3][:-1]), 2)))
            old_second = current_second
        else:
            pass
    '''Recording the end second and converting it into pure seconds'''
    end_second = (current_second.split(":")[1])
    end_second_decimal = second_converter(end_second)

    print("This is the end seconds: " + str(end_second) +
          " This is the start second: " + str(start_second))
    
    '''Calculating the lengths of the lists to plot the axes of the respective plots'''
    x_points_length = len(x_points)
    y_points_length = len(y_points)
    z_points_length = len(z_points)

    x_time = []
    y_time = []
    z_time = []

    '''Creating the X-Axis for the time to be plotted'''
    for time in range(0, x_points_length):
        x_time.append(time)

    for time in range(0, y_points_length):
        y_time.append(time)

    for time in range(0, z_points_length):
        z_time.append(time)

    '''Returning all the variables recorded to the plotting functions'''
    return x_points, y_points, z_points, x_time, y_time, z_time, start_second, end_second, start_second_decimal, end_second_decimal


x_points_1, y_points_1, z_points_1, x_time_1, y_time_1, z_time_1, start_second_1, end_second_1, start_second_decimal_1, end_second_decimal_1 = file_reader(
    '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav1_pos.txt', 'pos')
x_points_2, y_points_2, z_points_2, x_time_2, y_time_2, z_time_2, start_second_1, end_second_2, start_second_decimal_2, end_second_decimal_2 = file_reader(
    '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/uav2_pos.txt', 'pos')
x_points_3, y_points_3, z_points_3, x_time_3, y_time_3, z_time_3, start_second_3, end_second_3, start_second_decimal_3, end_second_decimal_3 = file_reader(
    '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/survivor1_pos.txt', 'pos')
x_points_4, y_points_4, z_points_4, x_time_4, y_time_4, z_time_4, start_second_4, end_second_4, start_second_decimal_4, end_second_decimal_4 = file_reader(
    '/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/survivor2_pos.txt', 'pos')

'''Plotting the agents properties vs time '''

plt.plot(y_time_1, y_points_1, 'r')
plt.title('UAV 1 X Coordinates vs Time')
plt.xlabel("Seconds (s)")
plt.ylabel("X Position")
plt.legend(['X Position'], loc=1)
plt.savefig('/home/sarthak/catkin_ws/src/assets/UAV1CoordinatesX.png')
plt.show()

plt.plot(x_time_1, x_points_1, 'g')
plt.title('UAV 1 Y Coordinates vs Time')
plt.xlabel("Seconds (s)")
plt.ylabel("Y Position")
plt.legend(['Y Position'], loc=1)
plt.savefig('/home/sarthak/catkin_ws/src/assets/UAV1CoordinatesY.png')
plt.show()

plt.plot(z_time_1, z_points_1, 'b')
plt.title('UAV 1 Z Coordinates vs Time')
plt.xlabel("Seconds (s)")
plt.ylabel("Z Position")
plt.legend(['Z Position'], loc=1)
plt.savefig('/home/sarthak/catkin_ws/src/assets/UAV1CoordinatesZ.png')
plt.show()

plt.plot(y_time_2, y_points_2, 'r')
plt.title('UAV 2 X Coordinates vs Time')
plt.xlabel("Seconds (s)")
plt.ylabel("X Position")
plt.legend(['X Position'], loc=1)
plt.savefig('/home/sarthak/catkin_ws/src/assets/UAV2CoordinatesX.png')
plt.show()

plt.plot(x_time_2, x_points_2, 'g')
plt.title('UAV 2 Y Coordinates vs Time')
plt.xlabel("Seconds (s)")
plt.ylabel("Y Position")
plt.legend(['Y Position'], loc=1)
plt.savefig('/home/sarthak/catkin_ws/src/assets/UAV2CoordinatesY.png')
plt.show()

plt.plot(z_time_2, z_points_2, 'b')
plt.title('UAV 2 Z Coordinates vs Time')
plt.xlabel("Seconds (s)")
plt.ylabel("Z Position")
plt.legend(['Z Position'], loc=1)
plt.savefig('/home/sarthak/catkin_ws/src/assets/UAV2CoordinatesZ.png')
plt.show()

plt.plot(y_time_3, y_points_3, 'r')
plt.xlabel("Seconds (s)")
plt.ylabel("X Position")
plt.legend(['X Position'], loc=1)
plt.title('Survivor 1 X Coordinates vs Time')
plt.savefig('/home/sarthak/catkin_ws/src/assets/Survivor1X.png')
plt.show()

plt.plot(x_time_3, x_points_3, 'g')
plt.xlabel("Seconds (s)")
plt.ylabel("Y Position")
plt.legend(['Y Position'], loc=1)
plt.title('Survivor 1 Y Coordinates vs Time')
plt.savefig('/home/sarthak/catkin_ws/src/assets/Survivor1Y.png')
plt.show()

plt.plot(z_time_3, z_points_3, 'b')
plt.xlabel("Seconds (s)")
plt.ylabel("Z Position")
plt.legend(['Z Position'], loc=1)
plt.title('Survivor 1 Z Coordinates vs Time')
plt.savefig('/home/sarthak/catkin_ws/src/assets/Survivor1Z.png')
plt.show()

plt.plot(y_time_4, y_points_4, 'r')
plt.xlabel("Seconds (s)")
plt.ylabel("X Position")
plt.legend(['X Position'], loc=1)
plt.title('Survivor 2 X Coordinates vs Time')
plt.savefig('/home/sarthak/catkin_ws/src/assets/Survivor2X.png')
plt.show()

plt.plot(x_time_4, x_points_4, 'g')
plt.xlabel("Seconds (s)")
plt.ylabel("Y Position")
plt.legend(['Y Position'], loc=1)
plt.title('Survivor 2 Y Coordinates vs Time')
plt.savefig('/home/sarthak/catkin_ws/src/assets/Survivor2Y.png')
plt.show()

plt.plot(z_time_4, z_points_4, 'b')
plt.xlabel("Seconds (s)")
plt.ylabel("Z Position")
plt.legend(['Z Position'], loc=1)
plt.title('Survivor 2 Z Coordinates vs Time')
plt.savefig('/home/sarthak/catkin_ws/src/assets/Survivor2Z.png')
plt.show()

'''These points are required to plot the boundaries of the environment in the 3D plot'''
x_points_env = [0, 18.5, 18.5, 0, 0]
y_points_env = [0, 0, 18.5, 18.5, 0]
z_points_env = [0, 0, 0, 0, 0]

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

'''Plotting the entire trajectory of UAV 1 in the environment'''
ax.plot(x_points_1, y_points_1, z_points_1,
        label='UAV 1 Trajectory', color='r')

'''Plotting only the start point of UAV 1 trajectory'''
ax.plot([x_points_1[0]], [y_points_1[0]], [z_points_1[0]],
        label='UAV 1 Start Point', color='g', marker='x')

'''Plotting only the end point of UAV 1 trajectory'''
ax.plot([x_points_1[len(x_points_1)-1]], [y_points_1[len(x_points_1)-1]], [z_points_1[len(x_points_1)-1]],
        label='UAV 1 End Point', color='r', marker='x')

'''Plotting the entire trajectory of UAV 2 in the environment'''
ax.plot(x_points_2, y_points_2, z_points_2,
        label='UAV 2 Trajectory', color='c')

'''Plotting only the start point of UAV 2 trajectory'''
ax.plot([x_points_2[0]], [y_points_2[0]], [z_points_2[0]],
        label='UAV 2 Start Point', color='g', marker='o')

'''Plotting only the end point of UAV 2 trajectory'''
ax.plot([x_points_2[len(x_points_2)-1]], [y_points_2[len(x_points_2)-1]], [z_points_2[len(x_points_2)-1]],
        label='UAV 2 End Point', color='r', marker='o')

'''Plotting the entire trajectory of the survivors in the environment'''
ax.plot(x_points_3, y_points_3, z_points_3,
        label='Survivor 1 Trajectory', color='y')

'''Plotting only the start point of the survivors trajectory'''
ax.plot([x_points_3[0]], [y_points_3[0]], [z_points_3[0]],
        label='Survivor 1 Start Point', color='g', marker='D')

'''Plotting only the end point of the survivors trajectory'''
ax.plot([x_points_3[len(x_points_3)-1]], [y_points_3[len(x_points_3)-1]], [z_points_3[len(x_points_3)-1]],
        label='Survivor 1 End Point', color='r', marker='D')

'''Plotting the entire trajectory of the survivors in the environment'''
ax.plot(x_points_4, y_points_4, z_points_4,
        label='Survivor 2 Trajectory', color='y')

'''Plotting only the start point of the survivors trajectory'''
ax.plot([x_points_4[0]], [y_points_4[0]], [z_points_4[0]],
        label='Survivor 2 Start Point', color='g', marker='D')

'''Plotting only the end point of the survivors trajectory'''
ax.plot([x_points_4[len(x_points_4)-1]], [y_points_4[len(x_points_4)-1]], [z_points_4[len(x_points_4)-1]],
        label='Survivor 2 End Point', color='r', marker='D')

'''Plotting the environment boundary'''
ax.plot(x_points_env, y_points_env, z_points_env,
        label='Environment Boundary', color='b', marker='>')

'''Calculating the difference between the observer node has been triggered till when the UAV returns to the base'''
difference = (end_second_decimal_1 - start_second_decimal_1)

print('Completed in: %s seconds: ' % (str(difference)))

ax.set_aspect('equal')

plt.title('UAV & Survivor Paths: Probabilistic Search. \nCompleted in: %s seconds' %
          difference, loc='left')
ax.set_xlabel('Y Axis')
ax.set_ylabel('X Axis')
ax.set_zlabel('Z Axis')
ax.legend(loc='best', title='Trajectories', numpoints=1)

plt.savefig('/home/sarthak/catkin_ws/src/assets/Trajectories.png')

plt.show()