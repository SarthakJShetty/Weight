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
import os.path

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
    '''Variables to hold the global position of the UAV'''
    global_x_position = 0
    global_y_position = 0
    '''Grab the global_position_x/y only once'''
    global_position_check = 0
    '''Lists to hold the X, Y and Z coordinates respectively'''
    x_points = []
    y_points = []
    z_points = []
    '''Reading the lines in the text files and holding it in the file_elements list'''
    for line in position_file:
        if (line != '\n') and ('START' not in line):
            file_elements.append(line)
        if(('START' in line) and (global_position_check == 0)):
            '''We continously publish the relative positon of the UAV on the global coordinate system, and we use this loop to check for it in the .txt dump, 
            and report it to the main code, so that it can be added to the local coordinate system accordingly.'''
            global_x_position = float(line.split(" ")[1])
            global_y_position = float(line.split(" ")[2])
            global_position_check = 1
    if('SECONDS' in file_elements[-1]):
        '''If the last recorded data line is a SECONDS line, we pop it here'''
        file_elements.pop(-1)
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

    '''Converting the local coordinates to the global coordinate system from the relative position published by the quad_node'''
    x_points = [(x_point + global_x_position) for x_point in x_points]
    y_points = [(y_point + global_y_position) for y_point in y_points]

    '''Returning all the variables recorded to the plotting functions'''
    return x_points, y_points, z_points, x_time, y_time, z_time, start_second, end_second, start_second_decimal, end_second_decimal

'''This variable checks the number of UAV file data available on disc'''
n_uav = 0

'''This line checks the number of UAV/Survivor paits found in the disc'''
while os.path.isfile('data/uav'+str(n_uav)+'_pos.txt') and os.path.isfile('data/survivor'+str(n_uav)+'_pos.txt'):
    n_uav+=1

'''All UAV data is collected in these lists. Numpy arrays are not used since pre-declaration is required'''
x_points_uav = [[] for uav in range(0, n_uav)]
y_points_uav = [[] for uav in range(0, n_uav)]
z_points_uav = [[] for uav in range(0, n_uav)]
x_time_uav = [[] for uav in range(0, n_uav)]
y_time_uav = [[] for uav in range(0, n_uav)]
z_time_uav = [[] for uav in range(0, n_uav)]
start_second_uav = [[] for uav in range(0, n_uav)]
end_second_uav = [[] for uav in range(0, n_uav)]
start_second_decimal_uav = [[] for uav in range(0, n_uav)]
end_second_decimal_uav = [[] for uav in range(0, n_uav)]

'''All Survivor data is collected in these lists. Numpy arrays are not used since pre-declaration is required'''
x_points_survivor = [[] for uav in range(0, n_uav)]
y_points_survivor = [[] for uav in range(0, n_uav)]
z_points_survivor = [[] for uav in range(0, n_uav)]
x_time_survivor = [[] for uav in range(0, n_uav)]
y_time_survivor = [[] for uav in range(0, n_uav)]
z_time_survivor = [[] for uav in range(0, n_uav)]
start_second_survivor = [[] for uav in range(0, n_uav)]
end_second_survivor = [[] for uav in range(0, n_uav)]
start_second_decimal_survivor = [[] for uav in range(0, n_uav)]
end_second_decimal_survivor = [[] for uav in range(0, n_uav)]

for uav in range(0, n_uav):
    '''Populating the UAV/Survivor list of lists prior to plotting'''
    x_points_uav[uav], y_points_uav[uav], z_points_uav[uav], x_time_uav[uav], y_time_uav[uav], z_time_uav[uav], start_second_uav[uav], end_second_uav[uav], start_second_decimal_uav[uav], end_second_decimal_uav[uav] = file_reader('data/uav'+str(uav)+'_pos.txt', 'pos')
    x_points_survivor[uav], y_points_survivor[uav], z_points_survivor[uav], x_time_survivor[uav], y_time_survivor[uav], z_time_survivor[uav], start_second_survivor[uav], end_second_survivor[uav], start_second_decimal_survivor[uav], end_second_decimal_survivor[uav] = file_reader('data/survivor'+str(uav)+'_pos.txt', 'pos')

'''We multiply with 2 because each UAV is paired with a survivor'''
fig, axs = plt.subplots(n_uav*2, 3)

'''subplot_counter keeps track of the subplot frame being accessed'''
subplot_counter = 0

'''uav_counter keeps track of which UAV/Survivor pair data is being plotted right now'''
uav_counter = 0

while subplot_counter < (n_uav * 2):
    '''Here we plot the position graphs for the various UAVs'''
    axs[subplot_counter, 0].plot(x_time_uav[uav_counter], x_points_uav[uav_counter], 'r')
    axs[subplot_counter, 0].set_title('UAV '+str(uav_counter+1)+' X Coordinates vs Time')
    axs[subplot_counter, 0].set_ylabel("X Position")
    axs[subplot_counter, 0].set_yticks([min(x_points_uav[uav_counter]), max(x_points_uav[uav_counter])])

    axs[subplot_counter, 1].plot(y_time_uav[uav_counter], y_points_uav[uav_counter], 'g')
    axs[subplot_counter, 1].set_title('UAV '+str(uav_counter+1)+' Y Coordinates vs Time')
    axs[subplot_counter, 1].set_ylabel("Y Position")
    axs[subplot_counter, 1].set_yticks([min(y_points_uav[uav_counter]), max(y_points_uav[uav_counter])])

    axs[subplot_counter, 2].plot(z_time_uav[uav_counter], z_points_uav[uav_counter], 'b')
    axs[subplot_counter, 2].set_title('UAV '+str(uav_counter+1)+' Z Coordinates vs Time')
    axs[subplot_counter, 2].set_ylabel("Z Position")
    axs[subplot_counter, 2].set_yticks([min(z_points_uav[uav_counter]), max(z_points_uav[uav_counter])])

    '''For the same UAV, plot the survivor's data as well'''
    subplot_counter+=1

    axs[subplot_counter, 0].plot(x_time_survivor[uav_counter], x_points_survivor[uav_counter], 'r')
    axs[subplot_counter, 0].set_title('Survivor '+str(uav_counter+1)+' X Coordinates vs Time')
    axs[subplot_counter, 0].set_ylabel("X Position")
    axs[subplot_counter, 0].set_yticks([min(x_points_survivor[uav_counter]), max(x_points_survivor[uav_counter])])

    axs[subplot_counter, 1].plot(y_time_survivor[uav_counter], y_points_survivor[uav_counter], 'g')
    axs[subplot_counter, 1].set_title('Survivor '+str(uav_counter+1)+' Y Coordinates vs Time')
    axs[subplot_counter, 1].set_ylabel("Y Position")
    axs[subplot_counter, 1].set_yticks([min(y_points_survivor[uav_counter]), max(y_points_survivor[uav_counter])])

    axs[subplot_counter, 2].plot(z_time_survivor[uav_counter], z_points_survivor[uav_counter], 'b')
    axs[subplot_counter, 2].set_title('Survivor '+str(uav_counter+1)+' Z Coordinates vs Time')
    axs[subplot_counter, 2].set_ylabel("Z Position")
    axs[subplot_counter, 2].set_yticks([min(z_points_survivor[uav_counter]), max(z_points_survivor[uav_counter])])

    if((subplot_counter+1) == (n_uav * 2)):
        '''We add the x-axis labl only to the last set of subplots to prevent overcrowding between the title and the x-axis labels'''
        axs[subplot_counter, 0].set_xlabel("Seconds (s)")
        axs[subplot_counter, 1].set_xlabel("Seconds (s)")
        axs[subplot_counter, 2].set_xlabel("Seconds (s)")

    '''After each iteration, we go to the next pair of subplots (UAV + Survivor combo) and the next agent (refers to the index of UAV/Survivor lists)'''
    subplot_counter+=1
    uav_counter+=1

plt.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=None, hspace=1.8)
plt.savefig('/home/sarthak/catkin_ws/src/assets/UAVSurvivorInfo.png')

plt.show()

'''These points are required to plot the boundaries of the environment in the 3D plot'''
x_points_env = [0, 19, 19, 0, 0]
y_points_env = [0, 0, 19, 19, 0]
z_points_env = [0, 0, 0, 0, 0]

'''Generating the 3D plot object'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

def get_cmap(n, name = 'hsv'):
    '''Here, we declare a simple function to generate a colormap to be utilized across the code'''
    return plt.cm.get_cmap(name, n)

'''Here, we generate the colormaps that will be used by the 3D plotting system'''
cmap = get_cmap(n_uav * 2)

'''List that will hold the starting and ending positions of the UAVs in the form of tuples, (x, y, z)'''
uav_starting = []
uav_ending = []

'''List that will hold the starting and ending positions of the Survivors in the form of tuples, (x, y, z)'''
survivor_starting = []
survivor_ending = []

for agent_counter in range(0, n_uav):

    '''Starting point of UAV trajectory in 3D'''
    uav_starting.append((x_points_uav[agent_counter][0], y_points_uav[agent_counter][0], z_points_uav[agent_counter][0]))
    uav_ending.append((x_points_uav[agent_counter][len(x_points_uav[agent_counter])-1], y_points_uav[agent_counter][len(y_points_uav[agent_counter])-1], z_points_uav[agent_counter][len(x_points_uav[agent_counter])-1]))

    '''Starting point of survivor trajectory in 3D'''
    survivor_starting.append((x_points_survivor[agent_counter][0], y_points_survivor[agent_counter][0], z_points_survivor[agent_counter][0]))
    survivor_ending.append((x_points_survivor[agent_counter][len(x_points_survivor[agent_counter])-1], y_points_survivor[agent_counter][len(y_points_survivor[agent_counter])-1], z_points_survivor[agent_counter][len(y_points_survivor[agent_counter])-1]))

    '''Plotting the entire trajectory of UAV x in the environment'''
    ax.plot(x_points_uav[agent_counter], y_points_uav[agent_counter], z_points_uav[agent_counter],
            label='UAV '+str(agent_counter+1)+' Trajectory', color = cmap(agent_counter))

    '''Plotting only the start point of UAV x trajectory'''
    ax.plot([uav_starting[agent_counter][0]], [uav_starting[agent_counter][1]], [uav_starting[agent_counter][2]],
            color='g', marker='x')

    '''Plotting only the end point of UAV x trajectory'''
    ax.plot([uav_ending[agent_counter][0]], [uav_ending[agent_counter][1]], [uav_ending[agent_counter][2]],
            color='r', marker='x')

    if (survivor_starting[agent_counter] != survivor_ending[agent_counter]):
        '''Plotting the entire trajectory of the survivors in the environment inly if there was some movemement'''
        ax.plot(x_points_survivor[agent_counter], y_points_survivor[agent_counter], z_points_survivor[agent_counter],
                label='Survivor '+str(agent_counter+1)+' Trajectory', color=cmap(agent_counter), marker = '<')

    '''Plotting only the start point of the survivors trajectory'''
    ax.plot([survivor_starting[agent_counter][0]], [survivor_starting[agent_counter][1]], [survivor_starting[agent_counter][2]],
            color='g', marker='D')

    '''Plotting only the end point of the survivors trajectory'''
    ax.plot([survivor_ending[agent_counter][0]], [survivor_ending[agent_counter][1]], [survivor_ending[agent_counter][2]],
            color='r', marker='D')

    if (agent_counter == (n_uav - 1)):
        '''We want only one set of starting and ending points points to be plotted for the UAV and the Survivor. We plot only for the (n_uav - 1)th UAV
        since it will be located at the end of the legend instead of the beginning'''

        '''Plotting only the start point of UAV x trajectory'''
        ax.plot([uav_starting[agent_counter][0]], [uav_starting[agent_counter][1]], [uav_starting[agent_counter][2]],
                label = 'UAV Starting Point', color='g', marker='x')

        '''Plotting only the end point of UAV x trajectory'''
        ax.plot([uav_ending[agent_counter][0]], [uav_ending[agent_counter][1]], [uav_ending[agent_counter][2]],
                label = 'UAV Ending Point', color='r', marker='x')    

        '''Plotting only the start point of the survivors trajectory'''
        ax.plot([survivor_starting[agent_counter][0]], [survivor_starting[agent_counter][1]], [survivor_starting[agent_counter][2]],
                label = 'Survivor Starting Point', color='g', marker='D')

        '''Plotting only the end point of the survivors trajectory'''
        ax.plot([survivor_ending[agent_counter][0]], [survivor_ending[agent_counter][1]], [survivor_ending[agent_counter][2]],
                label = 'Survivor Ending Point', color='r', marker='D')

'''Plotting the environment boundary'''
ax.plot(x_points_env, y_points_env, z_points_env,
        label='Environment Boundary', color='b')

'''Calculating the difference between the observer node has been triggered till when the UAV returns to the base'''
time_difference = (end_second_decimal_uav[0] - start_second_decimal_uav[0])

print('Completed in: %s seconds.' % (str(time_difference)))

ax.set_aspect('equal')

plt.title('UAV & Survivor Paths: Probabilistic Search. \nCompleted in: %s seconds' %
          time_difference, loc='left')

'''Labelling the axes here'''
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

ax.legend(loc='best', title='Legend', numpoints=1)

plt.savefig('/home/sarthak/catkin_ws/src/assets/Trajectories.png')

plt.show()

'''Here on out, we generate the X-Y Projection of the trajectories'''
fig_projection = plt.figure()
ax_projection = fig_projection.add_subplot(111)

for agent_counter in range(0, n_uav):
    '''Plotting the entire trajectory of UAV x in the environment'''
    ax_projection.plot(y_points_uav[agent_counter], x_points_uav[agent_counter],
            label='UAV '+str(agent_counter+1)+' Trajectory', color = cmap(agent_counter))

    '''Plotting only the start point of UAV x trajectory'''
    ax_projection.plot([uav_starting[agent_counter][1]], [uav_starting[agent_counter][0]],
            color='g', marker='x')

    '''Plotting only the end point of UAV x trajectory'''
    ax_projection.plot([uav_ending[agent_counter][1]], [uav_ending[agent_counter][0]],
            color='r', marker='x')

    '''Plotting the entire trajectory of the survivors in the environment'''
    ax_projection.plot(y_points_survivor[agent_counter], x_points_survivor[agent_counter],
            label='Survivor '+str(agent_counter+1)+' Trajectory', color=cmap(agent_counter), marker = '>')

    '''Plotting only the start point of the survivors trajectory'''
    ax_projection.plot([survivor_starting[agent_counter][1]], [survivor_starting[agent_counter][0]],
            color='g', marker='D')

    '''Plotting only the end point of the survivors trajectory'''
    ax_projection.plot([survivor_ending[agent_counter][1]], [survivor_ending[agent_counter][0]],
            color='r', marker='D')

'''Plotting the environment boundary'''
ax_projection.plot(y_points_env, x_points_env,
        label='Environment Boundary', color='b')

plt.title('X-Y Projection of Trajectories')

'''Flipping the labesl here to correspond to the right hand rule'''
plt.ylabel('X Axis')
plt.xlabel('Y Axis')

'''Flipping the markings on the re-assigned X axis to reflect the Gazebo environment'''
ax_projection.set_ylim(max(x_points_env) + 1, -1)

'''Setting the aspect ratio to ensure a balanced figure'''
ax_projection.set_aspect('equal')

plt.savefig('/home/sarthak/catkin_ws/src/assets/XYProjection.png')

plt.show()