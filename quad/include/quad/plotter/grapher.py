import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def file_reader(position_file_txt, plotting_parameter):
    position_file = open(position_file_txt)
    old_second = 0
    current_second = 0
    file_elements = []
    x_points = []
    y_points = []
    z_points = []
    for line in position_file:
        if line != '\n':
            file_elements.append(line)

    for line in file_elements:
        if "SECONDS" in line:
            current_second = line
        else:
            pass
        if((current_second != old_second) and (plotting_parameter == 'pos')):
            x_points.append(
                round(float(file_elements[file_elements.index(line)+1][:-1]), 2))
            y_points.append(
                round(float(file_elements[file_elements.index(line)+2][:-1]), 2))
            z_points.append(
                round(float(file_elements[file_elements.index(line)+3][:-1]), 2))
            #print(round(float(file_elements[file_elements.index(line)+1][:-1]), 2), round(float(file_elements[file_elements.index(line)+2][:-1]), 2), round(float(file_elements[file_elements.index(line)+3][:-1]), 2))
            old_second = current_second
        elif((current_second != old_second) and ((plotting_parameter == 'vel') or (plotting_parameter == 'acc'))):
            x_points.append(
                abs(round(float(file_elements[file_elements.index(line)+1][:-1]), 2)))
            y_points.append(
                abs(round(float(file_elements[file_elements.index(line)+2][:-1]), 2)))
            z_points.append(
                abs(round(float(file_elements[file_elements.index(line)+3][:-1]), 2)))
            #print(round(float(file_elements[file_elements.index(line)+1][:-1]), 2), round(float(file_elements[file_elements.index(line)+2][:-1]), 2), round(float(file_elements[file_elements.index(line)+3][:-1]), 2))
            old_second = current_second
        else:
            pass

    x_points_length = len(x_points)
    y_points_length = len(y_points)
    z_points_length = len(z_points)

    x_time = []
    y_time = []
    z_time = []

    for time in range(0, x_points_length):
        x_time.append(time)

    for time in range(0, y_points_length):
        y_time.append(time)

    for time in range(0, z_points_length):
        z_time.append(time)

    return x_points, y_points, z_points, x_time, y_time, z_time


x_points_1, y_points_1, z_points_1, x_time_1, y_time_1, z_time_1 = file_reader(
    'uav1_pos.txt', 'pos')
x_points_2, y_points_2, z_points_2, x_time_2, y_time_2, z_time_2 = file_reader(
    'uav2_pos.txt', 'pos')

plt.plot(x_time_1, x_points_1)
plt.plot(y_time_1, y_points_1)
plt.plot(z_time_1, z_points_1)
plt.title('UAV 1 Coordinate vs Time')
plt.xlabel("Seconds (s)")
plt.ylabel("X/Y/Z Position")
plt.legend(['X Position', 'Y Position', 'Z Position'])
plt.show()

plt.plot(x_time_2, x_points_2)
plt.plot(y_time_2, y_points_2)
plt.plot(z_time_2, z_points_2)
plt.title('Survivor\'s Coordinate vs Time')
plt.xlabel("Seconds (s)")
plt.ylabel("X/Y/Z Position")
plt.legend(['X Position', 'Y Position', 'Z Position'])
plt.show()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(x_points_1, y_points_1, z_points_1,
        label='UAV 1 Trajectory', color='r')
ax.plot([x_points_1[0]], [y_points_1[0]], [z_points_1[0]],
        label='UAV 1 Start Point', color='g', marker='o')
ax.plot([x_points_1[len(x_points_1)-1]], [y_points_1[len(x_points_1)-1]], [z_points_1[len(x_points_1)-1]],
        label='UAV 1 End Point', color='r', marker='o')
ax.plot(x_points_2, y_points_2, z_points_2,
        label='Survivor\'s Trajectory', color='b')
ax.plot([x_points_2[0]], [y_points_2[0]], [z_points_2[0]],
        label='Survivor\'s Start Point', color='g', marker='D')
ax.plot([x_points_2[len(x_points_2)-1]], [y_points_2[len(x_points_2)-1]], [z_points_2[len(x_points_2)-1]],
        label='Survivor\'s End Point', color='r', marker='D')

plt.title('UAV & Survivor Paths: Lawn-Mower Search', loc='left')
ax.set_xlabel('X-Coordinate')
ax.set_ylabel('Y-Coordinate')
ax.set_zlabel('Z-Coordinate')
ax.legend(loc='best', title='Trajectories', numpoints=1)

plt.show()
