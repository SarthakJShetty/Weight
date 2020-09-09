'''This script is written to visualize the split in the environment, amongst the UAVs. The split_dumper() functions, dumps a .csv file of the form splitMap_x.csv,
which has to be plotted.

This script is a fork of the densityPlotter.py, hence the similarity of the structure/comments.

-Sarthak
(01/09/2020)'''

import matplotlib.pyplot as plt
import csv
import numpy as np
from matplotlib import colors
from matplotlib.patches import Patch
import os.path

'''This variable checks the number of UAV file data available on disc'''
n_uav = 0

'''This line checks the number of UAV/Survivor paits found in the disc'''
while os.path.isfile('data/splitMap_'+str(n_uav)+'_pos.txt'):
    n_uav+=1

map_files = ['/home/sarthak/catkin_ws/src/quad/include/quad/plotter/data/splitMap_'+ str(uav) +'.csv' for uav in range(0, n_uav)]

def get_cmap(n, name = 'viridis'):
    '''Here, we declare a simple function to generate a colormap to be utilized across the code'''
    return plt.cm.get_cmap(name, n)

for map_file in map_files:
    '''Start row makes sure that the row-elements are counted only once.'''
    start_row = 0
    '''Variable to count the number of elements in each row.'''
    row = 0
    '''Variable to count the number of columns.'''
    column = 0

    with open(map_file) as csv_file:
        '''This loop is used to calculate the number of rows and columns in the csv file that has been dumped.
        We use this data to intialize the numpy arrays that we use to plot the density maps.'''
        read_file = csv.reader(csv_file, delimiter='\t')
        for element in read_file:
            column += 1
            for by_element in element:
                '''This check is to make sure that the number of row elements of only one row is calculated.'''
                if start_row is 0:
                    '''This if condition is requried because the delimiter condition in csv.reader creates a N+th element which is just '', a space '''
                    if by_element is not '':
                        row += 1
            if start_row is 0:
                '''After the number of row-elements has been counted, do not count anymore elements on successive columns, this variable keeps a track on that.'''
                start_row += 1

    '''Generating a numpy array to hold the density read from the .csv file.'''
    density = np.zeros([column, row])

    '''Using numpy arrays here for rows and columns, because colormesh takes input in these forms.'''
    column_array = np.arange(0, column)
    row_array = np.arange(0, row)

    with open(map_file) as csv_file:
        read_file = csv.reader(csv_file, delimiter='\t')
        for (y_element, element) in zip(column_array, read_file):
            for (x_element, by_element) in zip(row_array, element):
                if by_element is not '':
                    density[y_element, x_element] = by_element

    '''Here, we generate the series of colors from which we plot different colors for the different agents'''
    cmap = get_cmap(len(map_files))

    '''We plot the legend for the figures here'''
    legend_elements = [Patch(facecolor = cmap(uav), label='UAV ' + str(uav)) for uav in range(0, len(map_files))]

    '''Generating the subplot to extract the ax and the fig elements of the plot.'''
    fig, ax = plt.subplots()

    '''Plotting the weights here using matplotlib's colormesh functionality. Setting the colormap to 'jet' to ensure better contrast between smaller magnitudes.'''
    plt.pcolormesh(density, cmap = cmap)

    '''Flipping the coordinates here to ensure that ease of understanding between plots and the Gazebo environment.'''
    plt.ylim(column, 0)

    '''Adding a grid to improve readability of the map and anchors the colors generated as well.'''
    plt.grid(True)
    
    plt.title('Environment Split by UAV ' + ((map_file.split('/')[-1]).split('.'))[0].split('_')[1])

    '''Adding labels to the X and the Y axis to improve the readability of the maps here'''
    plt.xticks(row_array)
    plt.yticks(column_array)

    '''Axes here are flipped because we follow the right hand screw rule in 3D.'''
    plt.xlabel('Y Axis')
    plt.ylabel('X Axis')

    ax.set_aspect('equal')
    ax.legend(handles = legend_elements)

    '''Splitting the filename at the extension, extracting only the first part and appending a png to it to save it to disc.'''
    plt.savefig('/home/sarthak/catkin_ws/src/assets/' +
                (map_file.split('/')[-1]).split('.')[0]+'.png')
    plt.show()
