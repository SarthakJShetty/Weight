'''This script is written to plot the weightage map created as a density color mesh.
While running the code, the drone does not execute the entire weightmap. To ensure that the weights are being generated correctly, we've
built this function to analyze the weightage map generated as a matplotlib colormesh.

Similar to the rest of the modules written for this project, this densityplotter function is agnostic to the dimensions of the environment.
Tests are due to substantiate this claim. Will be updating a section in the README of repository dedicated to this.

-Sarthak
(20/12/2019)'''

import matplotlib.pyplot as plt
import csv
import numpy as np
import os.path

'''This variable checks the number of UAV file data available on disc'''
n_uav = 0

'''This line checks the number of UAV/Survivor paits found in the disc'''
while os.path.isfile('data/weightMap_'+str(n_uav)+'.csv'):
    n_uav+=1

'''Filename pointing to the weightMap csv file'''
weight_files = ['data/weightMap_'+ str(uav) +'.csv' for uav in range(0, n_uav)]
weight_files += ['data/priorityMap_'+ str(uav) +'.csv' for uav in range(0, n_uav)]


for weight_file in weight_files:
    '''Start row makes sure that the row-elements are counted only once.'''
    start_row = 0
    '''Variable to count the number of elements in each row.'''
    row = 0
    '''Variable to count the number of columns.'''
    column = 0

    with open(weight_file) as csv_file:
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

    with open(weight_file) as csv_file:
        read_file = csv.reader(csv_file, delimiter='\t')
        for (y_element, element) in zip(column_array, read_file):
            for (x_element, by_element) in zip(row_array, element):
                if by_element is not '':
                    density[y_element, x_element] = by_element
    fig, ax = plt.subplots()
    '''Importing a colormap here (Jet!) and reversing it to better infer the priority of the waypoints.'''
    colormap = plt.cm.get_cmap('viridis')

    '''Plotting the weights here using matplotlib's colormesh functionality. Setting the colormap to 'jet' to ensure better contrast between smaller magnitudes.'''
    plt.pcolormesh(density, cmap=colormap)

    '''Adding a colorbar to understand the value of the different weights & their intensity.'''
    cb = plt.colorbar()

    if 'weightMap.csv' in weight_file:
        cb.set_label('Increasing Weightage of Waypoints')
    elif 'priorityMap.csv' in weight_file:
        cb.set_label('Decreasing Priority of Waypoints')

    '''Flipping the coordinates here to ensure that ease of understanding between plots and the Gazebo environment.'''
    plt.ylim(column, 0)

    '''Adding a grid to improve readability of the map and anchors the colors generated as well.'''
    plt.grid(True)

    if 'weightMap' in weight_file.split('/')[-1]:
        plt.title('Density of Weights for UAV ' + ((weight_file.split('/')[-1]).split('.'))[0].split('_')[1])
    elif 'priorityMap' in weight_file.split('/')[-1]:
        plt.title('Priority of Waypoints for UAV ' + ((weight_file.split('/')[-1]).split('.'))[0].split('_')[1])
    
    ax.set_aspect('equal')
    
    '''Splitting the filename at the extension, extracting only the first part and appending a png to it to save it to disc.'''
    plt.savefig('/home/sarthak/catkin_ws/src/assets/' +
                (weight_file.split('/')[-1]).split('.')[0]+'.png')
    plt.show()
