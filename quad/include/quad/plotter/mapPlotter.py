'''This script is written to plot the exploration map that's being generated in real-time by the agents exploring the environment. The 2D map being generated 
is part of the weighted_map structure which has a int member called exploration which gets cycled and dumped to the disc using the exploration_dumper() function,
part of the weight.xpp package.

This script is a fork of the individualMapPlotter.py, hence the similarity of the structure/comments.

-Sarthak
(09/02/2020)'''

import matplotlib.pyplot as plt
import csv
import numpy as np
from matplotlib import colors
from matplotlib.patches import Patch
import os.path

def individualMapGenerator(map_file):
    '''This function generates the map from the .CSV file read from the disc'''
    '''Start row makes sure that the row-elements are counted only once.'''
    start_row = 0

    with open(map_file) as element:
        '''Iterating through each row of the CSV file here to figure out the rows and columns'''
        mapObj = csv.reader(element, delimiter = '\t')
        rows = 0
        '''We need to count the column elements only once, since they remain consistent throughout the map'''
        start_row = 0
        columns = 0
        for row_element in mapObj:
            '''Iterating through the rows here'''
            rows+=1
            if start_row == 0:
                '''If no row has been iterated through before this, then iterate through the row and figure out the number of column elements.
                Iterate through it only once though.'''
                start_row += 1
                for column_element in row_element:
                    if(column_element != ''):
                        columns+=1

    '''Generating a numpy array to hold the individualMap read from the .csv file.'''
    individualMap = np.zeros([rows, columns])

    '''Using numpy arrays here for rows and columns, because colormesh takes input in these forms.'''
    column_array = np.arange(0, columns)
    row_array = np.arange(0, rows)

    with open(map_file) as csv_file:
        read_file = csv.reader(csv_file, delimiter='\t')
        for (y_element, element) in zip(column_array, read_file):
            for (x_element, by_element) in zip(row_array, element):
                if by_element is not '':
                    individualMap[y_element, x_element] = by_element

    return individualMap

def mapPresenter(individualMap, map_file = None):
    '''This function presents each UAVs map using matplotlib functions'''
    if map_file == None:
        '''If no map_file is presented, the function assumes that the stitched map is being presented here'''
        map_file = 'data/explorationMap_All.csv'

    '''Here, we create a custom legend to label the 3 types of cell in the colormap being plotted below.'''
    legend_elements = [Patch(facecolor='red', edgecolor='r',
                        label='Unexplored Cell'),
                Patch(facecolor='blue', edgecolor='b',
                        label='Explored Cell'),
                Patch(facecolor='yellow', edgecolor='y',
                        label='Survivor Cell')]

    '''Generating the subplot to extract the ax and the fig elements of the plot. Fig component isn't used though'''
    _, ax = plt.subplots()

    '''Generating a custom colormap of 3 colors to plot the exploration status of the different cells.'''
    cmap = colors.ListedColormap(['red', 'blue', 'yellow'])

    '''Plotting the weights here using matplotlib's colormesh functionality. Setting the colormap to 'jet' to ensure better contrast between smaller magnitudes.'''
    plt.pcolormesh(individualMap, cmap = cmap)

    rows, columns = individualMap.shape

    '''Flipping the coordinates here to ensure that ease of understanding between plots and the Gazebo environment.'''
    plt.ylim(columns, 0)

    '''Adding a grid to improve readability of the map and anchors the colors generated as well.'''
    plt.grid(True)

    plt.title('Exploration of Environment by UAV: ' + ((map_file.split('/')[-1]).split('.'))[0].split('_')[-1])

    '''Adding labels to the X and the Y axis to improve the readability of the maps here'''
    plt.xticks(np.arange(0, rows))
    plt.yticks(np.arange(0, columns))

    '''Axes here are flipped because we follow the right hand screw rule in 3D.'''
    plt.xlabel('Y Axis')
    plt.ylabel('X Axis')

    ax.set_aspect('equal')
    ax.legend(handles = legend_elements)

    '''Splitting the filename at the extension, extracting only the first part and appending a png to it to save it to disc.'''
    plt.savefig('/home/sarthak/catkin_ws/src/assets/' +
                (map_file.split('/')[-1]).split('.')[0]+'.png')
    plt.show()

def mapStitcher(individualMaps):
    '''This function stitches the exploration maps generated together'''
    rows, columns = individualMaps[0].shape
    completeMap = np.zeros([rows, columns])
    for individualMap in individualMaps:
        '''Applyinf a .add function here to add each individualMap to the completeMap'''
        completeMap = np.add(completeMap, individualMap)

    return completeMap

'''This variable checks the number of UAV file data available on disc'''
n_uav = 0

'''This line checks the number of UAV/Survivor paits found in the disc'''
while os.path.isfile('data/explorationMap_'+str(n_uav)+'.csv'):
    n_uav+=1

map_files = ['data/explorationMap_'+ str(uav) +'.csv' for uav in range(0, n_uav)]

individualMaps = []

for map_file in map_files:
    '''Iteratively going through the CSVs of the various maps on disk'''
    individualMap = individualMapGenerator(map_file)
    '''Appending each map to a list of maps'''
    individualMaps.append(individualMap)
    '''Presenting each map generated'''
    mapPresenter(individualMap, map_file)

'''We call the mapSticher() function inside the mapPresenter, to generate the stitched map and then present it'''
mapPresenter(mapStitcher(individualMaps))