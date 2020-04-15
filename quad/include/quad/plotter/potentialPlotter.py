import numpy as np
import pandas as pd
from math import sqrt
from mpl_toolkits.mplot3d import Axes3D  
import matplotlib.pyplot as plt

def obstacleGenerator(objectsBoundary, obstaclePoints):
    '''Returning all the points that lie within and on the boundary of the given object, to generate the Repulsive Potential'''
    for objectBoundary in objectsBoundary:
        for lowerBound in range(objectBoundary[0][0], (objectBoundary[1][0]+1)):
            for higherBound in range(objectBoundary[0][1], (objectBoundary[1][1]+1)):
                    obstaclePoints.append((lowerBound, higherBound))
    return obstaclePoints

def attractivePotentialGenerator(environmentX, environmentY, goalX, goalY, attractivePotential, attractiveScalingFactor):
    '''This function generates the attractive potential for the environment, considering the environment dimension'''
    for yCounter in range(0, environmentY):
        for xCounter in range(0, environmentX):
            '''We increment by one here to ensure that the outer edges of the obstacle are also included in the obstacle list'''
            attractivePotential[yCounter, xCounter] = attractiveScalingFactor*(sqrt((yCounter - goalY)**2 + (xCounter - goalX)**2))
    return attractivePotential

def reductivePotentialGenerator(environmentX, environmentY, obstaclePoints, reductivePotential, distanceFactor, reductiveScalingFactor):
    '''This function generates the reductive potentials generated due to the presence of obstacles in the environment'''
    for yCounter in range(0, environmentY):
        for xCounter in range(0, environmentX):
            '''If the current point is not in the obstacle list, continue to this section and calculate the distance of the given point from the obstacles'''
            distancesToObstacle = [sqrt((yCounter - obstaclePoint[0])**2 + (xCounter - obstaclePoint[1])**2) for obstaclePoint in obstaclePoints]
            '''Cycling through the distance from each obstacle point in the list and the given point'''
            for distanceToObstacle in distancesToObstacle:
                if(distanceToObstacle < distanceFactor):
                    '''If the distance is less than distanceFactor then calculate the reductive potential'''
                    reductivePotential[yCounter, xCounter] = reductivePotential[yCounter, xCounter] + reductiveScalingFactor*((1/(sum(distancesToObstacle)) - 1/(distanceFactor))**2)
                else:
                    '''If the given point is distanceFactor away from the obstacle, then ignore and assign reductive potential as 0'''
                    reductivePotential[yCounter, xCounter] = reductivePotential[yCounter, xCounter] + 0
    return reductivePotential

def totalPotentialGenerator(attractivePotential, reductivePotential):
    '''Adding the attractive and reductive potentials before meshing'''
    totalPotential  = attractivePotential + reductivePotential
    return totalPotential

'''Declaring the environment dimensions'''
environmentX = 30
environmentY = 30

'''Location of the goal coordinates'''
goalX = 15
goalY = 15

'''Declaring the factors for the attractive and reductive potential allocators'''
attractiveScalingFactor = 10
reductiveScalingFactor = 100
distanceFactor = 3

'''Declaring the objects here'''
objectsBoundary = [[(20, 20), (25, 25)], [(5, 5), (10, 10)]]
obstaclePoints = []
obstaclePoints = obstacleGenerator(objectsBoundary, obstaclePoints)

'''Generating the numpy arrays for meshing the environment space'''
column = np.arange(0, environmentY, 1)
row = np.arange(0, environmentX, 1)

'''Meshing the 2 1-D arrays to get the coordinate space for allocation of potentials'''
rowArray, columnArray = np.meshgrid(row, column)

'''Initializing the Attractive and Reductive Potentials of the field here'''
attractivePotential = np.zeros([environmentY, environmentX])
reductivePotential = np.zeros([environmentY, environmentX])

'''Generating the attractive potential values here'''
attractivePotential = attractivePotentialGenerator(environmentX, environmentY, goalX, goalY, attractivePotential, attractiveScalingFactor)

'''Generating the reductive potential values here'''
reductivePotential = reductivePotentialGenerator(environmentX, environmentY, obstaclePoints, reductivePotential, distanceFactor, reductiveScalingFactor)

'''Reshaping the attractive potential numpy array to the prescribed meshing'''
attractivePotentialMesh = attractivePotential.reshape(columnArray.shape)

'''Reshaping the reductive potential numpy array to the prescribed meshing'''
reductivePotentialMesh = reductivePotential.reshape(columnArray.shape)

'''Here, we combine the reductive and the attractive potentials of each coordinate in the map'''
totalPotential = totalPotentialGenerator(attractivePotential, reductivePotential)

'''Meshing the totalPotential here'''
totalPotentialMesh = totalPotential.reshape(columnArray.shape)

'''Initalizing the 3D space for projection of the potential'''
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

'''Grabbing the colormap from the Matplotlib library'''
colormap = plt.cm.get_cmap('viridis')

'''Plotting the potential function here over the 3D space'''
ax.plot_surface(rowArray, columnArray, totalPotentialMesh, cmap=colormap)

'''Labelling the axes in the 3D space'''
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')

'''Presenting the manifold generated'''
plt.show()