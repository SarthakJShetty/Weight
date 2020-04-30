import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D  
import matplotlib.pyplot as plt

def obstaclePointsGenerator(objectsBoundary, obstaclePoints):
    '''Returning all the points that lie within and on the boundary of the given object, to generate the Repulsive Potential'''
    for objectBoundary in objectsBoundary:
        for lowerBound in range(objectBoundary[0][0], (objectBoundary[1][0]+1)):
            for higherBound in range(objectBoundary[0][1], (objectBoundary[1][1]+1)):
                    obstaclePoints.append((lowerBound, higherBound))
    return obstaclePoints

def attractivePotentialGenerator(environmentX, environmentY, goalX, goalY, attractivePotential, attractiveScalingFactor):
    '''This function generates the attractive potential for the environment, considering the environment dimension'''
    for xCounter in range(0, environmentX):
        for yCounter in range(0, environmentY):
            '''We increment by one here to ensure that the outer edges of the obstacle are also included in the obstacle list'''
            attractivePotential[xCounter, yCounter] = attractiveScalingFactor*(((yCounter - goalY)**2 + (xCounter - goalX)**2)**(1/2))
    return attractivePotential

def cartesianDistance(x1, y1, x2, y2):
    '''This function returns the cartesian distance between two given points.'''
    return ((y1 - y2)**2 + (x1 - x2)**2)**(1/2)

def reductivePotentialGenerator(environmentX, environmentY, obstaclePoints, reductivePotential, distanceFactor, reductiveScalingFactor):
    '''This function generates the reductive potentials generated due to the presence of obstacles in the environment'''
    for xCounter in range(0, environmentX):
        for yCounter in range(0, environmentY):
            '''If the current point is not in the obstacle list, continue to this section and calculate the distance of the given point from the obstacles'''
            distancesToObstacle = [cartesianDistance(xCounter, yCounter, obstaclePoint[0], obstaclePoint[1]) for obstaclePoint in obstaclePoints]
            '''Cycling through the distance from each obstacle point in the list and the given point'''
            for distanceToObstacle in distancesToObstacle:
                if(distanceToObstacle < distanceFactor):
                    '''If the distance is less than distanceFactor then calculate the reductive potential'''
                    if(distanceToObstacle != 0):
                        '''If the given (yCounter, xCounter) is not in obstaclePoints, calculate the reductivepotential'''
                        reductivePotential[xCounter, yCounter] = reductivePotential[xCounter, yCounter] + reductiveScalingFactor*((1/(sum(distancesToObstacle)) - 1/(distanceFactor))**2)
                    else:
                        '''If the given (yCounter, xCounter) is in obstaclePoints, then assign the highest reductivePotential to that point'''
                        reductivePotential[xCounter, yCounter] = reductivePotential[xCounter, yCounter] + np.max(reductivePotential)
                else:
                    '''If the given point is distanceFactor away from the obstacle, then ignore and assign reductive potential as 0'''
                    reductivePotential[xCounter, yCounter] = reductivePotential[xCounter, yCounter] + 0
    return reductivePotential

def totalPotentialGenerator(attractivePotential, reductivePotential):
    '''Adding the attractive and reductive potentials before meshing'''
    totalPotential  = attractivePotential + reductivePotential
    return totalPotential

def pathGenerator(totalPotential, environmentX, environmentY, goalX, goalY, startX, startY, radiusOfConsideration, pathCoordinates, potentialPointsOfConsideration, distancePointsOfConsideration):
    '''This function generates a list of points to the goal from the start coordinates
    Our algorithm is as follows:
    1. Find the point(s) of least potential within the radiusOfConsideration
    2. Backtrack it into the potentialPointsOfConsideration dictionary and collect the corresponding (X, Y) coordinate
    3. Find the distances for each of these short-listed points.
    4. Go to the point with the with the least potential, that's farthest away.
    5*, If more than 1 are returned from 3., go to the first point, since it doesnt matter (potentials field and radiusOfConsideration circle intersect at these points).
    6. Add this point to the pathCoordinates list
    7. Iterate with new startXm startY coordinates'''
    '''We loop through until the startX and startY converge with goalX and goalY'''
    while ((startX != goalX) or (startY != goalY)):
        startXValue = startX - radiusOfConsideration
        endXValue = startX + radiusOfConsideration
        startYValue = startY - radiusOfConsideration
        endYValue = startY + radiusOfConsideration
        '''In some cases, the bounding box used to grab the surrounding potentials may lie outside the environment, where points have null potentials.
        To ensure that the bounding box always lies within the operating environment, we put in place these checks, to limit the spillage to null potentials.'''
        if(startXValue < 0):
            startXValue = 0
        else:
            startXValue = startXValue
        if(endXValue > environmentX):
            endXValue = environmentX
        else:
            endXValue = endXValue
        if(startYValue < 0):
            startYValue = 0
        else:
            startYValue = startYValue
        if(endYValue > environmentY):
            endYValue = environmentY
        else:
            endYValue = endYValue
        '''We design a box of radius radiusOfConsideration around the given start(X, Y) coordinates. We use this space for determining the point with the least potential to move to.'''
        for xCounter in range(startXValue, endXValue):
            for yCounter in range(startYValue, endYValue):
                '''We use dictionaries to ensure traceability from a given potential/distance value to it's key (X, Y) which is in the form of a tuple key in the dictionary
                We create two dictionaries: 1. To note the potential the points being considered, 2. To note the distance of a point (X, Y) within the radius of consideration.'''
                potentialPointsOfConsideration[(xCounter, yCounter)] = totalPotential[xCounter, yCounter]
                distancePointsOfConsideration[(xCounter, yCounter)] = cartesianDistance(startX, startY, xCounter, yCounter)
        '''We find the minimumPotential here and grab the corresponding coordinate'''
        minimumPotential = min([potential for point, potential in potentialPointsOfConsideration.items()])
        minimumPotentialPoints = [point for point, potential in potentialPointsOfConsideration.items() if potential == minimumPotential]
        '''We grab the coordinates corresponding to the least potentials within the radiusOfConsideration'''
        maximumDistance = max([distancePointsOfConsideration[minimumPotentialPoint] for minimumPotentialPoint in minimumPotentialPoints])
        maximumDistancePoints = [point for point, distance in distancePointsOfConsideration.items() if distance == maximumDistance and point in minimumPotentialPoints]
        '''Here, we update start(X, Y) to the new point generated from the preceeding code'''
        startX = maximumDistancePoints[0][0]
        startY = maximumDistancePoints[0][1]
        '''At the end of each iteration we append the pathCoordinates with the new coordinate'''
        pathCoordinates.append((startX, startY))
    return pathCoordinates

'''Adding a variable here to switch between contour plotting and surface'''
contourPlotting = True

'''Declaring the environment dimensions'''
environmentX = 30
environmentY = 25

'''Location of the goal coordinates'''
goalX = 20
goalY = 20

'''Start location that has to be eventually routed to the goal(X, Y)'''
startX = 5
startY = 5

'''Declaring a dictionary to hold the coordinates and their corresponding distances while generating the path to the goal'''
potentialPointsOfConsideration = {}
distancePointsOfConsideration = {}

'''pathCoordinates holds the path charted from start(X, Y) to goal(X, Y)'''
pathCoordinates = []

'''We append the first set of start(X, Y) coordinates here, before the routing begins'''
pathCoordinates.append((startX, startY))

'''Declaring a radius of consideration to identify the lowest potential coordinate in the neighbourhood of the start(X, Y)'''
radiusOfConsideration = 2

'''Declaring the factors for the attractive and reductive potential allocators'''
attractiveScalingFactor = 1
reductiveScalingFactor = 50
distanceFactor = 2

'''Declaring the point objects here'''
objectsBoundary = [[(10, 10), (10, 10)]]
obstaclePoints = []
obstaclePoints = obstaclePointsGenerator(objectsBoundary, obstaclePoints)

'''Generating the numpy arrays for meshing the environment space'''
column = np.arange(0, environmentY, 1)
row = np.arange(0, environmentX, 1)

'''Meshing the 2 1-D arrays to get the coordinate space for allocation of potentials'''
columnArray, rowArray = np.meshgrid(column, row)

'''Initializing the Attractive and Reductive Potentials of the field here'''
attractivePotential = np.zeros([environmentX, environmentY])
reductivePotential = np.zeros([environmentX, environmentY])

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

'''The projection parameter is required only for plot_surface'''
if contourPlotting:
    ax = fig.add_subplot(111)
else:
    ax = fig.add_subplot(111, projection='3d')

'''Grabbing the colormap from the Matplotlib library'''
colormap = plt.cm.get_cmap('viridis')

'''Increasing the number of contours in the plot'''
contourLevels = np.arange(np.min(totalPotential), np.max(totalPotential), 4)

'''Plotting the potential function here over the 3D space'''
if contourPlotting:
    ax.contour(rowArray, columnArray, totalPotentialMesh, cmap=colormap, levels=contourLevels)
else:
    ax.plot_surface(rowArray, columnArray, totalPotentialMesh, cmap=colormap)

'''Labelling the axes in the 3D space'''
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')

'''Z label is required only for surface plots'''
if not contourPlotting:
    ax.set_zlabel('Z Axis')

'''Repurposing the colormap used to plot the surface to generate the colorbar'''
colorbar = plt.cm.ScalarMappable(cmap = colormap)
colorbar.set_array(totalPotential)
colorbar = plt.colorbar(colorbar)
colorbar.set_label('Increasing Potential')

'''Labelling the figure generated'''
plt.title('Potential Distribution Across ' + str(environmentX) + ' x '+ str(environmentY)+ ' Environment Space')

'''Setting the default aspect ratio as equal'''
ax.set_aspect('equal')

'''Adding referential grids to the figure to better figure out contour plots'''
plt.grid(True)

'''This line will be used while generating contour plots to clearly mark point obstacles'''
pathCoordinates = pathGenerator(totalPotential, environmentX, environmentY, goalX, goalY, startX, startY, radiusOfConsideration, pathCoordinates, potentialPointsOfConsideration, distancePointsOfConsideration)

'''Here, we scatter the obstacle for the countour plot. We use scatter instead of plot because we don't want the line connecting the two.'''
if contourPlotting:
    plt.scatter([[obstaclePoint[0]] for obstaclePoint in obstaclePoints], [[obstaclePoint[1]] for obstaclePoint in obstaclePoints])

'''Here, we plot the pathCoordinates from the pathGenerator'''
if contourPlotting:
    '''For contours we do not use the third dimension'''
    plt.plot([[pathCoordinate[0]] for pathCoordinate in pathCoordinates], [[pathCoordinate[1]] for pathCoordinate in pathCoordinates], markersize=5, markerfacecolor='r', markeredgecolor='r', marker='o')
else:
    '''For surface plots we use totalPotential to plot the points'''
    plt.plot([[pathCoordinate[0]] for pathCoordinate in pathCoordinates], [[pathCoordinate[1]] for pathCoordinate in pathCoordinates], [totalPotential[pathCoordinate[0], pathCoordinate[1]] for pathCoordinate in pathCoordinates], markerfacecolor='r', markeredgecolor='r', marker='o', markersize=10, alpha=1)

'''Presenting the manifold generated'''
plt.show()