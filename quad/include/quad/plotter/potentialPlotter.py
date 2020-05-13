import numpy as np
import pandas as pd
from math import sqrt
from mpl_toolkits.mplot3d import Axes3D  
import matplotlib.pyplot as plt

def cartesianDistance(x1, y1, x2, y2):
    '''This function returns the cartesian distance between two given points.'''
    return ((y1 - y2)**2 + (x1 - x2)**2)**(1/2)

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
            attractivePotential[xCounter, yCounter] = attractiveScalingFactor*(cartesianDistance(xCounter, yCounter, goalX, goalY))
    return attractivePotential

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

def attractiveElectricPotentialGenerator(environmentX, environmentY, goalX, goalY, portCharge, attractivePotential):
    '''This function generates the attractive potentials generated due to the presence of the port. It's assumed to be a point charge'''
    for xCounter in range(0, environmentX):
        for yCounter in range(0, environmentY):
            if((xCounter, yCounter)!=(goalX, goalY)):
                '''Standard formula for the electric potential due to a point charge V = (K) * (Q/r)'''
                attractivePotential[xCounter, yCounter] = -1 * portCharge/(cartesianDistance(xCounter, yCounter, goalX, goalY))
    '''To prevent the division by zero at the location of the port, we assign the least value - 1 potential to the port'''
    attractivePotential[goalX, goalY] = attractivePotential.min() - 1
    return attractivePotential

def reductiveElectricPotentialGenerator(environmentX, environmentY, goalX, goalY, shipCharge, obstaclePoints, reductivePotential):
    '''This function creates the reductive potentials generated due to the presence of ships. They are assumed to be unit point charges.'''
    for obstaclePoint in obstaclePoints:
        for xCounter in range(0, environmentX):
            for yCounter in range(0, environmentY):
                if((xCounter, yCounter)!=(obstaclePoint[0], obstaclePoint[1])):
                    '''Standard formula for the electric potential due to a point charge V = (K) * (Q/r)'''
                    reductivePotential[xCounter, yCounter] = reductivePotential[xCounter, yCounter] + shipCharge/(cartesianDistance(xCounter, yCounter, obstaclePoint[0], obstaclePoint[1]))
    for obstaclePoint in obstaclePoints:
        '''To prevent the division by zero at the location of the ships, we calulcate the maximum value encountered in the reductivePotential and assign them one more that value.''' 
        reductivePotential[obstaclePoint[0], obstaclePoint[1]] = reductivePotential.max() + 1
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

def timeForPathTraversal(pathCoordinates, startVelocity, finalVelocity, acceleration, timeTaken, totalTime):
    '''This function calculates the total time taken to traverse the points from start(X, Y) to goal(X, Y), generated by pathGenerator()'''
    '''We loop to only (N-1)th point since the last point would not have any (N+1)th point, and planning should terminate there'''
    for pathCoordinateIndex in range(0, len(pathCoordinates)-1):
        '''Using the equation of One Dimension motion here to find finalVelocity: v**2 = u**2 + 2*a*s'''
        finalVelocity = sqrt(startVelocity**2 + 2*acceleration*cartesianDistance(pathCoordinates[pathCoordinateIndex][0], pathCoordinates[pathCoordinateIndex][1], pathCoordinates[pathCoordinateIndex+1][0], pathCoordinates[pathCoordinateIndex+1][1]))
        '''Using the equation of One Dimension motion here to find timeTaken: v = u + a*t'''
        timeTaken = (finalVelocity - startVelocity)/(acceleration)
        '''The first start velocity will be 0 considering start from stationary point. Reassigning to the previous finalVelocity here.'''
        startVelocity = finalVelocity
        '''Adding the time taken to traverse each rectilinear unit here.'''
        totalTime += timeTaken
    return totalTime

'''Adding a variable here to switch between contour plotting and surface'''
contourPlotting = True

'''Declaring the environment dimensions'''
environmentX = 30
environmentY = 25

'''Location of the goal coordinates'''
goalX = 20
goalY = 20

'''Port is assumed to be negatively charged.'''
portCharge = 50
'''Ship is assumed to be unit positvely charged.'''
shipCharge = 1

'''Variables for the motion code added here. Agent starts from stationary condition hence, startVelocity is 0 unit per second.'''
startVelocity = 0
'''Initial finalVelocity assumed to be 0 units per second'''
finalVelocity = 0
'''timeTaken by the agent to traverse each length between consecutive waypoints. Added to the totalTime variable.'''
timeTaken = 0
'''totalTime taken to traverse the pathCoordinates'''
totalTime = 0
'''Assuming the acceleration to be 1 unit per second per second'''
acceleration = 1

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
objectsBoundary = [[(5, 5), (5, 5)], [(9, 9), (9, 9)], [(15, 17), (15, 17)]]
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
# attractivePotential = attractivePotentialGenerator(environmentX, environmentY, goalX, goalY, attractivePotential, attractiveScalingFactor)
attractivePotential = attractiveElectricPotentialGenerator(environmentX, environmentY, goalX, goalY, portCharge, attractivePotential)

'''Generating the reductive potential values here'''
# reductivePotential = reductivePotentialGenerator(environmentX, environmentY, obstaclePoints, reductivePotential, distanceFactor, reductiveScalingFactor)
reductivePotential = reductiveElectricPotentialGenerator(environmentX, environmentY, goalX, goalY, shipCharge, obstaclePoints, reductivePotential)

'''Reshaping the attractive potential numpy array to the prescribed meshing'''
attractivePotentialMesh = attractivePotential.reshape(columnArray.shape)

'''Reshaping the reductive potential numpy array to the prescribed meshing'''
reductivePotentialMesh = reductivePotential.reshape(columnArray.shape)

'''Here, we combine the reductive and the attractive potentials of each coordinate in the map'''
totalPotential = totalPotentialGenerator(attractivePotential, reductivePotential)

'''Meshing the totalPotential here'''
totalPotentialMesh = totalPotential.reshape(columnArray.shape)

'''This line will be used while generating contour plots to clearly mark point obstacles'''
pathCoordinates = pathGenerator(totalPotential, environmentX, environmentY, goalX, goalY, startX, startY, radiusOfConsideration, pathCoordinates, potentialPointsOfConsideration, distancePointsOfConsideration)

'''This function generates the time taken by the agent to traverse the pathCoordinates'''
timeTaken = timeForPathTraversal(pathCoordinates, startVelocity, finalVelocity, acceleration, timeTaken, totalTime)

'''Initalizing the 3D space for projection of the potential'''
fig = plt.figure()

'''The projection parameter is required only for plot_surface'''
if contourPlotting:
    ax = fig.add_subplot(111)
else:
    ax = fig.add_subplot(111, projection='3d')

'''Grabbing the colormap from the Matplotlib library'''
colormap = plt.cm.get_cmap('viridis')

'''Reintroducing the contourLevels variable here to improve readability of the contour plots'''
contourLevels = np.arange(np.min(totalPotential), np.max(totalPotential), 1)

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
plt.title('Potential Distribution Across ' + str(environmentX) + ' x '+ str(environmentY)+ ' Environment Space\n' + 'Time Taken: '+str(format(timeTaken, '0.3f')) + ' seconds')

'''Setting the default aspect ratio as equal'''
ax.set_aspect('equal')

'''Adding referential grids to the figure to better figure out contour plots'''
plt.grid(True)

'''Here, we scatter the obstacle for the countour plot. We use scatter instead of plot because we don't want the line connecting the two.'''
if contourPlotting:
    plt.scatter([[obstaclePoint[0]] for obstaclePoint in obstaclePoints], [[obstaclePoint[1]] for obstaclePoint in obstaclePoints])

'''Here, we plot the pathCoordinates from the pathGenerator'''
if contourPlotting:
    '''For contours we do not use the third dimension'''
    plt.plot([[pathCoordinate[0]] for pathCoordinate in pathCoordinates], [[pathCoordinate[1]] for pathCoordinate in pathCoordinates], markersize=5, markerfacecolor='r', markeredgecolor='r', marker='o')
else:
    '''For surface plots we use totalPotential to plot the points'''
    plt.plot([[pathCoordinate[0]] for pathCoordinate in pathCoordinates], [[pathCoordinate[1]] for pathCoordinate in pathCoordinates], [totalPotential[pathCoordinate[0], pathCoordinate[1]] for pathCoordinate in pathCoordinates], markerfacecolor='r', markeredgecolor='r', marker='o', markersize=5, alpha=1)

'''Presenting the manifold generated'''
plt.show()