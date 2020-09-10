import math
import random
import os, sys, csv
# import ps11_visualize
import numpy as np
from skopt import Optimizer
import pandas as pd
from skopt.space import Real, Integer
import skopt
from skopt import gp_minimize
from skopt.utils import use_named_args
from skopt import callbacks
from skopt.callbacks import CheckpointSaver
from skopt import load
# from pylab import plot, axis, title, ylabel, xlabel, show
import csv
from csv import writer
from Sim import ps11_visualize
from Sim import Dubins
from Sim import Greedy
import gc
from datetime import datetime
import tracemalloc
import linecache
from pulp import *
import time
import matplotlib.pyplot as plt
import matplotlib
from skopt.plots import plot_objective
from skopt.plots import plot_evaluations
from mpl_toolkits import mplot3d

from skopt.plots import plot_convergence

class Position(object):
    """
    A Position represents a location in a two-dimensional room.
    """
 
    def __init__(self, x, y, heading):
        """
        Initializes a position with coordinates (x, y).

        x: a real number indicating the x-coordinate
        y: a real number indicating the y-coordinate
        """
        self.x = x #int(x)
        self.y = y #int(y)
        self.heading = heading
        self.turningRadius = 0.02

    def getX(self):
        # print(self.x)
        return self.x

    def getY(self):
        # print(self.y)
        return self.y

    def getHeading(self):
        return self.heading

    def getPos(self):
        return int(self.x), int(self.y)

    def setHeading(self, angle):
        self.heading = angle

    def setPosition(self, x, y, angle):
        return Position(x, y, angle)


    # def getRightTurn(self, angle, speed, deltaTime=1):
    #
    #     angular_speed = (speed / self.turningRadius) * -1
    #     old_x, old_y = self.getX(), self.getY()
    #     phi_s = math.radians(angle)
    #     new_x = old_x + self.turningRadius * math.sin(phi_s) + self.turningRadius * math.sin(angular_speed *
    #                                                                                          deltaTime - phi_s)
    #     new_y = old_y + self.turningRadius * math.cos(phi_s) - self.turningRadius * math.cos(angular_speed * deltaTime - phi_s)
    #     new_heading = math.degrees(phi_s - angular_speed * deltaTime)
    #     return Position(new_x, new_y, new_heading)
    #
    # def getLeftTurn(self, angle, speed, deltaTime=1):
    #
    #     angular_speed = speed/ self.turningRadius
    #     old_x, old_y = self.getX(), self.getY()
    #     phi_s = math.radians(angle)
    #     new_x = old_x - self.turningRadius * math.sin(phi_s) + self.turningRadius * \
    #             math.sin(angular_speed*deltaTime + phi_s)
    #     new_y = old_y - self.turningRadius * math.cos(phi_s) + self.turningRadius * \
    #             math.cos(angular_speed*deltaTime + phi_s)
    #     new_heading = math.degrees(phi_s + angular_speed * deltaTime)
    #     return Position(new_x, new_y, new_heading)
    #
    #
    # def getStraightMove(self, angle, speed):
    #     old_x, old_y = self.getX(), self.getY()
    #     delta_y = speed * math.sin(math.radians(angle))
    #     delta_x = speed * math.cos(math.radians(angle))
    #     new_x = old_x + delta_x
    #     new_y = old_y + delta_y
    #     return Position(new_x, new_y, angle)

    def getConstrainedRandomAngle(self, constraint):
        return random.randrange(-constraint, constraint)

    # def getConstrainedRandomPosition(self, pos, loc_constraint, angle_constraint):
    #     x = int(pos.getX())
    #     y = int(pos.getY())
    #     angle = pos.getHeading()
    #     new_x = random.randint(max(x - loc_constraint, 0), min(x + loc_constraint, 600))
    #     new_y = random.randint(max(y - loc_constraint, 0), min(y + loc_constraint, 600))
    #     new_angle = -(math.atan2(y - new_y, x - new_x))
    #     return Position(new_x, new_y, new_angle)

    def getConstrainedRandomPosition(self, pos, loc_constraint, angle_constraint):
        x = int(pos.getX())
        y = int(pos.getY())
        angle = pos.getHeading()
        angle = (math.degrees(angle))
        random_angle = random.uniform(angle-angle_constraint, angle+angle_constraint)
        random_angle = math.radians(random_angle)
        new_x = x + (loc_constraint * math.cos(random_angle))
        new_y = y + (loc_constraint * math.sin(random_angle))
        new_angle = math.atan2(new_y - y, new_x - x)
        # print(f'old x: {x}, new x: {new_x}'
        #       f'\n old y: {y}, new y: {new_y}'
        #       f'\n old angle: {angle}, new angle: {math.degrees(new_angle)}')
        return Position(new_x, new_y, new_angle)



class RectangularRoom(object):
    """
    A RectangularRoom represents a rectangular region containing clean or dirty
    dredge_tiles.

    A room has a width and a height and contains (width * height) dredge_tiles. At any
    particular time1, each of these dredge_tiles is either clean or dirty.
    """

    def __init__(self, width, height):
        """
        Initializes a rectangular room with the specified width and height.
        Initially, no dredge_tiles in the room have been cleaned.

        width: an integer > 0
        height: an integer > 0
        """
        self.roomWidth = width
        self.roomHeight = height
        self.dredgeAreaHeight = int(height/2)
        self.dredgeAreaWidth = int(width/2)
        self.dredgeArea = []
        self.dredgeMatrix = np.zeros((self.dredgeAreaWidth, self.dredgeAreaHeight))
        self.resultantEnvForce = 2
        self.dumpLoc = (600, 0)
        self.dredgePerimeter = []
        self.closePerimeter = []

    def createDredgingLocations(self):
        """
        Creates a basic rectangular dredging area based on the size of the two variables:
         dredgeAreaWidth & dredgeAreaHeight
        """
        for m in range(self.dredgeAreaWidth):
            for n in range(self.dredgeAreaHeight):
                self.dredgeArea.append((m+self.dredgeAreaWidth, n+self.dredgeAreaHeight))

    def dredgeTileAtPosition(self, pos):
        """
        Mark the tile under the position POS as cleaned.
        Assumes that POS represents a valid position inside this room.
        Assumes hold is not full

        pos: a Position
        """
        # Convert postion to integer, add the position to the dictionary of clean positions; increment if necessary
        intPosition = (int(pos.getX())-1, int(pos.getY())-1)
        self.dredgeMatrix[intPosition[0]-(self.dredgeAreaHeight), intPosition[1]-(self.dredgeAreaHeight)] += 1

    def dredgeTileAtPosition(self, x, y):

        intPosition = (int(x)-1, int(y)-1)
        self.dredgeMatrix[intPosition[0]-(self.dredgeAreaHeight), intPosition[1]-(self.dredgeAreaHeight)] += 1

    # def checkDredged(self):
    #     boolMap = np.where(self.dredgeMatrix > 1)
    #     ohShite = np.sum(self.dredgeMatrix[boolMap])
    #     if ohShite > 0:
    #         print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")


    def isTileDredgable(self, pos):
        if not self.isPosInDredgeArea(pos.getX, pos.getY):
            return False
        if self.dredgeMatrix[int(pos.getX)-1-(self.dredgeAreaHeight),int(pos.getY)-1-(self.dredgeAreaHeight)] >= 1:
            return False
        return True

    def isTileDredgable(self, x, y):
        if not self.isPosInDredgeArea(x, y):
            return False
        if self.dredgeMatrix[int(x)-1-(self.dredgeAreaHeight), int(y)-1-(self.dredgeAreaHeight)] >= 1:
            return False
        return True

    def getNumTiles(self):

        return self.dredgeAreaHeight * self.dredgeAreaWidth

    def getNumCleanedTiles(self):
        """
        Return the total number of clean dredge_tiles in the room.

        returns: an integer
        """

        # boolMap = np.where(self.dredgeMatrix >= 1)
        count = 0
        for x in np.nditer(self.dredgeMatrix):
            if x > 0:
                count += 1
        return count

    def getRandomDredgePosition(self):
        """
        Return a random position inside the room.

        returns: a Position object.
        """
        # generate random numbers between 0 and width or height inclusive
        randomWidth = random.randint(self.dredgeAreaWidth, self.roomWidth)
        randomHeight = random.randint(self.dredgeAreaHeight, self.roomHeight)
        randomAngle = random.randint(0, 360)
        randomAngle = math.radians(randomAngle)
        return Position(randomWidth, randomHeight, randomAngle)

    def getinformedPosition(self, prevDredged, pos):
        tmp = np.where(prevDredged == np.amin(prevDredged))
        try:
            coord = [co for co in zip(tmp[0], tmp[1])]
            ind = random.randint(0, len(coord) - 1)
            new_angle = math.atan2(coord[ind][1] - pos.getY(), coord[ind][0] - pos.getX())
            return Position((coord[ind][0] + 300), (300 + coord[ind][1]), new_angle)
        except IndexError:
            print(f'tmp values: {tmp}')
            print(f'tmp size: {tmp.size}')
            print(f'tmp chosen y: {tmp[0][1]} and x:{tmp[0][0]}')
            if random.random > 0.5:
                new_angle = math.atan2(tmp[0][1] - pos.getY(), tmp[0][0] - pos.getX())
                return Position((tmp[0][0] + 300), (300 + tmp[0][1]), new_angle)
            else:
                new_angle = math.atan2(tmp[-1][1] - pos.getY(), tmp[-1][0] - pos.getX())
                return Position((tmp[-1][0] + 300), (300 + tmp[-1][1]), new_angle)


    def getRandomPosition(self):
        """
        Return a random position inside the room.

        returns: a Position object.
        """
        # generate random numbers between 0 and width or height inclusive
        randomWidth = random.randint(0, self.roomWidth - 1)
        randomHeight = random.randint(0, self.roomHeight - 1)
        randomAngle = random.randint(0, 360)
        return Position(randomWidth, randomHeight, randomAngle)

    def getRandomPosSetAngle1(self, pos, distance):
        x = int(pos.getX())
        y = int(pos.getY())
        new_x = random.randint(300, 600-distance)
        new_y = random.randint(300, 550)
        new_angle = math.atan2(new_y-y, new_x - x)
        return Position(new_x, new_y, new_angle)

    def getPerimeterPoint(self, distance):
        y = 300
        x = random.randint(300,(600-distance))
        angle = math.radians(90)
        return Position(x, y, angle)

    def getPerimeterPointReturn(self, pos):
        y = 300
        x = int(pos.getX())
        angle = math.radians(270)
        return Position(x, y, angle)

    def getbaselinePoint1(self, pos):
        x = int(pos.getX())
        y = int(pos.getY())
        angle = math.radians(90)
        new_y = y + random.randint(100,250)
        return Position(x, new_y, angle)

    def getbaselinePoint2(self, pos, distance):
        x = int(pos.getX())
        y = int(pos.getY())
        new_x = x+distance
        new_y = y
        new_angle = math.radians(270)
        return Position(new_x, new_y, new_angle)

    # def getRandomPosSetAngle2(self, pos, distance):
    #     x = int(pos.getX())
    #     y = int(pos.getY())
    #     new_x = x+distance
    #     new_y = y
    #     new_angle = math.atan2(new_y-self.dumpLocBaseline[1], new_x - self.dumpLocBaseline[0])
    #     return Position(new_x, new_y, new_angle)

    def isPositionInRoom(self, pos):
        """
        Return True if POS is inside the room.

        pos: a Position object.
        returns: True if POS is in the room, False otherwise.
        """
        # if Pos X or Y are greater than or eqaul to width of room they are outside the room.
        # What if X or Y are negative?
        if pos.getX() < 0: return False
        if pos.getY() < 0: return False

        if pos.getX() >= self.roomWidth: return False
        if pos.getY() >= self.roomHeight: return False
        return True

    def isPosInDredgeArea(self, x, y):
        if int(x) < self.dredgeAreaWidth: return False
        if int(y) < self.dredgeAreaHeight: return False

        if int(x) >= self.roomWidth: return False
        if int(y) >= self.roomHeight: return False

    # def getPerimeter(self):
    #     for i in range(self.dredgeAreaHeight):
    #         self.dredgePerimeter.append((0,i))
    #         self.closePerimeter.append((0,i))
    #         self.dredgePerimeter.append((self.dredgeAreaHeight, i))
    #         self.closePerimeter.append((self.dredgeAreaHeight, i))
    #         self.dredgePerimeter.append((i, self.dredgeAreaHeight))
    #         self.dredgePerimeter.append((i, 0))
        return True

class BaseShip(object):
    """
    Represents a robot cleaning a particular room.

    At all times the robot has a particular position and direction in
    the room.  The robot also has a fixed speed.

    Subclasses of BaseRobot should provide movement strategies by
    implementing updatePositionAndClean(), which simulates a single
    time1-step.
    """

    def __init__(self, room, speed, waypointSeperation,
        numOfWaypoints):
        """
        Initializes a Robot with the given speed in the specified
        room. The robot initially has a random direction d and a
        random position p in the room.

        The direction d is an integer satisfying 0 <= d < 360; it
        specifies an angle in degrees.

        p is a Position object giving the robot's position.

        room:  a RectangularRoom object.
        speed: a float (speed > 0)
        """
        self.robotSpeed = speed
        self.robotRoom = room
        self.robotPosition = Position(self.robotRoom.roomWidth, 0, math.radians(115))
        self.holdCapacity = 583 #based on a dredge track length and cell size (3500m/6m=583.333)
        self.hold = 0
        self.path = []
        self.pathDredgePercentage = []
        self.isPathPlanned = False
        self.environmentForce = 1
        self.environmentForceDirection = 0
        #parameters
        self.waypointSeperation = waypointSeperation
        self.numOfWaypoints = numOfWaypoints

    def getRobotPosition(self):
        """
        Return the position of the robot.

        returns: a Position object giving the robot's position.
        """

        # print "self.robotPosition is %s" %self.robotPosition
        return self.robotPosition


    def setRobotDirection(self, direction):
        """
        Set the direction of the robot to DIRECTION.

        direction: integer representing an angle in degrees
        """
        self.robotDirection = direction

    def isHoldFull(self):
        """
        return True if hold is less than max capacity
        return False if hold has reached max capacity
        hold: amount of dirt ship has collected

        """
        if self.hold >= self.holdCapacity:
            return True
        return False
class baseline(BaseShip):
    def perimeterPoint(self, room, dis):
        currentPosition = Position(room.dumpLoc[0], room.dumpLoc[1], math.radians(90))
        perimeterPos = room.getPerimeterPoint(dis)
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), perimeterPos.getX(),
                                              perimeterPos.getY(), perimeterPos.getHeading(),
                                              currentPosition.turningRadius)
        gc.collect()
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        self.robotPosition = newPos
        return dx, dy, dangle

    def firstDredgePoint(self, room, dis):
        currentPosition = self.getRobotPosition()
        perimeterPos = room.getbaselinePoint1(currentPosition)
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), perimeterPos.getX(),
                                              perimeterPos.getY(), perimeterPos.getHeading(),
                                              currentPosition.turningRadius)
        gc.collect()
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        self.robotPosition = newPos
        return dx, dy, dangle

    def secoundDredgePoint(self, room, dis):
        currentPosition = self.getRobotPosition()
        randomPos = room.getbaselinePoint2(currentPosition, dis)
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), randomPos.getX(),
                                              randomPos.getY(), randomPos.getHeading(),
                                              currentPosition.turningRadius)
        gc.collect()
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        self.robotPosition = newPos
        return dx, dy, dangle

    def perimeterPointReturn(self, room, dis):
        currentPosition = self.getRobotPosition()
        perimeterPos = room.getPerimeterPointReturn(currentPosition)
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), perimeterPos.getX(),
                                              perimeterPos.getY(), perimeterPos.getHeading(),
                                              currentPosition.turningRadius)
        gc.collect()
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        self.robotPosition = newPos
        return dx, dy, dangle

    def toDump(self):
        dump_x, dump_y = self.robotRoom.dumpLoc
        currentPosition = self.getRobotPosition()
        # endAngle = math.radians(270)
        endAngle = math.atan2(dump_y-currentPosition.getY(), dump_x - currentPosition.getX())
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), dump_x, dump_y, endAngle,
                                              currentPosition.turningRadius)
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        self.robotPosition = newPos
        self.hold = 0
        return dx, dy, dangle

class setDistanceWapoint(BaseShip):
    def firstDredgeRoute(self, room):
        currentPosition = self.getRobotPosition()
        randomPos = room.getRandomDredgePosition()
        endAngle = math.atan2(randomPos.getY() - currentPosition.getY(), randomPos.getX() - currentPosition.getX())
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              endAngle, randomPos.getX(),
                                              randomPos.getY(), randomPos.getHeading(),
                                              currentPosition.turningRadius)
        gc.collect()
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        self.robotPosition = newPos
        return dx, dy, dangle

    def firstDredgeRouteEven(self, room, recuringMatrix):
        currentPosition = self.getRobotPosition()
        evenPos = room.getinformedPosition(recuringMatrix, currentPosition)
        endAngle = math.atan2(evenPos.getY() - currentPosition.getY(), evenPos.getX() - currentPosition.getX())
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              endAngle, evenPos.getX(),
                                              evenPos.getY(), evenPos.getHeading(),
                                              currentPosition.turningRadius)
        gc.collect()
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        self.robotPosition = newPos
        return dx, dy, dangle

    def dredgeRoute(self, angleConstraint):
        currentPosition = self.getRobotPosition()
        constrainedPos = currentPosition.getConstrainedRandomPosition(currentPosition, self.waypointSeperation, angleConstraint)
        while int(currentPosition.getX()) == int(constrainedPos.getX()) and \
                int(currentPosition.getY()) == int(constrainedPos.getY()):
            constrainedPos = currentPosition.getConstrainedRandomPosition(currentPosition, self.waypointSeperation)
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), constrainedPos.getX(),
                                              constrainedPos.getY(), constrainedPos.getHeading(),
                                              currentPosition.turningRadius)
        gc.collect()
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        # print(f'x: {dx[-1]}, y: {dy[-1]}, angle: {math.degrees(dangle[-1])}')
        self.robotPosition = newPos
        return dx, dy, dangle

    def dredgeRouteEven(self, room, recuringMatrix):
        currentPosition = self.getRobotPosition()
        constrainedPos = room.getinformedPosition(recuringMatrix, currentPosition)
        while int(currentPosition.getX()) == int(constrainedPos.getX()) and \
                int(currentPosition.getY()) == int(constrainedPos.getY()):
            constrainedPos = room.getinformedPosition(currentPosition, self.waypointSeperation)
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), constrainedPos.getX(),
                                              constrainedPos.getY(), constrainedPos.getHeading(),
                                              currentPosition.turningRadius)
        gc.collect()
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])

        self.robotPosition = newPos
        return dx, dy, dangle

    def toDump(self):
        dump_x, dump_y = self.robotRoom.dumpLoc
        currentPosition = self.getRobotPosition()
        endAngle = (math.atan2(dump_y - currentPosition.getY(), dump_x - currentPosition.getX()))
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), dump_x, dump_y, endAngle,
                                              currentPosition.turningRadius)
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        self.robotPosition = newPos
        self.hold = 0
        return dx, dy, dangle

# class evenDistribution(BaseShip):
#
#     def firstDredgeRoute(self, room, recuringMatrix):
#         currentPosition = self.getRobotPosition()
#         evenPos = room.getinformedPosition(recuringMatrix, currentPosition)
#         dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
#                                               currentPosition.getHeading(), evenPos.getX(),
#                                               evenPos.getY(),evenPos.getHeading(),
#                                               currentPosition.turningRadius)
#         gc.collect()
#         newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
#         self.robotPosition = newPos
#         return dx, dy, dangle
#
#
#     def dredgeRoute(self, room, recuringMatrix):
#         currentPosition = self.getRobotPosition()
#         constrainedPos = room.getinformedPosition(recuringMatrix, currentPosition)
#         while int(currentPosition.getX()) == int(constrainedPos.getX()) and \
#                 int(currentPosition.getY()) == int(constrainedPos.getY()):
#             constrainedPos = currentPosition.getConstrainedRandomPosition(currentPosition, self.waypointSeperation)
#             # print("it actually happened")
#         # try:
#         dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
#                                               currentPosition.getHeading(), constrainedPos.getX(),
#                                               constrainedPos.getY(), constrainedPos.getHeading(),
#                                               currentPosition.turningRadius)
#         gc.collect()
#         # except:
#         #     MemoryError
#         #     print(f'length of one dangle: { len(px)}')
#         # if not self.isHoldFull(self.hold):
#         #     for i, tmp in enumerate(dx):
#         #         if self.robotRoom.isTileDredgable(tmp, dy[i]) and not self.robotRoom.isTileCleaned(tmp, dy[i]):
#         #             print("dredgeRoute")
#         #             print(tmp, dy[i])
#         #             self.robotRoom.dredgeTileAtPosition(tmp, dy[i])
#         #             self.hold += 1
#         # self.path.append([px, py, pangle])
#
#         # try:
#         newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
#         # except IndexError:
#         #     # print(f'current: {currentPosition.getPos()} \n'
#         #     #       f'planned: {constrainedPos.getPos()}')
#         #     print(f'currentPosition.getX(): {currentPosition.getX()} \n'
#         #           f'currentPosition.getY(): {currentPosition.getY()} \n'
#         #           f'currentPosition.getHeading(): {currentPosition.getHeading()} \n'
#         #           f'constrainedPos.getX(): {constrainedPos.getX()} \n'
#         #           f'constrainedPos.getY(): {constrainedPos.getY()} \n'
#         #           f'constrainedPos.getHeading(): {constrainedPos.getHeading()}')
#         #     if len(dx) == 0: newPos = currentPosition.setPosition(constrainedPos.getX(), dy[-1], dangle[-1])
#         #     elif len(dy) == 0: newPos = currentPosition.setPosition(dx[-1], constrainedPos.getY(), dangle[-1])
#         self.robotPosition = newPos
#         return dx, dy, dangle
#
#     # def currentToPerimeter(self):
#     #     dump_x, dump_y = self.robotRoom.dumpLoc
#     #     currentPosition = self.getRobotPosition()
#     #     perimeterIndex = random.randint(0, (len(self.robotRoom.closePerimeter) - 1))
#     #     perimeter_x, perimeter_y = self.robotRoom.closePerimeter[perimeterIndex]
#     #     endAngle = (math.atan2(currentPosition.getX() - dump_x, currentPosition.getY() - dump_y))
#     #     px, py, pangle = Dubins.getDubinsPath(currentPosition.getX(), currentPosition.getY(),
#     #                                           currentPosition.getHeading(), perimeter_x, perimeter_y, endAngle,
#     #                                           currentPosition.turningRadius)
#     #     print(len(px))
#     #     if not self.isHoldFull(self.hold):
#     #         for i, tmp in enumerate(px):
#     #             if self.robotRoom.isTileDredgable and not self.robotRoom.isTileCleaned(tmp, py[i]):
#     #                 self.robotRoom.dredgeTileAtPosition(tmp, py[i])
#     #                 self.hold += 1
#     #     # self.path.append([px, py, pangle])
#     #     newPos = currentPosition.setPosition(px[-1], py[-1], pangle[-1])
#     #     self.robotPosition = newPos
#     #     return px, py, pangle
#
#     def toDump(self):
#         dump_x, dump_y = self.robotRoom.dumpLoc
#         currentPosition = self.getRobotPosition()
#         endAngle = -(math.atan2(currentPosition.getY() - dump_y, currentPosition.getX() - dump_x))
#         dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
#                                               currentPosition.getHeading(), dump_x, dump_y, endAngle,
#                                               currentPosition.turningRadius)
#         newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
#         self.robotPosition = newPos
#         self.hold = 0
#         return dx, dy, dangle


def appendToCSV1(pathout, filename ,value):
    try:
        with open(pathout+filename, 'a+', newline='') as sb:
            csvWriter = csv.writer(sb)
            csvWriter.writerow(value)
    # except IOError:
    #     # print('not writable')
    #     # print(pathout+filename)
    except MemoryError:
        print(len(value))

def appendToCSV(pathout, filename, value1, value2, value3):
    try:
        with open(pathout+filename, 'a+', newline='') as sb:
            csvWriter = csv.writer(sb)
            for i, j in enumerate(value1):
                csvWriter.writerow([j, value2[i], value3[i]])
    # except IOError:
    #     # print('not writable')
    #     # print(pathout+filename)
    except MemoryError:
        print(len(value1))


def display_top(snapshot, key_type='lineno', limit=10):
    snapshot = snapshot.filter_traces((
        tracemalloc.Filter(False, "<frozen importlib._bootstrap>"),
        tracemalloc.Filter(False, "<unknown>"),
    ))
    top_stats = snapshot.statistics(key_type)

    print("Top %s lines" % limit)
    for index, stat in enumerate(top_stats[:limit], 1):
        frame = stat.traceback[0]
        print("#%s: %s:%s: %.1f KiB"
              % (index, frame.filename, frame.lineno, stat.size / 1024))
        line = linecache.getline(frame.filename, frame.lineno).strip()
        if line:
            print('    %s' % line)

    other = top_stats[limit:]
    if other:
        size = sum(stat.size for stat in other)
        print("%s other: %.1f KiB" % (len(other), size / 1024))
    total = sum(stat.size for stat in top_stats)
    print("Total allocated size: %.1f KiB" % (total / 1024))

def generateBaseline(speed, width, height, currentDist):

    pathLen = 0
    x = []
    y = []
    h = []
    testRoom = RectangularRoom(width, height)
    robot = baseline(testRoom, speed, 0, 0)
    # initialize for this trial
    distance = random.randint(100,200)

    tmp_x, tmp_y, tmp_h = robot.perimeterPoint(testRoom, distance)
    pathLen += len(tmp_x)
    x.extend(tmp_x)
    del tmp_x
    y.extend(tmp_y)
    del tmp_y
    h.extend(tmp_h)
    del tmp_h
    gc.collect()

    tmp_x, tmp_y, tmp_h = robot.firstDredgePoint(testRoom, distance)
    for i, tmp in enumerate(tmp_x):
        if (not robot.isHoldFull()) and testRoom.isTileDredgable(tmp, tmp_y[i]):
            testRoom.dredgeTileAtPosition(tmp, tmp_y[i])
            robot.hold += 1
    pathLen += len(tmp_x)
    x.extend(tmp_x)
    del tmp_x
    y.extend(tmp_y)
    del tmp_y
    h.extend(tmp_h)
    del tmp_h
    gc.collect()

    tmp_x, tmp_y, tmp_h = robot.secoundDredgePoint(testRoom, distance)
    for i, tmp in enumerate(tmp_x):
        if (not robot.isHoldFull()) and testRoom.isTileDredgable(tmp, tmp_y[i]):
            testRoom.dredgeTileAtPosition(tmp, tmp_y[i])
            robot.hold += 1
    pathLen += len(tmp_x)
    x.extend(tmp_x)
    del tmp_x
    y.extend(tmp_y)
    del tmp_y
    h.extend(tmp_h)
    del tmp_h
    gc.collect()

    tmp_x, tmp_y, tmp_h = robot.perimeterPointReturn(testRoom, distance)
    for i, tmp in enumerate(tmp_x):
        if (not robot.isHoldFull()) and testRoom.isTileDredgable(tmp, tmp_y[i]):
            testRoom.dredgeTileAtPosition(tmp, tmp_y[i])
            robot.hold += 1
    pathLen += len(tmp_x)
    x.extend(tmp_x)
    del tmp_x
    y.extend(tmp_y)
    del tmp_y
    h.extend(tmp_h)
    del tmp_h
    gc.collect()

    tmp_x, tmp_y, tmp_h = robot.toDump()
    if (not robot.isHoldFull()):
        for i, tmp in enumerate(tmp_x):
            if (not robot.isHoldFull()) and testRoom.isTileDredgable(tmp, tmp_y[i]):
                testRoom.dredgeTileAtPosition(tmp, tmp_y[i])
                robot.hold += 1
    pathLen += len(tmp_x)
    x.extend(tmp_x)
    del tmp_x
    y.extend(tmp_y)
    del tmp_y
    h.extend(tmp_h)
    del tmp_h
    gc.collect()
    currentDist = np.add(currentDist, testRoom.dredgeMatrix)

    return [x, y, h, testRoom.dredgeMatrix, pathLen, currentDist]


def generatePaths(speed, width, height, robot_type, waypointSeperation, numOfWaypoints, cutoff, currentDist, nextAngleConstraint):

    # tmpCollection = []
    # tmp_x = []
    # tmp_y = []
    # tmp_h = []
    pathLen = 0
    x = []
    y = []
    h = []
    testRoom = RectangularRoom(width, height)
    robot = robot_type(testRoom, speed, int(waypointSeperation), int(numOfWaypoints))
    sampled = random.random()
    # initialize for this trial
    if sampled < cutoff:
        tmp_x, tmp_y, tmp_h = robot.firstDredgeRoute(testRoom)
        for i, tmp in enumerate(tmp_x):
            if (not robot.isHoldFull()) and testRoom.isTileDredgable(tmp, tmp_y[i]):
                testRoom.dredgeTileAtPosition(tmp, tmp_y[i])
                robot.hold += 1
        pathLen += len(tmp_x)
        x.extend(tmp_x)
        del tmp_x
        y.extend(tmp_y)
        del tmp_y
        h.extend(tmp_h)
        del tmp_h
        gc.collect()
        for _ in range(robot.numOfWaypoints - 1):
            tmp_x, tmp_y, tmp_h = robot.dredgeRoute(nextAngleConstraint)
            for i, tmp in enumerate(tmp_x):
                if (not robot.isHoldFull()) and testRoom.isTileDredgable(tmp, tmp_y[i]):
                    testRoom.dredgeTileAtPosition(tmp, tmp_y[i])
                    robot.hold += 1
            pathLen += len(tmp_x)
            x.extend(tmp_x)
            del tmp_x
            y.extend(tmp_y)
            del tmp_y
            h.extend(tmp_h)
            del tmp_h
            gc.collect()
    else:
        tmp_x, tmp_y, tmp_h = robot.firstDredgeRouteEven(testRoom, currentDist)
        for i, tmp in enumerate(tmp_x):
            if (not robot.isHoldFull()) and testRoom.isTileDredgable(tmp, tmp_y[i]):
                testRoom.dredgeTileAtPosition(tmp, tmp_y[i])
                robot.hold += 1
        pathLen += len(tmp_x)
        x.extend(tmp_x)
        del tmp_x
        y.extend(tmp_y)
        del tmp_y
        h.extend(tmp_h)
        del tmp_h
        gc.collect()
        for _ in range(robot.numOfWaypoints-1):
            tmp_x, tmp_y, tmp_h = robot.dredgeRouteEven(testRoom, currentDist)
            for i, tmp in enumerate(tmp_x):
                if (not robot.isHoldFull()) and testRoom.isTileDredgable(tmp, tmp_y[i]):
                    testRoom.dredgeTileAtPosition(tmp, tmp_y[i])
                    robot.hold += 1
            pathLen += len(tmp_x)
            x.extend(tmp_x)
            del tmp_x
            y.extend(tmp_y)
            del tmp_y
            h.extend(tmp_h)
            del tmp_h
            gc.collect()

    tmp_x, tmp_y, tmp_h = robot.toDump()
    if (not robot.isHoldFull()):
        for i, tmp in enumerate(tmp_x):
            if (not robot.isHoldFull()) and testRoom.isTileDredgable(tmp, tmp_y[i]):
                testRoom.dredgeTileAtPosition(tmp, tmp_y[i])
                robot.hold += 1
    pathLen += len(tmp_x)
    x.extend(tmp_x)
    del tmp_x
    y.extend(tmp_y)
    del tmp_y
    h.extend(tmp_h)
    del tmp_h
    gc.collect()
    currentDist = np.add(currentDist, testRoom.dredgeMatrix)

    return [x, y, h, testRoom.dredgeMatrix, pathLen, currentDist]

def restart():
    print("argv was", sys.argv)
    print("sys.executable was", sys.executable)
    print("restart now")
    os.execv(sys.executable, ['python'] + sys.argv)

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)
def shiftedColorMap(cmap, start=0, midpoint=0.5, stop=1.0, name='shiftedcmap'):
    '''
    function taken from
    https://stackoverflow.com/questions/7404116/...
        ...defining-the-midpoint-of-a-colormap-in-matplotlib
    Function to offset the "center" of a colormap. Useful for
    data with a negative min and positive max and you want the
    middle of the colormap's dynamic range to be at zero

    Input
    -----
      cmap : The matplotlib colormap to be altered
      start : Offset from lowest point in the colormap's range.
          Defaults to 0.0 (no lower ofset). Should be between
          0.0 and `midpoint`.
      midpoint : The new center of the colormap. Defaults to
          0.5 (no shift). Should be between 0.0 and 1.0. In
          general, this should be  1 - vmax/(vmax + abs(vmin))
          For example if your data range from -15.0 to +5.0 and
          you want the center of the colormap at 0.0, `midpoint`
          should be set to  1 - 5/(5 + 15)) or 0.75
      stop : Offset from highets point in the colormap's range.
          Defaults to 1.0 (no upper ofset). Should be between
          `midpoint` and 1.0.
    '''
    cdict = {  'red': [],  'green': [], 'blue': [],  'alpha': []  }

    # regular index to compute the colors
    reg_index = np.linspace(start, stop, 257)

    # shifted index to match the data
    shift_index = np.hstack([
        np.linspace(0.0, midpoint, 128, endpoint=False),
        np.linspace(midpoint, 1.0, 129, endpoint=True)
    ])

    for ri, si in zip(reg_index, shift_index):
        r, g, b, a = cmap(ri)

        cdict['red'].append((si, r, r))
        cdict['green'].append((si, g, g))
        cdict['blue'].append((si, b, b))
        cdict['alpha'].append((si, a, a))

    newcmap = matplotlib.colors.LinearSegmentedColormap(name, cdict)
    plt.register_cmap(cmap=newcmap)

    return newcmap


# === Run code
# setSize = 9
numOfPaths = 1000
# (speed, width, height, min_coverage, robot_type, visualize, waypointSeperation, nextAngleConstraint, numOfWaypoints):
iterator = 0
# tracemalloc.start()
focPath = 'C:\\Users\\denma\\Documents\\Uni\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\FamilyofCurves\\'
# bestPath = 'D:\\Masters\\Thesis\\Git\\optimising_TSHD_path\\Sim\\BestPath\\'
bestPath = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\BestPath\\'
optPath = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\opt\\'
Matrices = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\Matrices\\'
bestScores = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\bestScores\\'
bestMatrices = "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\bestMatrices\\"
distributionMat = "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\EvenDistribution\\"
bestCov = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\BestCoveragePercent\\'
bestOverlap = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\BestOverlap\\'
bestPathLen = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\BestPathLen\\'
optSpace = [Integer(1, 200, name='waypointSeperation'), Integer(1, 50, name='numOfWaypoints'), Real(0, 1, name='cutoff'), Integer(0,180,name='angle')]
@use_named_args(dimensions=optSpace)
def objective(waypointSeperation, numOfWaypoints, cutoff, angle):
    start_time = time.time()
    width = 600
    print("Running Simulation")
    print(f'waypointSeperation: {waypointSeperation}')
    print(f'numOfWaypoints: {numOfWaypoints}')
    print(f'cutoff: {cutoff}'
          f'\n angle: {angle}')
    hyp_string = str(waypointSeperation) + '_' + str(numOfWaypoints)+'_'+str(cutoff)+'_'+str(angle)
    try:
        current_dataset = pd.read_csv(optPath + hyp_string, delimiter=',')
        num_entries = len(current_dataset.index)+1
        print(f'num_entries:{num_entries}')
    except FileNotFoundError:
        num_entries = 0
        print(f'number of entries are 0, should be the first run???????????????????????????????????????????????????????')
        np.save(distributionMat + hyp_string, np.zeros((300, 300)))
        os.mkdir(Matrices + hyp_string)
    while num_entries <= (numOfPaths-1):
        # print(f'Number of entries = {num_entries}'
        #       f'Number of paths = {(numOfPaths-1)}')
        # for k in range(setSize):
        #     if not num_entries < (numOfPaths-1):
        #         print("ending early.")
        #         print(f'Number of entries = {num_entries}'
        #               f'Number of paths = {(numOfPaths - 1)}')
        #         os.system(
        #             'python "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\Simulator.py"')
        mat = np.load(distributionMat + hyp_string +'.npy')
        RobotAvg = generatePaths(0.5, width, width, setDistanceWapoint, waypointSeperation,
            numOfWaypoints, cutoff, mat, angle)
        # [x, y, h, testRoom.dredgeMatrix, pathLen]
        appendToCSV(focPath, hyp_string, RobotAvg[0], RobotAvg[1], RobotAvg[2])
        np.save(Matrices+hyp_string+'\\'+str(num_entries), RobotAvg[3])
        np.save(distributionMat + hyp_string, RobotAvg[5])
        # pd.DataFrame(RobotAvg[4]).to_csv(Matrices+hyp_string, mode='a', header=False, index=None)
        appendToCSV1(optPath, hyp_string, [RobotAvg[4]])
        # snapshot = tracemalloc.take_snapshot()
        # top_stats = snapshot.statistics('lineno')
        gc.collect()
        num_entries += 1

            # if

        # os.system('python "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\Simulator.py"')  #restart
    # try:
    #     df = pd.read_csv(bestScores + hyp_string, delimiter=',', header=None)
    #     best = df.iloc[0]
    #     best = float(best)
    #     print("been here... seen that... ")
    #     """ Code to do averages """
    #     # df = df.astype('float64')
    #     # print(df)
    #     # num_entries = len(df.index)
    #     # if num_entries >= 10:
    #     #     best = df.sum(axis=0)
    #     #     best = best/num_entries
    #     #     print(f'already evaluated 10 times, best is: {best.values[0]}')
    #     #     x =best.at[0]
    #     #     return x
    #     # else:
    #     #     opt_data = pd.read_csv(optPath + hyp_string, delimiter=',', names=['coverage', 'path_length'])
    #     #     pl_list = opt_data['path_length'].values.tolist()
    #     #     c_list = opt_data['coverage'].values.tolist()
    #     #     subsetSelection = Greedy.Greedy(pl_list, Matrices + hyp_string, c_list, 500, int(width / 2))
    #     #     print(f'number of random greedy searches: {500}')
    #     #     best, indexs = subsetSelection.runGreedy()
    #     #     appendToCSV1(bestPath, hyp_string, indexs)
    #     #     appendToCSV1(bestScores, hyp_string, [best])
    #     #     return best
    #     print("--- %s seconds ---" % (time.time() - start_time))
    #     gc.collect()
    #     return best
    # except:
    # opt_data = pd.read_csv(optPath+hyp_string, delimiter = ',', names = ['coverage', 'path_length'])
    print(f'time to generate {numOfPaths-1} paths: {time.time() - start_time}')
    opt_data = pd.read_csv(optPath+hyp_string, delimiter = ',', names = ['path_length'])
    pl_list = opt_data['path_length'].values.tolist()
    print(f'number of paths = {len(pl_list)}. Expected number = {numOfPaths-1}')
    greed_time = time.time()
    subsetSelection = Greedy.Greedy(pl_list, Matrices+hyp_string+'\\', numOfPaths/4, int(width/2), bestMatrices+hyp_string+"\\")
    best, indexs, totalCov, totalPathlen, totalOverlap = subsetSelection.runGreedy()
    print(f'time to do {numOfPaths/4} greedy searches: {time.time() - greed_time}')
    appendToCSV1(bestPath, hyp_string, indexs)
    appendToCSV1(bestScores, hyp_string, [best])
    appendToCSV1(bestCov, hyp_string, [totalCov])
    appendToCSV1(bestOverlap, hyp_string, [totalOverlap])
    appendToCSV1(bestPathLen, hyp_string, [totalPathlen])
    print("--- %s Overall seconds ---" % (time.time() - start_time))
    gc.collect()
    return best

def baselineLoop():
    focPath = 'C:\\Users\\denma\\Documents\\Uni\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\FamilyofCurves\\'
    bestPath = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\BestPath\\'
    bestCov = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\BestCoveragePercent\\'
    bestOverlap = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\BestOverlap\\'
    bestPathLen = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\BestPathLen\\'
    optPath = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\opt\\'
    Matrices = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\Matrices\\'
    bestScores = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\bestScores\\'
    bestMatrices = "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\bestMatrices\\"
    distributionMat = "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\EvenDistribution\\"
    plot_path = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\baseline\\plots\\'
    start_time = time.time()
    # np.save(distributionMat + "baseline", np.zeros((300, 300)))
    width = 600
    pathLim = 8000
    hyp_string = "baseline"
    saveme = str(pathLim)
    iterator = 7999
    # while iterator < pathLim:
    #     mat = np.load(distributionMat + hyp_string +'.npy')
    #     # speed, width, height, waypointSeperation, nextAngleConstraint, numOfWaypoints, currentDist
    #     RobotAvg = generateBaseline(0.5, width, width, mat)
    #     appendToCSV(focPath, hyp_string, RobotAvg[0], RobotAvg[1], RobotAvg[2])
    #     np.save(Matrices+hyp_string+str(iterator), RobotAvg[3])
    #     np.save(distributionMat + hyp_string, RobotAvg[5])
    #     # pd.DataFrame(RobotAvg[4]).to_csv(Matrices+hyp_string, mode='a', header=False, index=None)
    #     appendToCSV1(optPath, hyp_string, [RobotAvg[4]])
    #     # snapshot = tracemalloc.take_snapshot()
    #     # top_stats = snapshot.statistics('lineno')
    #     gc.collect()
    #     iterator+=1

    opt_data = pd.read_csv(optPath+hyp_string, delimiter = ',', names = ['path_length'])
    pl_list = opt_data['path_length'].values.tolist()
    print(f'number of paths = {len(pl_list)}. Expected number = {pathLim}')
    print(f'Time to generate {pathLim} paths: {time.time() - start_time}')
    greed_time = time.time()
    subsetSelection = Greedy.Greedy(pl_list, Matrices+hyp_string, pathLim/4, int(width/2), bestMatrices+hyp_string+"\\")
    print(f'number of random greedy searches: {pathLim/4}')
    best, indexs, totalCov, totalPathlen, totalOverlap = subsetSelection.runGreedy()
    print("--- %s Greedy seconds ---" % (time.time() - greed_time))
    appendToCSV1(bestPath, hyp_string+saveme, indexs)
    appendToCSV1(bestScores, hyp_string+saveme, [best])
    appendToCSV1(bestCov, hyp_string+saveme, [totalCov])
    appendToCSV1(bestOverlap, hyp_string+saveme, [totalOverlap])
    appendToCSV1(bestPathLen, hyp_string+saveme, [totalPathlen])
    print("--- %s Overall seconds ---" % (time.time() - start_time))
    print(f'coverage achieved: {totalCov}'
          f' \n total path length: {totalPathlen}')
    gc.collect()
    best_hyp = "baseline"

    best_inds = []
    with open(bestPath + best_hyp+saveme, mode='r') as csv_file:
        reader = csv.reader(csv_file, delimiter=',')
        for i, row in enumerate(reader):
            best_inds.extend(row)
            # if i == min_index:
            #     best_inds = row
    best_inds = list(map(int, best_inds))
    full_set = pd.read_csv(focPath + best_hyp, delimiter=',', header=None)
    # full_set.info()
    opt_data = pd.read_csv(optPath + best_hyp, delimiter=',', names=['path_length'])
    opt_data1 = pd.read_csv(optPath + best_hyp, delimiter=',', names=['path_length']).values.astype("int").squeeze()
    initopt_data = opt_data1.tolist()
    cov_length = []
    true_index = []
    overlapMatrix = np.zeros((300, 300))
    bestSzie = 0
    bestind = 0

    x = full_set.iloc[:, 0]
    # y.append(full_set.iloc[236:236+237-5,1])
    y = full_set.iloc[:, 1]
    # x.append(full_set.iloc[:, 0])
    # y.append(full_set.iloc[:, 1])
    plt.plot(x, y, label=best_hyp)
    # plt.legend()
    plt.grid(True)
    plt.xlabel("x co-ordinates")
    plt.ylabel("y co-ordinates")
    plt.xlim([-100, 800])
    plt.ylim([-100, 800])
    plt.viridis()
    # plt.axis("equal")
    plt.title("All path from baseline")
    plt.savefig(plot_path + 'allpaths' + saveme)
    plt.show()


    shortest = int(opt_data['path_length'].idxmin())
    id = opt_data['path_length'].iloc[shortest]
    id1 = opt_data['path_length'].iloc[0:shortest]
    id2 = int(id1.sum())
    x = full_set.iloc[id2:id2 + id, 0]
    # y.append(full_set.iloc[236:236+237-5,1])
    y = full_set.iloc[id2:id2 + id, 1]
    # x.append(full_set.iloc[:, 0])
    # y.append(full_set.iloc[:, 1])
    plt.plot(x, y, label=best_hyp)
    # plt.legend()
    plt.grid(True)
    plt.xlabel("x co-ordinates")
    plt.ylabel("y co-ordinates")
    plt.xlim([-100, 800])
    plt.ylim([-100, 800])
    plt.viridis()
    # plt.axis("equal")
    plt.title("Shortest path from baseline")
    plt.savefig(plot_path + 'shortpaths' + saveme)
    plt.show()

    shortest = int(opt_data['path_length'].idxmax())
    id = opt_data['path_length'].iloc[shortest]
    id1 = opt_data['path_length'].iloc[0:shortest]
    id2 = int(id1.sum())
    x = full_set.iloc[id2:id2 + id, 0]
    # y.append(full_set.iloc[236:236+237-5,1])
    y = full_set.iloc[id2:id2 + id, 1]
    # x.append(full_set.iloc[:, 0])
    # y.append(full_set.iloc[:, 1])
    plt.plot(x, y, label=best_hyp)
    # plt.legend()
    plt.grid(True)
    plt.xlabel("x co-ordinates")
    plt.ylabel("y co-ordinates")
    plt.xlim([-100, 800])
    plt.ylim([-100, 800])
    plt.viridis()
    # plt.axis("equal")
    plt.title("Longest path from baseline")
    plt.savefig(plot_path + 'longpaths' + saveme)
    plt.show()

    for ind, row in enumerate(best_inds):
        tmpMatrix = np.load(Matrices + best_hyp + str(row) + '.npy')
        overlapMatrix = np.add(overlapMatrix, tmpMatrix)
        boolMap = np.where(tmpMatrix > 0)
        size = tmpMatrix[boolMap].size
        if size > bestSzie:
            bestSzie = size
            bestind = row
    boolMap = np.where(overlapMatrix > 0)
    cover = overlapMatrix[boolMap].size

    percent_covered = (cover / (300 * 300)) * 100
    print(f'covered percentage: {percent_covered}')
    id1 = opt_data['path_length'].iloc[0:bestind]
    bes = int(opt_data['path_length'].iloc[bestind])
    id2 = int(id1.sum())
    x = full_set.iloc[id2:id2 + bes, 0]
    # y.append(full_set.iloc[236:236+237-5,1])
    y = full_set.iloc[id2:id2 + bes, 1]
    # x.append(full_set.iloc[:, 0])
    # y.append(full_set.iloc[:, 1])
    plt.plot(x, y, label=best_hyp)
    # plt.legend()
    plt.grid(True)
    plt.xlabel("x co-ordinates")
    plt.ylabel("y co-ordinates")
    plt.xlim([-100, 800])
    plt.ylim([-100, 800])
    plt.viridis()
    # plt.axis("equal")
    plt.title("Path with most area covered from baseline")
    plt.savefig(plot_path + 'mostcovpaths' + saveme)
    plt.show()

    plt.imshow(overlapMatrix)
    plt.colorbar()
    plt.title("Dredged Locations from baseline")
    plt.savefig(plot_path + 'dredged_locations' + saveme)
    plt.show()
    orig_cmap = matplotlib.cm.viridis
    shifted_cmap = shiftedColorMap(orig_cmap, midpoint=0, name='shifted')

    plt.imshow(overlapMatrix, interpolation=None, cmap=shifted_cmap)
    plt.colorbar()
    plt.title("Dredged Locations shifted colourmap from baseline")
    plt.savefig(plot_path + 'dredged_locations_shifted' + saveme)
    plt.show()
    tally = 0
    start_time = time.time()
    for z in np.nditer(overlapMatrix):
        if z > 0:
            tally += 1
    print("--- %s seconds ---" % (time.time() - start_time))
    print(tally)
    start_time = time.time()
    boolMap = np.where(overlapMatrix > 0)
    size = overlapMatrix[boolMap].size
    print("--- %s seconds ---" % (time.time() - start_time))
    print(f'size: {size}')


  # keyword arguments will be passed to `skopt.dump`
if False:
    baselineLoop()
else:
    if True:
        try:
            res = load('C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\check\\checkpoint.pkl')
            checkpoint_saver = CheckpointSaver(
                "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\check\\checkpoint.pkl", )
            x0 = res.x_iters
            y0 = res.func_vals
            print(f'previous valuees: \n{x0}\n{y0}')
            gp_minimize(objective,  # the function to minimize
                        optSpace,  # the bounds on each dimension of x
                        x0=x0, # already examined values for x
                        y0=y0, # observed values for x0
                        acq_func="gp_hedge",  # the acquisition function
                        n_calls=50,  # the number of evaluations of f
                        callback=[checkpoint_saver],
                        n_random_starts=5,
                        random_state=1234,  # the random seed
                        verbose=True,
                        n_jobs=-1)

        except:
            print("###################################### no file to Load ###############################################")
            checkpoint_saver = CheckpointSaver(
                "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\check\\checkpoint.pkl", )
            res = gp_minimize(objective,  # the function to minimize
                              optSpace,  # the bounds on each dimension of x
                              acq_func="gp_hedge",  # the acquisition function
                              n_calls=50,  # the number of evaluations of f
                              callback=[checkpoint_saver],
                              n_random_starts=5,  # the number of random initialization points
                              noise=0.01,  # the noise level (optional)
                              random_state=1234,  # the random seed
                              verbose=True,
                              n_jobs=-1)
    else:
        # focPath = 'D:\\Thesis_results\\FamilyofCurves\\'
        # # bestPath = 'D:\\Masters\\Thesis\\Git\\optimising_TSHD_path\\Sim\\BestPath\\'
        # bestPath = 'D:\\Thesis_results\\BestPath\\'
        # optPath = 'D:\\Thesis_results\\opt\\'
        # Matrices = 'D:\\Thesis_results\\Matrices\\'
        # bestScores = 'D:\\Thesis_results\\bestScores\\'

        plot_path = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\plots\\'

        res = load('C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\check\\checkpoint.pkl')
        # res = load('D:\\Thesis_results\\check\\checkpoint.pkl')
        # plot_path = 'D:\\Thesis_results\\plots\\'
        best_hyp = str(res.x[0]) + '_' + str(res.x[1]) + '_' + str(res.x[2]) + '_' +str(res.x[3])
        # best_hyp = str(hyp[0]) + '_' + str(hyp[1]) + '_' + str(hyp[2]) + '_' +str(hyp[3])
        saveme = best_hyp + '_'+ str(numOfPaths) + '.png'
        plot_convergence(res)
        plt.savefig(plot_path+'convergence'+saveme)
        plt.show()
        print(f'Best path length: {res.fun}')
        print(f'best hyperparameters: {res.x}')
        print(f'set1:\n {res.x_iters}')
        print(f'set1:\n {res.func_vals}')
        _ = plot_evaluations(res)
        plt.show()
        plt.savefig(plot_path + 'evalutions'+'.jpeg')
        _ = plot_objective(res, n_samples=50)
        plt.show()
        plt.savefig(plot_path + 'objectives'+ '.jpeg')


        # best_score = pd.read_csv(bestScores+best_hyp, delimiter=',', header=None)
        # min_index = int(best_score.idxmin()
        # print(int(min_index))

        # if min_index == 0:
        #     best_indexs = pd.read_csv(bestPath+best_hyp, header=None, sep=',', engine='python', index_col=None, nrows=1)
        # else:
        #     best_indexs = pd.read_csv(bestPath+best_hyp, header=None, sep=',', engine='python', index_col=None, skiprows=min_index, nrows=1)
        best_inds = []
        with open(bestPath+best_hyp, mode='r') as csv_file:
            reader = csv.reader(csv_file, delimiter=',')
            for i, row in enumerate(reader):
                best_inds.extend(row)
                # if i == min_index:
                #     best_inds = row
        best_inds = list(map(int, best_inds))
        full_set = pd.read_csv(focPath + best_hyp, delimiter=',', header=None)
        # full_set.info()
        opt_data = pd.read_csv(optPath + best_hyp, delimiter=',', names=['path_length'])
        opt_data1 = pd.read_csv(optPath + best_hyp, delimiter=',', names=['path_length']).values.astype("int").squeeze()
        initopt_data = opt_data1.tolist()
        cov_length = []
        true_index = []
        overlapMatrix = np.zeros((300,300))
        bestSzie = 0
        bestind = 0
        # opt_data1 = []
        # with open(optPath + best_hyp, mode='r') as csv_file:
        #     reader = csv.reader(csv_file, delimiter=',')
        #     print(reader)
        #     for k, l in enumerate(reader):
        #         for j in l:
        #             print(j)
        #             opt_data1.extend(int(j))




        shortest = int(opt_data['path_length'].idxmin())
        id = opt_data['path_length'].iloc[shortest]
        id1 = opt_data['path_length'].iloc[0:shortest]
        id2 = int(id1.sum())
        x = full_set.iloc[id2:id2+id, 0]
        # y.append(full_set.iloc[236:236+237-5,1])
        y = full_set.iloc[id2:id2+id, 1]
        # x.append(full_set.iloc[:, 0])
        # y.append(full_set.iloc[:, 1])
        plt.plot(x, y, label=best_hyp)
        # plt.legend()
        plt.grid(True)
        plt.xlabel("x co-ordinates")
        plt.ylabel("y co-ordinates")
        plt.xlim([-100, 800])
        plt.ylim([-100, 800])
        plt.viridis()
        # plt.axis("equal")
        plt.title("Shortest path from hyp_parameters: 18_30_2.317_0.015")
        plt.savefig(plot_path + 'shortpaths' + saveme)
        plt.show()

        shortest = int(opt_data['path_length'].idxmax())
        id = opt_data['path_length'].iloc[shortest]
        id1 = opt_data['path_length'].iloc[0:shortest]
        id2 = int(id1.sum())
        x = full_set.iloc[id2:id2 + id, 0]
        # y.append(full_set.iloc[236:236+237-5,1])
        y = full_set.iloc[id2:id2 + id, 1]
        # x.append(full_set.iloc[:, 0])
        # y.append(full_set.iloc[:, 1])
        plt.plot(x, y, label=best_hyp)
        # plt.legend()
        plt.grid(True)
        plt.xlabel("x co-ordinates")
        plt.ylabel("y co-ordinates")
        plt.xlim([-100, 800])
        plt.ylim([-100, 800])
        plt.viridis()
        # plt.axis("equal")
        plt.title("Longest path from hyp_parameters: 18_30_2.317_0.015")
        plt.savefig(plot_path + 'longpaths' + saveme)
        plt.show()

        for ind, row in enumerate(best_inds):
            tmpMatrix = np.load(Matrices + best_hyp + '\\' + str(row) + '.npy')
            overlapMatrix = np.add(overlapMatrix, tmpMatrix)
            boolMap = np.where(tmpMatrix > 0)
            size = tmpMatrix[boolMap].size
            if size > bestSzie:
                bestSzie = size
                bestind = row


        id1 = opt_data['path_length'].iloc[0:bestind]
        bes = int(opt_data['path_length'].iloc[bestind])
        id2 = int(id1.sum())
        x = full_set.iloc[id2:id2+bes, 0]
        # y.append(full_set.iloc[236:236+237-5,1])
        y = full_set.iloc[id2:id2+bes, 1]
        # x.append(full_set.iloc[:, 0])
        # y.append(full_set.iloc[:, 1])
        plt.plot(x, y, label=best_hyp)
        # plt.legend()
        plt.grid(True)
        plt.xlabel("x co-ordinates")
        plt.ylabel("y co-ordinates")
        plt.xlim([-100, 800])
        plt.ylim([-100, 800])
        plt.viridis()
        # plt.axis("equal")
        plt.title("Path with most area covered from hyp_parameters: 18_30_2.317_0.015")
        plt.savefig(plot_path + 'mostcovpaths' + saveme)
        plt.show()


        # try:
        #     start_time = time.time()
        #     overlapMatrix = np.load(bestMatrices+best_hyp+".npy")
        #     print("--- %s seconds ---" % (time.time() - start_time))
        # except:
        #     overlapMatrix = np.zeros((300,300))
        #     for i, chunk in enumerate(pd.read_csv(Matrices+best_hyp, delimiter=',', header=None, chunksize=300, index_col=None, low_memory = False)):
        #         for j, k in enumerate(best_inds):
        #             if k == i:
        #                 arr = chunk.to_numpy()
        #                 overlapMatrix = np.add(overlapMatrix, arr)
        #                 # np.save(self.bestMatrices+str(k), arr)
        #     # boolMap = np.where(overlapMatrix > 1)
        #     # total_overlap = np.sum(overlapMatrix[boolMap])
        #     np.save(bestMatrices+best_hyp, overlapMatrix)
        plt.imshow(overlapMatrix)
        plt.colorbar()
        plt.title("Dredged Locations")
        plt.savefig(plot_path + 'dredged_locations'+saveme)
        plt.show()
        orig_cmap = matplotlib.cm.viridis
        shifted_cmap = shiftedColorMap(orig_cmap, midpoint=0, name='shifted')

        plt.imshow(overlapMatrix, interpolation=None, cmap=shifted_cmap)
        plt.colorbar()
        plt.title("Dredged Locations shifted colourmap")
        plt.savefig(plot_path + 'dredged_locations_shifted'+saveme)
        plt.show()
        tally=0
        start_time = time.time()
        for z in np.nditer(overlapMatrix):
            if z > 0:
                tally +=1
        print("--- %s seconds ---" % (time.time() - start_time))
        print(tally)
        start_time = time.time()
        boolMap = np.where(overlapMatrix > 0)
        size = overlapMatrix[boolMap].size
        print("--- %s seconds ---" % (time.time() - start_time))
        print(f'size: {size}')
        # scores = []
        # x = []
        # y = []
        # for k in res.x_iters:
        #     x.append(k[0])
        #     y.append(k[1])
        # # for k in np.nditer(res.func_vals):
        # #     scores
        #     # scores.extend(pd.read_csv(bestScores + str(l[0])+'_'+str(l[1]) , delimiter=',', header=None).to_list)
        #
        # # Make data.
        # X = x
        # Y = y
        # Z = res.func_vals
        #
        # fig = plt.figure()
        # ax = fig.gca(projection='3d')
        # ax.scatter(X, Y, Z)
        # ax.set_xlabel('Waypoint Seperation')
        # ax.set_ylabel('Number of Waypoints')
        # ax.set_zlabel('Minimum Score Returned by Cost Function')
        # plt.show()