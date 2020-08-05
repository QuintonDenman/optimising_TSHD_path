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
import time
from Sim import Dubins
from Sim import Greedy
import gc
from datetime import datetime
import tracemalloc
import linecache
from pulp import *
import itertools
import time


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
        self.turningRadius = 6

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


    # def getNewPosition(self, angle, speed):
    #     """
    #     Computes and returns the new Position after a single clock-tick has
    #     passed, with this object as the current position, and with the
    #     specified angle and speed.
    #
    #     Does NOT test whether the returned position fits inside the room.
    #
    #     angle: integer representing angle in degrees, 0 <= angle < 360
    #     speed: positive float representing speed
    #
    #     Returns: a Position object representing the new position.
    #     """
    #     # if angle > 30:
    #     #     angle = 30
    #     # elif angle < -30:
    #     #     angle = -30
    #     old_x, old_y = self.getX(), self.getY()
    #     # Compute the change in position
    #     delta_y = speed * math.cos(math.radians(angle))
    #     delta_x = speed * math.sin(math.radians(angle))
    #     # Add that to the existing position
    #     new_x = old_x + delta_x
    #     new_y = old_y + delta_y
    #     return Position(new_x, new_y)

    def getRightTurn(self, angle, speed, deltaTime=1):
        # angle = math.radians(angle)
        # old_x, old_y = self.getX(), self.getY()
        # relative_start_angle = 90-angle
        # delta_phi = (speed*deltaTime) / (2 * math.pi * self.turningRadius)
        # alpha = (math.pi - delta_phi)/2
        # beta = alpha + (relative_start_angle - math.pi/2)
        # A = self.turningRadius * (math.sin(delta_phi) / math.sin(alpha))
        # delta_x = A*math.cos(beta)
        # delta_y = A*math.sin(beta)
        # new_x = old_x + delta_x
        # new_y = delta_y - old_y #origin of tkinter is always top left
        # new_angle = relative_start_angle + delta_phi
        # return Position(new_x, new_y, math.degrees(new_angle))
        angular_speed = (speed / self.turningRadius) * -1
        old_x, old_y = self.getX(), self.getY()
        phi_s = math.radians(angle)
        new_x = old_x + self.turningRadius * math.sin(phi_s) + self.turningRadius * math.sin(angular_speed *
                                                                                             deltaTime - phi_s)
        new_y = old_y + self.turningRadius * math.cos(phi_s) - self.turningRadius * math.cos(angular_speed * deltaTime - phi_s)
        new_heading = math.degrees(phi_s - angular_speed * deltaTime)
        return Position(new_x, new_y, new_heading)

    def getLeftTurn(self, angle, speed, deltaTime=1):
        # old_x, old_y = self.getX(), self.getY()
        # relative_start_angle = 90-angle
        # delta_phi = (speed*deltaTime) / (2 * math.pi * self.turningRadius)
        # alpha = (math.pi - delta_phi)/2
        # beta = alpha - (relative_start_angle - math.pi/2)
        # A = self.turningRadius * (math.sin(delta_phi) / math.sin(alpha))
        # delta_x = A*math.cos(beta)
        # delta_y = A*math.sin(beta)
        # new_x = old_x + delta_x
        # new_y = delta_y - old_y #origin of tkinter is always top left
        # new_angle = relative_start_angle+delta_phi
        # return Position(new_x, new_y, math.degrees(new_angle))
        angular_speed = speed/ self.turningRadius
        old_x, old_y = self.getX(), self.getY()
        phi_s = math.radians(angle)
        new_x = old_x - self.turningRadius * math.sin(phi_s) + self.turningRadius * \
                math.sin(angular_speed*deltaTime + phi_s)
        new_y = old_y - self.turningRadius * math.cos(phi_s) + self.turningRadius * \
                math.cos(angular_speed*deltaTime + phi_s)
        new_heading = math.degrees(phi_s + angular_speed * deltaTime)
        return Position(new_x, new_y, new_heading)





    def getStraightMove(self, angle, speed):
        old_x, old_y = self.getX(), self.getY()
        delta_y = speed * math.sin(math.radians(angle))
        delta_x = speed * math.cos(math.radians(angle))
        new_x = old_x + delta_x
        new_y = old_y + delta_y
        return Position(new_x, new_y, angle)

    # def getStraightMove(self, angle, speed, time=1):
    #     x, y = self.getX(), self.getY()
    #     for i in range(time):
    #         delta_y = speed * math.cos(math.radians(angle))
    #         delta_x = speed * math.sin(math.radians(angle))
    #         x = x + delta_x
    #         y = y + delta_y
    #     return Position(x, y, angle)

    def getConstrainedRandomAngle(self, constraint):
        return random.randrange(-constraint, constraint)

    def getConstrainedRandomPosition(self, pos, loc_constraint, angle_constraint):
        x = int(pos.getX())
        y = int(pos.getY())
        angle = pos.getHeading()
        new_x = random.randint(max(x - loc_constraint, 0), min(x + loc_constraint, 70)) #TODO: fix hardcoded dimension
        new_y = random.randint(max(y - loc_constraint, 0), min(y + loc_constraint, 70))
        new_angle = (math.atan2(x - new_x, y - new_y))
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
        self.cleanTiles = {}  # a dictionary of Position objects
        self.dredgeMatrix = np.zeros((self.dredgeAreaWidth, self.dredgeAreaHeight))
        self.resultantEnvForce = 2
        self.dumpLoc = (width, height)
        self.dredgePerimeter = []
        self.closePerimeter = []

    def createDredgingLocations(self):
        """
        Creates a basic rectangular dredging area based on the size of the two variables:
         dredgeAreaWidth & dredgeAreaHeight
        """
        for m in range(self.dredgeAreaWidth):
            for n in range(self.dredgeAreaHeight):
                self.dredgeArea.append((m, n))
        # print(f'dredgeArea={self.dredgeArea}')

    def dredgeTileAtPosition(self, pos):
        """
        Mark the tile under the position POS as cleaned.
        Assumes that POS represents a valid position inside this room.
        Assumes hold is not full

        pos: a Position
        """
        # Convert postion to integer, add the position to the dictionary of clean positions; increment if necessary
        intPosition = (int(pos.getX()), int(pos.getY()))
        self.cleanTiles[intPosition] = self.cleanTiles.get(intPosition, 0) + 1
        self.dredgeMatrix[intPosition[0]-36, intPosition[1]-36] += 1

    def dredgeTileAtPosition(self, x, y):

        intPosition = (int(x), int(y))
        self.cleanTiles[intPosition] = self.cleanTiles.get(intPosition, 0) + 1
        self.dredgeMatrix[intPosition[0]-36, intPosition[1]-36] += 1

    def getCleanTiles(self):
        return self.cleanTiles


    def isTileCleaned(self, m, n):
        """
        Return True if the tile (m, n) has been cleaned.

        Assumes that (m, n) represents a valid tile inside the room.

        m: an integer
        n: an integer
        returns: True if (m, n) is cleaned, False otherwise
        """
        questionedPosition = (int(m), int(n))
        if questionedPosition in self.cleanTiles:
            return True
        return False

    def resetTiles(self):
        self.cleanTiles = {}
        return

    def isTileDredgable(self, pos):
        """
        Return True if the tile (m, n) is within the dredging area.

        Assumes that (m, n) represents a valid tile inside the room.
        Assumes that the list dredgeArea has been populated

        m: an integer
        n: an integer
        returns: True if (m, n) is within the dredging area , False otherwise
        """
        questionedPosition = (int(pos.getX()), int(pos.getY()))
        if questionedPosition in self.dredgeArea:

            return True
        return False

    def getNumTiles(self):
        """
        Return the total number of dredge_tiles in the room.

        returns: an integer
        """
        return self.roomWidth * self.roomHeight

    def getNumCleanedTiles(self):
        """
        Return the total number of clean dredge_tiles in the room.

        returns: an integer
        """
        # Counts dictionary entries.  Assumes that dictionary value cannot be 0, i.e., no tile can become unclean after being cleaned.
        return len(self.cleanTiles)

    def getRandomDredgePosition(self):
        """
        Return a random position inside the room.

        returns: a Position object.
        """
        # generate random numbers between 0 and width or height inclusive
        randomWidth = random.randint(0, self.dredgeAreaWidth - 1)
        randomHeight = random.randint(0, self.dredgeAreaHeight - 1)
        randomAngle = random.randint(0, 360)
        return Position(randomWidth, randomHeight, randomAngle)

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

    def isPositionInDredgingArea(self, pos):
        """
        Return True if POS is inside the dredging area.

        pos: a Position object.
        returns: True if POS is in the dredging area, False otherwise.
        """
        if pos.getX() < 0: return False #Uneccessary if you assume dredging area is in the room
        if pos.getY() < self.dredgeAreaHeight: return False

        if pos.getX() >= self.dredgeAreaWidth: return False
        if pos.getY() >= self.dredgeAreaHeight: return False #Uneccessary if you assume dredging area is in the room

    def getPerimeter(self):
        for i in range(self.dredgeAreaHeight):
            self.dredgePerimeter.append((0,i))
            self.closePerimeter.append((0,i))
            self.dredgePerimeter.append((self.dredgeAreaHeight, i))
            self.closePerimeter.append((self.dredgeAreaHeight, i))
            self.dredgePerimeter.append((i, self.dredgeAreaHeight))
            self.dredgePerimeter.append((i, 0))
        return

class BaseShip(object):
    """
    Represents a robot cleaning a particular room.

    At all times the robot has a particular position and direction in
    the room.  The robot also has a fixed speed.

    Subclasses of BaseRobot should provide movement strategies by
    implementing updatePositionAndClean(), which simulates a single
    time1-step.
    """

    def __init__(self, room, speed, waypointSeperation, nextAngleConstraint,
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
        self.robotPosition = Position(self.robotRoom.roomWidth/2, self.robotRoom.roomHeight/2, 0)
        self.holdCapacity = 583 #based on a dredge track length and cell size (3500m/6m=583.333)
        self.hold = 0
        self.path = []
        self.pathDredgePercentage = []
        self.isPathPlanned = False
        self.environmentForce = 1
        self.environmentForceDirection = 0
        #parameters
        self.waypointSeperation = waypointSeperation
        self.nextAngleConstraint = nextAngleConstraint
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

    def isHoldFull(self, hold):
        """
        return True if hold is less than max capacity
        return False if hold has reached max capacity
        hold: amount of dirt ship has collected

        """
        if hold >= self.holdCapacity:
            return True
        return False

class RandomAstarShip(BaseShip):
    """
        A Robot is a BaseRobot with the standard movement strategy.

        At each time1-step, a Robot attempts to move in its current
        direction; when it hits a wall, it chooses a new direction
        randomly.
        """

    def updatePositionAndClean(self):
        """
        Simulate the passage of a single time1-step.

        Move the robot to a new position and if tile is in the dredgable zone mark the tile as having
        been cleaned.
        """

        notAtWall = True

        if self.isHoldFull(self.hold):
            currentPosition = self.getRobotPosition()
            # calculate new direction by using current location and target tile
            goalX, goalY = self.path[-1]
            newDirection = math.degrees(math.atan2(goalX-currentPosition.getX(), goalY-currentPosition.getY()))
            #TODO: implement ship dynamics instead of a point turn, direction would then be final loc
            newPosition = currentPosition.getNewPosition(newDirection, self.robotSpeed)
            self.robotPosition = newPosition
            if (int(self.getRobotPosition().getX()), int(self.getRobotPosition().getY())) == self.path[-1]:
                self.hold = 0
            else:
                notAtWall = False

        while notAtWall:
            currentPosition = self.getRobotPosition()
            nextPosition = currentPosition.getNewPosition(self.getRobotDirection(), self.robotSpeed)
            if self.robotRoom.isPositionInRoom(nextPosition):
                self.robotPosition = nextPosition
                # tell room this tile is clean
                if self.robotRoom.isTileDredgable(self.robotPosition) and not self.robotRoom.isTileCleaned(
                        self.robotPosition.getX(), self.robotPosition.getY()) and not self.isHoldFull(self.hold):
                    self.robotRoom.dredgeTileAtPosition(self.robotPosition)
                    self.hold += 1
                notAtWall = False
            else:  # pick a new direction at random
                self.robotDirection = random.randint(0, 360)


class DynamicsTest(BaseShip):
    def moveForward(self):
        current_position = self.getRobotPosition()
        new_position = current_position.getStraightMove(current_position.getHeading(), self.robotSpeed)
        self.robotPosition = new_position
        print(f'My angle is: {new_position.getHeading()}')

    def moveLeft(self):
        current_position = self.getRobotPosition()
        new_position = current_position.getLeftTurn(current_position.getHeading(), self.robotSpeed)
        self.robotPosition = new_position
        print(f'My left angle is: {new_position.getHeading()}')
    def moveRight(self):
        current_position = self.getRobotPosition()
        new_position = current_position.getLeftTurn(current_position.getHeading(), self.robotSpeed)
        self.robotPosition = new_position
        print(f'My riight angle is: {new_position.getHeading()}')


class PerimeterSetDistance(BaseShip):
    """
        A Robot is a BaseRobot with the standard movement strategy.

        At each time1-step, a Robot attempts to move in its current
        direction; when it hits a wall, it chooses a new direction
        randomly.
        """

    def findOptDredgeRoute(self):
        dump_x, dump_y = self.robotRoom.dumpLoc
        currentPosition = self.getRobotPosition()
        startAngle = 225
        perimeterIndex = random.randint(0, (len(self.robotRoom.closePerimeter) - 1))
        perimeter_x, perimeter_y = self.robotRoom.closePerimeter[perimeterIndex]
        endAngle = (math.atan2(dump_x - perimeter_x, dump_y - perimeter_y))
        px, py, pangle = Dubins.getDubinsPath(dump_x, dump_y, startAngle, perimeter_x, perimeter_y, endAngle,
                                              currentPosition.turningRadius)
        newPos = currentPosition.setPosition(px[-1], py[-1], pangle[-1])
        self.robotPosition = newPos
        while not self.isHoldFull(self.hold):
                currentPosition = self.getRobotPosition()
                constrainedPos = currentPosition.getConstrainedRandomPosition()
                startDredgeArea = self.robotRoom.getNumCleanedTiles() / self.robotRoom.getNumTiles()
                px, py, pangle = Dubins.getDubinsPath(currentPosition.getX(), currentPosition.getY(),
                                                      math.radians(currentPosition.getHeading()), constrainedPos.getX(),
                                                      constrainedPos.getY(), math.radians(constrainedPos.getHeading()),
                                                      currentPosition.turningRadius)
                for i, tmp in enumerate(px):
                    if not self.robotRoom.isTileCleaned(tmp, py[i]):
                        self.robotRoom.dredgeTileAtPosition(tmp, py[i])
                        self.hold += 1
                        # print(self.hold)
                newDredgeArea = self.robotRoom.getNumCleanedTiles() / self.robotRoom.getNumTiles()
                self.path.append([startDredgeArea-newDredgeArea, px, py, pangle])
                newPos = currentPosition.setPosition(px[-1], py[-1], pangle[-1])
                self.robotPosition = newPos
        currentPosition = self.getRobotPosition()
        endAngle = (math.atan2(currentPosition.getX() - dump_x, currentPosition.getY() - dump_y))
        px, py, pangle = Dubins.getDubinsPath(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), dump_x, dump_y, endAngle,
                                              currentPosition.turningRadius)

        newPos = currentPosition.setPosition(px[-1], py[-1], pangle[-1])
        self.robotPosition = newPos
        self.hold = 0

class setDistanceWapoint(BaseShip):

    '''
    Parameters:
    self.waypointSeperation = 10
    self.nextAngleConstraint = 30
    self.numOfWaypoints = 3
    '''
    # def dumpToPerimeter(self):
    #     dump_x, dump_y = self.robotRoom.dumpLoc
    #     currentPosition = self.getRobotPosition()
    #     startAngle = 225
    #     perimeterIndex = random.randint(0, (len(self.robotRoom.closePerimeter) - 1))
    #     perimeter_x, perimeter_y = self.robotRoom.closePerimeter[perimeterIndex]
    #     endAngle = (math.atan2(dump_x - perimeter_x, dump_y - perimeter_y))
    #     dx, dy, dangle = Dubins.getDubinsPath(dump_x, dump_y, startAngle, perimeter_x, perimeter_y, endAngle,
    #                                           currentPosition.turningRadius)
    #     # self.path.append([px, py, pangle])
    #     newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
    #     self.robotPosition = newPos
    #     return dx, dy, dangle
    def dredgeRoute(self):
        currentPosition = self.getRobotPosition()
        constrainedPos = currentPosition.getConstrainedRandomPosition(currentPosition, self.waypointSeperation,
                                                                      self.nextAngleConstraint)
        while int(currentPosition.getX()) == int(constrainedPos.getX()) and \
                int(currentPosition.getY()) == int(constrainedPos.getY()):
            constrainedPos = currentPosition.getConstrainedRandomPosition(currentPosition, self.waypointSeperation,
                                                                          self.nextAngleConstraint)
            # print("it actually happened")
        # try:
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              math.radians(currentPosition.getHeading()), constrainedPos.getX(),
                                              constrainedPos.getY(), math.radians(constrainedPos.getHeading()),
                                              currentPosition.turningRadius)
        gc.collect()
        # except:
        #     MemoryError
        #     print(f'length of one dangle: { len(px)}')
        if not self.isHoldFull(self.hold):
            for i, tmp in enumerate(dx):
                if self.robotRoom.isTileDredgable and not self.robotRoom.isTileCleaned(tmp, dy[i]):
                    self.robotRoom.dredgeTileAtPosition(tmp, dy[i])
                    self.hold += 1
        # self.path.append([px, py, pangle])

        try:
            newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        except IndexError:
            # print(f'current: {currentPosition.getPos()} \n'
            #       f'planned: {constrainedPos.getPos()}')
            print(f'currentPosition.getX(): {currentPosition.getX()} \n'
                  f'currentPosition.getY(): {currentPosition.getY()} \n'
                  f'currentPosition.getHeading(): {currentPosition.getHeading()} \n'
                  f'constrainedPos.getX(): {constrainedPos.getX()} \n'
                  f'constrainedPos.getY(): {constrainedPos.getY()} \n'
                  f'constrainedPos.getHeading(): {constrainedPos.getHeading()}')
            if len(dx) == 0: newPos = currentPosition.setPosition(constrainedPos.getX(), dy[-1], dangle[-1])
            elif len(dy) == 0: newPos = currentPosition.setPosition(dx[-1], constrainedPos.getY(), dangle[-1])
        self.robotPosition = newPos
        return dx, dy, dangle

    # def currentToPerimeter(self):
    #     dump_x, dump_y = self.robotRoom.dumpLoc
    #     currentPosition = self.getRobotPosition()
    #     perimeterIndex = random.randint(0, (len(self.robotRoom.closePerimeter) - 1))
    #     perimeter_x, perimeter_y = self.robotRoom.closePerimeter[perimeterIndex]
    #     endAngle = (math.atan2(currentPosition.getX() - dump_x, currentPosition.getY() - dump_y))
    #     px, py, pangle = Dubins.getDubinsPath(currentPosition.getX(), currentPosition.getY(),
    #                                           currentPosition.getHeading(), perimeter_x, perimeter_y, endAngle,
    #                                           currentPosition.turningRadius)
    #     print(len(px))
    #     if not self.isHoldFull(self.hold):
    #         for i, tmp in enumerate(px):
    #             if self.robotRoom.isTileDredgable and not self.robotRoom.isTileCleaned(tmp, py[i]):
    #                 self.robotRoom.dredgeTileAtPosition(tmp, py[i])
    #                 self.hold += 1
    #     # self.path.append([px, py, pangle])
    #     newPos = currentPosition.setPosition(px[-1], py[-1], pangle[-1])
    #     self.robotPosition = newPos
    #     return px, py, pangle

    def toDump(self):
        dump_x, dump_y = self.robotRoom.dumpLoc
        currentPosition = self.getRobotPosition()
        endAngle = (math.atan2(currentPosition.getX() - dump_x, currentPosition.getY() - dump_y))
        dx, dy, dangle = Dubins.main(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), dump_x, dump_y, endAngle,
                                              currentPosition.turningRadius)
        if not self.isHoldFull(self.hold):
            for i, tmp in enumerate(dx):
                if self.robotRoom.isTileDredgable and not self.robotRoom.isTileCleaned(tmp, dy[i]):
                    self.robotRoom.dredgeTileAtPosition(tmp, dy[i])
                    self.hold += 1
        # self.path.append([px, py, pangle])
        newPos = currentPosition.setPosition(dx[-1], dy[-1], dangle[-1])
        self.robotPosition = newPos
        self.hold = 0
        gc.collect()
        return dx, dy, dangle




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
            csvWriter.writerow(value1)
            csvWriter.writerow(value2)
            csvWriter.writerow(value3)
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


def generatePaths(speed, width, height, min_coverage, robot_type, visualize, waypointSeperation, nextAngleConstraint,
        numOfWaypoints, filename):

    # tmpCollection = []
    # tmp_x = []
    # tmp_y = []
    # tmp_h = []
    pathLen = 0
    x = []
    y = []
    h = []
    # dredgedLoc = []
    # print "Trial %i:" % m,
    # if visualize: anim = ps11_visualize.RobotVisualization(width, height, int(width/2), int(height/2), .02)
    # create the room
    testRoom = RectangularRoom(width, height)
    testRoom.createDredgingLocations()
    testRoom.getPerimeter()
    robot = robot_type(testRoom, speed, int(waypointSeperation), nextAngleConstraint,
        int(numOfWaypoints))

    # initialize for this trial
    percentClean = 0.0000000
    numberOfCurves = 0
    # while percentClean < min_coverage:  # clean until percent clean >= min coverage

    # if visualize: anim.update(testRoom, [robot])
    # tmpCollection.append(robot.dumpToPerimeter())
    # tmp_x, tmp_y, tmp_h = robot.dumpToPerimeter()
    for _ in range(robot.numOfWaypoints):
        tmp_x, tmp_y, tmp_h = robot.dredgeRoute()
        pathLen += len(tmp_x)
        x.append(tmp_x)
        del tmp_x
        y.append(tmp_y)
        del tmp_y
        h.append(tmp_h)
        del tmp_h
        gc.collect()


    tmp_x, tmp_y, tmp_h = robot.toDump()
    pathLen += len(tmp_x)
    x.append(tmp_x)
    del tmp_x
    y.append(tmp_y)
    del tmp_y
    h.append(tmp_h)
    del tmp_h
    gc.collect()

    percentClean = float(testRoom.getNumCleanedTiles()) / float(testRoom.getNumTiles())
    # cleanTiles = testRoom.getCleanTiles()
    # for i, _ in cleanTiles.items():
    #     dredgedLoc.append(i)
    # while tmp < 20:
    #     tmp += 1
    #     if visualize: anim.update(testRoom, [robot])
    #     # tmpCollection.append(robot.dumpToPerimeter())
    #     tmp_x, tmp_y, tmp_h = robot.dumpToPerimeter()
    #     for _ in range(robot.numOfWaypoints):
    #         tmpCollection.append(robot.dredgeRoute())
    #
    #     tmpCollection.append(robot.toDump())
    #     numberOfCurves += 1
    #     percentClean = float(testRoom.getNumCleanedTiles()) / float(testRoom.getNumTiles())
    #     gc.collect()
    # print(percentClean)
    # if visualize: anim.done()
    # trialsCollection.append(progressList)
    # coveragePath.append(tmpCoverage)

    return [x, y, h, percentClean, testRoom.dredgeMatrix, pathLen]

def restart():
    print("argv was", sys.argv)
    print("sys.executable was", sys.executable)
    print("restart now")
    os.execv(sys.executable, ['python'] + sys.argv)



# === Run code
setSize = 200
numOfPaths = 2000
# (speed, width, height, min_coverage, robot_type, visualize, waypointSeperation, nextAngleConstraint, numOfWaypoints):
iterator = 0
tracemalloc.start()
optSpace = [Integer(1, 10, name='waypointSeperation'), Integer(1, 5, name='numOfWaypoints')]
@use_named_args(dimensions=optSpace)
def objective(waypointSeperation, numOfWaypoints):
    width = 70
    print("Running Simulation")
    print(f'waypointSeperation: {waypointSeperation}')
    print(f'numOfWaypoints: {numOfWaypoints}')
    # focPath = 'D:\\Masters\\Thesis\\Git\\optimising_TSHD_path\\Sim\\FamilyofCurves\\'
    focPath = 'C:\\Users\\denma\\Documents\\Uni\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\FamilyofCurves\\'
    # bestPath = 'D:\\Masters\\Thesis\\Git\\optimising_TSHD_path\\Sim\\BestPath\\'
    bestPath = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\BestPath\\'
    optPath = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\opt\\'
    Matrices = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\Matrices\\'
    hyp_string = str(waypointSeperation) + '_' + str(numOfWaypoints)
    try:
        current_dataset = pd.read_csv(focPath + hyp_string, delimiter=',')
        # print(current_dataset)
        num_entries = len(current_dataset.index)
    except FileNotFoundError:
        num_entries = 0
        print(f'number of entries are 0, should be the first run???????????????????????????????????????????????????????')
    while num_entries < (numOfPaths*3)-1:
        print(f'Number of entries = {num_entries}'
              f'Number of paths = {numOfPaths*3}')
        for k in range(setSize):
            start_time = time.time()
            RobotAvg = generatePaths(0.5, width, width, 0.9, setDistanceWapoint, False, waypointSeperation, 30,
                numOfWaypoints, focPath+hyp_string)
            # saveBest(focPath, dt_string, [RobotAvg[1]])
            appendToCSV(focPath, hyp_string, RobotAvg[0], RobotAvg[1], RobotAvg[2])
            # print(RobotAvg[3])
            # print(RobotAvg[4])
            # print(RobotAvg[5])
            pd.DataFrame(RobotAvg[4]).to_csv(Matrices+hyp_string, mode='a', header=False, index=None)
            appendToCSV1(optPath, hyp_string, [RobotAvg[3], RobotAvg[5]])
            # snapshot = tracemalloc.take_snapshot()
            # top_stats = snapshot.statistics('lineno')
            gc.collect()

            if k % 10 == 0 :
                print(f'set {k} of {setSize}')
                print("--- %s seconds ---" % (time.time() - start_time))
                # overlap_data = pd.read_csv(Matrices + hyp_string, delimiter=',', header=None)
                # print(f'overlap data straight from csv: {overlap_data}')
                # print(f'overlap data straight from csv: {overlap_data.shape[0] / 35}')
                # print(f'overlap data straight from csv: {overlap_data.shape}')
                # print(f'written to: {RobotAvg[4]}')
                # print(f'written to: {RobotAvg[4].shape}')
                # print(f'written to: {RobotAvg[4].shape[0]/35}')
                # print(RobotAvg[4].shape[0]/35)
                # print("[ Top 10 ]")
                # for stat in top_stats[:10]:
                #     print(stat)
        os.system('python "C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\Simulator.py"')  #restart
        # pathCollection.append(RobotAvg[1])

    # data, initialSolution, initialTemp, iterationLimit, finalTemp, tempReduction, neighborOperator,
    # iterationPerTemp=100, alpha=10, beta=5)
    # subsetSelection = LpP[roblem("Subset Selection", LpMinimize)
    # subsetSelection += lpSum(objective[]])
    opt_data = pd.read_csv(optPath+hyp_string, delimiter = ',', names = ['coverage', 'path_length'])
    overlap_data = pd.read_csv(Matrices+hyp_string,  delimiter = ',', header = None)
    pd.set_option("display.max_rows", None, "display.max_columns", None)
    # dingle = opt_data['dredged_locations'].values.tolist()
    pl_list = opt_data['path_length'].values.tolist()
    # dl_list = opt_data['dredged_locations'].values.tolist()
    c_list = opt_data['coverage'].values.tolist()
    subsetSelection = Greedy.Greedy(pl_list, overlap_data, c_list, 500, int(width/2))
    print(f'number of random greedy searches: {numOfPaths}')
    best, indexs = subsetSelection.runGreedy()
    # print(index, best)
    # df = pd.read_csv(focPath+dt_string)
    # print(df.shape)
    # bestFamilyofCurves = df[index]
    appendToCSV1(bestPath, hyp_string+'_'+str(best), indexs)
    return best


checkpoint_saver = CheckpointSaver("C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\check\\checkpoint.pkl",)  # keyword arguments will be passed to `skopt.dump`
try:
    res = load('C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\check\\checkpoint.pkl')
    x0 = res.x_iters
    print(x0)
    y0 = res.func_vals
    print(y0)
    gp_minimize(objective,  # the function to minimize
                optSpace,  # the bounds on each dimension of x
                x0=x0, # already examined values for x
                y0=y0, # observed values for x0
                acq_func="gp_hedge",  # the acquisition function
                n_calls=100,  # the number of evaluations of f
                callback=[checkpoint_saver],
                n_random_starts=5,  # the number of random initialization points
                noise=0.1 ** 2,  # the noise level (optional)
                random_state=1234,  # the random seed
                verbose=True,
                n_jobs=-1)

except:
    print("###################################### no file to Load ###############################################")
    res = gp_minimize(objective,  # the function to minimize
                      optSpace,  # the bounds on each dimension of x
                      acq_func="gp_hedge",  # the acquisition function
                      n_calls=30,  # the number of evaluations of f
                      callback=[checkpoint_saver],
                      n_random_starts=5,  # the number of random initialization points
                      noise=0.1 ** 2,  # the noise level (optional)
                      random_state=1234,  # the random seed
                      verbose=True,
                      n_jobs=-1)
#
# opt = Optimizer(Integer(1, 15, name='waypointSeperation'))


print(f'Best path length: {res.fun}')
print(res.space)
print(res.specs)

