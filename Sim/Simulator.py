import math
import random
import os, sys, csv
import Sim.ps11_visualize
import numpy as np
from skopt import Optimizer
import pandas as pd
from skopt.space import Real, Integer
from skopt.utils import use_named_args
from pylab import plot, axis, title, ylabel, xlabel, show
import csv
from csv import writer
from Sim import ps11_visualize
import time
from Sim import Dubins
from Sim import SimulatedAnealing
import gc
from datetime import datetime
from bayes_opt import BayesianOptimization


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
        self.resultantEnvForce = 2
        self.dumpLoc = (width, height)
        self.dredgePerimeter = []
        self.closePerimeter = []

    def createDredgingLocations(self):
        """"
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

    def dredgeTileAtPosition(self, x, y):

        intPosition = (int(x), int(y))
        self.cleanTiles[intPosition] = self.cleanTiles.get(intPosition, 0) + 1


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
    def dumpToPerimeter(self):
        dump_x, dump_y = self.robotRoom.dumpLoc
        currentPosition = self.getRobotPosition()
        startAngle = 225
        perimeterIndex = random.randint(0, (len(self.robotRoom.closePerimeter) - 1))
        perimeter_x, perimeter_y = self.robotRoom.closePerimeter[perimeterIndex]
        endAngle = (math.atan2(dump_x - perimeter_x, dump_y - perimeter_y))
        px, py, pangle = Dubins.getDubinsPath(dump_x, dump_y, startAngle, perimeter_x, perimeter_y, endAngle,
                                              currentPosition.turningRadius)
        self.path.append([px, py, pangle])
        newPos = currentPosition.setPosition(px[-1], py[-1], pangle[-1])
        self.robotPosition = newPos
    def dredgeRoute(self):
        currentPosition = self.getRobotPosition()
        constrainedPos = currentPosition.getConstrainedRandomPosition(currentPosition, self.waypointSeperation,
                                                                      self.nextAngleConstraint)
        while int(currentPosition.getX()) == int(constrainedPos.getX()) and \
                int(currentPosition.getY()) == int(constrainedPos.getY()):
            constrainedPos = currentPosition.getConstrainedRandomPosition(currentPosition, self.waypointSeperation,
                                                                          self.nextAngleConstraint)
            print("it actually happened")
        px, py, pangle = Dubins.getDubinsPath(currentPosition.getX(), currentPosition.getY(),
                                              math.radians(currentPosition.getHeading()), constrainedPos.getX(),
                                              constrainedPos.getY(), math.radians(constrainedPos.getHeading()),
                                              currentPosition.turningRadius)
        if not self.isHoldFull(self.hold):
            for i, tmp in enumerate(px):
                if self.robotRoom.isTileDredgable and not self.robotRoom.isTileCleaned(tmp, py[i]):
                    self.robotRoom.dredgeTileAtPosition(tmp, py[i])
                    self.hold += 1
        self.path.append([px, py, pangle])

        try:
            newPos = currentPosition.setPosition(px[-1], py[-1], pangle[-1])
        except IndexError:
            # print(f'current: {currentPosition.getPos()} \n'
            #       f'planned: {constrainedPos.getPos()}')
            print(f'currentPosition.getX(): {currentPosition.getX()} \n'
                  f'currentPosition.getY(): {currentPosition.getY()} \n'
                  f'currentPosition.getHeading(): {currentPosition.getHeading()} \n'
                  f'constrainedPos.getX(): {constrainedPos.getX()} \n'
                  f'constrainedPos.getY(): {constrainedPos.getY()} \n'
                  f'constrainedPos.getHeading(): {constrainedPos.getHeading()}')
            if len(px) == 0: newPos = currentPosition.setPosition(constrainedPos.getX(), py[-1], pangle[-1])
            elif len(py) == 0: newPos = currentPosition.setPosition(px[-1], constrainedPos.getY(), pangle[-1])
        self.robotPosition = newPos
        return px, py, pangle

    def currentToPerimeter(self):
        dump_x, dump_y = self.robotRoom.dumpLoc
        currentPosition = self.getRobotPosition()
        perimeterIndex = random.randint(0, (len(self.robotRoom.closePerimeter) - 1))
        perimeter_x, perimeter_y = self.robotRoom.closePerimeter[perimeterIndex]
        endAngle = (math.atan2(currentPosition.getX() - dump_x, currentPosition.getY() - dump_y))
        px, py, pangle = Dubins.getDubinsPath(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), perimeter_x, perimeter_y, endAngle,
                                              currentPosition.turningRadius)
        if not self.isHoldFull(self.hold):
            for i, tmp in enumerate(px):
                if self.robotRoom.isTileDredgable and not self.robotRoom.isTileCleaned(tmp, py[i]):
                    self.robotRoom.dredgeTileAtPosition(tmp, py[i])
                    self.hold += 1
        self.path.append([px, py, pangle])
        newPos = currentPosition.setPosition(px[-1], py[-1], pangle[-1])
        self.robotPosition = newPos
        return px, py, pangle

    def toDump(self):
        dump_x, dump_y = self.robotRoom.dumpLoc
        currentPosition = self.getRobotPosition()
        endAngle = (math.atan2(currentPosition.getX() - dump_x, currentPosition.getY() - dump_y))
        px, py, pangle = Dubins.getDubinsPath(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(), dump_x, dump_y, endAngle,
                                              currentPosition.turningRadius)
        if not self.isHoldFull(self.hold):
            for i, tmp in enumerate(px):
                if self.robotRoom.isTileDredgable and not self.robotRoom.isTileCleaned(tmp, py[i]):
                    self.robotRoom.dredgeTileAtPosition(tmp, py[i])
                    self.hold += 1
        self.path.append([px, py, pangle])
        newPos = currentPosition.setPosition(px[-1], py[-1], pangle[-1])
        self.robotPosition = newPos
        self.hold = 0

class randomWaypoint(BaseShip):

    '''
    Parameters:
    self.waypointSeperation = 10
    self.nextAngleConstraint = 30
    self.numOfWaypoints = 3
    '''

    def findAllroutes(self):
        dump_x, dump_y = self.robotRoom.dumpLoc
        startAngle = 225 #TODO: make this a dependent vairable based on location youre heading to.
        currentPosition = self.getRobotPosition()
        startDredgeArea = self.robotRoom.getNumCleanedTiles() / self.robotRoom.getNumTiles()
        for _ in range (self.numOfWaypoints):
            px, py, pangle = Dubins.getDubinsPath(currentPosition.getX(), currentPosition.getY(),
                                              currentPosition.getHeading(),
                                              random.randint(0, self.robotRoom.dredgeAreaWidth),
                                              random.randint(self.robotRoom.dredgeAreaHeight,
                                                             self.robotRoom.roomHeight),
                                              math.radians(random.random(0,360)), currentPosition.turningRadius)
            if not self.isHoldFull(self.hold):
                for i in range(px):
                    if self.robotRoom.isTileDredgable(px[i], py[i]):
                        if not self.robotRoom.isTileCleaned(px[i], py[i]):
                            self.robotRoom.dredgeTileAtPosition(px[i], py[i])
            newDredgeArea = self.robotRoom.getNumCleanedTiles() / self.robotRoom.getNumTiles()
            self.path.append([px, py, pangle])
            self.pathDredgePercentage.append(startDredgeArea-newDredgeArea)
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
def generatePaths(speed, width, height, min_coverage, robot_type, visualize, waypointSeperation, nextAngleConstraint,
        numOfWaypoints):

    pathCollection = []
    coveragePath =[]
    tmpCollection = []
    tmpCoverage = []
    # tmpDredgeRoute =[]
    # print "Trial %i:" % m,
    if visualize: anim = ps11_visualize.RobotVisualization(width, height, int(width/2), int(height/2), .02)
    # create the room
    testRoom = RectangularRoom(width, height)
    testRoom.createDredgingLocations()
    testRoom.getPerimeter()
    robot = robot_type(testRoom, speed, int(waypointSeperation), nextAngleConstraint,
        int(numOfWaypoints))

    # initialize for this trial
    percentClean = 0.0000000
    progressList = []
    numberOfCurves = 0
    while percentClean < min_coverage:  # clean until percent clean >= min coverage
        if visualize: anim.update(testRoom, [robot])
        robot.dumpToPerimeter()
        for _ in range(robot.numOfWaypoints):
            robot.dredgeRoute()
            # tmpDredgeRoute = robot.dredgeRoute()
            # tmpCoverage.append([tmpDredgeRoute]) #just the coverage of the dredging area
        # robot.currentToPerimeter()
        robot.toDump()
        numberOfCurves += 1
        percentClean = float(testRoom.getNumCleanedTiles()) / float(testRoom.getNumTiles())
        # print(f'percent clean {percentClean}')
        # print(f'percent cleaned: {percentClean}')
        # progressList.append(percentClean)
        tmpCollection.append(robot.path) #Path is the entire route from start to end
    if visualize: anim.done()
    # trialsCollection.append(progressList)
    # coveragePath.append(tmpCoverage)
    pathCollection.append(tmpCollection)

        # print "%i robot(s) took %i clock-ticks to clean %i %% of a %ix%i room." %(num_robots, len(progressList), int(min_coverage * 100), width, height)
    # averageOfTrials = calcAvgLengthList(trialsCollection)
    # print "On average, the %i robot(s) took %i clock ticks to %f clean a %i x %i room." %(num_robots, int(averageOfTrials), min_coverage, width, height)
    return numberOfCurves, pathCollection

def createPathSimulation(num_robots, speed, width, height, num_trials, robot_type, visualize, ):


    trialsCollection = []  # list to hold lists of date from each trial
    for m in range(num_trials):  # for each trial
        # print "Trial %i:" % m,
        if visualize: anim = ps11_visualize.RobotVisualization(num_robots, width, height, int(width/2), int(height/2), .02)
        # create the room
        testRoom = RectangularRoom(width, height)
        testRoom.createDredgingLocations()
        testRoom.getPerimeter()
        # create robots and put them in a list
        robotList = []
        for i in range(num_robots):
            robotList.append(robot_type(testRoom, speed, testRoom.dredgeArea))

        # initialize for this trial
        percentClean = 0.0000000
        progressList = []
        x = 0
        k = 1000
        while x < k:
            print(x)
            if visualize: anim.update(testRoom, robotList)
            for eachRobot in robotList:  #for each time1-step make each robot clean
                testRoom.resetTiles()
                eachRobot.findOptDredgeRoute()
            x += 1
        if visualize: anim.done()
        trialsCollection.append(progressList)
        coverage = []

        for idx, val in enumerate(eachRobot.path):
            if idx < 10:
                coverage.append((idx, val[0]))
            else:
                coverage.sort(key=lambda j: j[1])
                for k, cov in enumerate(coverage):
                    if val[0] > cov[1]:
                        coverage[k] = (idx, val[0])
                        break
        print(coverage)



        # print "%i robot(s) took %i clock-ticks to clean %i %% of a %ix%i room." %(num_robots, len(progressList), int(min_coverage * 100), width, height)
    # averageOfTrials = calcAvgLengthList(trialsCollection)
    # print "On average, the %i robot(s) took %i clock ticks to %f clean a %i x %i room." %(num_robots, int(averageOfTrials), min_coverage, width, height)
    return trialsCollection

def runSimulation(num_robots, speed, width, height, min_coverage, num_trials, robot_type, visualize):
    """
    Runs NUM_TRIALS trials of the simulation and returns a list of
    lists, one per trial. The list for a trial has an element for each
    timestep of that trial, the value of which is the percentage of
    the room that is clean after that timestep. Each trial stops when
    MIN_COVERAGE of the room is clean.

    The simulation is run with NUM_ROBOTS robots of type ROBOT_TYPE,
    each with speed SPEED, in a room of dimensions WIDTH x HEIGHT.

    Visualization is turned on when boolean VISUALIZE is set to True.

    num_robots: an int (num_robots > 0)
    speed: a float (speed > 0)
    width: an int (width > 0)
    height: an int (height > 0)
    min_coverage: a float (0 <= min_coverage <= 1.0)
    num_trials: an int (num_trials > 0)
    robot_type: class of robot to be instantiated (e.g. Robot or
                RandomWalkRobot)
    visualize: a boolean (True to turn on visualization)

    test case:
    avg = runSimulation(10, 1.0, 15, 20, 0.8, 30, Robot, False)

    """
    k = 5
    trialsCollection = []  # list to hold lists of date from each trial
    for m in range(num_trials):  # for each trial

        # print "Trial %i:" % m,
        if visualize: anim = ps11_visualize.RobotVisualization(num_robots, width, height, int(width / 2),
                                                               int(height / 2), .02)
        # create the room
        testRoom = RectangularRoom(width, height)
        testRoom.createDredgingLocations()
        # create robots and put them in a list
        robotList = []
        for i in range(num_robots):
            robotList.append(robot_type(testRoom, speed, testRoom.dredgeArea))

        # initialize for this trial
        percentClean = 0.0000000
        progressList = []
        initial_time = time.time()

        while ((time.time()-initial_time) < k):
            if visualize: anim.update(testRoom, robotList)
            for eachRobot in robotList:  # for each time1-step make each robot clean
                eachRobot.moveForward()
        initial_time = time.time()
        while ((time.time()-initial_time) < k):
            if visualize: anim.update(testRoom, robotList)
            for eachRobot in robotList:  # for each time1-step make each robot clean
                eachRobot.moveLeft()
        initial_time = time.time()
        while ((time.time()-initial_time) < k):
            if visualize: anim.update(testRoom, robotList)
            for eachRobot in robotList:  # for each time1-step make each robot clean
                eachRobot.moveForward()
        initial_time = time.time()
        while ((time.time()-initial_time) < k):
            if visualize: anim.update(testRoom, robotList)
            for eachRobot in robotList:  # for each time1-step make each robot clean
                eachRobot.moveRight()
        if visualize: anim.done()
        trialsCollection.append(progressList)
        # print "%i robot(s) took %i clock-ticks to clean %i %% of a %ix%i room." %(num_robots, len(progressList), int(min_coverage * 100), width, height)
    # averageOfTrials = calcAvgLengthList(trialsCollection)
    # print "On average, the %i robot(s) took %i clock ticks to %f clean a %i x %i room." %(num_robots, int(averageOfTrials), min_coverage, width, height)
    return trialsCollection

    # trialsCollection = []  # list to hold lists of date from each trial
    # for m in range(num_trials):  # for each trial
    #     # print "Trial %i:" % m,
    #     if visualize: anim = ps11_visualize.RobotVisualization(num_robots, width, height, int(width/2), int(height/2), .02)
    #     # create the room
    #     testRoom = RectangularRoom(width, height)
    #     testRoom.createDredgingLocations()
    #     # create robots and put them in a list
    #     robotList = []
    #     for i in range(num_robots):
    #         robotList.append(robot_type(testRoom, speed, testRoom.dredgeArea))
    #
    #     # initialize for this trial
    #     percentClean = 0.0000000
    #     progressList = []
    #     while percentClean < min_coverage:  # clean until percent clean >= min coverage
    #         if visualize: anim.update(testRoom, robotList)
    #         for eachRobot in robotList:  # for each time1-step make each robot clean
    #             if eachRobot.isPathPlanned == False and eachRobot.isHoldFull(eachRobot.hold):
    #                 pos = eachRobot.getRobotPosition()
    #                 eachRobot.path = astar(testRoom.roomWidth, testRoom.roomHeight, (int(pos.getX()), int(pos.getY())),
    #                                                                                      testRoom.dumpLoc)
    #                 eachRobot.isPathPlanned = True
    #             eachRobot.updatePositionAndClean()
    #             if eachRobot.hold == 0: #terrible do this with more finesse
    #                 eachRobot.isPathPlanned = False
    #         percentClean = float(testRoom.getNumCleanedTiles()) / float(testRoom.getNumTiles())
    #         progressList.append(percentClean)
    #     if visualize: anim.done()
    #     trialsCollection.append(progressList)
    #
    #     # print "%i robot(s) took %i clock-ticks to clean %i %% of a %ix%i room." %(num_robots, len(progressList), int(min_coverage * 100), width, height)
    # # averageOfTrials = calcAvgLengthList(trialsCollection)
    # # print "On average, the %i robot(s) took %i clock ticks to %f clean a %i x %i room." %(num_robots, int(averageOfTrials), min_coverage, width, height)
    # return trialsCollection

def computeMeans(list_of_lists):
    """
    Returns a list as long as the longest list in LIST_OF_LISTS, where
    the value at index i is the average of the values at index i in
    all of LIST_OF_LISTS' lists.

    Lists shorter than the longest list are padded with their final
    value to be the same length.
    """
    # Find length of longest list
    longest = 0
    for lst in list_of_lists:
        if len(lst) > longest:
            longest = len(lst)
    # Get totals
    tots = [0] * (longest)
    for lst in list_of_lists:
        for i in range(longest):
            if i < len(lst):
                tots[i] += lst[i]
            else:
                tots[i] += lst[-1]
    # Convert tots to an array to make averaging across each index easier
    tots = pylab.array(tots)
    # Compute means
    means = tots / float(len(list_of_lists))
    return means


def calcAvgLengthList(listOfLists):
    """
    Takes a list of lists and then calculates the average length of the lists
    """
    sumOfLengths = 0
    averageLength = 0
    for eachList in listOfLists:
        sumOfLengths += len(eachList)
    averageLength = sumOfLengths / len(listOfLists)
    # print averageLength
    return averageLength


def showPlot1():
    """
    Produces a plot showing dependence of cleaning time1 on room size.

    How long does it take a single robot to clean 75% of each of the following types of rooms: 5x5, 10x10, 15x15, 20x20, 25x25? Output a figure that plots the mean time1 (on the Y-axis) against the area of the room.
    """
    print
    """How long does it take a single robot to clean 75% of each of the following types of rooms: 5x5, 10x10, 15x15, 20x20, 25x25? Output a figure that plots the mean time1 (on the Y-axis) against the area of the room."""

    square_size = [5, 10, 15, 20, 25]
    listOfMeanTimes = []
    for each in square_size:
        trialsCollection = runSimulation(1, 1.0, each, each, 0.75, 25, Robot, False)
        averageOfTrials = calcAvgLengthList(trialsCollection)
        print
        "On average, the robot took %i clock ticks to clean 75%% of a %i x %i room." % (
        int(averageOfTrials), each, each)
        listOfMeanTimes.append((each, int(averageOfTrials)))
    write_lists_csv(listOfMeanTimes, "robot-x-size-of-square-room.csv", ["Size of Square Room", "Mean Time"])


def showPlot2():
    """
    Produces a plot showing dependence of cleaning time1 on number of robots.

    How long does it take to clean 75% of a 25x25 room with each of 1-10 robots? Output a figure that plots the mean time1 (on the Y-axis) against the number of robots.

    """
    listOfMeanTimes = []
    for each in range(1, 11):
        trialsCollection = runSimulation(each, 1.0, 25, 25, 0.75, 25, Robot, False)
        averageOfTrials = calcAvgLengthList(trialsCollection)
        # print "On average, the robot took %i clock ticks to clean 75%% of a %i x %i room." % (int(averageOfTrials), each, each)
        listOfMeanTimes.append((each, int(averageOfTrials)))
    write_lists_csv(listOfMeanTimes, "numRobot-x-square-room.csv", ["Num of Robots", "Mean Time"])


def showPlot3():
    """
    Produces a plot showing dependence of cleaning time1 on room shape.
    How long does it take two robots to clean 75% of rooms with dimensions 20x20, 25x16, 40x10, 50x8, 80x5, and 100x4? (Notice that the rooms have the same area.) Output a figure that plots the mean time1 (on the Y-axis) against the ratio of width to height.
    """
    room_size = [(20, 20), (25, 16), (40, 10), (50, 8), (80, 5), (100, 4)]
    listOfMeanTimes = []
    for each in room_size:
        trialsCollection = runSimulation(1, 1.0, each[0], each[1], 0.75, 25, Robot, False)
        averageOfTrials = calcAvgLengthList(trialsCollection)
        print
        "On average, the robot took %i clock ticks to clean 75%% of a %i x %i room." % (
        int(averageOfTrials), each[0], each[1])
        listOfMeanTimes.append((each, int(averageOfTrials)))
    # print "write_lists_csv(listOfMeanTimes,"room-shape.csv", ["Means"])"
    write_lists_csv(listOfMeanTimes, "room-shape.csv", ["Room Dimensions", "Means"])


def showPlot4():
    """
    Produces a plot showing cleaning time1 vs. percentage cleaned, for
    each of 1-5 robots.
    How does the time1 it takes to clean a 25x25 room vary as min_coverage changes? Output a figure that plots mean time1 (on the Y-axis) against the percentage cleaned, for each of 1-5 robots. Your plot will have multiple curves.
    """
    percentClean = [.25, .5, .75, .8, .9, 1]
    listOfMeanTimes = []
    for i in range(1, 11):
        for each in percentClean:
            trialsCollection = runSimulation(i, 1.0, 25, 25, each, 25, Robot, False)
            averageOfTrials = calcAvgLengthList(trialsCollection)
            listOfMeanTimes.append((i, each, int(averageOfTrials)))
    write_lists_csv(listOfMeanTimes, "robots-percentClean.csv", ["Num. Robots", "Percent cleaned", "Means"])



def write_lists_csv(block_list, file_name, headers):
    """
    Takes a list or list of lists, a files location//name, and a list of headers
    Writes the itemsof the lists as rows in a CSV file.  Each item of the list is a comma-separated value.
    Returns the location of the CSV file.
    """
    fileWriter = csv.writer(open(file_name, 'wb'), delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    fileWriter.writerow(headers)
    for each in block_list:
        fileWriter.writerow(each)

    # fileWriter = open(file_name, 'w')
    # fileWriter.writerow(headers)
    # for item in block_list:
    #   fileWriter.write("%s\n" % item)


def showPlot1A():
    """
    Produces a plot showing dependence of cleaning time1 on room size.
    """

    room_sizes = ((5, 5), (10, 10), (15, 15), (20, 20), (25, 25))
    room_areas = ()
    for rs in room_sizes:
        room_areas += (rs[0] * rs[1],)

    room_size_time = ()
    num_trials = 100
    min_coverage = 0.75
    robots_num = 1
    for rs in room_sizes:
        ll = runSimulation(robots_num, 1, rs[0], rs[1], min_coverage, num_trials, Robot, False)
        assert len(ll) == num_trials, "Some error, num_trials != len(ll) "
        room_size_time += (sum([len(l) for l in ll]) / float(num_trials),)
    plot(room_areas, room_size_time, linestyle='--', lw=2, marker='o', \
         markeredgecolor='k', markerfacecolor='r', markersize=8)
    axis([20, 630, 0, 1050])
    title('One robot. Time to clean 75% room size for different room sizes')
    ylabel("Average time1 over %s trials" % num_trials)
    xlabel("Room area")
    show()

def saveBest(pathout, filename ,value):
    try:
        with open(pathout+filename, 'a+', newline='') as sb:
            csvWriter = csv.writer(sb)
            csvWriter.writerow(value)
            print('wrote')
    except IOError:
        print('not writable')
        print(pathout+filename)

# === Run code
setSize = 100
# (speed, width, height, min_coverage, robot_type, visualize, waypointSeperation, nextAngleConstraint, numOfWaypoints):
iterator = 0
def mainLoop(waypointSeperation, numOfWaypoints):
    print("Running Simulation")
    focPath = 'C:\\Users\\denma\\Documents\\Uni\Thesis\\Simulator\\optimising_TSHD_path\\Sim\FamilyofCurves\\'
    bestPath = 'C:\\Users\\denma\\Documents\\Uni\\Thesis\\Simulator\\optimising_TSHD_path\\Sim\\BestPath\\'
    curveScores = []
    now = datetime.now()
    dt_string = now.strftime("%d-%m-%H-%M-%S")
    for k in range(setSize):
        print(f'set {k} of 100')
        RobotAvg = generatePaths(0.5, 70, 70, 0.9, setDistanceWapoint, False, waypointSeperation, 30,
            numOfWaypoints)
        saveBest(focPath, dt_string, [RobotAvg[1]])
        curveScores.append(RobotAvg[0])
    # data, initialSolution, initialTemp, iterationLimit, finalTemp, tempReduction, neighborOperator,
    # iterationPerTemp=100, alpha=10, beta=5)
    gc.collect()
    subsetSelection = SimulatedAnealing.SimulatedAnnealing(curveScores, int(setSize/2), -1, "linear")
    print(f'number of random greedy searches: {int(setSize / 2)}')
    best, index = subsetSelection.runGreedy()
    print(index, best)
    df = pd.read_csv(focPath+dt_string)
    print(df.shape)
    bestFamilyofCurves = df[index]
    saveBest(bestPath, dt_string, )
    return best

pbounds = {'waypointSeperation': (1, 15), 'numOfWaypoints': (1, 15)}
optimizer = BayesianOptimization(
    f=mainLoop,
    pbounds=pbounds,
    random_state=1,
)

optimizer.maximize(
    init_points=50,
    n_iter=1000,
)
print(optimizer.max)

# optSpace = [Integer(1, 15, name='waypointSeperation'), Integer(1, 15, name='numOfWapoints')]

mainLoop(15,15)