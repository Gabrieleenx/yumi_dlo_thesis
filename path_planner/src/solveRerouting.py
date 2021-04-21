import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils


class Individual(object):
    def __init__(self, mode):
        self.mode
 
        self.rightPickupPoint = 0
        self.leftPickupPoint = 0

        self.rightPickupInReach = 0
        self.leftPickupInReach = 0

        self.rightEndPosition = np.zeros(3)
        self.rightEndOrientation = 0
        self.leftEndPosition = np.zeros(3)
        self.leftEndOrientation = 0

        self.absolutePosition = np.zeros(3)
        self.absoluteOrientationZ = 0
        

class Solve(object):
    def __init__(self):
        self.DLO = np.zeros((3,70))
        self.map = []
        self.rewardFunctions = []
        self.rightPickupRange = np.zeros(2)
        self.leftPickupRange = np.zeros(2)
        self.mode
        self.population = []
        self.targetFixture = 0
        self.previousFixture = -1
        self.reachLeftCentrum = np.array([0.138, 0.106, 0.462])
        self.reachRightCentrum = np.array([0.138, -0.106, 0.462])

    def updateInit(self, DLO, map_, rewardFunctions, mode, grippWidth, targetFixture, previousFixture, tfListener):
        self.DLO = DLO
        self.mode = mode
        self.grippWidth = grippWidth
        self.targetFixture = targetFixture
        self.previousFixture = previousFixture
        self.map = map_
        self.rewardFunctions = rewardFunctions

        if mode == 'individual':
            leftGrippPoint, rightGrippPoint = utils.calcGrippPoints(targetFixture, map_, DLO, grippWidth, clipPoint)
            if self.previousFixture < 0:
                start = 0
            else:
                minDist, point, minIndex = utils.closesPointDLO(self.DLO, self.map[previousFixture].getClippPosition())
                start = point

            if leftGrippPoint > rightGrippPoint:
                leftStart = leftGrippPoint - self.grippWidth/2
                leftEnd = DLO.getLength()
                rightStart = start
                rightEnd = rightGrippPoint + self.grippWidth/2
            else:
                rightStart = leftGrippPoint - self.grippWidth/2
                rightEnd = DLO.getLength()
                leftStart = start
                leftEnd = rightGrippPoint + self.grippWidth/2
            self.rightPickupRange = np.array([rightStart, rightEnd])
            self.leftPickupRange = np.array([leftStart, leftEnd])
                
        (Link7ToGripperRight, _) = tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        self.Link7ToGripperRight = np.asarray(Link7ToGripperRight)
        (Link7ToGripperLeft, _) = tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))
        self.Link7ToGripperLeft = np.asarray(Link7ToGripperLeft)

  

    def solve(self, populationSize, numGenerations):
        # generate initial population
        
        self.population = []
        for i in range(populationSize):
            individual = self.generateInitIndividual()
            self.population.append(individual)

        for i in range(numGenerations):
            pass
            # evaluate
            # resample 
 
        # check solution is feasible  
        # return solution 

    def generateInitIndividual(self):
        individual = Individual(self.mode)
        low = self.leftPickupRange[0]
        high = leftPickupRange[1]
        individual.leftPickupPoint = np.random.default_rng().uniform(low=low, high=high)

        low = self.rightPickupRange[0]
        high = rightPickupRange[1]
        individual.rightPickupPoint = np.random.default_rng().uniform(low=low, high=high)

        if self.previousFixture < 0:
            # check if pickup point is in reach
            pointRight = self.DLO.getCoord(individual.rightPickupPoint)
            pointRight += self.Link7ToGripperRight
            if np.linalg.norm(pointRight - self.reachRightCentrum) >= self.reach:
                individual.rightPickupInReach = 0
            else: 
                individual.rightPickupInReach = 1

            pointLeft = self.DLO.getCoord(individual.leftPickupPoint)
            pointLeft += self.Link7ToGripperLeft
            if np.linalg.norm(pointLeft - self.reachLeftCentrum) >= self.reach:
                individual.leftPickupInReach = 0
            else: 
                individual.leftPickupInReach = 1

            if individual.leftPickupInReach == 1 and individual.rightPickupInReach == 1:
                if np.random.random() <0.5:
                    newRightX = np.random.normal(pointRight[0], 0.4)
                    newRightY = np.random.normal(pointRight[1], 0.4)
                    newRightZ = 0.03    
                    individual.rightEndPosition = np.array([newRightX, newRightY, newRightZ])
                    pickupDist = abs(individual.rightPickupPoint - individual.leftPickupPoint) 
                    
                    
                    individual.leftEndPosition =
                else:
                    pass 
            elif individual.leftPickupInReach == 1:
                pass
            elif individual.rightPickupInReach == 1:
                pass

        else:
            pass

        return individual


    def generateSample(self, base):
        pass
        # return new individual

    def resample(self):

    def evaluate(self):