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

        self.score = 0


def fixturePenalty(position, map_, minDist):
    for i in range(len(map_)):
        posFixture = map_[i].getBasePosition()
        dist = np.linalg.norm(posFixture[0:2]-position[0:2]) # only distance in xy plane thats relevant
        score = 0
        if dist < minDist:
            score += -3
            score += dist - minDist
    return score


def distanceMovedPenalty(initPose, endPose):
    dist = np.linalg.norm(initPose[0:2] - endPose[0:2])
    return - dist*3

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
        self.leftGrippPoint = 0 
        self.rightGrippPoint = 0
        self.fixtureRadius = 0.06

    def updateInit(self, DLO, map_, rewardFunctions, mode, grippWidth, targetFixture, previousFixture, tfListener):
        self.DLO = DLO
        self.mode = mode
        self.grippWidth = grippWidth
        self.targetFixture = targetFixture
        self.previousFixture = previousFixture
        self.map = map_
        self.rewardFunctions = rewardFunctions

        if mode == 'individual':
            self.leftGrippPoint, self.rightGrippPoint = utils.calcGrippPoints(targetFixture, map_, DLO, grippWidth, clipPoint)
            if self.previousFixture < 0:
                start = 0
            else:
                minDist, point, minIndex = utils.closesPointDLO(self.DLO, self.map[previousFixture].getClippPosition())
                start = point

            if self.leftGrippPoint > self.rightGrippPoint:
                leftStart = self.leftGrippPoint 
                leftEnd = DLO.getLength()
                rightStart = start
                rightEnd = self.rightGrippPoint 
            else:
                rightStart = self.leftGrippPoint 
                rightEnd = DLO.getLength()
                leftStart = start
                leftEnd = self.rightGrippPoint 
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
            if self.mode == 'individual':
                individual = self.generateInitIndividual(posMSTD=0.3, rotDegSTD=30)
                self.population.append(individual)
            else:
                pass
        for i in range(numGenerations):
            pass
            # evaluate
            # resample 
 
        # check solution is feasible  
        # return solution 

    def generateInitIndividual(self, posMSTD, rotDegSTD):
        individual = Individual(self.mode)
        low = self.leftPickupRange[0]
        high = leftPickupRange[1]
        individual.leftPickupPoint = np.random.default_rng().uniform(low=low, high=high)

        low = self.rightPickupRange[0]
        high = rightPickupRange[1]
        individual.rightPickupPoint = np.random.default_rng().uniform(low=low, high=high)

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

        # if both pickup points are in reach 
        #if individual.leftPickupInReach == 1 and individual.rightPickupInReach == 1:
        dy = pointLeft[1] - pointRight[1]
        dx = pointLeft[0] - pointRight[0]
        angle = np.arctan2(dy, dx)
        if np.random.random() < 0.5:
            newRightX = np.random.normal(pointRight[0], posMSTD)
            newRightY = np.random.normal(pointRight[1], posMSTD)
            newZ = 0.03    
            individual.rightEndPosition = np.array([newRightX, newRightY, newZ])
            pickupDist = abs(individual.rightPickupPoint - individual.leftPickupPoint) 
            angleNew = np.random.normal(angle, rotDegSTD * np.pi/180)
            newLeftX = newRightX + pickupDist*np.cos(angleNew) 
            newLeftY = newRightY + pickupDist*np.sin(angleNew)
            individual.leftEndPosition = np.array([newLeftX, newLeftY, newZ])
            
        else:
            newLeftX = np.random.normal(pointLeft[0], posMSTD)
            newLeftY = np.random.normal(pointLeft[1], posMSTD)
            newZ = 0.03    
            individual.leftEndPosition = np.array([newLeftX, newLeftY, newZ])
            pickupDist = abs(individual.rightPickupPoint - individual.leftPickupPoint) 
            angleNew = np.random.normal(angle + np.pi, rotDegSTD * np.pi/180)
            newRightX = newLeftX + pickupDist*np.cos(angleNew) 
            newRightY = newLeftY + pickupDist*np.sin(angleNew)
            individual.rightEndPosition = np.array([newRightX, newRightY, newZ])

        individual.rightEndOrientation = angle
        individual.leftEndOrientation = angle
        '''
        # if Left pickup points are in reach 
        elif individual.leftPickupInReach == 1:
            newLeftX = np.random.normal(pointLeft[0], posMSTD)
            newLeftY = np.random.normal(pointLeft[1], posMSTD)
            newZ = 0.03    
            individual.leftEndPosition = np.array([newLeftX, newLeftY, newZ])
            rotZLeft = getZRotationCable(individual.leftPickupPoint, self.DLO) - np.pi/2
            individual.leftEndOrientation = tf.transformations.quaternion_from_euler(rotZLeft, 0, np.pi, 'rzyx')
        # if right pickup points are in reach 
        elif individual.rightPickupInReach == 1:
            newRightX = np.random.normal(pointRight[0], posMSTD)
            newRightY = np.random.normal(pointRight[1], posMSTD)
            newZ = 0.03    
            individual.rightEndPosition = np.array([newRightX, newRightY, newZ])
            rotZRight = getZRotationCable(individual.rightPickupPoint, self.DLO) - np.pi/2
            individual.rightEndOrientation = tf.transformations.quaternion_from_euler(rotZRight, 0, np.pi, 'rzyx')
        '''
        return individual
        
    def generateSample(self, base):
        pass
        # return new individual

    def resample(self):
        # Normalize sum scores
        pass

    def evaluate(self, individual):
        score = 0 
        if self.mode == 'individual':
            initRightPos = self.DLO.getCoord(self.rightGrippPoint)
            initLeftPos = self.DLO.getCoord(self.leftGrippPoint)
            l1 = abs(individual.rightPickupPoint - self.rightGrippPoint)
            l2 = abs(self.leftGrippPoint - self.rightGrippPoint)
            l3 = abs(self.leftGrippPoint - individual.leftPickupPoint)

            if individual.leftPickupInReach == 1 and individual.rightPickupInReach == 1:
                # Both grippers pickup reward 
                score += 3

                # "Predict rope"
                leftEndPosition = individual.leftEndPosition
                rightEndPosition = individual.rightEndPosition
                vec = utils.normalize(leftEndPosition - rightEndPosition)
                rightEndPickupPoint = rightEndPosition + vec * l1
                leftEndPickupPoint = rightEndPosition + vec * (l1 + l2)

            elif individual.leftPickupInReach == 1:
                score += 1

                # "Predict rope"
                leftEndPosition = individual.leftEndPosition
                rightEndPosition = individual.rightEndPosition

                dist = np.linalg.norm(leftEndPosition - initLeftPos)
                if dist > l3:
                    vec = utils.normalize(leftEndPosition - initLeftPos)
                    leftEndPickupPoint = initLeftPos + vec * (dist-l3)
                else:
                    leftEndPickupPoint = initLeftPos

                dist =  np.linalg.norm(leftEndPickupPoint - initRightPos)
                if dist > l2:
                    vec = utils.normalize(leftEndPickupPoint - initRightPos)
                    rightEndPickupPoint = initRightPos + vec * (dist-l2)
                else:
                    rightEndPickupPoint = initRightPos

            elif individual.rightPickupInReach == 1:
                score += 1
                # "Predict rope"
                leftEndPosition = individual.leftEndPosition
                rightEndPosition = individual.rightEndPosition

                dist = np.linalg.norm(rightEndPosition - initRightPos)
                if dist > l1:
                    vec = utils.normalize(rightEndPosition - initRightPos)
                    rightEndPickupPoint = initRightPos + vec * (dist-l1)
                else:
                    rightEndPickupPoint = initRightPos

                dist =  np.linalg.norm(rightEndPickupPoint - initLeftPos)
                if dist > l2:
                    vec = utils.normalize(rightEndPickupPoint - initLeftPos)
                    leftEndPickupPoint = initLeftPos + vec * (dist-l2)
                else:
                    leftEndPickupPoint = initLeftPos
            else:
                return -= 30    

            # To close to fixture, penalty 
            score += fixturePenalty(leftEndPosition, self.map, self.fixtureRadius)
            score += fixturePenalty(rightEndPosition, self.map, self.fixtureRadius)
            score += fixturePenalty(rightEndPickupPoint, self.map, self.fixtureRadius)
            score += fixturePenalty(leftEndPickupPoint, self.map, self.fixtureRadius)

            # Distance moved penalty

            score += distanceMovedPenalty(initRightPos, rightEndPickupPoint)
            score += distanceMovedPenalty(initLeftPos, leftEndPickupPoint)

            # Score for distance between pickup points

            score += np.linalg.norm(rightEndPickupPoint - leftEndPickupPoint)

            # penalty for end pickup out of reach 

            # penalty for crossing

            # reward for inside rope constraint 


        return score