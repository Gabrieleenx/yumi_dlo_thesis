import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils, utilsSolve

class Solve(object):
    def __init__(self):
        self.DLO = np.zeros((3,70))
        self.map = []
        self.rightPickupRange = np.zeros(2)
        self.leftPickupRange = np.zeros(2)
        self.mode = 'individual'
        self.population = []
        self.targetFixture = 0
        self.previousFixture = -1
        self.reachLeftCentrum = np.array([0.138, 0.106, 0.462])
        self.reachRightCentrum = np.array([0.138, -0.106, 0.462])
        self.leftGrippPoint = 0 
        self.rightGrippPoint = 0
        self.fixtureRadius = 0.06
        self.previousFixtureDLOLength = 0
        self.cableSlack = 0
        self.reach = 0.559 - 0.03 
        self.individualSTD = np.array([0.05, 0.05, 0.05, 0.05, 15 * np.pi/180])
        self.combinedSTD = np.array([0.05, 0.05, 10 * np.pi/180])

        self.crossOverProbability = 0.4
        self.mutationProbabiliy = 0.8
        self.flippProbability = 0.05

        self.initAngle = 0
        self.initPosRight = np.zeros(3)
        self.initPosLeft = np.zeros(3)
        self.initAbsPos = np.zeros(3)

    def updateInit(task):
        self.task = task
        self.mode = self.task.mode # for convinience
        self.DLO = task.DLO # for convinience 
        if task.mode == 'individual':
            clipPoint = utils.calcClipPoint(targetFixture=task.targetFixture,\
                                            previousFixture=task.previousFixture,\
                                            map_=task.map,\
                                            cableSlack=task.DLO,\
                                            DLO=task.DLO)
                
            self.leftGrippPoint, self.rightGrippPoint = utils.calcGrippPoints(targetFixture=task.targetFixture,\
                                                                            map_=task.map,\
                                                                            DLO=task.DLO,\
                                                                            grippWidth=task.grippWidth,\
                                                                            clipPoint)
                                                                        
            self.rightPickupRange, self.leftPickupRange, self.initAngle = \
                                    utilsSolve.pickupRangeAndAngle(task=task,\
                                                                    rightGrippPoint=self.rightGrippPoint,\
                                                                    leftGrippPoint=self.leftGrippPoint)
    
        else:
            self.rightGrippPoint, self.leftGrippPoint, self.initAbsPos, self.initAngle = \
                                                utilsSolve.absPosAngleGrippPoints(task=task)

        
        (Link7ToGripperRight, _) = tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        self.Link7ToGripperRight = np.asarray(Link7ToGripperRight)
        (Link7ToGripperLeft, _) = tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))
        self.Link7ToGripperLeft = np.asarray(Link7ToGripperLeft)
    
  
    def solve(self, populationSize, numGenerations):

        # generate initial population
        scores = np.zeros(populationSize)
        self.population = []
        for i in range(populationSize):
            individual = self.generateInitIndividual()
            self.population.append(individual)

        # main loop over generations 
        for i in range(numGenerations):

            # evaluate population
            for j in range(populationSize):
                if self.mode == 'individual':
                    score = self.evaluateIndividual(self.population[j])
                else:
                    score = self.evaluateCombined(self.population[j])
                scores[j] = score
            print('generation ', i ,' scores max' , np.max(scores))

            # normalize scores
            scores, maxIndex = utilsSolve.normalizeSumScores(scores)

            # resample 
            tempPopulation = []

            for j in range(populationSize):
                randVal = np.random.random() 
                if randVal < self.crossOverProbability:
                    parrentOneIndex = sampleIndex(populationSize, scores)
                    parrentTwoIndex = sampleIndex(populationSize, scores)
                    individal = self.crossOver(self.population[parrentOneIndex], self.population[parrentTwoIndex])
                else:
                    individualIndex = sampleIndex(populationSize, scores)
                    individual = self.population[individualIndex]

                randVal = np.random.random() 

                if randVal < self.mutationProbabiliy:
                    individual = self.mutateIndividual(individual)

                tempPopulation.append(individual)

            # save best individual from previous generation 
            tempPopulation[0] = self.population[maxIndex]

            # update population
            self.population = tempPopulation.copy()

        return self.population[0] 


    def generateInitIndividual(self):

        individual = Individual(self.mode)
        if self.mode == 'individual':
            # initial pickup point, uniform in range
            low = self.leftPickupRange[0]
            high = self.leftPickupRange[1]
            individual.parametersIndividual[1] = np.random.default_rng().uniform(low=low, high=high)

            low = self.rightPickupRange[0]
            high = self.rightPickupRange[1]
            individual.parametersIndividual[0] = np.random.default_rng().uniform(low=low, high=high)

            # generate new position and orientation from normal distrobution 
            pointRight = self.DLO.getCoord(individual.parametersIndividual[0])
            pointLeft =  self.DLO.getCoord(individual.parametersIndividual[1])
            meanPickupX = 0.5*(pointRight[0] + pointLeft[0])
            meanPickupY = 0.5*(pointRight[1] + pointLeft[1])
            newAbsX = np.random.normal(pointRight[0], self.individualSTD[2]*4)
            newAbsY = np.random.normal(pointRight[1], self.individualSTD[3]*4)

            dy = pointLeft[1] - pointRight[1]
            dx = pointLeft[0] - pointRight[0]
            angle = np.arctan2(dy, dx) - np.pi/2

            newAbsAgnle = np.random.normal(angle, self.individualSTD[4]*4)

            individual.parametersIndividual[2] = newAbsX
            individual.parametersIndividual[3] = newAbsY
            individual.parametersIndividual[4] = newAbsAgnle

        elif self.mode == 'combined':

            individual.grippWidth = self.task.grippWidth
            if np.random.random() < 0.5:
                angle = np.random.normal(np.pi/2, 4*self.combinedSTD[2])
            else:
                angle = np.random.normal(-np.pi/2, 4*self.combinedSTD[2])
                
            newAbsX = np.random.normal(self.initAbsPos[0], self.combinedSTD[0]*4)
            newAbsY = np.random.normal(self.initAbsPos[1], self.combinedSTD[1]*4)
            individual.parametersIndividual[0] = newAbsX
            individual.parametersIndividual[1] = newAbsY
            individual.parametersCombined[2] = angle

        return individual
        
    def mutateIndividual(self, seedIndividual):
        individual = Individual(self.mode)
        if self.mode == 'individual':

            pickupPoints = seedIndividual.getPickupPoints()
            low = self.leftPickupRange[0]
            high = self.leftPickupRange[1]
            newLeftPickup = np.random.normal(pickupPoints[1] , self.individualSTD[1])
            individual.parametersIndividual[1] = np.clip(newLeftPickup, low, high)

            low = self.rightPickupRange[0]
            high = self.rightPickupRange[1]
            newRightPickup =  np.random.normal(pickupPoints[0], self.individualSTD[0])
            individual.parametersIndividual[0] = np.clip(newRightPickup, low, high)

            newAbsX = np.random.normal(seedIndividual.parametersIndividual[2], self.individualSTD[2])
            newAbsY = np.random.normal(seedIndividual.parametersIndividual[3], self.individualSTD[3])
            newAbsAgnle = np.random.normal(seedIndividual.parametersIndividual[4], self.individualSTD[4])

            individual.parametersIndividual[2] = newAbsX
            individual.parametersIndividual[3] = newAbsY
            individual.parametersIndividual[4] = newAbsAgnle
        elif self.mode == 'combined':
            newAbsX = np.random.normal(seedIndividual.parametersCombined[0], self.combinedSTD[0])
            newAbsY = np.random.normal(seedIndividual.parametersCombined[1], self.combinedSTD[1])

            if np.random.random() < self.flippProbability:
                newAbsAgnle = np.random.normal(-seedIndividual.parametersCombined[2], self.combinedSTD[2])
            else:
                newAbsAgnle = np.random.normal(seedIndividual.parametersCombined[2], self.combinedSTD[2])

            individual.parametersCombined[0] = newAbsX
            individual.parametersCombined[1] = newAbsY
            individual.parametersCombined[2] = newAbsAgnle

        return individual

    def crossOver(self, parentOne, parentTwo):
        individual = Individual(self.mode)
        if self.mode == 'individual':
            numElements = np.size(parentOne.parametersIndividual)
        else:
            numElements = np.size(parentOne.parametersCombined)

        crossOverPoint = np.random.randint(0,numElements)
        newParameters = np.zeros(numElements)

        if self.mode == 'individual':
            newParameters[0:crossOverPoint] = parentOne.parametersIndividual[0:crossOverPoint]
            newParameters[crossOverPoint:numElements] = parentTwo.parametersIndividual[crossOverPoint:numElements]
            individual.parametersIndividual = newParameters
        else:
            newParameters[0:crossOverPoint] = parentOne.parametersCombined[0:crossOverPoint]
            newParameters[crossOverPoint:numElements] = parentTwo.parametersCombined[crossOverPoint:numElements]
            individual.parametersCombined = newParameters
        
        return individual

    def evaluateIndividual(self, individual):
        score = 0 
        # final positions
        rightPos, leftPos, quatRight, quatLeft = individual.getRightLeftPosQuat(np.array([0.03, 0.03]))
        pickupPoints = individual.getPickupPoints()

        # To close to fixture, for end points, penalty 
        scoreFixtureRight, validFixtureRight = fixturePenalty(rightPos, self.map, self.fixtureRadius)
        scoreFixtureLeft, validFixtureLeft = fixturePenalty(leftPos, self.map, self.fixtureRadius)

        score += scoreFixtureRight
        score += scoreFixtureLeft
    
        individual.pickupLeftValid = validFixtureLeft
        individual.pickupRightValid = validFixtureRight
        
        # check if pickup point is in reach
        pointRight = self.DLO.getCoord(pickupPoints[0])
        pointRight += self.Link7ToGripperRight
        if np.linalg.norm(pointRight - self.reachRightCentrum) >= self.reach:
            individual.pickupRightValid = 0
        else: 
            individual.pickupRightValid = 1

        pointLeft = self.DLO.getCoord(pickupPoints[1])
        pointLeft += self.Link7ToGripperLeft
        if np.linalg.norm(pointLeft - self.reachLeftCentrum) >= self.reach:
            individual.pickupLeftValid = 0
        else: 
            individual.pickupLeftValid = 1

        # check if end point is in reach
        pointRight = np.copy(rightPos)
        pointRight += self.Link7ToGripperRight
        if np.linalg.norm(pointRight - self.reachRightCentrum) >= self.reach:
            individual.pickupRightValid = 0
        else: 
            individual.pickupRightValid = 1

        pointLeft = np.copy(leftPos)
        pointLeft += self.Link7ToGripperLeft
        if np.linalg.norm(pointLeft - self.reachLeftCentrum) >= self.reach:
            individual.pickupLeftValid = 0
        else: 
            individual.pickupLeftValid = 1

        # penalty for crossing
        angle = individual.parametersIndividual[4]
        if angle < -(np.pi/2 + (20*np.pi/180)) or angle > (np.pi/2 + (20*np.pi/180)):
            score += - 5 


        # penalty for outside side rope constraint 
        if self.previousFixture == -1:
            pass
        else:
            fixtureClippPosition = self.map[self.previousFixture].getClippPosition()

        
            lengthRight =  abs(self.previousFixtureDLOLength - pickupPoints[0])
            lengthLeft =  abs(self.previousFixtureDLOLength - pickupPoints[1])
            if individual.pickupLeftValid == 1 and individual.pickupRightValid == 1:
                if lengthRight < lengthLeft:
                    closesPoint  = rightPos
                    lengthRope = lengthRight
                else:
                    closesPoint = leftPos
                    lengthRope = lengthLeft
            elif individual.pickupLeftValid == 1:
                closesPoint = leftPos
                lengthRope = lengthLeft
            elif individual.pickupRightValid == 1:
                closesPoint = rightPos
                lengthRope = lengthRight
            else:
                closesPoint = np.zeros(3)
                lengthRope = 1        

            closestDist = np.linalg.norm(fixtureClippPosition - closesPoint)
            if closestDist > lengthLeft:
                score -= 4 + (closestDist - lengthRope)


        # "Predict rope" -----------------
        initRightPos = self.DLO.getCoord(self.rightGrippPoint)
        initLeftPos = self.DLO.getCoord(self.leftGrippPoint)

        l1 = abs(pickupPoints[0] - self.rightGrippPoint)
        l2 = abs(self.leftGrippPoint - self.rightGrippPoint)
        l3 = abs(self.leftGrippPoint - pickupPoints[1])
      
        if individual.pickupLeftValid == 1 and individual.pickupRightValid == 1:
            # Both grippers pickup reward 
            score += 3

            vec = utils.normalize(leftPos - rightPos)
            rightEndPickupPoint = rightPos + vec * l1
            leftEndPickupPoint = rightPos + vec * (l1 + l2)

        elif individual.pickupLeftValid == 1:
            score += 1

            dist = np.linalg.norm(leftPos - initLeftPos)
            if dist > l3:
                vec = utils.normalize(leftPos - initLeftPos)
                leftEndPickupPoint = initLeftPos + vec * (dist-l3)
            else:
                leftEndPickupPoint = initLeftPos

            dist =  np.linalg.norm(leftEndPickupPoint - initRightPos)
            if dist > l2:
                vec = utils.normalize(leftEndPickupPoint - initRightPos)
                rightEndPickupPoint = initRightPos + vec * (dist-l2)
            else:
                rightEndPickupPoint = initRightPos

        elif individual.pickupRightValid == 1:
            score += 1

            dist = np.linalg.norm(rightPos - initRightPos)
            if dist > l1:
                vec = utils.normalize(rightPos - initRightPos)
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
            return -3    

        # new pickup points to close to fixture
        scoreFixtureRight, validFixtureRight = fixturePenalty(rightEndPickupPoint, self.map, self.fixtureRadius)
        scoreFixtureLeft, validFixtureLeft = fixturePenalty(leftEndPickupPoint, self.map, self.fixtureRadius)

        score += scoreFixtureRight
        score += scoreFixtureLeft

        # penalty for pickup points too close 

        if np.linalg.norm(rightEndPickupPoint - leftEndPickupPoint) < 0.12:
            score += -2

        if np.linalg.norm(initRightPos - initLeftPos) < 0.12:
            score += -2

        # penalty for end pickup out of reach 
        pointRight = rightEndPickupPoint
        pointRight += self.Link7ToGripperRight

        if np.linalg.norm(pointRight - self.reachRightCentrum) >= self.reach-0.02:
            score += -(np.linalg.norm(pointRight - self.reachRightCentrum) - self.reach-0.02)
        else: 
            score += 2
        
        pointLeft = leftEndPickupPoint
        pointLeft += self.Link7ToGripperLeft
        if np.linalg.norm(pointLeft - self.reachLeftCentrum) >= self.reach-0.02:
            score += -(np.linalg.norm(pointLeft - self.reachLeftCentrum) - self.reach-0.02)
        else: 
            score += 2

        # penalty for distance from target pickup
        score += -3*abs(pickupPoints[0] - self.rightGrippPoint)
        score += -3*abs(pickupPoints[1] - self.leftGrippPoint)

        # angle close to init angle reward
        score += 0.5 / (abs(self.initAngle - individual.parametersIndividual[4]) + 1)

        # Distance moved penalty

        score += 2*distanceMovedPenalty(initRightPos, rightEndPickupPoint)
        score += 2*distanceMovedPenalty(initLeftPos, leftEndPickupPoint)

        
        return score
    

    def evaluateCombined(self, individual):
        score = 0 
        # final positions
        rightPos, leftPos, quatRight, quatLeft = individual.getRightLeftPosQuat(np.array([0.03, 0.03]))

        # To close to fixture, for end points, penalty 
        scoreFixtureRight, validFixtureRight = fixturePenalty(rightPos, self.map, self.fixtureRadius)
        scoreFixtureLeft, validFixtureLeft = fixturePenalty(leftPos, self.map, self.fixtureRadius)

        score += scoreFixtureRight
        score += scoreFixtureLeft

        if validFixtureLeft == 1 and validFixtureRight == 1:
            individual.combinedValid = 1
        else:
            individual.combinedValid = 0
        

        # check if end point is in reach
        pointRight = np.copy(rightPos)
        pointRight += self.Link7ToGripperRight
        if np.linalg.norm(pointRight - self.reachRightCentrum) >= self.reach -0.03 or np.linalg.norm(pointRight - self.reachLeftCentrum) >= self.reach-0.03:
            individual.combinedValid = 0
        else: 
            individual.combinedValid = 1

        pointLeft = np.copy(leftPos)
        pointLeft += self.Link7ToGripperLeft
        if np.linalg.norm(pointLeft - self.reachLeftCentrum) >= self.reach-0.03 or np.linalg.norm(pointLeft - self.reachRightCentrum) >= self.reach-0.03:
            individual.combinedValid = 0
        else: 
            individual.combinedValid = 1

        # penalty for crossing
        angle = individual.parametersCombined[2]
        if angle < -(np.pi/2 + (20*np.pi/180)) or angle > (np.pi/2 + (20*np.pi/180)):
            score += - 5 
        
        # reward for angle close to pi/2 or -pi/2
        if abs(abs(angle) - np.pi/2) < (20*np.pi/180):
            score += 2
            score += 8/(abs(abs(angle) - np.pi/2) + 1)
        else:
            score += -1

        # penalty for outside side rope constraint 
        if self.previousFixture == -1:
            pass
        else:
            fixtureClippPosition = self.map[self.previousFixture].getClippPosition()

            lengthRight =  abs(self.previousFixtureDLOLength -  self.rightGrippPoint )
            lengthLeft =  abs(self.previousFixtureDLOLength -  self.leftGrippPoint )
            if lengthRight < lengthLeft:
                closesPoint  = rightPos
                lengthRope = lengthRight
            else:
                closesPoint = leftPos
                lengthRope = lengthLeft
            

            closestDist = np.linalg.norm(fixtureClippPosition - closesPoint)
            if closestDist > lengthLeft:
                score -= 4 + (closestDist - lengthRope)

        # Distance moved penalty
        score += 2*distanceMovedPenalty(self.initPosRight , rightPos)
        score += 2*distanceMovedPenalty(self.initPosLeft , leftPos)

        if individual.combinedValid == 0:
            score += -4
        
        return score