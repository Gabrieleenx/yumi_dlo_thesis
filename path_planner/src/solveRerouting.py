import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils, utilsSolve, collisionCheck

class Solve(object):
    def __init__(self):
        self.DLO = np.zeros((3,70))
        self.map = []
        self.rightPickupRange = np.zeros(2)
        self.leftPickupRange = np.zeros(2)
        self.mode = 'individual'
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
        self.currentGrippPositionRight = np.zeros(3)
        self.currentGrippPositionLeft = np.zeros(3)
        self.crossOverProbability = 0.9
        self.mutationConstant = 1 # mutatoin probability c/n, c = mutationConstant, n = num genes
        #self.flippProbability = 0.05

        self.initAbsPos = np.zeros(3)

    def updateInit(self, task):
        self.task = task
        self.mode = task.mode # for convinience
        self.DLO = task.DLO # for convinience 
        self.map = task.map # for convinience 

        if task.mode == 'individual':
            clipPoint = utils.calcClipPoint(targetFixture=task.targetFixture,\
                                            previousFixture=task.previousFixture,\
                                            map_=task.map,\
                                            cableSlack=task.cableSlack,\
                                            DLO=task.DLO,\
                                            gripWidth=task.grippWidth)
                
            self.leftGrippPoint, self.rightGrippPoint = utils.calcGrippPoints(targetFixture=task.targetFixture,\
                                                                            map_=task.map,\
                                                                            DLO=task.DLO,\
                                                                            grippWidth=task.grippWidth,\
                                                                            clipPoint=clipPoint)
                                                                        
            self.rightPickupRange, self.leftPickupRange, self.initAngle = \
                                    utilsSolve.pickupRangeAndAngle(task=task,\
                                                    rightGrippPoint=self.rightGrippPoint,\
                                                    leftGrippPoint=self.leftGrippPoint)
            self.initRightGrippPos = self.DLO.getCoord(self.rightGrippPoint)
            self.initLeftGrippPos = self.DLO.getCoord(self.leftGrippPoint)
            self.currentGrippPositionRight = task.gripperRight.getPosition()
            self.currentGrippPositionLeft = task.gripperLeft.getPosition()

        else:
            self.rightGrippPoint, self.leftGrippPoint, self.initAbsPos, self.initAngle = \
                                                utilsSolve.absPosAngleGrippPoints(task=task)

  
    def solve(self, populationSize, numGenerations):

        # generate initial population
        scores = np.zeros(populationSize)
        population = []
        for i in range(populationSize):
            individual = utilsSolve.generateInitIndividual(task=self.task,\
                                                    leftPickupRange=self.leftPickupRange,\
                                                    rightPickupRange=self.rightPickupRange,\
                                                    individualSTD=self.individualSTD,\
                                                    combinedSTD=self.combinedSTD,\
                                                    initAbsPos=self.initAbsPos)
            population.append(individual)

        # main loop over generations 
        for i in range(numGenerations):

            # evaluate population
            for j in range(populationSize):
                if self.mode == 'individual':
                    score = self.evaluateIndividual(population[j])
                else:
                    score = self.evaluateCombined(population[j])
                scores[j] = score
            print('generation ', i ,' scores max ' , np.max(scores))

            # normalize scores
            scores, maxIndex = utilsSolve.normalizeSumScores(scores)

            # resample 
            tempPopulation = []

            for j in range(populationSize):
                randVal = np.random.random() 
                if randVal < self.crossOverProbability:
                    parrentOneIndex = utilsSolve.sampleIndex(populationSize, scores)
                    parrentTwoIndex = utilsSolve.sampleIndex(populationSize, scores)
                    individual = utilsSolve.crossOver(task=self.task,\
                                            parentOne=population[parrentOneIndex],\
                                            parentTwo=population[parrentTwoIndex])
                else:
                    individualIndex = utilsSolve.sampleIndex(populationSize, scores)
                    individual = population[individualIndex]

                randVal = np.random.random() 

                individual = utilsSolve.mutateIndividual(task=self.task,\
                                                seedIndividual=individual,\
                                                leftPickupRange=self.leftPickupRange,\
                                                rightPickupRange=self.rightPickupRange,\
                                                individualSTD=self.individualSTD,
                                                combinedSTD=self.combinedSTD,
                                                mutationConstant=self.mutationConstant)

                tempPopulation.append(individual)

            # save best individual from previous generation 
            tempPopulation[0] = population[maxIndex]

            # update population
            population = tempPopulation.copy()
 
        return population[0] 


    def evaluateIndividual(self, individual):
        # init
        score = 0 
        individual.pickupLeftValid = True
        individual.pickupRightValid = True
        # final positions
        rightPos, leftPos, quatRight, quatLeft = individual.getRightLeftPosQuat(np.array([0.03, 0.03]))
        pickupPoints = individual.getPickupPoints()
        
        # pickup points 
        tempRightPos, tempLeftPos, tempRightQuat, tempLeftQuat, valid_ = utils.calcGrippPosRot(self.DLO, leftGrippPoint=pickupPoints[1], rightGrippPoint=pickupPoints[0],\
                                 targetHeightRight=0, targetHeightLeft=0)
       
        
        # To close to fixture, for end points, penalty 
        
        scoreFixtureRight, validFixtureRight = utilsSolve.fixturePenalty(position=rightPos, map_=self.map)
        scoreFixtureLeft, validFixtureLeft = utilsSolve.fixturePenalty(position=leftPos, map_=self.map)

        score += scoreFixtureRight
        score += scoreFixtureLeft
        individual.pickupRightValid = individual.pickupRightValid and validFixtureRight
        individual.pickupLeftValid = individual.pickupLeftValid and validFixtureLeft
        

        # To close to fixture, pickup, penalty 
        
        scoreFixtureRight, validFixtureRight = utilsSolve.fixturePenalty(position=tempRightPos, map_=self.map)
        scoreFixtureLeft, validFixtureLeft = utilsSolve.fixturePenalty(position=tempLeftPos, map_=self.map)

        score += scoreFixtureRight
        score += scoreFixtureLeft
        individual.pickupRightValid = individual.pickupRightValid and validFixtureRight
        individual.pickupLeftValid = individual.pickupLeftValid and validFixtureLeft
        
        # check if pickup point is in reach
        score_, valid_ = utilsSolve.outsideReachPenalty(position=tempRightPos,\
                                quat=np.array([1,0,0,0]), reachCentrum=self.reachRightCentrum, reach=self.reach)
        score += score_
        individual.pickupRightValid = individual.pickupRightValid and valid_

        score_, valid_ = utilsSolve.outsideReachPenalty(position=tempLeftPos,\
                                quat=np.array([1,0,0,0]), reachCentrum=self.reachLeftCentrum, reach=self.reach)
        score += score_
        individual.pickupLeftValid = individual.pickupLeftValid and valid_

        # check if end point is in reach
        score_, valid_ = utilsSolve.outsideReachPenalty(position=rightPos, quat=quatRight,\
                                         reachCentrum=self.reachRightCentrum, reach=self.reach)
        score += score_
        individual.pickupRightValid = individual.pickupRightValid and valid_

        score_, valid_ = utilsSolve.outsideReachPenalty(position=leftPos, quat=quatLeft,\
                                         reachCentrum=self.reachLeftCentrum, reach=self.reach)
        score += score_
        individual.pickupLeftValid = individual.pickupLeftValid and valid_
        
        # penalty for crossing to far on x axis
        score_, valid_ = utilsSolve. minXposPenalty(position=rightPos)
        score += score_
        individual.pickupRightValid = individual.pickupRightValid and valid_

        score_, valid_ = utilsSolve. minXposPenalty(position=leftPos)
        score += score_
        individual.pickupLeftValid = individual.pickupLeftValid and valid_
        
        # penalty for crossing to far on y axis
        angle = individual.parametersIndividual[4]
        if angle < -(np.pi/2 + (20*np.pi/180)) or angle > (np.pi/2 + (20*np.pi/180)):
            score += - 2 

        # penalty for crossing pickup 
        if not utilsSolve.checkCrossing(pointR=tempRightPos, pointL=tempLeftPos):
            score += - 2 


        # penalty for outside side rope constraint 
        score_, valid_  = utilsSolve.ropeConstraint(task=self.task, individual=individual)
        score += score_
        individual.pickupRightValid = individual.pickupRightValid and valid_
        individual.pickupLeftValid = individual.pickupLeftValid and valid_
        
        # "Predict rope" ----------------
        
        rightEndPickupPoint, leftEndPickupPoint = utilsSolve.predictRope(task=self.task, individual=individual,\
                                    leftGrippPoint=self.leftGrippPoint, rightGrippPoint=self.rightGrippPoint)
        
        # new pickup points to close to fixture
        
        scoreFixtureRight, validFixtureRight = utilsSolve.fixturePenalty(position=rightEndPickupPoint, map_=self.map)
        scoreFixtureLeft, validFixtureLeft = utilsSolve.fixturePenalty(position=leftEndPickupPoint, map_=self.map)

        score += scoreFixtureRight
        score += scoreFixtureLeft
        individual.pickupRightValid = individual.pickupRightValid and validFixtureRight
        individual.pickupLeftValid = individual.pickupLeftValid and validFixtureLeft

        # penalty for end pickup out of reach 
        
        score_, valid_ = utilsSolve.outsideReachPenalty(position=rightEndPickupPoint,\
                                quat=np.array([1,0,0,0]), reachCentrum=self.reachRightCentrum, reach=self.reach-0.04)
        score += score_
        individual.pickupRightValid = individual.pickupRightValid and valid_

        score_, valid_ = utilsSolve.outsideReachPenalty(position=leftEndPickupPoint, quat=quatLeft,\
                                         reachCentrum=self.reachLeftCentrum, reach=self.reach-0.04)
        score += score_
        individual.pickupLeftValid = individual.pickupLeftValid and valid_
        
        # penalty for pickup points too close 
        
        if np.linalg.norm(rightEndPickupPoint - leftEndPickupPoint) < 0.12:
            score += -2
        
        if np.linalg.norm(tempRightPos - tempLeftPos) < 0.12:
            score += -2
            if individual.pickupRightValid and individual.pickupLeftValid:
                individual.pickupRightValid = False
                individual.pickupLeftValid = False

        # penalty for distance from target pickup
        
        score += -abs(pickupPoints[0] - self.rightGrippPoint)
        score += -abs(pickupPoints[1] - self.leftGrippPoint)
        
        # angle close to init angle reward
        
        score += 0.5 / (abs(self.initAngle - individual.parametersIndividual[4]) + 1)
        
        # Distance moved penalty
        
        score_, valid_ = utilsSolve.distanceMovedPenalty(self.initRightGrippPos, rightEndPickupPoint)
        score += score_
        score_, valid_ = utilsSolve.distanceMovedPenalty(self.initLeftGrippPos, leftEndPickupPoint)
        score += score_

        # if both not valid
        #if not (individual.pickupRightValid or individual.pickupLeftValid):
        #    score += -5
        
        return score
    

    def evaluateCombined(self, individual):
        score = 0 
        # final positions
        rightPos, leftPos, quatRight, quatLeft = individual.getRightLeftPosQuat(np.array([0.00, 0.00]))

        # To close to fixture, for end points, penalty 
        
        scoreFixtureRight, validFixtureRight = utilsSolve.fixturePenalty(position=rightPos, map_=self.map)
        scoreFixtureLeft, validFixtureLeft = utilsSolve.fixturePenalty(position=leftPos, map_=self.map)

        score += scoreFixtureRight
        score += scoreFixtureLeft
        individual.combinedValid = individual.combinedValid and validFixtureRight and validFixtureLeft

        # check if end point is in reach

        score_, valid_ = utilsSolve.outsideReachPenalty(position=rightPos, quat=quatRight,\
                                         reachCentrum=self.reachRightCentrum, reach=self.reach-0.03)
        score += score_
        individual.combinedValid = individual.combinedValid and valid_

        score_, valid_ = utilsSolve.outsideReachPenalty(position=leftPos, quat=quatLeft,\
                                         reachCentrum=self.reachLeftCentrum, reach=self.reach-0.03)
        score += score_
        individual.combinedValid = individual.combinedValid and valid_
        # left position needs to be in reach for right as they will switch 
        score_, valid_ = utilsSolve.outsideReachPenalty(position=rightPos, quat=quatRight,\
                                         reachCentrum=self.reachLeftCentrum, reach=self.reach-0.03)
        score += score_
        individual.combinedValid = individual.combinedValid and valid_

        score_, valid_ = utilsSolve.outsideReachPenalty(position=leftPos, quat=quatLeft,\
                                         reachCentrum=self.reachRightCentrum, reach=self.reach-0.03)
        score += score_
        individual.combinedValid = individual.combinedValid and valid_


        # penalty for crossing
        angle = individual.parametersCombined[2]
        if angle < -(np.pi/2 + (20*np.pi/180)) or angle > (np.pi/2 + (20*np.pi/180)):
            score += - 2 
        
        # reward for angle close to pi/2 or -pi/2
        if abs(abs(angle) - np.pi/2) < (20*np.pi/180):
            score += 1
            score += 2/(abs(abs(angle) - np.pi/2) + 1)
        else:
            score += -1

        # penalty for outside side rope constraint 
        score_, valid_ = utilsSolve.ropeConstraintCombined(task=self.task,\
                                                        individual=individual,\
                                                        rightGrippPoint=self.rightGrippPoint,\
                                                        leftGrippPoint=self.leftGrippPoint)
        score += score_
        individual.combinedValid = individual.combinedValid and valid_

        # Distance moved penalty
        initPosRight = self.task.gripperRight.getPosition() 
        initPosLeft = self.task.gripperLeft.getPosition()
        score_, valid_ = utilsSolve.distanceMovedPenalty(initPosRight , rightPos)
        score += 2*score_
        individual.combinedValid = individual.combinedValid and valid_

        score_, valid_ = utilsSolve.distanceMovedPenalty(initPosLeft , leftPos)
        score += 2*score_
        individual.combinedValid = individual.combinedValid and valid_

        return score

