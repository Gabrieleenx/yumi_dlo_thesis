import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils


class Individual(object):
    def __init__(self, mode):
        # Non Tuneable values ------------------------------- 
        self.mode = mode

        self.pickupLeftValid = 0       
        self.pickupRightValid = 0
        self.score = 0

        # Tuneable parameters -----------------------------------
        
        # parametersIndividual = [pickupR,  pickupL, absPosX, absPosY, absRotZ]

        self.parametersIndividual = np.zeros(5)

    def getPickupPoints(self):
        return self.parametersIndividual[0:2]
    
    def getRightLeftPosQuat(self, targetHeight):
        # targetHeight is in [right, left]
        if self.mode == 'individual':
            grippWidth = abs(self.parametersIndividual[0] - self.parametersIndividual[1])        
            angle = self.parametersIndividual[4] + np.pi / 2
            dx = grippWidth * np.cos(angle) / 2
            dy = grippWidth * np.sin(angle) / 2

            leftPos = np.zeros(3)

            leftPos[0] = self.parametersIndividual[2] + dx
            leftPos[1] = self.parametersIndividual[3] + dy
            leftPos[2] = targetHeight[1]

            rightPos = np.zeros(3)
            rightPos[0] = self.parametersIndividual[2] - dx
            rightPos[1] = self.parametersIndividual[3] - dy
            rightPos[2] = targetHeight[0]

            quatRight = tf.transformations.quaternion_from_euler(self.parametersIndividual[4], 0, 180*np.pi/180, 'rzyx')
            quatLeft = quatRight
        else:
            grippWidth = 0 # TODO for combined mode
            rightPos = np.zeros(3)
            leftPos = np.zeros(3)
            quatRight = np.zeros(4)
            quatLeft = np.zeros(4)

        return rightPos, leftPos, quatRight, quatLeft

def sampleIndex(populationSize, scores):
    value = np.random.random()
    sumValue = 0
    seedIndex = 0
    for k in range(populationSize):
        sumValue += scores[k]
        if sumValue > value:
            seedIndex = k   
            break
    return seedIndex

def fixturePenalty(position, map_, minDist):
    valid = 1
    score = 0
    for i in range(len(map_)):
        posFixture = map_[i].getBasePosition()
        dist = np.linalg.norm(posFixture[0:2]-position[0:2]) # only distance in xy plane thats relevant
        if dist < minDist:
            score += -3
            score += dist - minDist
            valid = 0
    return score, valid


def distanceMovedPenalty(initPose, endPose):
    dist = np.linalg.norm(initPose[0:2] - endPose[0:2])
    return - dist*3

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
        self.crossOverProbability = 0.4
        self.mutationProbabiliy = 0.8
        self.initAngle = 0

    def updateInit(self, DLO, map_, mode, grippWidth, targetFixture, previousFixture, tfListener, cableSlack):
        self.DLO = DLO
        self.mode = mode
        self.grippWidth = grippWidth
        self.targetFixture = targetFixture
        self.previousFixture = previousFixture
        self.map = map_
        self.cableSlack = cableSlack

        if mode == 'individual':
            clipPoint = utils.calcClipPoint(targetFixture, previousFixture, map_, cableSlack, DLO)
            self.leftGrippPoint, self.rightGrippPoint = utils.calcGrippPoints(targetFixture, map_, DLO, grippWidth, clipPoint)
            
            if self.previousFixture < 0:
                start = 0.05
            else:
                minDist, point, minIndex = utils.closesPointDLO(self.DLO, self.map[previousFixture].getClippPosition())
                start = point
                self.previousFixtureDLOLength = point

            if self.leftGrippPoint > self.rightGrippPoint:
                leftStart = self.leftGrippPoint 
                leftEnd = DLO.getLength()-0.05
                rightStart = start
                rightEnd = self.rightGrippPoint 
            else:
                rightStart = self.rightGrippPoint 
                rightEnd = DLO.getLength()- 0.05
                leftStart = start
                leftEnd = self.leftGrippPoint 

            pointRight = self.DLO.getCoord(self.rightGrippPoint)
            pointLeft =  self.DLO.getCoord(self.leftGrippPoint)    
            dy = pointLeft[1] - pointRight[1]
            dx = pointLeft[0] - pointRight[0]
            angle = np.arctan2(dy, dx) - np.pi/2    

            self.initAngle = angle

            self.rightPickupRange = np.array([rightStart, rightEnd])
            self.leftPickupRange = np.array([leftStart, leftEnd])

        (Link7ToGripperRight, _) = tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        self.Link7ToGripperRight = np.asarray(Link7ToGripperRight)
        (Link7ToGripperLeft, _) = tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))
        self.Link7ToGripperLeft = np.asarray(Link7ToGripperLeft)

  
    def solve(self, populationSize, numGenerations):

        # generate initial population
        scores = np.zeros(populationSize)
        self.population = []
        for i in range(populationSize):
            if self.mode == 'individual':
                individual = self.generateInitIndividual()
                self.population.append(individual)
            else:
                pass

        for i in range(numGenerations):
            for j in range(populationSize):
                # evaluate
                score = self.evaluate(self.population[j])
                scores[j] = score
            # normalize scores
            print(' scores max' , np.max(scores))
            maxIndex = np.argmax(scores)
            print('generation ', i ,' scores ', scores)

            scores = scores + abs(np.min(scores)) + 1e-3
            scores = scores/np.sum(scores)
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

            tempPopulation[0] = self.population[maxIndex]
            self.population = tempPopulation.copy()

            rightPos, leftPos, quatRight, quatLeft = self.population[0].getRightLeftPosQuat(np.array([0.03, 0.03]))
            scoreFixtureLeft, validFixtureLeft = fixturePenalty(leftPos, self.map, self.fixtureRadius)
            
            print('rightPickupPoint = ', self.population[0].parametersIndividual[0], '\n', \
                'leftPickupPoint = ', self.population[0].parametersIndividual[1], '\n',\
                'rightEndPosition = ', rightPos, '\n',\
                'leftEndPosition = ', leftPos, '\n',\
                'angle = ', self.population[0].parametersIndividual[4], '\n',\
                'scoreFixtureLeft = ', scoreFixtureLeft, '\n',\
                'Valid ', self.population[0].pickupRightValid, ' ', self.population[0].pickupLeftValid)
            

        print('score, chosen ', self.evaluate(self.population[0]))

        # check solution is feasible

        return self.population[0] 

    def generateInitIndividual(self):

        individual = Individual(self.mode)
        
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

        return individual
        
    def mutateIndividual(self, seedIndividual):
        individual = Individual(self.mode)
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
        return individual

    def crossOver(self, parentOne, parentTwo):
        individual = Individual(self.mode)

        numElements = np.size(parentOne.parametersIndividual)
        crossOverPoint = np.random.randint(0,numElements)
        newParameters = np.zeros(numElements)

        if self.mode == 'individual':
            newParameters[0:crossOverPoint] = parentOne.parametersIndividual[0:crossOverPoint]
            newParameters[crossOverPoint:numElements] = parentTwo.parametersIndividual[crossOverPoint:numElements]
            individual.parametersIndividual = newParameters
        else:
            pass
        
        return individual

    def evaluate(self, individual):
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

        # inside of reach verfication 
        
        
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
        if self.mode == 'individual':
            angle = individual.parametersIndividual[4]
            if angle < -(np.pi/2 + (20*np.pi/180)) or angle > (np.pi/2 + (20*np.pi/180)):
                score += - 5 


        # penalty for outside side rope constraint 
        if self.previousFixture == -1:
            pass
        else:
            fixtureClippPosition = self.map[self.previousFixture].getClippPosition()

            if self.mode == 'individual':
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

            else:
                # TODO for combined mode 
                closesPoint = np.zeros(3)
                lengthRope = 1

            closestDist = np.linalg.norm(fixtureClippPosition - closesPoint)
            if closestDist > lengthLeft:
                score -= 4 + (dist - lengthRope)



        if self.mode == 'individual':
            initRightPos = self.DLO.getCoord(self.rightGrippPoint)
            initLeftPos = self.DLO.getCoord(self.leftGrippPoint)

            l1 = abs(pickupPoints[0] - self.rightGrippPoint)
            l2 = abs(self.leftGrippPoint - self.rightGrippPoint)
            l3 = abs(self.leftGrippPoint - pickupPoints[1])

            # "Predict rope"
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
                return -5    

            # new pickup points to close to fixture
            scoreFixtureRight, validFixtureRight = fixturePenalty(rightEndPickupPoint, self.map, self.fixtureRadius)
            scoreFixtureLeft, validFixtureLeft = fixturePenalty(leftEndPickupPoint, self.map, self.fixtureRadius)

            score += scoreFixtureRight
            score += scoreFixtureLeft

            # penalty for pickup points too close 

            if np.linalg.norm(rightEndPickupPoint - leftEndPickupPoint) < 0.12:
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
    

