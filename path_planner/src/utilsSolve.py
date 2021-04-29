#!/usr/bin/env python3

import numpy as np
import tf
import utils
import rospy
# ------------- Checks -------------------------------------
# these are put global for significant performance gain, only used in checkPositionWithinReach() function 
transformer = tf.TransformerROS(True, rospy.Duration(0.1)) # also used in trajectoryPointToNpArray()    
positionG = np.array([0,0,0.135]) # gripper offset
tfMatrixG = transformer.fromTranslationRotation(translation=positionG, rotation=np.array([0,0,0,1]))
invTfMAtrixG = np.linalg.pinv(tfMatrixG)

def checkPositionWithinReach(position, quat, reachCentrum, reach):
    # checks if a position is within reach and returns bool and distance
    #transformer = tf.TransformerROS(True, rospy.Duration(0.1))
    tfMatrixP = transformer.fromTranslationRotation(translation=position, rotation=quat)
    tfMatrixGripperBase = tfMatrixP.dot(invTfMAtrixG)
    positionGrippBase = tfMatrixGripperBase[0:3,3]
    within = True
    dist = np.linalg.norm(positionGrippBase - reachCentrum)
    if dist >= reach:
        within = False
    return within, dist


def trajectoryPointToNpArray(trajectoryPoint, mode):
    # return np array from trajectory point msg, mode ['individual' or 'comibined']
    if mode == 'individual':
        posRight = np.asarray(trajectoryPoint.positionRight)
        posLeft = np.asarray(trajectoryPoint.positionLeft)
        quatRight = np.asarray(trajectoryPoint.orientationRight)
        quatLeft = np.asarray(trajectoryPoint.orientationLeft)
        return posRight, quatRight, posLeft, quatLeft
    else:
        posAbsolut = np.asarray(trajectoryPoint.positionAbsolute)
        posRelative = np.asarray(trajectoryPoint.positionRelative)
        quatAbsolute = np.asarray(trajectoryPoint.orientationAbsolute)
        quatRelative = np.asarray(trajectoryPoint.orientationRelative)
        return posAbsolut, quatAbsolute, posRelative, quatRelative


def calcAbsolutToIndividualPos(posAbsolut, quatAbsolute, posRelative):
    deltaRelative = posRelative/2
    tfMatrix = transformer.fromTranslationRotation(translation=posAbsolut, rotation=quatAbsolute)
    deltaRelativehomogeneous = np.array([deltaRelative[0],deltaRelative[1],deltaRelative[2], 1]) 
    posRight = tfMatrix.dot(deltaRelativehomogeneous)
    deltaRelativehomogeneous = np.array([-deltaRelative[0],-deltaRelative[1],-deltaRelative[2], 1]) 
    posLeft = tfMatrix.dot(deltaRelativehomogeneous)
    return posRight[0:3], posLeft[0:3]


def checkTaskWithinReach(task):
    # checks if all trajectory point in a task is within the arms reach, returns True or False 
    within = True
    reach = 0.53 # max 0.559
    reachLeftCentrum = np.array([0.138, 0.106, 0.462])
    reachRightCentrum = np.array([0.138, -0.106, 0.462])
    mode = task.mode
    for i in range(len(task.trajectory)):
        trajectoryPoint=task.trajectory[i]
        if task.mode == 'individual':
            posRight, quatRight, posLeft, quatLeft = \
                    trajectoryPointToNpArray(trajectoryPoint=trajectoryPoint, mode=mode)
        else:
            posAbsolut, quatAbsolute, posRelative, quatRelative= \
                    trajectoryPointToNpArray(trajectoryPoint=trajectoryPoint, mode=mode)
            posRight, posLeft = calcAbsolutToIndividualPos(posAbsolut=posAbsolut,\
                                            quatAbsolute=quatAbsolute, posRelative=posRelative)
            quatLeft = quatAbsolute
            quatRight = quatAbsolute

        withinCheck, _ = checkPositionWithinReach(position=posLeft, quat=quatLeft, reachCentrum=reachLeftCentrum, reach=reach)
        within = within and withinCheck
        withinCheck, _ = checkPositionWithinReach(position=posRight, quat=quatRight, reachCentrum=reachRightCentrum, reach=reach)
        within = within and withinCheck

    return within


def checkIfTrajectoriesPassToClose(task):
    minGripperDistance = 0.12
    traj = task.trajectory
    for i in range(len(traj)):
        if i == 0:
            pointA0 = task.gripperRight.getPosition()
            pointA1 = np.asarray(traj[i].positionRight)
            pointB0 = task.gripperLeft.getPosition()
            pointB1 = np.asarray(traj[i].positionLeft)
        else:
            pointA0 = np.asarray(traj[i-1].positionRight)
            pointA1 = np.asarray(traj[i].positionRight)
            pointB0 = np.asarray(traj[i-1].positionLeft)
            pointB1 = np.asarray(traj[i].positionLeft)

        closestDist = utils.closestDistLineToLineSegment(pointA0=pointA0,\
                            pointA1=pointA1, pointB0=pointB0, pointB1=pointB1)

        if closestDist <= minGripperDistance:
            return False

    return True


def checkCrossing(pointR, pointL):
    diff = pointL - pointR
    angle = np.arctan2(diff[1], diff[0])
    angleDiff = utils.calcAngleDiff(angle1=angle, angle2=np.pi/2)
    if abs(angleDiff) > np.pi/2 + 20 * np.pi /180:
        return False
    return True

def checkIfNotLeftRightArmCross(task):
    minGripperDistance = 0.12
    traj = task.trajectory
    for i in range(len(traj)):
        if task.mode == 'individual':
            pointA1 = np.asarray(traj[i].positionRight)
            pointB1 = np.asarray(traj[i].positionLeft)
        else:
            posAbsolut =  np.asarray(traj[i].positionAbsolute)
            quatAbsolute =  np.asarray(traj[i].orientationAbsolute)
            posRelative =  np.asarray(traj[i].positionRelative)
            pointA1, pointB1 = calcAbsolutToIndividualPos(posAbsolut=posAbsolut,\
                                quatAbsolute=quatAbsolute, posRelative=posRelative)

        if not checkCrossing(pointR=pointA1, pointL=pointB1):
            return False

    return True

def checkCloseToFixtures(task):
    map_ = task.map
    traj = task.trajectory

    for j in range(len(traj)):
        leftCoord = np.asarray(traj[j].positionLeft)
        rightCoord = np.asarray(traj[j].positionRight)

        for i in range(len(map_)):
            fixture = map_[i]
            fixturePos = fixture.getBasePosition()
            fixtureRadius = fixture.getFixtureRadius()
            if np.linalg.norm(fixturePos - leftCoord) <= fixtureRadius:
                return False

            if np.linalg.norm(fixturePos - rightCoord) <= fixtureRadius:
                return False

    return True




# Evaluation functions ---------------------------------

def fixturePenalty(position, map_):
    valid = 1
    score = 0
    for i in range(len(map_)):
        posFixture = map_[i].getBasePosition()
        minDist = map_[i].getFixtureRadius()+0.01
        dist = np.linalg.norm(posFixture[0:2]-position[0:2]) # only distance in xy plane thats relevant
        if dist <= minDist:
            score += -2
            score += dist - minDist
            valid = 0
    return score, valid


def distanceMovedPenalty(initPose, endPose):
    dist = np.linalg.norm(initPose[0:2] - endPose[0:2])
    score = -dist * 2
    return score, True


def outsideReachPenalty(position, quat, reachCentrum, reach):
    within, dist = checkPositionWithinReach(position=position,\
                    quat=quat, reachCentrum=reachCentrum, reach=reach)
    valid = within
    score = 1
    if valid == False:
        score = - 1/(dist+1)
    return score, valid


def ropeConstraint(task, individual):
    valid = True
    score = 0 
    if task.previousFixture == -1:
        return 0, valid
    else:
        rightPos, leftPos, quatRight, quatLeft = individual.getRightLeftPosQuat(np.array([0.03, 0.03]))
        fixtureClippPosition = task.map[task.previousFixture].getClippPosition()
        minDist, point, minIndex = utils.closesPointDLO(DLO=task.DLO, pos=fixtureClippPosition)
        pickupPoints = individual.getPickupPoints()
        lengthRight = abs(point - pickupPoints[0])
        lengthLeft = abs(point - pickupPoints[1])
        if individual.pickupLeftValid and individual.pickupRightValid:
            if lengthRight < lengthLeft:
                closesPoint  = rightPos
                lengthRope = lengthRight
            else:
                closesPoint = leftPos
                lengthRope = lengthLeft
        elif individual.pickupLeftValid:
            closesPoint = leftPos
            lengthRope = lengthLeft
        elif individual.pickupRightValid:
            closesPoint = rightPos
            lengthRope = lengthRight
        else:
            closesPoint = np.zeros(3)
            lengthRope = 1        

        closestDist = np.linalg.norm(fixtureClippPosition - closesPoint)
        if closestDist > lengthLeft:
            score -= 2 + (closestDist - lengthRope)
            valid = False
    return score, valid


def ropeConstraintCombined(task, individual, rightGrippPoint, leftGrippPoint):
    valid = True
    score = 0
    if task.previousFixture == -1:
        pass
    else:
        fixtureClippPosition = task.map[task.previousFixture].getClippPosition()
        minDist, point, minIndex = utils.closesPointDLO(DLO=task.DLO, pos=fixtureClippPosition)
        rightPos, leftPos, quatRight, quatLeft = individual.getRightLeftPosQuat(np.array([0.03, 0.03]))

        lengthRight =  abs(point -  rightGrippPoint )
        lengthLeft =  abs(point -  leftGrippPoint )
        if lengthRight < lengthLeft:
            closesPoint  = rightPos
            lengthRope = lengthRight
        else:
            closesPoint = leftPos
            lengthRope = lengthLeft
        

        closestDist = np.linalg.norm(fixtureClippPosition - closesPoint)
        if closestDist > lengthLeft:
            score -= 2 + (closestDist - lengthRope)
            valid = False
            
    return score, valid

def predictRope(task, individual, leftGrippPoint, rightGrippPoint):
    initRightPos = task.DLO.getCoord(rightGrippPoint)
    initLeftPos = task.DLO.getCoord(leftGrippPoint)
    pickupPoints = individual.getPickupPoints()
    rightPos, leftPos, quatRight, quatLeft = individual.getRightLeftPosQuat(np.array([0.03, 0.03]))

    l1 = abs(pickupPoints[0] - rightGrippPoint)
    l2 = abs(leftGrippPoint - rightGrippPoint)
    l3 = abs(leftGrippPoint - pickupPoints[1])
    
    if individual.pickupLeftValid == 1 and individual.pickupRightValid == 1:

        vec = utils.normalize(leftPos - rightPos)
        rightEndPickupPoint = rightPos + vec * l1
        leftEndPickupPoint = rightPos + vec * (l1 + l2)

    elif individual.pickupLeftValid == 1:

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
        return np.zeros(3), np.zeros(3)    

    return rightEndPickupPoint, leftEndPickupPoint


# Help classes and functions for solver ----------------

class Individual(object):
    def __init__(self, mode):
        # Non Tuneable values ------------------------------- 
        self.mode = mode
        self.combinedValid = True
        self.pickupLeftValid = True       
        self.pickupRightValid = True
        self.score = 0
        self.grippWidth = 0 # combined 

        # Tuneable parameters -----------------------------------
        
        # parametersIndividual = [pickupR,  pickupL, absPosX, absPosY, absRotZ]
        self.parametersIndividual = np.zeros(5)

        # parametersCombined = [absPosX, absPosY, absRotZ]
        self.parametersCombined = np.zeros(3)


    def getPickupPoints(self):
        return self.parametersIndividual[0:2]
    
    def getRightLeftPosQuat(self, targetHeight):
        # targetHeight is in [right, left]
        if self.mode == 'individual':
            grippWidth = abs(self.parametersIndividual[0] - self.parametersIndividual[1])
            posAbsolut = np.array([self.parametersIndividual[2], self.parametersIndividual[3], targetHeight[0]])
            posRelative = np.array([0, grippWidth, 0])
            quatRight = tf.transformations.quaternion_from_euler(self.parametersIndividual[4], 0, 180*np.pi/180, 'rzyx')
            quatLeft = quatRight
            rightPos, leftPos = calcAbsolutToIndividualPos(posAbsolut=posAbsolut,\
                                                     quatAbsolute=quatRight, posRelative=posRelative)   
            #print('rightPos ', rightPos, ' leftPos ', leftPos, ' posAbsolut ', posAbsolut)
            '''     
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
            '''
        else:
            posAbsolut = np.array([self.parametersCombined[0], self.parametersCombined[1], targetHeight[0]])
            posRelative = np.array([0, self.grippWidth, 0])
            quatRight = tf.transformations.quaternion_from_euler(self.parametersCombined[2], 0, 180*np.pi/180, 'rzyx')
            quatLeft = quatRight
            rightPos, leftPos = calcAbsolutToIndividualPos(posAbsolut=posAbsolut,\
                                                     quatAbsolute=quatRight, posRelative=posRelative)   
            '''
            angle = self.parametersCombined[2] + np.pi / 2
            dx = self.grippWidth * np.cos(angle) / 2
            dy = self.grippWidth * np.sin(angle) / 2
            rightPos = np.zeros(3)
            rightPos[0] = self.parametersCombined[0] - dx
            rightPos[1] = self.parametersCombined[1] - dy
            rightPos[2] = targetHeight[0]

            leftPos = np.zeros(3)
            leftPos[0] = self.parametersCombined[0] + dx
            leftPos[1] = self.parametersCombined[1] + dy
            leftPos[2] = targetHeight[0]

            quatRight = tf.transformations.quaternion_from_euler(self.parametersCombined[2], 0, 180*np.pi/180, 'rzyx')
            quatLeft = quatRight
            '''
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



def pickupRangeAndAngle(task, rightGrippPoint, leftGrippPoint):
    endDLOMargin = 0.05
    if task.previousFixture < 0:
        start = endDLOMargin
    else:
        minDist, point, minIndex = utils.closesPointDLO(DLO=task.DLO,\
                                pos=task.map[task.previousFixture].getClippPosition())
        start = point

    if leftGrippPoint > rightGrippPoint:
        leftStart = leftGrippPoint 
        leftEnd = task.DLO.getLength() - endDLOMargin
        rightStart = start
        rightEnd = rightGrippPoint 
    else:
        rightStart = rightGrippPoint 
        rightEnd = task.DLO.getLength() - endDLOMargin
        leftStart = start
        leftEnd = leftGrippPoint 

    pointRight = task.DLO.getCoord(rightGrippPoint)
    pointLeft =  task.DLO.getCoord(leftGrippPoint)    
    dy = pointLeft[1] - pointRight[1]
    dx = pointLeft[0] - pointRight[0]
    
    angle = np.arctan2(dy, dx) - np.pi/2    
    rightPickupRange = np.array([rightStart, rightEnd])
    leftPickupRange = np.array([leftStart, leftEnd])
    
    return rightPickupRange, leftPickupRange, angle

def absPosAngleGrippPoints(task):
    posRight = task.gripperRight.getPosition()
    posLeft = task.gripperLeft.getPosition()
    dy = posLeft[1] - posRight[1]
    dx = posLeft[0] - posRight[0]
    initAngle = np.arctan2(dy, dx) - np.pi/2    
    initAbsPos = 0.5 * (posRight  + posLeft)
    minDist, point, minIndex = utils.closesPointDLO(task.DLO, posRight)
    rightGrippPoint = point
    minDist, point, minIndex = utils.closesPointDLO(task.DLO, posLeft)
    leftGrippPoint = point
        
    return rightGrippPoint, leftGrippPoint, initAbsPos, initAngle


def normalizeSumScores(scores):
    maxIndex = np.argmax(scores)
    scores = scores + abs(np.min(scores)) + 1e-3
    medianValue = np.median(scores)
    scores[scores < medianValue] = 0
    scores = scores/np.sum(scores)
    return scores, maxIndex


def generateInitIndividual(task, leftPickupRange, rightPickupRange, individualSTD, combinedSTD, initAbsPos):

        individual = Individual(task.mode)
        if task.mode == 'individual':
            # initial pickup point, uniform in range
            low = leftPickupRange[0]
            high = leftPickupRange[1]
            individual.parametersIndividual[1] = np.random.default_rng().uniform(low=low, high=high)

            low = rightPickupRange[0]
            high = rightPickupRange[1]
            individual.parametersIndividual[0] = np.random.default_rng().uniform(low=low, high=high)

            # generate new position and orientation from normal distrobution 
            pointRight = task.DLO.getCoord(individual.parametersIndividual[0])
            pointLeft =  task.DLO.getCoord(individual.parametersIndividual[1])
            meanPickupX = 0.5*(pointRight[0] + pointLeft[0])
            meanPickupY = 0.5*(pointRight[1] + pointLeft[1])
            newAbsX = np.random.normal(meanPickupX, individualSTD[2]*4)
            newAbsY = np.random.normal(meanPickupY, individualSTD[3]*4)

            dy = pointLeft[1] - pointRight[1]
            dx = pointLeft[0] - pointRight[0]
            angle = np.arctan2(dy, dx) - np.pi/2

            newAbsAgnle = np.random.normal(angle, individualSTD[4]*4)

            individual.parametersIndividual[2] = newAbsX
            individual.parametersIndividual[3] = newAbsY
            individual.parametersIndividual[4] = newAbsAgnle

        elif task.mode == 'combined':

            individual.grippWidth = task.grippWidth
            if np.random.random() < 0.5:
                angle = np.random.normal(np.pi/2, 4*combinedSTD[2])
            else:
                angle = np.random.normal(-np.pi/2, 4*combinedSTD[2])
                
            newAbsX = np.random.normal(initAbsPos[0], combinedSTD[0]*4)
            newAbsY = np.random.normal(initAbsPos[1], combinedSTD[1]*4)
            individual.parametersIndividual[0] = newAbsX
            individual.parametersIndividual[1] = newAbsY
            individual.parametersCombined[2] = angle

        return individual

def mutateIndividual(task, seedIndividual, leftPickupRange, rightPickupRange, individualSTD, combinedSTD):
        individual = Individual(task.mode)
        if task.mode == 'individual':

            pickupPoints = seedIndividual.getPickupPoints()
            low = leftPickupRange[0]
            high = leftPickupRange[1]
            newLeftPickup = np.random.normal(pickupPoints[1] , individualSTD[1])
            individual.parametersIndividual[1] = np.clip(newLeftPickup, low, high)

            low = rightPickupRange[0]
            high = rightPickupRange[1]
            newRightPickup =  np.random.normal(pickupPoints[0], individualSTD[0])
            individual.parametersIndividual[0] = np.clip(newRightPickup, low, high)

            newAbsX = np.random.normal(seedIndividual.parametersIndividual[2], individualSTD[2])
            newAbsY = np.random.normal(seedIndividual.parametersIndividual[3], individualSTD[3])
            newAbsAgnle = np.random.normal(seedIndividual.parametersIndividual[4], individualSTD[4])

            individual.parametersIndividual[2] = newAbsX
            individual.parametersIndividual[3] = newAbsY
            individual.parametersIndividual[4] = newAbsAgnle
        elif task.mode == 'combined':
            newAbsX = np.random.normal(seedIndividual.parametersCombined[0], combinedSTD[0])
            newAbsY = np.random.normal(seedIndividual.parametersCombined[1], combinedSTD[1])
            flippProbability = 0.5
            if np.random.random() < flippProbability:
                newAbsAgnle = np.random.normal(-seedIndividual.parametersCombined[2], combinedSTD[2])
            else:
                newAbsAgnle = np.random.normal(seedIndividual.parametersCombined[2], combinedSTD[2])

            individual.parametersCombined[0] = newAbsX
            individual.parametersCombined[1] = newAbsY
            individual.parametersCombined[2] = newAbsAgnle

        return individual
    

def crossOver(task, parentOne, parentTwo):
    individual = Individual(task.mode)
    if task.mode == 'individual':
        numElements = np.size(parentOne.parametersIndividual)
    else:
        numElements = np.size(parentOne.parametersCombined)

    crossOverPoint = np.random.randint(0,numElements)
    newParameters = np.zeros(numElements)

    if task.mode == 'individual':
        newParameters[0:crossOverPoint] = parentOne.parametersIndividual[0:crossOverPoint]
        newParameters[crossOverPoint:numElements] = parentTwo.parametersIndividual[crossOverPoint:numElements]
        individual.parametersIndividual = newParameters
    else:
        newParameters[0:crossOverPoint] = parentOne.parametersCombined[0:crossOverPoint]
        newParameters[crossOverPoint:numElements] = parentTwo.parametersCombined[crossOverPoint:numElements]
        individual.parametersCombined = newParameters
    
    return individual


