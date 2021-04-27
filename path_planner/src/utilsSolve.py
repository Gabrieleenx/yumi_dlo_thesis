#!/usr/bin/env python3

import numpy as np
import tf
import utils

# ------------- Checks -------------------------------------

def checkPositionWithinReach(position, reachCentrum, reach):
    # checks if a position is within reach and returns bool and distance
    within = True
    dist = np.linalg.norm(position - reachCentrum)
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
    transformer = tf.TransformerROS(True, rospy.Duration(0.1))
    tfMatrix = transformer.fromTranslationRotation(translation=posAbsolut, rotation=quatAbsolute)
    deltaRelativehomogeneous = np.array([deltaRelative[0],deltaRelative[1],deltaRelative[2], 1]) 
    posRight = tfMatrix.dot(deltaRelativehomogeneous)
    deltaRelativehomogeneous = np.array([-deltaRelative[0],-deltaRelative[1],-deltaRelative[2], 1]) 
    posLeft = tfMatrix.dot(deltaRelativehomogeneous)
    return posRight, posLeft


def checkTaskWithinReach(task):
    # checks if all trajectory point in a task is within the arms reach, returns True or False 
    within = True
    reach = 0.53 # max 0.559
    reachLeftCentrum = np.array([0.138, 0.106, 0.462])
    reachRightCentrum = np.array([0.138, -0.106, 0.462])
    mode = task.mode
    for i in range(task.numSubTasks):
        trajectoryPoint=task.trajectory[i]
        if task.mode == 'individual':
            posRight, quatRight, posLeft, quatLeft = \
                    trajectoryPointToNpArray(trajectoryPoint=trajectoryPoint, mode=mode)
        else:
            posAbsolut, quatAbsolute, posRelative, quatRelative= \
                    trajectoryPointToNpArray(trajectoryPoint=trajectoryPoint, mode=mode)
            posRight, posLeft = calcAbsolutToIndividualPos(posAbsolut=posAbsolut,\
                                            quatAbsolute=quatAbsolute, posRelative=posRelative)

        withinCheck, _ = checkPositionWithinReach(position=posLeft, reachCentrum=reachLeftCentrum, reach=reach)
        within = within and withinCheck
        withinCheck, _ = checkPositionWithinReach(position=posRight, reachCentrum=reachRightCentrum, reach=reach)
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

        diff = pointB1 - pointA1
        angle = np.arctan2(diff[1], diff[0])
        angleDiff = utils.calcAngleDiff(angle1=angle, angle2=np.pi/2)
        if abs(angleDiff) > np.pi/2 + 20 * np.pi /180:
            return False

    return True

def checkCloseToFixtures(task):
    map_ = task.map
    traj = task.trajectory

    for j in range(task.numSubTasks):
        leftCoord = np.asarray(traj[j].positionLeft)
        rightCoord = np.asarray(traj[j].positionRight)

        for i in range(len(map_)):
            fixture = map_[i]
            fixturePos = fixture.getBasePosition()
            fixtureRadious = fixture.getFixtureRadious()
            if np.linalg.norm(fixturePos - leftCoord) <= fixtureRadious:
                return False

            if np.linalg.norm(fixturePos - rightCoord) <= fixtureRadious:
                return False

    return True




# Evaluation functions ---------------------------------

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
    score = -dist * 3
    return score, True

# Help classes and functions for solver ----------------

class Individual(object):
    def __init__(self, mode):
        # Non Tuneable values ------------------------------- 
        self.mode = mode
        self.combinedValid = 0
        self.pickupLeftValid = 0       
        self.pickupRightValid = 0
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
                                pos=task.map[previousFixture].getClippPosition())
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
    scores = scores/np.sum(scores)
    return scores, maxIndex

    
# old temp stuff

'''
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
        
        elif self.mode == 'combined':
            (baseToGripperRight, _) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
            (baseToGripperLeft, _) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
            self.initPosRight = np.asarray(baseToGripperRight)
            self.initPosLeft = np.asarray(baseToGripperLeft)
            dy = self.initPosLeft[1] - self.initPosRight[1]
            dx = self.initPosLeft[0] - self.initPosRight[0]
            self.initAngle = np.arctan2(dy, dx) - np.pi/2    
            self.initAbsPos = 0.5 * (self.initPosRight  + self.initPosLeft)
            minDist, point, minIndex = utils.closesPointDLO(self.DLO, baseToGripperRight)
            self.rightGrippPoint = point
            minDist, point, minIndex = utils.closesPointDLO(self.DLO, baseToGripperLeft)
            self.leftGrippPoint = point


        (Link7ToGripperRight, _) = tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        self.Link7ToGripperRight = np.asarray(Link7ToGripperRight)
        (Link7ToGripperLeft, _) = tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))
        self.Link7ToGripperLeft = np.asarray(Link7ToGripperLeft)
        '''