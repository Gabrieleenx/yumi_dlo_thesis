#!/usr/bin/env python3

import numpy as np
import tf


class FramePose(object):
    def __init__(self):
        self.position = np.zeros(3)
        self.quaternion = np.zeros(4)

    def getQuaternion(self):
        return self.quaternion

    def getPosition(self):
        return self.position

    def update(self, position, orientation):
        self.position = np.copy(np.asarray(position))
        self.quaternion = np.copy(np.asarray(orientation))
    
    def copyClass(self):
        classCopy = FramePose()
        classCopy.quaternion = np.copy(self.quaternion)
        classCopy.position = np.copy(self.position) 
        return classCopy


class DLO(object):
    def __init__(self):
        #self.points = np.zeros((100,3))
        #self.lengthList = np.zeros(100)
        self.totalLenght = 0 
        self.pointsRecived = 0
        self.constrainLength = 0

    def update(self, points):
        numPoints = np.shape(points)[0]
        self.points = np.copy(points)
        self.lengthList = np.zeros(numPoints)

        for i in range(numPoints-1):
            diff = points[i+1] - points[i]
            dist = np.linalg.norm(diff)
            self.lengthList[i+1] = self.lengthList[i] + dist
        
        self.totalLenght = self.lengthList[-1]
        self.pointsRecived = 1

    def getCoord(self, length):
        if length < 0 or length > self.totalLenght:
            return False

        index1 = np.argwhere(self.lengthList >= length)[0,0]
        index0 = index1 - 1
        if index0 < 0:
            index0 = 0

        diff = self.points[index1] - self.points[index0]

        point = self.points[index0] + normalize(diff) * (length - self.lengthList[index0])

        return point

    def getLength(self):
        return self.totalLenght


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm


def calcAbsoluteAndRelative(yumiGrippPoseR, yumiGrippPoseL, transformer):
    translationRightArm = yumiGrippPoseR.getPosition()
    translationLeftArm = yumiGrippPoseL.getPosition()
    rotationRightArm = yumiGrippPoseR.getQuaternion()
    rotationLeftArm = yumiGrippPoseL.getQuaternion()

    tfMatrixRight = transformer.fromTranslationRotation(translation=translationRightArm, rotation=rotationRightArm)
    tfMatrixLeft = transformer.fromTranslationRotation(translation=translationLeftArm, rotation=rotationLeftArm)
    tfMatrixLeftInv = np.linalg.pinv(tfMatrixLeft)

    translationRelativeLeftRight = tfMatrixLeftInv.dot(np.hstack([self.translationRightArm, 1]))[0:3]
    rotLeftRight = tfMatrixLeftInv.dot(tfMatrixRight)
    relativeOrientation = tf.transformations.quaternion_from_matrix(rotLeftRight)

    avgQ = np.vstack([rotationRightArm, rotationLeftArm])
    absoluteOrientation = averageQuaternions(avgQ)  
    absolutePosition = 0.5*(translationRightArm + translationLeftArm)

    transformation1 = transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=self.absoluteOrientation)
    transformationInv1 = np.linalg.pinv(transformation1)
    transformation2 = transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=self.rotationLeftArm)

    leftToAbsoluteFrameRot = transformationInv1.dot(transformation2)
    homogeneousLeftRelative = np.hstack([translationRelativeLeftRight,1])
    homogeneousAbsouluteRelative = leftToAbsoluteFrameRot.dot(homogeneousLeftRelative)
    realativPosition = homogeneousAbsouluteRelative[0:3]

    return absolutePosition, absoluteOrientation, realativPosition, relativeOrientation


class FixtureObject(object):
    # cable goes through in y direction 
    # quaterniorn w,ix,iy,iz
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation
        #self.index = index
        self.fixtureHeight = 0.06
        self.previousCableLength = 0 # How much of the cable is already attached to fixtures 


def calcClipPoint(targetFixture, previousFixture, map_, cableSlack):
    # calculates the point on the rope (from right) that will be in target fixture
    length = 0
    if targetFixture >= 0 and previousFixture >= 0:
        fixture0 = map_[previousFixture]
        fixture1 = map_[targetFixture]
        length = np.linalg.norm(fixture1.position - fixture0.position)
        length += fixture0.previousCableLength

    length += cableSlack

    return length


def getZRotationCable(length, DLO):
    length0 = length-0.01
    length1 = length+0.01
    if length0 < 0 or length1 > DLO.getLength():
        print('error in getZRotationCable, length outside domain, length0 = ', length0, ' length1 = ', length1)
        return 0

    point0 = DLO.getCoord(length0)
    point1 = DLO.getCoord(length1)

    dy = point1[1] - point0[1]
    dx = point1[0] - point0[0]
    rotZ = np.arctan2(dy,dx)
    return rotZ - np.pi/2


def getPointTime(gripperRight, gripperLeft, posTargetLeft, posTargetRight,\
                        rotTargetLeft, rotTargetRight, avgSpeed, avgRotVel, shortestTime):
    # get max distance, for calc time 
    distRight = np.linalg.norm(posTargetRight - gripperRight.getPosition())
    distLeft = np.linalg.norm(posTargetLeft - gripperLeft.getPosition())
    maxDist = max(distLeft, distRight)
    timeSpeed = max(maxDist/avgSpeed, shortestTime)

    rotDistRight = getTotalRadians(gripperRight.getQuaternion(), rotTargetRight)
    rotDistLeft = getTotalRadians(gripperLeft.getQuaternion(), rotTargetLeft)
    maxRot = max(rotDistRight, rotDistLeft)
    timeRot = max(maxRot/avgRotVel, shortestTime)

    return max(timeSpeed, timeRot)


def getTotalRadians(currentQ, targetQ):
    if currentQ.dot(targetQ) < 0:
        currentQ = -currentQ

    skewTarget = np.array([[0, -targetQ[2], targetQ[1]],\
                            [targetQ[2],0,-targetQ[0]],\
                                [-targetQ[1],targetQ[0],0]])

    errorOrientation = currentQ[3]*targetQ[0:3] - targetQ[3]*currentQ[0:3] - skewTarget.dot(currentQ[0:3] )
    norm = np.linalg.norm(errorOrientation)
    return norm


def getOffestCableConstraint(map_, fixtrueIndex, gripperTargetPosition, DLO):
    if fixtrueIndex >= 0:
        cableAttachmentPostion = map_[fixtrueIndex].position 
        cableAttachmentPostion[2] += map_[fixtrueIndex].fixtureHeight 

        dist = np.linalg.norm(gripperTargetPosition - cableAttachmentPostion)
        cableLenght = DLO.constrainLength
        
        if dist < cableLenght:
            return np.zeros(3)

        offsetFactor = (dist - cableLenght)/dist
        d = cableAttachmentPostion - gripperTargetPosition

        return d*offsetFactor

    else:
        return np.zeros(3)


def checkIfWithinTol(pos, targetPos, posTol, quat, targetQuat, quatTol):
    dist = np.linalg.norm(pos - targetPos)
    rot = getTotalRadians(quat, targetQuat)
    if dist <= posTol and rot <= quatTol:
        return True
    else:
        return False 
