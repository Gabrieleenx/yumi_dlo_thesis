#!/usr/bin/env python3

import numpy as np
import rospy
import tf
import utilsSolve

class FramePose(object):
    def __init__(self):
        self.position = np.zeros(3)
        self.quaternion = np.zeros(4)
        self.flipped = -1

    def getQuaternion(self):
        return np.copy(self.quaternion)

    def getPosition(self):
        return np.copy(self.position)

    def update(self, position, orientation):
        self.position = np.copy(np.asarray(position))
        self.quaternion = np.copy(np.asarray(orientation))
    
    def copyClass(self):
        classCopy = FramePose()
        classCopy.quaternion = np.copy(self.quaternion)
        classCopy.position = np.copy(self.position) 
        classCopy.flipped = self.flipped
        return classCopy


class DLO(object):
    def __init__(self):
        #self.points = np.zeros((100,3))
        #self.lengthList = np.zeros(100)
        self.totalLenght = 0 
        self.pointsRecived = 0

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
            print('Length outside rope dimentions!')
            return False

        index1 = np.argwhere(self.lengthList >= length)[0,0]
        index0 = index1 - 1
        if index0 < 0:
            index0 = 0

        diff = self.points[index1] - self.points[index0]

        point = self.points[index0] + normalize(diff) * (length - self.lengthList[index0])

        return point
    
    def getPartLength(self, index):
        if index < 0 or index > np.size(self.lengthList, axis=0):
            return False

        return self.lengthList[index]

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

    avgQ = np.vstack([rotationRightArm, rotationLeftArm])
    absoluteOrientation = averageQuaternions(avgQ)  
    absolutePosition = 0.5*(translationRightArm + translationLeftArm)

    transformationAbsolute = transformer.fromTranslationRotation(translation=absolutePosition, rotation=absoluteOrientation)
    transformationAbsoluteInv = np.linalg.pinv(transformationAbsolute)

    transformationRightFromAbs = transformationAbsoluteInv.dot(tfMatrixRight)
    transformationLeftFromAbs = transformationAbsoluteInv.dot(tfMatrixLeft)
    quatRightAbs = tf.transformations.quaternion_from_matrix(transformationRightFromAbs)
    posRightAbs = tf.transformations.translation_from_matrix(transformationRightFromAbs)
    quatLeftAbs = tf.transformations.quaternion_from_matrix(transformationLeftFromAbs)
    posLeftAbs = tf.transformations.translation_from_matrix(transformationLeftFromAbs)

    relativeOrientation = tf.transformations.quaternion_multiply(quatRightAbs, tf.transformations.quaternion_conjugate(quatLeftAbs))
    realativPosition = posRightAbs - posLeftAbs

    return absolutePosition, absoluteOrientation, realativPosition, relativeOrientation


# taken (Modified) from https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (x,y,z,w), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    # from (x,y,z,w) to (w,x,y,z)
    if Q[0].dot(Q[1]) < 0:
        Q[0] = -Q[0]

    Q = np.roll(Q,1,axis=1)
    # Number of quaternions to average
    M = Q.shape[0]
    A = np.zeros(shape=(4,4))
    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A
    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    avgQ = np.real(eigenVectors[:,0])
    # from (w,x,y,z) to (x,y,z,w) 
    avgQ = np.roll(avgQ,-1)

    return avgQ


class FixtureObject(object):
    # cable goes through in y direction 
    # quaterniorn w,ix,iy,iz
    def __init__(self, position, orientation, fixtureHeight, fixtureRadius):
        self.position = position
        self.orientation = orientation
        self.fixtureHeight = fixtureHeight
        self.fixtureRadius = fixtureRadius
        transformer = tf.TransformerROS(True, rospy.Duration(0.1)) 
        tfMatrix1 = transformer.fromTranslationRotation(translation=position, rotation=orientation)
        self.position2 = tfMatrix1.dot(np.array([0.025,0,0,1]))[0:3]

    def getBasePosition(self):
        return np.copy(self.position)

    def getBasePosition2(self):
        return np.copy(self.position2)

    def getClippPosition(self):
        clippPosition = np.array([self.position[0], self.position[1], self.position[2]+ self.fixtureHeight])
        return clippPosition
        
    def getOrientation(self):
        return np.copy(self.orientation)

    def getFixtureHeight(self):
        return self.fixtureHeight

    def getFixtureRadius(self):
        return self.fixtureRadius


def calcClipPoint(targetFixture, previousFixture, map_, cableSlack, DLO, gripWidth):
    # calculates the point on the rope (from right) that will be in target fixture
    clipPoint = 0

    if targetFixture >= 0 and previousFixture >= 0:
        fixture0 = map_[previousFixture]
        fixture1 = map_[targetFixture] 
        diff = fixture1.getClippPosition() - fixture0.getClippPosition()
        clipPoint = np.linalg.norm(diff)
        minDist, point, minIndex = closesPointDLO(DLO, fixture0.getClippPosition())
        clipPoint += DLO.getPartLength(minIndex)

        # change clip point depending on orientation 
        angleOne = np.arctan2(diff[1], diff[0]) # atan2(y,x)
        fixtureEuler = tf.transformations.euler_from_quaternion(fixture1.getOrientation(), 'sxyz')
        angleTwo = fixtureEuler[2] + np.pi/2 # euler orienation of y axis around z base. 
        diffAngle = abs(calcAngleDiff(angle1=angleOne, angle2=angleTwo))
        clipPoint += gripWidth/2 *(np.sin(diffAngle-np.pi/2) + 1)
    
    clipPoint += cableSlack
    return clipPoint


def getZRotationCable(length, DLO):
    length0 = length-0.01
    length1 = length+0.01
    if length0 < 0 or length1 > DLO.getLength():
        print('error in getZRotationCable, length outside domain, length0 = ', length0, ' length1 = ', length1, ' DLO Length = ', DLO.getLength())
        return 0

    point0 = DLO.getCoord(length0)
    point1 = DLO.getCoord(length1)

    dy = point1[1] - point0[1]
    dx = point1[0] - point0[0]
    rotZ = np.arctan2(dy,dx)
    return rotZ #  - np.pi/2


def getPointTime(gripperRight, gripperLeft, posTargetRight, posTargetLeft, \
                        rotTargetRight, rotTargetLeft, avgSpeed=0.02, avgRotVel=0.07, shortestTime=2):

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


def getOffestCableConstraint(map_, previousFixture, gripperTargetPosition, DLO, targetFixture, cableSlack):
    if previousFixture >= 0:
        cableAttachmentPostion = map_[previousFixture].getClippPosition()

        dist = np.linalg.norm(gripperTargetPosition - cableAttachmentPostion)

        fixture0 = map_[previousFixture]
        fixture1 = map_[targetFixture]
        length = np.linalg.norm(fixture1.getClippPosition() - fixture0.getClippPosition())
        cableLenght = length + cableSlack
        
        if dist < cableLenght:
            return np.zeros(3)

        offsetFactor = (dist - cableLenght)/dist
        d = cableAttachmentPostion - gripperTargetPosition

        return d*offsetFactor

    else:
        return np.zeros(3)

def calcAngleDiff(angle1, angle2):
    angleDiff = angle1 - angle2
    if angleDiff > np.pi:
        angleDiff -= 2*np.pi
    if angleDiff < - np.pi:
        angleDiff += 2*np.pi
    return angleDiff

def checkIfWithinTol(pos, targetPos, posTol, quat, targetQuat, quatTol):
    dist = np.linalg.norm(pos - targetPos)
    rot = getTotalRadians(quat, targetQuat)
    if dist <= posTol and rot <= quatTol:
        return True
    else:
        return False 


def closesPointDLO(DLO, pos):
    diff = DLO.points - pos
    dist = np.linalg.norm(diff, axis=1)
    minDist = np.min(dist)
    minIndex = np.argmin(dist)
    point = DLO.lengthList[minIndex]
    return minDist, point, minIndex

def rotateX180(q):
    return tf.transformations.quaternion_multiply(q, np.array([1,0,0,0]))


def calcGrippPoints(targetFixture, map_, DLO, grippWidth, clipPoint):

    rotZClippPoint = getZRotationCable(clipPoint, DLO)
    if clipPoint-grippWidth/2 < 0 or clipPoint+grippWidth/2 > DLO.getLength():
        print('Error, pickup points are outside the cable')
        return -1, -1
    point0 = DLO.getCoord(clipPoint-grippWidth/2)
    point1 = DLO.getCoord(clipPoint+grippWidth/2) 
    dy = point1[1] - point0[1]
    dx = point1[0] - point0[0]
    rotZ = np.arctan2(dy,dx)
    fixtureQuat = map_[targetFixture].getOrientation()
    fixtureEuler = tf.transformations.euler_from_quaternion(fixtureQuat, 'sxyz')

    # 0 to Left along, x axis. 
    rotSlack = 20 * np.pi /180 

    angleDiff = calcAngleDiff(rotZ, np.pi/2)

    if abs(angleDiff) < np.pi/2 + rotSlack:
        if fixtureEuler[2] > np.pi/2 + rotSlack and  fixtureEuler[2] < np.pi/2 - rotSlack:
            leftGrippPoint = clipPoint + grippWidth/2
            rightGrippPoint = clipPoint - grippWidth/2
        elif abs(angleDiff) < np.pi/2 - rotSlack:
            leftGrippPoint = clipPoint + grippWidth/2
            rightGrippPoint = clipPoint - grippWidth/2
        elif fixtureEuler[2] > -np.pi/2 and  fixtureEuler[2] < np.pi/2:
            leftGrippPoint = clipPoint + grippWidth/2
            rightGrippPoint = clipPoint - grippWidth/2
        else:
            leftGrippPoint = clipPoint - grippWidth/2
            rightGrippPoint = clipPoint + grippWidth/2
    else:
        leftGrippPoint = clipPoint - grippWidth/2
        rightGrippPoint = clipPoint + grippWidth/2

    return leftGrippPoint, rightGrippPoint

def calcGrippPosRot(DLO, leftGrippPoint, rightGrippPoint, targetHeightRight, targetHeightLeft):
    if leftGrippPoint < 0 or leftGrippPoint > DLO.getLength():
        return np.zeros(3), np.zeros(3), np.zeros(4), np.zeros(4), False
    if rightGrippPoint < 0 or rightGrippPoint > DLO.getLength():
        return np.zeros(3), np.zeros(3), np.zeros(4), np.zeros(4), False
    
    positionRight = DLO.getCoord(rightGrippPoint)
    positionLeft = DLO.getCoord(leftGrippPoint)
    positionRight[2] += targetHeightRight
    positionLeft[2] += targetHeightLeft

    # get  orientation
    if leftGrippPoint > rightGrippPoint:
        rotZRight = getZRotationCable(rightGrippPoint, DLO) - np.pi/2
        rotZLeft = getZRotationCable(leftGrippPoint, DLO) - np.pi/2
        quatRight = tf.transformations.quaternion_from_euler(rotZRight, 0, np.pi, 'rzyx')
        quatLeft = tf.transformations.quaternion_from_euler(rotZLeft, 0, np.pi, 'rzyx')
    else:
        rotZRight = getZRotationCable(rightGrippPoint, DLO) + np.pi/2
        rotZLeft = getZRotationCable(leftGrippPoint, DLO) + np.pi/2
        quatRight = tf.transformations.quaternion_from_euler(rotZRight, 0, np.pi, 'rzyx')
        quatLeft = tf.transformations.quaternion_from_euler(rotZLeft, 0, np.pi, 'rzyx')
    return positionRight, positionLeft, quatRight, quatLeft, True



def closestDistLineToLineSegment(pointA0, pointA1, pointB0, pointB1):
    #Input endpoints for to line segments A and B, assumes constant movment along line in direction they are defined

    #Lines redefined to LA = pointA0 + s*u and LB = pointB0 + t*v
    u = pointA1 - pointA0
    v = pointB1 - pointB0  
    w0 = pointA0 - pointB0
    a = u.dot(u)
    b = u.dot(v)
    c = v.dot(v)
    d = u.dot(w0)
    e = v.dot(w0)    

    if np.linalg.norm(u-v) <= 1e-10:
        # moving in same direction at same speed
        t_cpa = 0

    else:
        t_cpa = -w0.dot(u-v)/(np.linalg.norm(u-v)**2)

    if t_cpa < 0:
        t_cpa = 0
    elif t_cpa > 1:
        t_cpa = 1

    pA = pointA0 + t_cpa*u
    pB = pointB0 + t_cpa*v
    dist = np.linalg.norm(pA - pB)

    return dist
