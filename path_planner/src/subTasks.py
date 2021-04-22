#!/usr/bin/env python3

import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils

'''
Subtask needs to contain the following

class subTask(object):
    def __init__(self, grippers):
        self.gripper = grippers # in mm, [Right, left]
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', ...] 
        self.verificationArgs = ['var1',..]   at least one variable for these even if nothing is used 
        self.pointTime = 1

    def getTrajectoryPoint(self, inputs):
        gripperLeft = inputs[0]
        gripperRight = inputs[1]

        do things ... 

        gripperLeft.update(positionLeft, orientationLeft)
        gripperRight.update(positionRight, orientationRight)
        return [trajectoryPoint]

    def verification(self, input_):
        return bool 
'''

class GoToHeightIndividual(object):
    # Works for both comined and individual control, preservs orientation
    def __init__(self, targetHeight, grippers):
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', 'self.mode', 'self.tfListener']
        self.verificationArgs = ['self.taskDone']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.gripper = grippers # in mm, [Right, left]
        self.pointTime = 1

    def getTrajectoryPoint(self, inputs):
        gripperLeft = inputs[0]
        gripperRight = inputs[1]
        mode = inputs[2]
        tfListener = inputs[3]

        (worldToBase, _) = tfListener.lookupTransform('/world', '/yumi_base_link', rospy.Time(0))
        targertHeightBase = self.targetHeight - worldToBase[2]
        trajectoryPoint = Trajectory_point()

        if mode == 'individual':
            positionLeft = gripperLeft.getPosition()
            positionRight = gripperRight.getPosition()
            orientationLeft = gripperLeft.getQuaternion()
            orientationRight = gripperRight.getQuaternion()

            maxDist = max(abs(positionLeft[2] - targertHeightBase[1]), abs(positionRight[2] - targertHeightBase[0]))
            positionRight[2] = targertHeightBase[0]
            positionLeft[2] = targertHeightBase[1]
            self.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)

            trajectoryPoint.positionRight = positionRight.tolist()
            trajectoryPoint.positionLeft = positionLeft.tolist()
            trajectoryPoint.orientationLeft = orientationLeft.tolist()
            trajectoryPoint.orientationRight = orientationRight.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = self.pointTime

            gripperLeft.update(positionLeft, orientationLeft)
            gripperRight.update(positionRight, orientationRight)

        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]
    
    def verification(self, input_):
        return True

class GoToHeightCombined(object):
    # Works for both comined and individual control, preservs orientation
    def __init__(self, targetHeight, grippers):
        self.inputArgs = ['absoluteTemp', 'relativeTemp', 'self.mode', 'self.tfListener']
        self.verificationArgs = ['self.taskDone']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.gripper = grippers # in mm, [Right, left]
        self.pointTime = 1

    def getTrajectoryPoint(self, inputs):
        absolute = inputs[0]
        relative = inputs[1]
        mode = inputs[2]
        tfListener = inputs[3]

        (worldToBase, _) = tfListener.lookupTransform('/world', '/yumi_base_link', rospy.Time(0))
        targertHeightBase = self.targetHeight - worldToBase[2]
        trajectoryPoint = Trajectory_point()
        
        if mode == 'combined':
            absPos = absolute.getPosition()
            absRot = absolute.getQuaternion()
            relPos = relative.getPosition()
            relRot  = relative.getQuaternion()

            maxDist = abs(absPos[2] - targertHeightBase[0])
            absPos[2] = targertHeightBase[0]
            self.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)

            trajectoryPoint.positionRight = absPos.tolist()
            trajectoryPoint.positionLeft = relPos.tolist()
            trajectoryPoint.orientationLeft = relRot.tolist()
            trajectoryPoint.orientationRight = absRot.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = self.pointTime 

            relative.update(relPos, relRot)
            absolute.update(absPos, absRot)

        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]
    
    def verification(self, input_):
        return True

class OverCableIndividual(object): # only for individual control 
    def __init__(self, targetHeight, grippers, grippWidth):
        self.inputArgs = ['self.map', 'self.DLO', 'gripperLeftTemp', 'gripperRightTemp', 'self.targetFixture',\
             'self.previousFixture', 'self.cableSlack', 'self.tfListener']
        self.verificationArgs = ['self.DLO']
        self.gripper = grippers # in mm, [Right, left]
        self.targetHeight = targetHeight # height from cable
        self.grippWidth = grippWidth
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.avgRotVel = 0.4 # it will use which ever is slower between avgRotVel and avgVelocity 
        self.currentTarget = [0]
        self.pointTime = 1
        self.leftGrippPoint = 0
        self.rightGrippPoint = 0

    def getTrajectoryPoint(self, inputs):
        map_ = inputs[0]
        DLO = inputs[1]
        gripperLeft = inputs[2]
        gripperRight = inputs[3]
        targetFixture = inputs[4]
        previousFixture = inputs[5]
        cableSlack = inputs[6]
        tfListener = inputs[7]
        
        clipPoint = utils.calcClipPoint(targetFixture, previousFixture, map_, cableSlack, DLO)
        self.leftGrippPoint, self.rightGrippPoint = utils.calcGrippPoints(targetFixture, map_, DLO, self.grippWidth, clipPoint)

        if self.leftGrippPoint == -1 or self.rightGrippPoint == -1:
            return []

        positionRight, positionLeft,quatRight, quatLeft, inRange = utils.calcGrippPosRot(DLO,\
                        self.leftGrippPoint, self.rightGrippPoint, self.targetHeight[0], self.targetHeight[1])

        if inRange == False:
            return []
        self.currentTarget = [positionRight, quatRight, positionLeft, quatLeft]
        
        # calc time
        self.pointTime = utils.getPointTime(gripperRight, gripperLeft, positionRight, positionLeft,\
                        quatRight, quatLeft, self.avgVelocity, 0.4, self.shortestTime)

        trajectoryPoint = Trajectory_point()
        trajectoryPoint.positionRight = positionRight.tolist()
        trajectoryPoint.positionLeft = positionLeft.tolist()
        trajectoryPoint.orientationLeft = quatLeft.tolist()
        trajectoryPoint.orientationRight = quatRight.tolist()
        trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
        trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
        trajectoryPoint.pointTime = self.pointTime

        gripperRight.update(positionRight, quatRight)
        gripperLeft.update(positionLeft, quatLeft)
    
        return [trajectoryPoint]

    def verification(self, input_):
        DLO = input_[0]
        positionRight, positionLeft,quatRight, quatLeft, inRange = utils.calcGrippPosRot(DLO,\
                        self.leftGrippPoint, self.rightGrippPoint, self.targetHeight[0], self.targetHeight[1])
        
        if inRange == False:
            print('Error, pickup points are outside the cable')
            return False

        checkRight = utils.checkIfWithinTol(positionRight, self.currentTarget[0], 0.02,\
                    quatRight, self.currentTarget[1], 0.4)

        checkLeft = utils.checkIfWithinTol(positionLeft, self.currentTarget[2], 0.02,\
                    quatLeft, self.currentTarget[3], 0.4)

        if checkRight == True and checkLeft == True:
            return True
        else:
            return False


class HoldPositionIndividual(object):
    def __init__(self, time_, grippers):
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', 'self.mode']
        self.verificationArgs = ['self.taskDone']
        self.pointTime = time_ # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.gripper = grippers # in mm, [Right, left]

    def getTrajectoryPoint(self, inputs):
        gripperLeft = inputs[0]
        gripperRight = inputs[1]
        mode = inputs[2]

        trajectoryPoint = Trajectory_point()

        if mode == 'individual':
            positionLeft = gripperLeft.getPosition()
            positionRight = gripperRight.getPosition()
            orientationLeft = gripperLeft.getQuaternion()
            orientationRight = gripperRight.getQuaternion()

            trajectoryPoint.positionRight = positionRight.tolist()
            trajectoryPoint.positionLeft = positionLeft.tolist()
            trajectoryPoint.orientationLeft = orientationLeft.tolist()
            trajectoryPoint.orientationRight = orientationRight.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = self.pointTime

            gripperLeft.update(positionLeft, orientationLeft)
            gripperRight.update(positionRight, orientationRight)
        
        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]
    
    def verification(self, input_):
        return True


class HoldPositionCombined(object):
    def __init__(self, time_, grippers):
        self.inputArgs = ['relativeTemp', 'absoluteTemp', 'self.mode']
        self.verificationArgs = ['self.taskDone']
        self.pointTime = time_ # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.gripper = grippers # in mm, [Right, left]

    def getTrajectoryPoint(self, inputs):
        relative = inputs[0]
        absolute = inputs[1]
        mode = inputs[2]

        trajectoryPoint = Trajectory_point()
        
        if mode == 'combined':
            absPos = absolute.getPosition()
            absRot = absolute.getQuaternion()
            relPos = relative.getPosition()
            relRot  = relative.getQuaternion()


            trajectoryPoint.positionRight = absPos.tolist()
            trajectoryPoint.positionLeft = relPos.tolist()
            trajectoryPoint.orientationLeft = relRot.tolist()
            trajectoryPoint.orientationRight = absRot.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = self.pointTime

            relative.update(relPos, relRot)
            absolute.update(absPos, absRot)

        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]
    
    def verification(self, input_):
        return True


class GoToHeightWithCableIndividual(object):
    def __init__(self, targetHeight, grippers):
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', 'self.mode', 'self.tfListener', 'self.map', 'self.previousFixture',\
             'self.DLO', 'self.targetFixture', 'self.cableSlack']
        self.verificationArgs = ['self.DLO', 'self.gripperRight', 'self.gripperLeft']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.gripper = grippers # in mm, [Right, left]
        self.tol = 0.06 # if cable gripp position is off more then 4 cm then task faild and also
                        # some margin for noise and delay
        self.pointTime = 1

    def getTrajectoryPoint(self, inputs):
        gripperLeft = inputs[0]
        gripperRight = inputs[1]
        mode = inputs[2]
        tfListener = inputs[3]
        map_ = inputs[4]
        previousFixture = inputs[5]
        DLO = inputs[6]
        targetFixture = inputs[7]
        cableSlack = inputs[8]

        (worldToBase, _) = tfListener.lookupTransform('/world', '/yumi_base_link', rospy.Time(0))
        targertHeightBase = self.targetHeight - worldToBase[2]
        trajectoryPoint = Trajectory_point()

        if mode == 'individual':
            positionLeft = gripperLeft.getPosition()
            positionRight = gripperRight.getPosition()
            orientationLeft = gripperLeft.getQuaternion()
            orientationRight = gripperRight.getQuaternion()

            maxDist = max(abs(positionLeft[2] - targertHeightBase[1]), abs(positionRight[2] - targertHeightBase[0]))
            positionRight[2] = targertHeightBase[0]
            positionLeft[2] = targertHeightBase[1]

            offsetPosition = utils.getOffestCableConstraint(map_, previousFixture, positionRight, DLO, targetFixture, cableSlack)
            
            positionRight += offsetPosition
            positionLeft += offsetPosition
            self.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)

            trajectoryPoint.positionRight = positionRight.tolist()
            trajectoryPoint.positionLeft = positionLeft.tolist()
            trajectoryPoint.orientationLeft = orientationLeft.tolist()
            trajectoryPoint.orientationRight = orientationRight.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = self.pointTime

            gripperLeft.update(positionLeft, orientationLeft)
            gripperRight.update(positionRight, orientationRight)

        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]   

    def verification(self, input_):
        DLO = input_[0]
        gripperRight = input_[1]
        gripperLeft = input_[2]

        minDistRight, pointRight, minIndex = utils.closesPointDLO(DLO, gripperRight.getPosition())
        minDistLeft, pointLeft, minIndex = utils.closesPointDLO(DLO, gripperLeft.getPosition())

        if minDistLeft < self.tol and minDistRight < self.tol:
            return True
        else:
            return False



class GoToHeightWithCableCombined(object):
    def __init__(self, targetHeight, grippers):
        self.inputArgs = ['relativeTemp', 'absoluteTemp', 'self.mode', 'self.tfListener', 'self.map', 'self.previousFixture',\
             'self.DLO', 'self.targetFixture', 'self.cableSlack']
        self.verificationArgs = ['self.DLO', 'self.gripperRight', 'self.gripperLeft']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.gripper = grippers # in mm, [Right, left]
        self.tol = 0.06 # if cable gripp position is off more then 4 cm then task faild and also
                        # some margin for noise and delay
        self.pointTime = 1

    def getTrajectoryPoint(self, inputs):
        relative = inputs[0]
        absolute = inputs[1]
        mode = inputs[2]
        tfListener = inputs[3]
        map_ = inputs[4]
        previousFixture = inputs[5]
        DLO = inputs[6]
        targetFixture = inputs[7]
        cableSlack = inputs[8]

        (worldToBase, _) = tfListener.lookupTransform('/world', '/yumi_base_link', rospy.Time(0))
        targertHeightBase = self.targetHeight - worldToBase[2]
        trajectoryPoint = Trajectory_point()
        
        if mode == 'combined':
            # TODO offset constraint thing for combined 
            absPos = absolute.getPosition()
            absRot = absolute.getQuaternion()
            relPos = relative.getPosition()
            relRot  = relative.getQuaternion()

            maxDist = abs(absPos[2] - targertHeightBase[0])
            absPos[2] = targertHeightBase[0]
            self.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)

            trajectoryPoint.positionRight = absPos.tolist()
            trajectoryPoint.positionLeft = relPos.tolist()
            trajectoryPoint.orientationLeft = relRot.tolist()
            trajectoryPoint.orientationRight = absRot.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = self.pointTime

            relative.update(relPos, relRot)
            absolute.update(absPos, absRot)

        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]   

    def verification(self, input_):
        DLO = input_[0]
        gripperRight = input_[1]
        gripperLeft = input_[2]

        minDistRight, pointRight, minIndex = utils.closesPointDLO(DLO, gripperRight.getPosition())
        minDistLeft, pointLeft, minIndex = utils.closesPointDLO(DLO, gripperLeft.getPosition())

        if minDistLeft < self.tol and minDistRight < self.tol:
            return True
        else:
            return False


class OverFixtureCombinded(object):
    def __init__(self, grippers, gripperWidth, targetHeight):
        self.gripper = grippers # in mm, [Right, left]
        self.inputArgs = ['relativeTemp', 'absoluteTemp', 'self.map',\
                 'self.previousFixture', 'self.targetFixture', 'self.tfListener', 'self.DLO'] 
        self.verificationArgs = ['self.taskDone'] #  at least one variable for these even if nothing is used 
        self.pointTime = 1
        self.gripperWidth = gripperWidth
        self.targetHeight = targetHeight # height over fixture, single float 
        self.avgVelocity = 0.02 # m/s
        self.avgRotVel = 0.2 # rad/s 
        self.shortestTime = 1

    def getTrajectoryPoint(self, inputs):
        relative = inputs[0]
        absolute = inputs[1]
        map_ =  inputs[2]
        previousFixture = inputs[3]
        targetFixture = inputs[4]
        tfListener = inputs[5]
        DLO = inputs[6]

        targetFixtureObj = map_[targetFixture]
        absPos = targetFixtureObj.getClippPosition()
        absPos[2] +=  self.targetHeight
        rot = targetFixtureObj.getOrientation()

        if absolute.flipped == -1:
            absPrevQuat = absolute.getQuaternion()
            minDist, length, minIndex = utils.closesPointDLO(DLO, absolute.getPosition())
            cableRotZ = utils.getZRotationCable(length, DLO) - np.pi/2 
            startAbsEuler = tf.transformations.euler_from_quaternion(absPrevQuat, 'sxyz')
            if abs(cableRotZ - startAbsEuler[2]) > np.pi/2:
                absolute.flipped = 1

        if absolute.flipped == 1:
            # rotate z 180
            rot = tf.transformations.quaternion_multiply(rot, np.array([0,0,1,0]))

        absRot = utils.rotateX180(rot)
        relRot = np.array([0,0,0,1])
        relPos = np.array([0, self.gripperWidth, 0])

        self.pointTime = utils.getPointTime(absolute, relative, absPos, relPos,\
                        absRot, relRot, self.avgVelocity, self.avgRotVel, self.shortestTime)
        trajectoryPoint = Trajectory_point()

        trajectoryPoint.positionRight = absPos.tolist()
        trajectoryPoint.positionLeft = relPos.tolist()
        trajectoryPoint.orientationLeft = relRot.tolist()
        trajectoryPoint.orientationRight = absRot.tolist()
        trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
        trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
        trajectoryPoint.pointTime = self.pointTime

        relative.update(relPos, relRot)
        absolute.update(absPos, absRot)
        return [trajectoryPoint]

    def verification(self, input_):
        return True