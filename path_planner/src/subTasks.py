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
class GoToHeightIndividualResetOrientatin(object):
    # Works for both comined and individual control, preservs orientation
    def __init__(self, targetHeight, grippers):
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', 'self.mode', 'self.tfListener', 'self.jointPosition']
        self.verificationArgs = ['self.taskDone']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.gripper = grippers # in mm, [Right, left]
        self.pointTime = 1

    def getTrajectoryPoint(self, inputs):
        gripperLeft = inputs[0]
        gripperRight = inputs[1]
        mode = inputs[2]
        tfListener = inputs[3]
        jointPosition = inputs[4]

        (worldToBase, _) = tfListener.lookupTransform('/world', '/yumi_base_link', rospy.Time(0))
        targertHeightBase = self.targetHeight - worldToBase[2]
        trajectoryPoint = Trajectory_point()

        if mode == 'individual':
            positionLeft = gripperLeft.getPosition()
            positionRight = gripperRight.getPosition()
            
            if abs(jointPosition[6]) > np.pi/0.7 or abs(jointPosition[13]) > np.pi/0.7: 
                orientationRight = tf.transformations.quaternion_from_euler(-jointPosition[6]/2, 0, np.pi, 'rzyx')
                orientationLeft = tf.transformations.quaternion_from_euler(-jointPosition[13]/2, 0, np.pi, 'rzyx')
            else: 
                orientationLeft = np.array([1,0,0,0])
                orientationRight = np.array([1,0,0,0])

            positionRight[2] = targertHeightBase[0]
            positionLeft[2] = targertHeightBase[1]

            self.pointTime = utils.getPointTime(gripperRight=gripperRight,\
                                        gripperLeft=gripperLeft,\
                                        posTargetRight=positionRight,\
                                        posTargetLeft=positionLeft, \
                                        rotTargetRight=orientationRight,\
                                        rotTargetLeft=orientationLeft)
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


class GoToHeightIndividual(object):
    # Works for both comined and individual control, preservs orientation
    def __init__(self, targetHeight, grippers):
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', 'self.mode', 'self.tfListener']
        self.verificationArgs = ['self.taskDone']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
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

            positionRight[2] = targertHeightBase[0]
            positionLeft[2] = targertHeightBase[1]

            self.pointTime = utils.getPointTime(gripperRight=gripperRight,\
                                        gripperLeft=gripperLeft,\
                                        posTargetRight=positionRight,\
                                        posTargetLeft=positionLeft, \
                                        rotTargetRight=orientationRight,\
                                        rotTargetLeft=orientationLeft)
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


class GoToHeightWithCableIndividual(object):
    def __init__(self, targetHeight, grippers):
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', 'self.mode', 'self.tfListener']

        self.verificationArgs = ['self.DLO', 'self.gripperRight', 'self.gripperLeft']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.gripper = grippers # in mm, [Right, left]
        self.tol = 0.07 # if cable gripp position is off more then 4 cm then task faild and also
                        # some margin for noise and delay
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

            positionRight[2] = targertHeightBase[0]
            positionLeft[2] = targertHeightBase[1]

            self.pointTime = utils.getPointTime(gripperRight=gripperRight,\
                                        gripperLeft=gripperLeft,\
                                        posTargetRight=positionRight,\
                                        posTargetLeft=positionLeft, \
                                        rotTargetRight=orientationRight,\
                                        rotTargetLeft=orientationLeft)
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


class GoToHeightCombined(object):
    # Works for both comined and individual control, preservs orientation
    def __init__(self, targetHeight, grippers):
        self.inputArgs = ['absoluteTemp', 'relativeTemp', 'self.mode', 'self.tfListener']
        self.verificationArgs = ['self.taskDone']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
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

            absPos[2] = targertHeightBase[0]

            self.pointTime = utils.getPointTime(gripperRight=absolute,\
                            gripperLeft=relative,\
                            posTargetRight=absPos,\
                            posTargetLeft=relPos, \
                            rotTargetRight=absRot,\
                            rotTargetLeft=relRot)
            trajectoryPoint.positionAbsolute = absPos.tolist()
            trajectoryPoint.positionRelative = relPos.tolist()
            trajectoryPoint.orientationRelative = relRot.tolist()
            trajectoryPoint.orientationAbsolute = absRot.tolist()
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
        self.verificationArgs = ['self.DLO', 'self.gripperRight', 'self.gripperLeft']
        self.gripper = grippers # in mm, [Right, left]
        self.targetHeight = targetHeight # height from cable
        self.grippWidth = grippWidth
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
            print('Error, not in range')
            return []

        positionRight, positionLeft,quatRight, quatLeft, inRange = utils.calcGrippPosRot(DLO,\
                        self.leftGrippPoint, self.rightGrippPoint, self.targetHeight[0], self.targetHeight[1])
        positionRight[2] = np.maximum(positionRight[2], 0.009)
        positionLeft[2] = np.maximum(positionLeft[2], 0.009)

        if inRange == False:
            print('Error, not in range')
            return []

        self.currentTarget = [positionRight, quatRight, positionLeft, quatLeft]
        
        # calc time
        self.pointTime = utils.getPointTime(gripperRight=gripperRight,\
                            gripperLeft=gripperLeft,\
                            posTargetRight=positionRight,\
                            posTargetLeft=positionLeft, \
                            rotTargetRight=quatRight,\
                            rotTargetLeft=quatLeft)
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
        gripperRight = input_[1]
        gripperLeft = input_[2]

        if gripperLeft.getPosition()[2] < 0.09 or gripperRight.getPosition()[2] < 0.09:
            return True

        positionRight, positionLeft,quatRight, quatLeft, inRange = utils.calcGrippPosRot(DLO,\
                        self.leftGrippPoint, self.rightGrippPoint, self.targetHeight[0], self.targetHeight[1])
        
        if inRange == False:
            print('Error, pickup points are outside the cable')
            return False

        checkRight = utils.checkIfWithinTol(positionRight, self.currentTarget[0], 0.04,\
                    quatRight, self.currentTarget[1], 0.4)

        checkLeft = utils.checkIfWithinTol(positionLeft, self.currentTarget[2], 0.04,\
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


            trajectoryPoint.positionAbsolute = absPos.tolist()
            trajectoryPoint.positionRelative = relPos.tolist()
            trajectoryPoint.orientationRelative = relRot.tolist()
            trajectoryPoint.orientationAbsolute = absRot.tolist()
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

        if absolute.flipped == 1:
            # rotate z 180
            rot = tf.transformations.quaternion_multiply(rot, np.array([0,0,1,0]))

        absRot = utils.rotateX180(rot)
        relRot = np.array([0,0,0,1])
        relPos = np.array([0, self.gripperWidth, 0])
        
        self.pointTime = utils.getPointTime(gripperRight=absolute,\
                                        gripperLeft=relative,\
                                        posTargetRight=absPos,\
                                        posTargetLeft=relPos, \
                                        rotTargetRight=absRot,\
                                        rotTargetLeft=relRot)
        
        trajectoryPoint = Trajectory_point()

        trajectoryPoint.positionAbsolute = absPos.tolist()
        trajectoryPoint.positionRelative = relPos.tolist()
        trajectoryPoint.orientationRelative = relRot.tolist()
        trajectoryPoint.orientationAbsolute = absRot.tolist()
        trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
        trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
        trajectoryPoint.pointTime = self.pointTime

        relative.update(relPos, relRot)
        absolute.update(absPos, absRot)
        return [trajectoryPoint]

    def verification(self, input_):
        return True


class CableReroutingOverIndividual(object): # only for individual control 
    def __init__(self, individual, targetHeight, grippers):
        self.inputArgs = ['self.DLO', 'gripperLeftTemp', 'gripperRightTemp']
        self.verificationArgs = ['self.DLO', 'self.gripperRight', 'self.gripperLeft']
        self.gripper = grippers # in mm, [Right, left]
        self.targetHeight = targetHeight # height from cable
        self.individal = individual

        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.avgRotVel = 0.4 # it will use which ever is slower between avgRotVel and avgVelocity 
        self.currentTarget = [0]
        self.pointTime = 1
        self.leftGrippPoint = 0
        self.rightGrippPoint = 0

    def getTrajectoryPoint(self, inputs):
        DLO = inputs[0]
        gripperLeft = inputs[1]
        gripperRight = inputs[2]
        pickupPoints = self.individal.getPickupPoints()
        self.rightGrippPoint = pickupPoints[0]
        self.leftGrippPoint = pickupPoints[1]
        positionRight, positionLeft,quatRight, quatLeft, inRange = utils.calcGrippPosRot(DLO,\
                        self.leftGrippPoint, self.rightGrippPoint, self.targetHeight[0], self.targetHeight[1])
        positionRight[2] = np.maximum(positionRight[2], 0.009)
        positionLeft[2] = np.maximum(positionLeft[2], 0.009)
        if inRange == False:
            print('Error, not in range')

            return []
        
        if self.individal.pickupRightValid == 0:
            positionRight = np.array([0.1, -0.35, 0.1]) # gripperRight.getPosition()
            quatRight = np.array([1,0,0,0])#gripperRight.getQuaternion()

        if self.individal.pickupLeftValid == 0:
            positionLeft = np.array([0.1, 0.35, 0.1]) # gripperLeft.getPosition()
            quatLeft = np.array([1,0,0,0])#gripperLeft.getQuaternion()

        self.currentTarget = [positionRight, quatRight, positionLeft, quatLeft]
        
        # calc time
        self.pointTime = utils.getPointTime(gripperRight=gripperRight,\
                                        gripperLeft=gripperLeft,\
                                        posTargetRight=positionRight,\
                                        posTargetLeft=positionLeft, \
                                        rotTargetRight=quatRight,\
                                        rotTargetLeft=quatLeft)
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
        gripperRight = input_[1]
        gripperLeft = input_[2]

        if gripperLeft.getPosition()[2] < 0.09 or gripperRight.getPosition()[2] < 0.09:
            return True

        positionRight, positionLeft,quatRight, quatLeft, inRange = utils.calcGrippPosRot(DLO,\
                        self.leftGrippPoint, self.rightGrippPoint, self.targetHeight[0], self.targetHeight[1])
        
        if inRange == False:
            print('Error, pickup points are outside the cable')
            return False

        if self.individal.pickupRightValid == 1:
            checkRight = utils.checkIfWithinTol(positionRight, self.currentTarget[0], 0.04,\
                                                quatRight, self.currentTarget[1], 0.4)
        else: 
            checkRight = True

        if self.individal.pickupLeftValid == 1:
            checkLeft = utils.checkIfWithinTol(positionLeft, self.currentTarget[2], 0.04,\
                                                quatLeft, self.currentTarget[3], 0.4)
        else: 
            checkLeft = True

    
        if checkRight == True and checkLeft == True:
            return True
        else:
            return False


class CableReroutingEndPosIndividual(object): # only for individual control 
    def __init__(self, individual, targetHeight, grippers):
        self.inputArgs = ['self.DLO', 'gripperLeftTemp', 'gripperRightTemp', 'self.tfListener']
        self.verificationArgs = ['self.DLO', 'self.gripperRight', 'self.gripperLeft']
        self.gripper = grippers # in mm, [Right, left]
        self.targetHeight = targetHeight # height from cable
        self.individual = individual

        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.avgRotVel = 0.4 # it will use which ever is slower between avgRotVel and avgVelocity 
        self.pointTime = 1
        

    def getTrajectoryPoint(self, inputs):
        DLO = inputs[0]
        gripperLeft = inputs[1]
        gripperRight = inputs[2]
        tfListener = inputs[3]

        (worldToBase, _) = tfListener.lookupTransform('/world', '/yumi_base_link', rospy.Time(0))
        targertHeightBase = self.targetHeight - worldToBase[2]

        rightPos, leftPos, quatRight, quatLeft = self.individual.getRightLeftPosQuat(np.array([0.03, 0.03]))

        if self.individual.pickupRightValid == 1:
            positionRight = rightPos
            positionRight[2] = targertHeightBase[0]
        else:
            positionRight = np.array([0.1, -0.35, 0.1]) # gripperRight.getPosition()
            quatRight = np.array([1,0,0,0])#gripperRight.getQuaternion()

        if self.individual.pickupLeftValid == 1:
            positionLeft = leftPos
            positionLeft[2] = targertHeightBase[1]
        else:
            positionLeft = np.array([0.1, 0.35, 0.1]) # gripperLeft.getPosition()
            quatLeft = np.array([1,0,0,0])#gripperLeft.getQuaternion()

        # calc time
        self.pointTime = utils.getPointTime(gripperRight=gripperRight,\
                                                gripperLeft=gripperLeft,\
                                                posTargetRight=positionRight,\
                                                posTargetLeft=positionLeft, \
                                                rotTargetRight=quatRight,\
                                                rotTargetLeft=quatLeft)
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

        return True


class ReroutingCombined(object):
    def __init__(self, absPosXY, absRotZ, targetHeight, grippers, gripperWidth):
        self.gripper = grippers # in mm, [Right, left]
        self.inputArgs = ['relativeTemp', 'absoluteTemp', 'self.tfListener'] 
        self.verificationArgs = ['self.taskDone'] #  at least one variable for these even if nothing is used 
        self.pointTime = 1
        self.gripperWidth = gripperWidth
        self.targetHeight = targetHeight # height over world, single float 
        self.avgVelocity = 0.02 # m/s
        self.avgRotVel = 0.2 # rad/s 
        self.shortestTime = 1
        self.absPosXY = absPosXY
        self.absRotZ = absRotZ

    def getTrajectoryPoint(self, inputs):
        relative = inputs[0]
        absolute = inputs[1]
        tfListener = inputs[2]

        (worldToBase, _) = tfListener.lookupTransform('/world', '/yumi_base_link', rospy.Time(0))
        absPos = np.zeros(3)
        absPos[0] = self.absPosXY[0]
        absPos[1] = self.absPosXY[1]
        absPos[2] = self.targetHeight- worldToBase[2]
        absRot = tf.transformations.quaternion_from_euler(self.absRotZ, 0, 180*np.pi/180, 'rzyx')
        relRot = np.array([0,0,0,1])
        relPos = np.array([0, self.gripperWidth, 0])

        self.pointTime = utils.getPointTime(gripperRight=absolute,\
                                            gripperLeft=relative,\
                                            posTargetRight=absPos,\
                                            posTargetLeft=relPos, \
                                            rotTargetRight=absRot,\
                                            rotTargetLeft=relRot)
        trajectoryPoint = Trajectory_point()

        trajectoryPoint.positionAbsolute = absPos.tolist()
        trajectoryPoint.positionRelative = relPos.tolist()
        trajectoryPoint.orientationRelative = relRot.tolist()
        trajectoryPoint.orientationAbsolute = absRot.tolist()
        trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
        trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
        trajectoryPoint.pointTime = self.pointTime

        relative.update(relPos, relRot)
        absolute.update(absPos, absRot)
        return [trajectoryPoint]

    def verification(self, input_):
        return True