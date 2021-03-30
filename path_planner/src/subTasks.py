#!/usr/bin/env python3

import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils


class GoToHeight(object):
    # Works for both comined and individual control, preservs orientation
    def __init__(self, targetHeight, gippers):
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', 'self.mode', 'self.tfListener']
        self.verificationArgs = ['self.taskDone']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.gripper = gippers # in mm, [Right, left]

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
            
            trajectoryPoint.positionRight = positionRight.tolist()
            trajectoryPoint.positionLeft = positionLeft.tolist()
            trajectoryPoint.orientationLeft = orientationLeft.tolist()
            trajectoryPoint.orientationRight = orientationRight.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)

            gripperLeft.update(positionLeft, orientationLeft)
            gripperRight.update(positionRight, orientationRight)

        
        elif mode == 'combined':
            absPos = gripperRight.getPosition()
            absRot = gripperRight.getQuaternion()
            relPos = gripperLeft.getPosition()
            relRot  = gripperLeft.getQuaternion()

            maxDist = abs(absPos[2] - targertHeightBase[0])
            absPos[2] = targertHeightBase[0]
            trajectoryPoint.positionRight = absPos.tolist()
            trajectoryPoint.positionLeft = relPos.tolist()
            trajectoryPoint.orientationLeft = relRot.tolist()
            trajectoryPoint.orientationRight = absRot.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)

            gripperLeft.update(relPos, relRot)
            gripperRight.update(absPos, absRot)

        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]
    
    def verification(self, input_):
        return True

class OverCable(object): # only for individual control 
    def __init__(self, targetHeight, gippers):
        self.inputArgs = ['self.map', 'self.DLO', 'gripperLeftTemp', 'gripperRightTemp', 'self.targetFixture',\
             'self.previousFixture', 'self.cableSlack', 'self.tfListener']
        self.verificationArgs = ['self.DLO']
        self.gripper = gippers # in mm, [Right, left]
        self.targetHeight = targetHeight # height from cable
        self.grippWidth = 0.15
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.avgRotVel = 0.4 # it will use which ever is slower between avgRotVel and avgVelocity 
        self.point0 = 0
        self.point1 = 0
        self.currentTarget = [0]

    def getTrajectoryPoint(self, inputs):
        map_ = inputs[0]
        DLO = inputs[1]
        gripperLeft = inputs[2]
        gripperRight = inputs[3]
        targetFixture = inputs[4]
        previousFixture = inputs[5]
        cableSlack = inputs[6]
        tfListener = inputs[7]

        #(worldToBase, _) = tfListener.lookupTransform('/world', '/yumi_base_link', rospy.Time(0))
        #targertHeightBase = self.targetHeight + worldToBase[2]

        clipPoint = utils.calcClipPoint(targetFixture, previousFixture, map_, cableSlack)

        self.point0 = clipPoint - self.grippWidth/2 
        self.point1 = clipPoint + self.grippWidth/2
        
        if previousFixture >= 0:
            DLO.constrainLength = point0 - map_[previousFixture].previousCableLength
        # TODO which arm to which point 
        # get position 
        if self.point0 > 0 and self.point1 < DLO.getLength():
            position0 = DLO.getCoord(self.point0)
            position1 = DLO.getCoord(self.point1)
            position0[2] += self.targetHeight[0]
            position1[2] += self.targetHeight[1]

        else:
            print('Error, pickup points are outside the cable')
            return []
       
        # get  orientation
        rotZ0 = utils.getZRotationCable(self.point0, DLO)
        rotZ1 = utils.getZRotationCable(self.point1, DLO)
        quat0 = tf.transformations.quaternion_from_euler(rotZ0, 0, np.pi, 'rzyx')
        quat1 = tf.transformations.quaternion_from_euler(rotZ1, 0, np.pi, 'rzyx')

        self.currentTarget = [position0, quat0, position1, quat1]
        # calc time
        time = utils.getPointTime(gripperRight, gripperLeft, position1, position0,\
                        quat1, quat0, self.avgVelocity, 0.4, self.shortestTime)

        trajectoryPoint = Trajectory_point()
        trajectoryPoint.positionRight = position0.tolist()
        trajectoryPoint.positionLeft = position1.tolist()
        trajectoryPoint.orientationLeft = quat1
        trajectoryPoint.orientationRight = quat0
        trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
        trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
        trajectoryPoint.pointTime = time

        gripperLeft.update(position1, quat1)
        gripperRight.update(position0, quat0)

        return [trajectoryPoint]

    def verification(self, input_):
        DLO = input_[0]

        if self.point0 > 0 and self.point1 < DLO.getLength():
            position0 = DLO.getCoord(self.point0)
            position1 = DLO.getCoord(self.point1)

            position0[2] += self.targetHeight[0]
            position1[2] += self.targetHeight[1]

        else:
            print('Error, pickup points are outside the cable')
            return []
       
        # get  orientation
        rotZ0 = utils.getZRotationCable(self.point0, DLO)
        rotZ1 = utils.getZRotationCable(self.point1, DLO)
        quat0 = tf.transformations.quaternion_from_euler(rotZ0, 0, np.pi, 'rzyx')
        quat1 = tf.transformations.quaternion_from_euler(rotZ1, 0, np.pi, 'rzyx')

        check0 = utils.checkIfWithinTol(position0, self.currentTarget[0], 0.02,\
                    quat0, self.currentTarget[1], 0.4)

        check1 = utils.checkIfWithinTol(position1, self.currentTarget[2], 0.02,\
                    quat1, self.currentTarget[3], 0.4)

        if check0 == True and check1 == True:
            return True
        else:
            return False


class HoldPosition(object):
    # Works for both comined and individual control, preservs orientation
    def __init__(self, time_, gippers):
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', 'self.mode']
        self.verificationArgs = ['self.taskDone']
        self.time = time_ # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.gripper = gippers # in mm, [Right, left]

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
            trajectoryPoint.pointTime = self.time

            gripperLeft.update(positionLeft, orientationLeft)
            gripperRight.update(positionRight, orientationRight)
        
        elif mode == 'combined':
            absPos, absRot, relPos, relRot = utils.calcAbsoluteAndRelative(gripperRight, gripperLeft, self.transformer)

            trajectoryPoint.positionRight = absPos.tolist()
            trajectoryPoint.positionLeft = relPos.tolist()
            trajectoryPoint.orientationLeft = relRot.tolist()
            trajectoryPoint.orientationRight = absRot.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = self.time

            gripperLeft.update(relPos, relRot)
            gripperRight.update(absPos, absRot)

        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]
    
    def verification(self, input_):
        return True


class GoToHeightWithCable(object):
    # Works for both comined and individual control, preservs orientation
    def __init__(self, targetHeight, gippers):
        self.inputArgs = ['gripperLeftTemp', 'gripperRightTemp', 'self.mode', 'self.tfListener', 'self.map', 'self.previousFixture', 'self.DLO']
        self.verificationArgs = ['self.taskDone']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.gripper = gippers # in mm, [Right, left]

    def getTrajectoryPoint(self, inputs):
        gripperLeft = inputs[0]
        gripperRight = inputs[1]
        mode = inputs[2]
        tfListener = inputs[3]
        map_ = inputs[4]
        fixtrueIndex = inputs[5]
        DLO = inputs[6]

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

            offsetPosition = utils.getOffestCableConstraint(map_, fixtrueIndex, positionRight, DLO)
            
            positionRight += offsetPosition
            positionLeft += offsetPosition

            trajectoryPoint.positionRight = positionRight.tolist()
            trajectoryPoint.positionLeft = positionLeft.tolist()
            trajectoryPoint.orientationLeft = orientationLeft.tolist()
            trajectoryPoint.orientationRight = orientationRight.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)

            gripperLeft.update(positionLeft, orientationLeft)
            gripperRight.update(positionRight, orientationRight)

        
        elif mode == 'combined':
            # TODO offset constraint thing for combined 
            absPos = gripperRight.getPosition()
            absRot = gripperRight.getQuaternion()
            relPos = gripperLeft.getPosition()
            relRot  = gripperLeft.getQuaternion()

            maxDist = abs(absPos[2] - targertHeightBase[0])
            absPos[2] = targertHeightBase[0]
            trajectoryPoint.positionRight = absPos.tolist()
            trajectoryPoint.positionLeft = relPos.tolist()
            trajectoryPoint.orientationLeft = relRot.tolist()
            trajectoryPoint.orientationRight = absRot.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)

            gripperLeft.update(relPos, relRot)
            gripperRight.update(absPos, absRot)

        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]   

    def verification(self, input_):
        # TODO
        return True