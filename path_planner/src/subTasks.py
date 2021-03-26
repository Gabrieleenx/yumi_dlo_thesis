#!/usr/bin/env python3

import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils


class GoToHeight(object):
    # Works for both comined and individual control, preservs orientation
    def __init__(self, targetHeight, gippers):
        self.inputArgs = ['self.gripperLeft', 'self.gripperRight', 'self.mode', 'self.tfListener']
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
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
        
        elif mode == 'combined':
            absPos, absRot, relPos, relRot = utils.calcAbsoluteAndRelative(gripperRight, gripperLeft, self.transformer)

            maxDist = abs(absPos[2] - targertHeightBase[0])
            absPos[2] = targertHeightBase[0]
            trajectoryPoint.positionRight = absPos.tolist()
            trajectoryPoint.positionLeft = relPos.tolist()
            trajectoryPoint.orientationLeft = relRot.tolist()
            trajectoryPoint.orientationRight = absRot.tolist()
            trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
            trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
            trajectoryPoint.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)

        else: 
            print('Error, mode not valid in subtask')
            return []
        
        return [trajectoryPoint]
    

class OverCable(object): # only for individual control 
    def __init__(self, targetHeight, gippers):
        self.inputArgs = ['self.map', 'self.DLO', 'self.gripperLeft', 'self.gripperRight', 'self.targetFixture',\
             'self.previousFixture', 'self.cableSlack', 'self.tfListener']
        self.gripper = gippers # in mm, [Right, left]
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.grippWidth = 0.15
        self.avgVelocity = 0.02
        self.shortestTime = 1
        self.avgRotVel = 0.4 # it will use which ever is slower between avgRotVel and avgVelocity 

    def getTrajectoryPoint(self, inputs):
        map_ = inputs[0]
        DLO = inputs[1]
        gripperLeft = inputs[2]
        gripperRight = inputs[3]
        targetFixture = inputs[4]
        previousFixture = inputs[5]
        cableSlack = inputs[6]
        tfListener = inputs[7]

        (worldToBase, _) = tfListener.lookupTransform('/world', '/yumi_base_link', rospy.Time(0))
        targertHeightBase = self.targetHeight - worldToBase[2]

        clipPoint = utils.calcClipPoint(targetFixture, previousFixture, map_, cableSlack)

        point0 = clipPoint - self.grippWidth/2 
        point1 = clipPoint + self.grippWidth/2

        # TODO which arm to which point 
        # get position 
        if point0 > 0 and point1 < DLO.getLength():
            position0 = DLO.getCoord(point0)
            position1 = DLO.getCoord(point1)

            position0[2] = targertHeightBase[0]
            position1[2] = targertHeightBase[1]

        else:
            print('Error, pickup points are outside the cable')
            return []
       
        # get  orientation
        rotZ0 = utils.getZRotationCable(point0, DLO)
        rotZ1 = utils.getZRotationCable(point1, DLO)
        quat0 = tf.transformations.quaternion_from_euler(rotZ0, 0, np.pi, 'rzyx')
        quat1 = tf.transformations.quaternion_from_euler(rotZ1, 0, np.pi, 'rzyx')

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

        return [trajectoryPoint]
