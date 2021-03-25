#!/usr/bin/env python3

import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils

'''
class subTask(object):
    def __init__(self):
        self.inputArgs = []

    def getTrajectoryPoint(self, inputs):
        pass
'''

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
    

class OverCable(object):
    def __init__(self, targetHeight, gippers):
        self.inputArgs = ['self.map', 'self.DLO', 'self.gripperLeft', 'self.gripperRight', 'self.targetFixture',\
             'self.previousFixture', 'self.cableSlack', 'self.tfListener']
        self.gripper = gippers # in mm, [Right, left]
        self.targetHeight = targetHeight # height from world frame (i.e. table) and not yumi_base_link
            # Otherise the frame is yumi_base_link, [Right, Left], for combined only right is used
        self.grippWidth = 0.15
        self.avgVelocity = 0.02
        self.shortestTime = 1

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

        clipPoint = calcClipPoint(targetFixture, previousFixture, map_, cableSlack)

        point0 = clipPoint - self.grippWidth/2 
        point1 = clipPoint + self.grippWidth/2

        # TODO which arm to which point 

        if point0 > 0 and point1 < DLO.getLength():
            position0 = DLO.getCoord(point0)
            position1 = DLO.getCoord(point0)

            position0[2] = targertHeightBase[0]
            position1[2] = targertHeightBase[1]

        else:
            print('Error, pickup points are outside the cable')
            return []

        dist0 = np.linalg.norm(position0 - gripperRight.getPosition())
        dist1 = np.linalg.norm(position1 - gripperLeft.getPosition())
        maxDist = max(dist0, dist1)

        trajectoryPoint = Trajectory_point()
        trajectoryPoint.positionRight = position0.tolist()
        trajectoryPoint.positionLeft = position1.tolist()
        trajectoryPoint.orientationLeft = [1,0,0,0]
        trajectoryPoint.orientationRight = [1,0,0,0]
        trajectoryPoint.gripperLeft = [self.gripper[1],self.gripper[1]]
        trajectoryPoint.gripperRight = [self.gripper[0],self.gripper[0]]
        trajectoryPoint.pointTime = max(maxDist/self.avgVelocity, self.shortestTime)
