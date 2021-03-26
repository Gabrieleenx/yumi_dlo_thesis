#!/usr/bin/env python3

import numpy as np
import rospy
import subTasks
from controller.msg import Trajectory_point, Trajectory_msg
import tf


class Task(object):
    # base class for all tasks
    def __init__(self, mode):
        self.newTrajectory = 1
        self.taskDone = 0
        self.subTasks = []
        self.startSubTaskIdx = 0
        self.mode = mode
        self.tfListener = tf.TransformListener()

    def getNewTrajectory(self):
        return self.newTrajectory

    def getTrajectory(self):
        self.newTrajectory = 0
        msg = Trajectory_msg()
        msg.header.stamp = rospy.Time.now()
        msg.mode = self.mode
        msg.forceControl = 0 # hardcoded for now
        msg.maxForce = 4.0
        trajectory = []

        gripperRightTemp = self.gripperRight.copyClass()
        gripperLeftTemp = self.gripperLeft.copyClass()

        if self.mode == 'combined':
            position0, orientation0, position1, orientation1 = utils.calcAbsoluteAndRelative(gripperRight, gripperLeft, self.transformer)
            gripperRightTemp.update(position0, orientation0)
            gripperLeftTemp.update(position1, orientation1)

        for i in range(self.startSubTaskIdx, len(self.subTasks)):
            inputArgs = self.subTasks[i].inputArgs
            input_ = []
            for j in range(len(inputArgs)):
                input_.append(eval(inputArgs[j]))

            trajectoryPoint = self.subTasks[i].getTrajectoryPoint(input_)
            trajectory.extend(trajectoryPoint)

        msg.trajectory = trajectory
        return msg

    def getTaskDone(self):
        return self.taskDone

class GrabCable(Task):
    def __init__(self, targetFixture, previousFixture, cableSlack):
        super(GrabCable, self).__init__('individual')   
        goToHeight = subTasks.GoToHeight(np.array([0.2,0.2]), np.array([0,0])) 
        overCable = subTasks.OverCable(np.array([0.2,0.2]), np.array([20,20]))
        onCable = subTasks.OverCable(np.array([0.0,0.0]), np.array([20,20]))
        grippCable = subTasks.HoldPosition(3, np.array([0,0]))

        self.subTasks = [goToHeight, overCable, onCable, grippCable, goToHeight]
        self.targetFixture = targetFixture # int, starting from 0, which fixture is the target,
            # will be matched with map elements.
        self.previousFixture = previousFixture # # int, starting from -1 (-1 means no previous fixture),
            # which fixture is the current fixture the cable is attached to.
        self.cableSlack = cableSlack # float in meters, if no previous it defines how much the cable 
            # should stick out from the fixture, and if previous it defines how much longer the cable should
            # be then the closest distance between the current and target fixture.  
            
    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight):
        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        # TODO track progress and taskDone


class ClippIntoFixture(Task):
     def __init__(self, targetFixture, previousFixture, cableSlack):
        super(ClippIntoFixture, self).__init__('combined')    
        
        self.subTasks = []
        self.targetFixture = targetFixture # int, starting from 0, which fixture is the target,
            # will be matched with map elements.
        self.previousFixture = previousFixture # # int, starting from 0 (-1 means no previous fixture),
            # which fixture is the current fixture the cable is attached to.
        self.cableSlack = cableSlack # float in meters, if no previous it defines how much the cable 
            # should stick out from the fixture, and if previous it defines how much longer the cable should
            # be then the closest distance between the current and target fixture.  


 



