#!/usr/bin/env python3

import numpy as np
import rospy
import subTasks, utils
from controller.msg import Trajectory_point, Trajectory_msg
import tf


class Task(object):
    # base class for all tasks
    def __init__(self, mode):
        self.newTrajectory = 1
        self.newTrajectoryCheck = 1
        self.taskDone = 0
        self.subTasks = []
        self.goToSubTask = []
        self.startSubTaskIdx = 0
        self.mode = mode
        self.tfListener = tf.TransformListener()
        self.subTaskCheck = 1
        self.time = rospy.Time.now()
        self.lastSubtask = 0
        self.numSubTasks = 0
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))

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
            position0, orientation0, position1, orientation1 = utils.calcAbsoluteAndRelative(self.gripperRight, self.gripperLeft, self.transformer)
            gripperRightTemp.update(position0, orientation0)
            gripperLeftTemp.update(position1, orientation1)

        for i in range(self.startSubTaskIdx, len(self.subTasks)):

            inputArgs = self.subTasks[i].inputArgs
            input_ = []
            for j in range(len(inputArgs)):
                input_.append(eval(inputArgs[j]))
                    
            trajectoryPoint = self.subTasks[i].getTrajectoryPoint(input_)

            trajectory.extend(trajectoryPoint)

        # check trajectory, modify trajectory?

        msg.trajectory = trajectory
        return msg

    def trackProgress(self, currentSubTask):

        if currentSubTask != 0 and self.newTrajectoryCheck == 1:     
            return

        self.newTrajectoryCheck = 0
        currentSubTask = self.startSubTaskIdx + currentSubTask

        # check if task is done
        if currentSubTask == self.numSubTasks-1 and self.lastSubtask == self.numSubTasks-2:
            self.time = rospy.Time.now().to_sec()
        if currentSubTask == self.numSubTasks-1:
            pointTime = self.subTasks[currentSubTask].pointTime 
            if pointTime < rospy.Time.now().to_sec() - self.time:
                self.taskDone = 1

        # verify subtask 
        inputArgs = self.subTasks[currentSubTask].verificationArgs
        input_ = []
        for j in range(len(inputArgs)):
            input_.append(eval(inputArgs[j]))

        verified = self.subTasks[currentSubTask].verification(input_) # Do better verification 
        if verified == True: 
            self.lastSubtask = currentSubTask
            return
        elif verified == False:
            self.startSubTaskIdx = self.goToSubTask[currentSubTask]
            self.time = rospy.Time.now().to_sec()
            self.newTrajectory = 1
            self.newTrajectoryCheck = 1



    def getTaskDone(self):
        return self.taskDone


class GrabCable(Task):
    def __init__(self, targetFixture, previousFixture, cableSlack):
        super(GrabCable, self).__init__('individual')   
        goToHeight = subTasks.GoToHeight(np.array([0.10,0.10]), np.array([20,20])) 
        overCable = subTasks.OverCable(np.array([0.10,0.10]), np.array([20,20]), 0.15)
        onCable = subTasks.OverCable(np.array([0.009,0.002]), np.array([20,20]), 0.15)
        grippCable = subTasks.HoldPosition(3, np.array([0,0]))
        goToHeightWithCable = subTasks.GoToHeightWithCable(np.array([0.1,0.1]), np.array([0,0])) 

        self.subTasks = [goToHeight, overCable, onCable, grippCable, goToHeightWithCable]
        self.goToSubTask = [0,1,1,1,1]
        self.numSubTasks = len(self.subTasks)

        self.targetFixture = targetFixture # int, starting from 0, which fixture is the target,
            # will be matched with map elements.
        self.previousFixture = previousFixture # # int, starting from -1 (-1 means no previous fixture),
            # which fixture is the current fixture the cable is attached to.
        self.cableSlack = cableSlack # float in meters, if no previous it defines how much the cable 
            # should stick out from the fixture, and if previous it defines how much longer the cable should
            # be then the closest distance between the current and target fixture.  
            
    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask):

        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.trackProgress(currentSubTask)


class ClippIntoFixture(Task):
    def __init__(self, targetFixture, previousFixture, cableSlack):
        super(ClippIntoFixture, self).__init__('combined')    
        overFixture = subTasks.OverFixture(np.array([0,0]), 0.16, 0.05)
        lowerOverFixture = subTasks.OverFixture(np.array([0,0]), 0.16, -0.02)
        #clippIntoFixture = subTasks.ClipIntoFixture()
        openGrippers = subTasks.HoldPosition(3, np.array([20,20]))
        goToHeight = subTasks.GoToHeight(np.array([0.10,0.10]), np.array([20,20])) 

        self.subTasks = [overFixture, lowerOverFixture, openGrippers, goToHeight]
        self.goToSubTask = [0,0,0,0]
        self.numSubTasks = len(self.subTasks)

        self.targetFixture = targetFixture # int, starting from 0, which fixture is the target,
            # will be matched with map elements.
        self.previousFixture = previousFixture # # int, starting from 0 (-1 means no previous fixture),
            # which fixture is the current fixture the cable is attached to.
        self.cableSlack = cableSlack # float in meters, if no previous it defines how much the cable 
            # should stick out from the fixture, and if previous it defines how much longer the cable should
            # be then the closest distance between the current and target fixture.  

    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask):

        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.trackProgress(currentSubTask)

 



