#!/usr/bin/env python3

import numpy as np
import rospy
import subTasks
from controller.msg import Trajectory_point, Trajectory_msg

'''
class Task(object):
    # base class for all tasks
    def __init__(self, mode):
        self.newMessage = 1
        self.taskDone = 0
        self.subTasks = []
        self.currentSubTask = 0
        self.mode = mode

    def update(map, DLO, gripperLeft, gripperRight):
        if self.currentSubTask > len[self.subTasks]:
            self.taskDone = 1
            return

        inputArg = self.subTasks[self.currentSubTask].inputArg
        input_ = []
        
        for i in range(len(inputArg)):
            input_.append(eval(inputArg[i]))

        self.subTasks[self.currentSubTask].update(input_)

        self.currentSubTask += self.subTasks[self.currentSubTask].taskState()

        self.newMessage = self.subTasks[self.currentSubTask].newMessage()
         
    def newMessage():
        return self.newMessage

    def getMessage():
        self.newMessage = 0
        msg = Trajectory_msg()
        msg.header.stamp = rospy.Time.now()
        msg.mode = self.mode
        msg.forceControl = 0 # hardcoded for now
        msg.maxForce = 4.0
        trajectory = []
        for i in range(self.currentSubTask, len[self.subTasks]):
            trajectory.extend(self.subTasks[i].getTrajectory())
        msg.trajectory = trajectory
        return msg

    def taskDone():
        return self.taskDone
'''

class Task(object):
    # base class for all tasks
    def __init__(self, mode):
        self.newTrajectory = 1
        self.taskDone = 0
        self.subTasks = []
        self.startSubTaskIdx = 0
        self.mode = mode

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
        for i in range(self.startSubTaskIdx, len(self.subTasks)):
            inputArgs = self.subTasks[i].inputArgs
            input_ = []
            for j in range(len(inputArgs)):
                input_.append(eval(inputArgs[j]))
            trajectory.extend(self.subTasks[i].getTrajectoryPoint(input_))

        msg.trajectory = trajectory
        return msg

    def getTaskDone(self):
        return self.taskDone

class GrabCable(Task):
    def __init__(self, targetFixture, previousFixture, cableSlack):
        super(GrabCable, self).__init__('individual')   
        goToHeight = subTasks.GoToHeight(np.array([0.2,0.2]), np.array([0,0])) 
        self.subTasks = [goToHeight]
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


 



