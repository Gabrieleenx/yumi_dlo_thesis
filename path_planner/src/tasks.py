#!/usr/bin/env python3

import numpy as np
import rospy
import subTasks

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


class GrabCable(Task):
     def __init__(self):
        super(ClippIntoFixture, self).__init__('individual')    
        self.subTasks = []

class ClippIntoFixture(Task):
     def __init__(self):
        super(ClippIntoFixture, self).__init__('combined')    
        






