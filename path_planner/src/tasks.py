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
        self.time = rospy.Time.now()
        self.lastSubtask = 0
        self.numSubTasks = 0
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.trajectory = []
        self.targetFixture = 0
        self.previousFixture = -1
        self.cableSlack = 0
        self.nextTaskStep = 1 # when task don how to progress

    def getNewTrajectory(self):
        return self.newTrajectory

    def getNextTaskStep(self):
        return self.nextTaskStep

    def getInfo(self):
        return self.trajectory, self.mode, self.targetFixture, self.previousFixture, self.cableSlack, self.grippWidth

    def getMsg(self):
        self.newTrajectory = 0
        msg = Trajectory_msg()
        msg.header.stamp = rospy.Time.now()
        msg.mode = self.mode
        msg.forceControl = 0 # hardcoded for now
        msg.maxForce = 4.0
        self.trajectory = []



        if self.mode == 'combined':
            absolutePosition, absoluteOrientation, relativePosition, relativeOrientation = \
                        utils.calcAbsoluteAndRelative(self.gripperRight, self.gripperLeft, self.transformer)
            absoluteTemp = self.gripperRight.copyClass()
            relativeTemp = self.gripperLeft.copyClass()
            absoluteTemp.update(absolutePosition, absoluteOrientation)
            relativeTemp.update(relativePosition, relativeOrientation)
            absoluteTemp.flipped = -1
        else:
            gripperRightTemp = self.gripperRight.copyClass()
            gripperLeftTemp = self.gripperLeft.copyClass()

        for i in range(self.startSubTaskIdx, len(self.subTasks)):

            inputArgs = self.subTasks[i].inputArgs
            input_ = []
            for j in range(len(inputArgs)):
                input_.append(eval(inputArgs[j]))
                    
            trajectoryPoint = self.subTasks[i].getTrajectoryPoint(input_)

            self.trajectory.extend(trajectoryPoint)

        

        msg.trajectory = self.trajectory
        return msg

    def trackProgress(self, currentSubTask):
        # change this code 

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

    def resetTask(self):
        self.newTrajectory = 1
        self.newTrajectoryCheck = 1
        self.taskDone = 0
        self.startSubTaskIdx = 0
        self.time = rospy.Time.now()
        self.lastSubtask = 0
        self.trajectory = []


class GrabCable(Task):
    def __init__(self, targetFixture, previousFixture, cableSlack, grippWidth=0.15):
        super(GrabCable, self).__init__('individual')   
        goToHeight = subTasks.GoToHeightIndividual(np.array([0.10,0.10]), np.array([20,20])) 
        overCable = subTasks.OverCableIndividual(np.array([0.10,0.10]), np.array([20,20]), 0.15)
        onCable = subTasks.OverCableIndividual(np.array([0.01,-0.005]), np.array([20,20]), 0.15)
        grippCable = subTasks.HoldPositionIndividual(3, np.array([0,0]))
        goToHeightWithCable = subTasks.GoToHeightWithCableIndividual(np.array([0.1,0.1]), np.array([0,0])) 

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
        self.grippWidth = grippWidth
   
    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask):

        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.trackProgress(currentSubTask)


class ClippIntoFixture(Task):
    def __init__(self, targetFixture, previousFixture, cableSlack, grippWidth=0.15):
        super(ClippIntoFixture, self).__init__('combined')    
        overFixture = subTasks.OverFixtureCombinded(np.array([0,0]), 0.16, 0.05)
        lowerOverFixture = subTasks.OverFixtureCombinded(np.array([0,0]), 0.16, -0.02)
        #clippIntoFixture = subTasks.ClipIntoFixture()
        openGrippers = subTasks.HoldPositionCombined(3, np.array([20,20]))
        goToHeight = subTasks.GoToHeightCombined(np.array([0.10,0.10]), np.array([20,20])) 

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
        self.grippWidth = grippWidth


    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask):

        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.trackProgress(currentSubTask)

 
class Rerouting(Task):
    def __init__(self): 
        super(Rerouting, self).__init__('individual')   
    
    def initilize(self, mode, individual):
        self.mode = mode

        if self.mode == 'individual':
            self.nextTaskStep = 0
            goToHeight = subTasks.GoToHeightIndividual(np.array([0.10,0.10]), np.array([20,20])) 
            overCable = subTasks.CableReroutingOverIndividual(individual, np.array([0.10,0.10]), np.array([20,20]))
            onCable = subTasks.CableReroutingOverIndividual(individual, np.array([0.0,0.0]), np.array([20,20]))
            gotToEndPosition = subTasks.CableReroutingEndPosIndividual(individual, np.array([0.10,0.10]), np.array([0,0]))

            if individual.pickupRightValid == 1 and individual.pickupLeftValid == 1:
                goDown = subTasks.GoToHeightIndividual(np.array([0.03,0.03]), np.array([0,0]))
                grippCable = subTasks.HoldPositionIndividual(3, np.array([0,0]))
                goToHeightWithCable = subTasks.GoToHeightIndividual(np.array([0.10,0.10]), np.array([0,0])) 

            elif individual.pickupRightValid == 1:
                goDown = subTasks.GoToHeightIndividual(np.array([0.03,0.1]), np.array([0,20]))
                grippCable = subTasks.HoldPositionIndividual(3, np.array([0,20])) 
                goToHeightWithCable = subTasks.GoToHeightIndividual(np.array([0.10,0.10]), np.array([0,20])) 

            elif individual.pickupLeftValid == 1:
                goDown = subTasks.GoToHeightIndividual(np.array([0.1,0.03]), np.array([20,0]))
                grippCable = subTasks.HoldPositionIndividual(3, np.array([20,0])) 
                goToHeightWithCable = subTasks.GoToHeightIndividual(np.array([0.10,0.10]), np.array([20,0])) 

            else:
                goDown = subTasks.GoToHeightIndividual(np.array([0.10,0.10]), np.array([20,20]))  
                grippCable = subTasks.HoldPositionIndividual(3, np.array([20,20]))
                goToHeightWithCable = subTasks.GoToHeightIndividual(np.array([0.10,0.10]), np.array([20,20])) 

            releaseCable = subTasks.HoldPositionIndividual(3, np.array([20,20]))
 
            self.subTasks = [goToHeight, overCable, onCable, grippCable, goToHeightWithCable, gotToEndPosition, goDown, releaseCable, goToHeight]
            self.goToSubTask = [0,1,1,1,1,1,1]
            self.numSubTasks = len(self.subTasks)

        elif self.mode == 'combined':
            self.nextTaskStep -= 1
            pass
       

    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask):
        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.trackProgress(currentSubTask)

