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
        self.lastSubtask = -1
        self.numSubTasks = 0
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.trajectory = []
        self.targetFixture = 0
        self.previousFixture = -1
        self.cableSlack = 0
        self.nextTaskStep = 1 # Step for next task in roadmap, 1 = next task, -1 gives previous task

    def getNewTrajectory(self):
        return self.newTrajectory

    def getNextTaskStep(self):
        return self.nextTaskStep

    def setNextTaskStep(self, nextTaskStep):
        self.nextTaskStep = nextTaskStep

    def getInfo(self):
        return self.trajectory, self.mode, self.targetFixture, self.previousFixture, self.cableSlack, self.grippWidth

    def getMsg(self):
        # calculate trajectory message. 
        self.newTrajectory = 0
        msg = Trajectory_msg()
        msg.header.stamp = rospy.Time.now()
        msg.mode = self.mode

        self.trajectory = []

        # temporal gripper positions, used by the subtasks. 
        if self.mode == 'coordinated':
            absolutePosition, absoluteOrientation, relativePosition, relativeOrientation = \
                        utils.calcAbsoluteAndRelative(self.gripperRight, self.gripperLeft, self.transformer)
            absoluteTemp = self.gripperRight.copyClass()
            relativeTemp = self.gripperLeft.copyClass()
            absoluteTemp.update(absolutePosition, absoluteOrientation)
            relativeTemp.update(relativePosition, relativeOrientation)
            absoluteTemp.flipped = -1
             
            minDist, lengthRight, minIndex = utils.closesPointDLO(self.DLO, self.gripperRight.getPosition())
            minDist, lengthLeft, minIndex = utils.closesPointDLO(self.DLO, self.gripperLeft.getPosition())
            if lengthRight > lengthLeft:
                absoluteTemp.flipped = 1
            
        else:
            gripperRightTemp = self.gripperRight.copyClass()
            gripperLeftTemp = self.gripperLeft.copyClass()

        # get the inputs to the current sub task 
        for i in range(self.startSubTaskIdx, len(self.subTasks)):

            inputArgs = self.subTasks[i].inputArgs
            input_ = []
            for j in range(len(inputArgs)):
                input_.append(eval(inputArgs[j]))

            # Calculate the trajectory point    
            trajectoryPoint = self.subTasks[i].getTrajectoryPoint(input_)
            # addpend to list to create the trajectory parameters. 
            self.trajectory.extend(trajectoryPoint)

        msg.trajectory = self.trajectory
        return msg

    def trackProgress(self, currentSubTask):
        # gets called continiusly and monitors progress. 
        # currentSubTask is recived from the controller. 
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

        verified = self.subTasks[currentSubTask].verification(input_)  
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
        self.lastSubtask = -1
        self.trajectory = []


class GrabCable(Task):
    # This task is responsible for picking up the DLO 
    def __init__(self, targetFixture, previousFixture, cableSlack, grippWidth=0.15):
        super(GrabCable, self).__init__('individual')   
        goToHeight = subTasks.GoToHeightIndividual(np.array([0.10,0.10]), np.array([20,20])) 
        overCable = subTasks.OverCableIndividual(np.array([0.10,0.10]), np.array([20,20]), 0.15)
        onCable = subTasks.OverCableIndividual(np.array([-0.012,-0.012]), np.array([20,20]), 0.15)
        grippCable = subTasks.HoldPositionIndividual(3, np.array([0,0]))
        goToHeightWithCable = subTasks.GoToHeightWithCableIndividual(np.array([0.1,0.1]), np.array([0,0])) 

        self.subTasks = [goToHeight, overCable, onCable, grippCable, goToHeightWithCable]
        self.goToSubTask = [0,1,1,1,0]
        self.numSubTasks = len(self.subTasks)

        self.targetFixture = targetFixture # int, starting from 0, which fixture is the target,
            # will be matched with map elements.
        self.previousFixture = previousFixture # # int, starting from -1 (-1 means no previous fixture),
            # which fixture is the current fixture the cable is attached to.
        self.cableSlack = cableSlack # float in meters, if no previous it defines how much the cable 
            # should stick out from the fixture, and if previous it defines how much longer the cable should
            # be then the closest distance between the current and target fixture.  
        self.grippWidth = grippWidth
   
    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask, jointPosition):

        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.jointPosition = jointPosition
        self.trackProgress(currentSubTask)


class ClippIntoFixture(Task):
    # This task is responsible for clipping the DLO into a fixture. 
    def __init__(self, targetFixture, previousFixture, cableSlack, grippWidth=0.15):
        super(ClippIntoFixture, self).__init__('coordinated')    
        overFixture = subTasks.OverFixtureCombinded(np.array([0,0]), grippWidth+0.003, 0.05)
        lowerOverFixture = subTasks.OverFixtureCombinded(np.array([0,0]), grippWidth+0.003, -0.033)
        openGrippers = subTasks.HoldPositionCombined(3, np.array([20,20]))
        goToHeight = subTasks.GoToHeightCombined(np.array([0.10,0.10]), np.array([20,20])) 
        verifyClipped = subTasks.VerifyClippedFixture(time_=1, grippers=np.array([20,20]), MaxDLODist=0.04)

        self.subTasks = [overFixture, lowerOverFixture, openGrippers, goToHeight, verifyClipped]
        self.goToSubTask = [0,0,0,0,0]
        self.numSubTasks = len(self.subTasks)

        self.targetFixture = targetFixture # int, starting from 0, which fixture is the target,
            # will be matched with map elements.
        self.previousFixture = previousFixture # # int, starting from 0 (-1 means no previous fixture),
            # which fixture is the current fixture the cable is attached to.
        self.cableSlack = cableSlack # float in meters, if no previous it defines how much the cable 
            # should stick out from the fixture, and if previous it defines how much longer the cable should
            # be then the closest distance between the current and target fixture.  
        self.grippWidth = grippWidth


    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask, jointPosition):

        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.jointPosition = jointPosition
        self.trackProgress(currentSubTask)

 
class Rerouting(Task):
    # This task is responible for converting any solution from the genetic algorithim to trajectory paramters. 
    def __init__(self): 
        super(Rerouting, self).__init__('individual')   
    
    def initilize(self, targetFixture, previousFixture, mode, individual, grippWidth):
        self.mode = mode
        self.grippWidth = grippWidth

        if self.mode == 'individual':
            self.nextTaskStep = 0
            goToHeight = subTasks.GoToHeightIndividual(np.array([0.10,0.10]), np.array([20,20])) 
            overCable = subTasks.CableReroutingOverIndividual(individual, np.array([0.10,0.10]), np.array([20,20]))
            onCable = subTasks.CableReroutingOverIndividual(individual, np.array([-0.018,-0.018]), np.array([20,20]))
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

        elif self.mode == 'coordinated':
            self.nextTaskStep = -1
            
            absPosXY = individual.parametersCombined[0:2]
            absRotZ = individual.parametersCombined[2]
            moveTo = subTasks.ReroutingCombined(absPosXY, absRotZ, 0.10, np.array([0,0]), self.grippWidth)
            moveDown = subTasks.GoToHeightCombined(np.array([0.03,0.03]), np.array([0,0])) 
            releaseCable = subTasks.HoldPositionCombined(3, np.array([20,20]))
            goToHeight = subTasks.GoToHeightCombined(np.array([0.10,0.10]), np.array([20,20])) 
            absRotZ = 0 - 0.6*absRotZ
            rotateToNetural = subTasks.ReroutingCombined(absPosXY, absRotZ, 0.10, np.array([20,20]), self.grippWidth)
            self.subTasks = [moveTo, moveDown, releaseCable, goToHeight, rotateToNetural]
            self.goToSubTask = [0,0,0,0,0]
            self.numSubTasks = len(self.subTasks)

       

    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask, jointPosition):
        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.jointPosition = jointPosition
        self.trackProgress(currentSubTask)


class HoldPosition(Task):
    # this task justs holds position. 
    def __init__(self, targetFixture=0, previousFixture=-1, cableSlack=0.05, grippWidth=0.15):
        super(HoldPosition, self).__init__('individual')   
        holdPosition = subTasks.HoldPositionIndividual(3, np.array([0,0]))

        self.subTasks = [holdPosition]
        self.goToSubTask = [0]
        self.numSubTasks = len(self.subTasks)

        self.targetFixture = targetFixture # int, starting from 0, which fixture is the target,
            # will be matched with map elements.
        self.previousFixture = previousFixture # # int, starting from -1 (-1 means no previous fixture),
            # which fixture is the current fixture the cable is attached to.
        self.cableSlack = cableSlack # float in meters, if no previous it defines how much the cable 
            # should stick out from the fixture, and if previous it defines how much longer the cable should
            # be then the closest distance between the current and target fixture.  
        self.grippWidth = grippWidth
        self.nextTaskStep = 0

   
    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask, jointPosition):

        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.jointPosition = jointPosition
        #self.trackProgress(currentSubTask)
        
class ResetOrientation(Task):
    # reseteds the orientaiton of the grippers. 
    def __init__(self, targetFixture=0, previousFixture=-1, cableSlack=0.05, grippWidth=0.15):
        super(ResetOrientation, self).__init__('individual')   
        resetOrientaiton = subTasks.GoToHeightIndividualResetOrientatin(np.array([0.1,0.1]), np.array([20,20]))

        self.subTasks = [resetOrientaiton, resetOrientaiton]
        self.goToSubTask = [0, 0]
        self.numSubTasks = len(self.subTasks)

        self.targetFixture = targetFixture # int, starting from 0, which fixture is the target,
            # will be matched with map elements.
        self.previousFixture = previousFixture # # int, starting from -1 (-1 means no previous fixture),
            # which fixture is the current fixture the cable is attached to.
        self.cableSlack = cableSlack # float in meters, if no previous it defines how much the cable 
            # should stick out from the fixture, and if previous it defines how much longer the cable should
            # be then the closest distance between the current and target fixture.  
        self.grippWidth = grippWidth
        self.nextTaskStep = 0

   
    def updateAndTrackProgress(self, map_, DLO, gripperLeft, gripperRight, currentSubTask, jointPosition):

        self.map = map_ 
        self.DLO = DLO
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.jointPosition = jointPosition
        self.trackProgress(currentSubTask)
        