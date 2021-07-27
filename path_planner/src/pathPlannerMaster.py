#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg, Jacobian_msg
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int64
import tf
import numpy as np
import utils, constraintsCheck, solveRerouting, dataLogger
import tasks
import threading

class PathPlanner(object):
    def __init__(self, listOfObjects, listOfTasks, logger):

        # initial enviorment and planned tasks
        self.map = listOfObjects
        self.tasks = listOfTasks
        self.numOfTasks = len(self.tasks)
        # keep track of robot 
        self.tfListener = tf.TransformListener()        
        self.gripperRight = utils.FramePose()
        self.gripperLeft = utils.FramePose() 
        self.jointPosition = np.zeros(14)
        # rope representation 
        self.DLO = utils.DLO()
        # keep track of state
        self.currentTask = 0 
        self.currentSubTask = 1
        self.instruction = 0 # 0 = normal, 1 = problem detected, but solvable, 2 = non recoverable problem
        # detect and solve problems
        self.solve = solveRerouting.Solve()
        self.rerouting = tasks.Rerouting()
        self.holdPosition = tasks.HoldPosition()
        self.resetOrientation = tasks.ResetOrientation()
        # mutex
        self.mtx_spr = threading.Lock()
        self.mtx_subTask = threading.Lock()
        self.mtx_jointPos = threading.Lock()
        # ros publisher for trajectory 
        self.pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
        # logger
        self.logger = logger

    def callback(self, data):
        # update DLO estimation
        numOfPoints = len(data.points)
        # placeholder for storing variables 
        DLOPoints = np.zeros((numOfPoints,3)) 
        # fill in placeholder
        for i in range(numOfPoints):
            DLOPoints[i,0] = data.points[i].x
            DLOPoints[i,1] = data.points[i].y
            DLOPoints[i,2] = data.points[i].z
        # update DLO threadsafe
        self.mtx_spr.acquire()
        self.logger.appendDLOPoints(data=DLOPoints)
        self.DLO.update(points=DLOPoints)
        self.mtx_spr.release()

    def callbackCurrentSubTask(self, data):
        # update about current subtask from controller, threadsafe
        self.mtx_subTask.acquire()
        self.currentSubTask = data.data
        self.mtx_subTask.release()

    def callbackJointPosition(self, data):
        self.mtx_jointPos.acquire()
        self.jointPosition = np.asarray(data.jointPosition)[0:14]
        self.mtx_jointPos.release()

    def genetateNewTrajectory(self, task):
        # alway tries to generate new trajectory in normal state
        print('Generate new trajectory = ', self.currentTask)                 
        self.logger.appendPathplannerState(data='Generate new trajectory, task = ' + str(self.currentTask))

        task = self.tasks[self.currentTask]
        task.updateAndTrackProgress(map_=self.map,\
                                    DLO=self.DLO,\
                                    gripperLeft=self.gripperLeft,\
                                    gripperRight=self.gripperRight,\
                                    currentSubTask=self.currentSubTask,\
                                    jointPosition=self.jointPosition)
        # generates message
        msg = task.getMsg() 
        # Check for imposible solutions
        self.instruction = constraintsCheck.check(task=task, logger=self.logger)
        
        self.instruction = 0

        # skip solver for test, to be removed 
        #if self.instruction == 1:
        #    self.instruction = 2

        # if problem detected
        if self.instruction == 1:
            self.logger.appendPathplannerState(data='Hold position sent')
            self.holdPosition.resetTask()
            self.holdPosition.updateAndTrackProgress(map_=self.map,\
                                            DLO=self.DLO,\
                                            gripperLeft=self.gripperLeft,\
                                            gripperRight=self.gripperRight,\
                                            currentSubTask=self.currentSubTask,\
                                            jointPosition=self.jointPosition)   
            msg = self.holdPosition.getMsg()
            self.pub.publish(msg)
            # logger
            self.logger.appendMsgSent(data=msg)
            nonValidSolution = 1
            numAttempts = 0 # keep track of how many attempts of findin valid solution
            
            while nonValidSolution == 1: 
                print('Searching for solution')
                numAttempts += 1
                if numAttempts > 3:
                    print('Number of attempts exceded 3')
                    print('Non recoverable problem detected in path planning!')
                    self.instruction = 2
                    self.logger.appendPathplannerState(data='Number of attempts exceded, non solvable state')
                    return

                self.logger.appendPathplannerState(data='Searching for solution, attempt = '+ str(numAttempts))
                # setup solver
                self.solve.updateInit(task=task)
                # solve optim problem, evolution based stochastic solver. 
                individual = self.solve.solve(populationSize=100, numGenerations=20)
                self.logger.appendSolutionIndividual(data=individual)
                if not ((individual.pickupRightValid or individual.pickupLeftValid) and individual.combinedValid):
                    print('Non valid solution')
                    self.logger.appendPathplannerState(data='Non valid solution, from evaluation')
                    continue

                # setup new task 
                self.rerouting.resetTask()
                self.rerouting.initilize(targetFixture=task.targetFixture,\
                                        previousFixture=task.previousFixture,\
                                        mode=task.mode,\
                                        individual=individual,\
                                        grippWidth=task.grippWidth)
                task_ = self.rerouting
                task_.updateAndTrackProgress(map_=self.map,\
                                            DLO=self.DLO,\
                                            gripperLeft=self.gripperLeft,\
                                            gripperRight=self.gripperRight,\
                                            currentSubTask=self.currentSubTask,\
                                            jointPosition=self.jointPosition)   
            
                # replace message from new task with solution
                msg = task_.getMsg()
                # Check for imposible solutions
                nonValidSolution = constraintsCheck.check(task=task_, logger=self.logger)
                if nonValidSolution == 3:
                    self.instruction = 3
            task = task_

        if self.instruction == 2:
            print('Non recoverable problem detected in path planning!')
            self.holdPosition.resetTask()
            self.holdPosition.updateAndTrackProgress(map_=self.map,\
                                            DLO=self.DLO,\
                                            gripperLeft=self.gripperLeft,\
                                            gripperRight=self.gripperRight,\
                                            currentSubTask=self.currentSubTask,\
                                            jointPosition=self.jointPosition)   
            msg = self.holdPosition.getMsg()
            self.pub.publish(msg)
            # logger
            self.logger.appendMsgSent(data=msg)
            return

        if self.instruction == 3:
            # reset orientation 
            print('reseting orientation')
            self.logger.appendPathplannerState(data='Reset orientation')
            self.resetOrientation.resetTask()     
            self.resetOrientation.updateAndTrackProgress(map_=self.map,\
                                            DLO=self.DLO,\
                                            gripperLeft=self.gripperLeft,\
                                            gripperRight=self.gripperRight,\
                                            currentSubTask=self.currentSubTask,\
                                            jointPosition=self.jointPosition)   
            msg = self.resetOrientation.getMsg()
        self.pub.publish(msg)
        self.logger.appendMsgSent(data=msg)

    def update(self):
        # Main function beeing called in a loop, generates trajcectories and checks execution 
        # if no infomation about the DLO has been recived 
        if self.DLO.pointsRecived == 0:
            print('No DLO points have been recived')
            return

        # updates to position of grippers  
        (posRight, orientationRight) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
        (posLeft, orientationLeft) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))
        self.gripperRight.update(position=posRight, orientation=orientationRight)
        self.gripperLeft.update(position=posLeft, orientation=orientationLeft)
        
        
        # threadsafe for DLO and subtasks
        self.mtx_spr.acquire()
        self.mtx_subTask.acquire()
        
        # if normal task or problem solving task is used 
        if self.instruction == 0:
            task = self.tasks[self.currentTask]
        elif self.instruction == 1:
            task = self.rerouting
        elif self.instruction == 3:
            task = self.resetOrientation
        else:
            # unlock mutex 
            self.mtx_subTask.release()
            self.mtx_spr.release()
            return

        self.mtx_jointPos.acquire()

        #logger
        self.logger.appendGripperPose(data=[posRight, orientationRight, posLeft, orientationLeft])
        self.logger.appendJointPosition(data=np.copy(self.jointPosition))
        self.logger.appendControllerState(data=self.currentSubTask)
        self.logger.appendCurrentTask(data=self.currentTask)
        
        # if a new trajectory is called for
        if task.getNewTrajectory() == 1:
            self.genetateNewTrajectory(task=task)

        # update task variables and call for veriication on subtasks
                
        task.updateAndTrackProgress(map_=self.map,\
                                    DLO=self.DLO,\
                                    gripperLeft=self.gripperLeft,\
                                    gripperRight=self.gripperRight,\
                                    currentSubTask=self.currentSubTask,\
                                    jointPosition=self.jointPosition)
        self.mtx_jointPos.release()

        if task.getNewTrajectory() == 1:
            self.logger.appendPathplannerState(data='Problem during execution detected')

        # if a task is finished 
        if task.getTaskDone() == 1:
            print('task done')
            self.logger.appendPathplannerState(data='Task Done ')

            # if normal, then next task is choosen
            if self.instruction == 0:
                # if at end of task list
                if self.currentTask < self.numOfTasks-1:
                    self.currentTask += task.getNextTaskStep()
                    self.tasks[self.currentTask].resetTask() 
                else:
                    self.logger.appendPathplannerState(data='All tasks completed')

            # if problem task is solved then same or prevous task is selected
            elif self.instruction == 1 or self.instruction == 3:
                # return to normal task
                self.instruction = 0 
                self.currentTask += task.getNextTaskStep()
                task = self.tasks[self.currentTask]
                self.tasks[self.currentTask].resetTask() 
            print('currentTask ', self.currentTask)

        # unlock mutex 
        self.mtx_subTask.release()
        self.mtx_spr.release()


    
def main():
    # initilize ros node
    rospy.init_node('pathPlanner', anonymous=True) 
    tfListener = tf.TransformListener()
    # sleep for listener to have time to pick up the transformes 
    rospy.sleep(0.5)
    
    # Logger
    logger = dataLogger.Logger()

    # add existing fixtures in list
    fixtureList = ['/Fixture1','/Fixture2','/Fixture3','/Fixture4']
    listOfObjects = []
    for i in range(len(fixtureList)):
        try:
            (pos, rot) = tfListener.lookupTransform('/yumi_base_link', fixtureList[i], rospy.Time(0))
            pos = np.asarray(pos)
            rot = np.asarray(rot)
            obj = utils.FixtureObject(position=pos,\
                                        orientation=rot,\
                                        fixtureHeight=0.06,\
                                        fixtureRadius=0.065)
            listOfObjects.extend([obj])
            logger.appendPathplannerState(data='Adding Fixture ' + str(i))
            logger.appendFixturesObj(data=obj)

        except:
            break

    # tasks -------------------
    # how much cable slack there should be between current and previous fixture
    slackList = [0.11, 0.04, 0.04, 0.04]
    listOfTasks  = []
    for i in range(len(listOfObjects)):
        grabCable = tasks.GrabCable(targetFixture=i, previousFixture=i-1, cableSlack=slackList[i])
        clippIntoFixture = tasks.ClippIntoFixture(targetFixture=i, previousFixture=i-1, cableSlack=slackList[i])
        listOfTasks.extend([grabCable, clippIntoFixture])
        logger.appendPathplannerState(data='Adding tasks, grabCable')
        logger.appendPathplannerState(data='Adding tasks, clippIntoFixture')

    pathPlanner = PathPlanner(listOfObjects=listOfObjects, listOfTasks=listOfTasks, logger=logger)

    # Ros subscribers 
    rospy.Subscriber("/spr/dlo_estimation", PointCloud, pathPlanner.callback, queue_size=2)
    rospy.Subscriber("/controller/sub_task", Int64, pathPlanner.callbackCurrentSubTask, queue_size=2)
    rospy.Subscriber("/Jacobian_R_L", Jacobian_msg, pathPlanner.callbackJointPosition, queue_size=2)

    # sleep for everthing to initilize
    rospy.sleep(0.45)

    # rate at which the pathplanner checks and validated in Hz
    rate = rospy.Rate(10)

    # main loop
    while not rospy.is_shutdown():
            
        pathPlanner.update()

        rate.sleep()

    print('Shuting down')
    # save data
    dataLogger.saveObjectPickle(fileName='test1.obj', object_=logger)
    print('Done!')

if __name__ == '__main__':
    main()

