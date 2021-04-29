#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int64
import tf
import numpy as np
import utils, constraintsCheck, solveRerouting
import tasks
import threading


class PathPlanner(object):
    def __init__(self, listOfObjects, listOfTasks):

        # initial enviorment and planned tasks
        self.map = listOfObjects
        self.tasks = listOfTasks
        self.numOfTasks = len(self.tasks)
        # keep track of robot 
        self.tfListener = tf.TransformListener()        
        self.gripperRight = utils.FramePose()
        self.gripperLeft = utils.FramePose() 
        # rope representation 
        self.DLO = utils.DLO()
        # keep track of state
        self.currentTask = 0 
        self.currentSubTask = 1
        self.instruction = 0 # 0 = normal, 1 = problem detected, but solvable, 2 = non recoverable problem
        # detect and solve problems
        self.solve = solveRerouting.Solve()
        self.rerouting = tasks.Rerouting()
        # mutex
        self.mtx_spr = threading.Lock()
        self.mtx_subTask = threading.Lock()
        # ros publisher for trajectory 
        self.pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)

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
        self.DLO.update(points=DLOPoints)
        self.mtx_spr.release()

    def callbackCurrentSubTask(self, data):
        # update about current subtask from controller, threadsafe
        self.mtx_subTask.acquire()
        self.currentSubTask = data.data
        self.mtx_subTask.release()

    def genetateNewTrajectory(self, task):
        # alway tries to generate new trajectory in normal state
        print('new traj = ', self.currentTask)                 

        task = self.tasks[self.currentTask]
        task.updateAndTrackProgress(map_=self.map,\
                                    DLO=self.DLO,\
                                    gripperLeft=self.gripperLeft,\
                                    gripperRight=self.gripperRight,\
                                    currentSubTask=self.currentSubTask)
        # generates message
        msg = task.getMsg() 
        # Check for imposible solutions
        self.instruction = constraintsCheck.check(task=task)
        print('instruction = ', self.instruction)                 
        # if problem detected
        if self.instruction == 1:
            print('instruction 1')
            nonValidSolution = 1
            numAttempts = 0 # keep track of how many attempts of findin valid solution
            
            while nonValidSolution != 0: 
                print('instruction 1, while')

                numAttempts += 1
                if numAttempts > 3:
                    print('Number of attempts exceded 3')
                    self.instruction = 2
                    break
                # setup solver
                self.solve.updateInit(task=task)
                # solve optim problem, evolution based stochastic solver. 
                individual = self.solve.solve(populationSize=100, numGenerations=20)
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
                                            currentSubTask=self.currentSubTask)               
                # replace message from new task with solution
                msg = task_.getMsg()
                # Check for imposible solutions
                nonValidSolution = constraintsCheck.check(task=task_)
            task = task_

        if self.instruction == 2:
            print('Non recoverable problem detected in path planning!')
            return
            
        self.pub.publish(msg)

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
        else:
            # unlock mutex 
            self.mtx_subTask.release()
            self.mtx_spr.release()
            return

        # if a new trajectory is called for
        if task.getNewTrajectory() == 1:
            self.genetateNewTrajectory(task=task)
        # update task variables and call for veriication on subtasks
        task.updateAndTrackProgress(map_=self.map,\
                                    DLO=self.DLO,\
                                    gripperLeft=self.gripperLeft,\
                                    gripperRight=self.gripperRight,\
                                    currentSubTask=self.currentSubTask)

        # if a task is finished 
        if task.getTaskDone() == 1:
            print('task done')
            # if normal, then next task is choosen
            if self.instruction == 0:
                # if at end of task list
                if self.currentTask < self.numOfTasks-1:
                    self.currentTask += task.getNextTaskStep()
                    self.tasks[self.currentTask].resetTask() 
            # if problem task is solved then same or prevous task is selected
            elif self.instruction == 1:
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
                                        fixtureRadius=0.06)
            listOfObjects.extend([obj])
        except:
            break

    # tasks -------------------
    # how much cable slack there should be between current and previous fixture
    slackList = [0.15, 0.04, 0.04, 0.04]
    listOfTasks  = []
    for i in range(len(listOfObjects)):
        grabCable = tasks.GrabCable(targetFixture=i, previousFixture=i-1, cableSlack=slackList[i])
        clippIntoFixture = tasks.ClippIntoFixture(targetFixture=i, previousFixture=i-1, cableSlack=slackList[i])
        listOfTasks.extend([grabCable, clippIntoFixture])

    pathPlanner = PathPlanner(listOfObjects=listOfObjects, listOfTasks=listOfTasks)

    # Ros subscribers 
    rospy.Subscriber("/spr/dlo_estimation", PointCloud, pathPlanner.callback, queue_size=2)
    rospy.Subscriber("/controller/sub_task", Int64, pathPlanner.callbackCurrentSubTask, queue_size=2)

    # sleep for everthing to initilize
    rospy.sleep(0.45)

    # rate at which the pathplanner checks and validated in Hz
    rate = rospy.Rate(10)

    # main loop
    while not rospy.is_shutdown():
            
        pathPlanner.update()

        rate.sleep()


if __name__ == '__main__':
    main()

