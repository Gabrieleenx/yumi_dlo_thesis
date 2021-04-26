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

# TODO 
# 1 improve traj generation
# 2 fix bugs
# 3 detect problems


class PathPlanner(object):
    def __init__(self, listOfObjects, listOfTasks):
        self.pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
        self.map = listOfObjects
        self.tasks = listOfTasks
        self.numOfTasks = len(self.tasks)
        self.currentTask = 0
        self.DLOPoints = np.zeros((100,3)) # matrix with 3d points 
        self.DLO = utils.DLO()
        self.tfListener = tf.TransformListener()
        self.gripperRight = utils.FramePose()
        self.gripperLeft = utils.FramePose()
        self.currentSubTask = 1
        self.instruction = 0
        self.mtx_spr = threading.Lock()
        self.mtx_subTask = threading.Lock()
        self.checkConstraints = constraintsCheck.CheckConstraints()
        self.solve = solveRerouting.Solve()
        self.rerouting = tasks.Rerouting()


    def callback(self, data):
        # update self.DLOEstiamtion
        numOfPoints = len(data.points)
        self.DLOPoints = np.zeros((numOfPoints,3)) # matrix with 3d points 

        self.DLOEstimation = np.zeros((numOfPoints,3))
        for i in range(numOfPoints):
            self.DLOPoints[i,0] = data.points[i].x
            self.DLOPoints[i,1] = data.points[i].y
            self.DLOPoints[i,2] = data.points[i].z

        self.mtx_spr.acquire()
        self.DLO.update(self.DLOPoints)
        self.mtx_spr.release()

    def callbackCurrentSubTask(self, data):
        self.mtx_subTask.acquire()
        self.currentSubTask = data.data
        self.mtx_subTask.release()

    def update(self):
        # update pose 
        if self.DLO.pointsRecived == 0:
            print('No DLO points have been recived')
            return

        (posRight, orientationRight) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
        (posLeft, orientationLeft) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))
        self.gripperRight.update(posRight, orientationRight)
        self.gripperLeft.update(posLeft, orientationLeft)
        self.mtx_spr.acquire()
        self.mtx_subTask.acquire()

        if self.instruction == 0:
            task = self.tasks[self.currentTask]
        elif self.instruction == 1:
            task = self.rerouting


        if task.getNewTrajectory() == 1:
            task = self.tasks[self.currentTask]
            task.updateAndTrackProgress(self.map, self.DLO, self.gripperLeft, self.gripperRight, self.currentSubTask)
            msg = task.getMsg()
            # Check for imposible solutions
            trajectory, mode, targetFixture, previousFixture, cableSlack, grippWidth = task.getInfo()
            self.instruction = self.checkConstraints.check(self.map, trajectory, self.DLO, mode,\
                        self.tfListener, targetFixture, previousFixture, cableSlack)

            if self.instruction == 1:
                self.solve.updateInit(self.DLO, self.map, mode, grippWidth,\
                    targetFixture, previousFixture, self.tfListener, cableSlack)
                individual = self.solve.solve(100, 20)
                self.rerouting.resetTask()
                self.rerouting.initilize(mode, individual, grippWidth)
                task = self.rerouting
                task.updateAndTrackProgress(self.map, self.DLO, self.gripperLeft, self.gripperRight, self.currentSubTask)
                msg = task.getMsg()

            self.pub.publish(msg)
  
        task.updateAndTrackProgress(self.map, self.DLO, self.gripperLeft, self.gripperRight, self.currentSubTask)

        if task.getTaskDone() == 1:
            print('task done')
            if self.instruction == 0:
                if self.currentTask < self.numOfTasks-1:
                    self.currentTask += task.getNextTaskStep()
                    self.tasks[self.currentTask].resetTask() 
                    print('currentTask ', self.currentTask)
                    
            elif self.instruction == 1:
                self.instruction = 0
                self.currentTask += task.getNextTaskStep()
                task = self.tasks[self.currentTask]
                self.tasks[self.currentTask].resetTask() 
                print('currentTask ', self.currentTask)

        self.mtx_subTask.release()
        self.mtx_spr.release()

    
def main():
    rospy.init_node('pathPlanner', anonymous=True) 
    tfListener = tf.TransformListener()
    rospy.sleep(0.5)
    # objectes ----------------
    fixtureList = ['/Fixture1','/Fixture2','/Fixture3','/Fixture4']
    listOfObjects = []
    for i in range(len(fixtureList)):
        try:
            (pos, rot) = tfListener.lookupTransform('/yumi_base_link', fixtureList[i], rospy.Time(0))
            pos = np.asarray(pos)
            rot = np.asarray(rot)
            obj0 = utils.FixtureObject(pos, rot, 0.06)

            listOfObjects.extend([obj0])
        except:
            break

    # tasks -------------------
    slackList = [0.15, 0.04, 0.04, 0.04]
    listOfTasks  = []
    for i in range(len(listOfObjects)):
        grabCable = tasks.GrabCable(i, i-1, slackList[i])
        clippIntoFixture = tasks.ClippIntoFixture(i, i-1, [i])
        listOfTasks.extend([grabCable, clippIntoFixture])

    pathPlanner = PathPlanner(listOfObjects, listOfTasks)
    rospy.Subscriber("/spr/dlo_estimation", PointCloud, pathPlanner.callback, queue_size=2)
    rospy.Subscriber("/controller/sub_task", Int64, pathPlanner.callbackCurrentSubTask, queue_size=2)

    rospy.sleep(0.15)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        pathPlanner.update()

        rate.sleep()


if __name__ == '__main__':
    main()

