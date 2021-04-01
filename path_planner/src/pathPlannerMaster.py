#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Int64
import tf
import numpy as np
import utils
import tasks
import threading
# TODO thread safe
# TODO Wait for first DLO

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
        self.mtx_spr = threading.Lock()
        self.mtx_subTask = threading.Lock()

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
        self.tasks[self.currentTask].updateAndTrackProgress(self.map, self.DLO, self.gripperLeft, self.gripperRight, self.currentSubTask)
        self.mtx_subTask.release()
        self.mtx_spr.release()

        if self.tasks[self.currentTask].getNewTrajectory() == 1:
            msg = self.tasks[self.currentTask].getTrajectory()
            self.pub.publish(msg)

        if self.tasks[self.currentTask].getTaskDone() == 1:
            print('task done')
            if self.currentTask < self.numOfTasks-1:
                self.currentTask += 1 
                self.tasks[self.currentTask].taskDone = 0 

    
def main():
    rospy.init_node('pathPlanner', anonymous=True) 
    # objectes ----------------
    obj0 = utils.FixtureObject(np.array([0.3, -0.1, 0]), np.array([1,0,0,0]), 0.06)
    listOfObjects = [obj0]
    # tasks -------------------
    grabCable = tasks.GrabCable(0, -1, 0.15)
    clippIntoFixture = tasks.ClippIntoFixture(0, -1, 0.15)

    listOfTasks = [grabCable, clippIntoFixture]
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

