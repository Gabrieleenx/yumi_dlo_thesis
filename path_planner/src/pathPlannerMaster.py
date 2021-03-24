#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg
from sensor_msgs.msg import PointCloud
import tf
import numpy as np
import utils
import tasks

class PathPlanner(object):
    def __init__(self, listOfObjects, listOfTasks):
        self.pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
        self.map = listOfObjects
        self.tasks = listOfTasks
        self.numOfTasks = len(self.tasks)
        self.currentTask = 0
        self.DLOPoints = np.zeros((50,3)) # matrix with 3d points 
        self.DLO = utils.DLO()
        self.tfListener = tf.TransformListener()
        self.gripperRight = utils.FramePose()
        self.gripperLeft = utils.FramePose()


    def callback(data):
        # update self.DLOEstiamtion
        numOfPoints = len(data.points)
        self.DLOEstimation = np.zeros((numOfPoints,3))
        for i in range(numOfPoints):
            self.DLOPoints[i,0] = data.points[i].x
            self.DLOPoints[i,1] = data.points[i].y
            self.DLOPoints[i,2] = data.points[i].z

        self.DLO.update(self.DLOPoints)

    def update(self):
        # update pose 
        (posRight, orientationRight) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
        (posLeft, orientationLeft) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))
        self.gripperRight.update(posRight, orientationRight)
        self.gripperLeft.update(posLeft, orientationLeft)

        self.tasks[self.currentTask].updateAndTrackProgress(self.map, self.DLO, self.gripperLeft, self.gripperRight)
        if self.tasks[self.currentTask].getNewTrajectory() == 1:
            msg = self.tasks[self.currentTask].getTrajectory()
            self.pub.publish(msg)

        if self.tasks[self.currentTask].getTaskDone() == 1:
            if self.currentTask < self.numOfTasks-1:
                self.currentTask += 1 

    
def main():
    rospy.init_node('pathPlanner', anonymous=True) 
    # objectes ----------------
    obj0 = FixtureObject(np.array(0.3, -0.1, 0), np.array([1,0,0,0]))
    listOfObjects = [obj0]
    # tasks -------------------
    grabCable = tasks.GrabCable(0, -1, 0.05)

    listOfTasks = [grabCable]
    pathPlanner = PathPlanner(listOfObjects, listOfTasks)
    rospy.Subscriber("/spr/dlo_estimation", PointCloud, pathPlanner.callback, queue_size=2, )
    rospy.sleep(0.15)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        pathPlanner.update()
        rate.sleep()


if __name__ == '__main__':
    main()

