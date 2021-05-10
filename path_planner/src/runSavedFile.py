#!/usr/bin/env python3

import numpy as np
import tf
import utils
import rospy
import pickle
from sensor_msgs.msg import JointState, PointCloud
from geometry_msgs.msg import Point32

import os

class Replay(object):
    def __init__(self, savedData, timeStep):
        self.savedData = savedData
        self.time = savedData.fixturesObj[0].time
        self.timeStep = timeStep
        self.jointPosPub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.jointPositionIndex = 0
        self.DLOPointsIndex = 0
        self.DLOPub = rospy.Publisher('/spr/dlo_estimation', PointCloud, queue_size=3)
        self.tfbrodcaster = tf.TransformBroadcaster()
        self.pathplannerStateIndex = 0

    def step(self):
        self.time += self.timeStep
    
        while self.savedData.jointPosition[self.jointPositionIndex].time < self.time:
            self.jointPositionIndex += 1

        jointPos = self.savedData.jointPosition[self.jointPositionIndex].data
        jointPos = np.hstack([jointPos, np.zeros(4)])
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', \
            'yumi_joint_6_r', 'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', \
                'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l', 'gripper_r_joint', 'gripper_r_joint_m',\
                    'gripper_l_joint', 'gripper_l_joint_m']
        msg.position = jointPos.tolist()
        self.jointPosPub.publish(msg)


        while self.savedData.DLOPoints[self.DLOPointsIndex].time < self.time:
            self.DLOPointsIndex += 1

        self.estimate = self.savedData.DLOPoints[self.DLOPointsIndex].data
        self.publishDLO()



        for i in range(len(self.savedData.fixturesObj)):
            fixtureObj = self.savedData.fixturesObj[i].data
            pos = fixtureObj.getBasePosition()
            quat = fixtureObj.getOrientation()

            self.tfbrodcaster.sendTransform(pos,
                                            quat,
                                            rospy.Time.now(),
                                            "fixture"+str(i),
                                            "yumi_base_link")


        while self.savedData.pathplannerState[self.pathplannerStateIndex].time < self.time:
            print(self.savedData.pathplannerState[self.pathplannerStateIndex].data)
            self.pathplannerStateIndex += 1


    def publishDLO(self):
        cloudpoints = self.estimate
        cloud_msg = PointCloud()
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = "yumi_base_link"

        for i in range(cloudpoints.shape[0]):
            cloud_msg.points.append(Point32(cloudpoints[i, 0], cloudpoints[i, 1], cloudpoints[i, 2])) 
            # Change to camera frame
        self.DLOPub.publish(cloud_msg)


    
def main():
    # initilize ros node
    rospy.init_node('savedData', anonymous=True) 
    script_dir = os.path.dirname(__file__)
    rel_path = "SavedData/Test1/test5good3Frixtures.obj"
    abs_file_path = os.path.join(script_dir, rel_path)
    print(abs_file_path)
        
    file_load = open(abs_file_path, 'rb')
    savedObj = pickle.load(file_load)
    file_load.close()

    replay = Replay(savedData=savedObj, timeStep=1/10)

    rate = rospy.Rate(10) 
    print('hi')
    while not rospy.is_shutdown():
        replay.step()
        rate.sleep()

    print(savedObj.fixturesObj[0].data.getBasePosition())

if __name__ == '__main__':
    main()
