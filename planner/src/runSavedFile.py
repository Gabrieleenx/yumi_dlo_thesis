#!/usr/bin/env python3

import numpy as np
import tf
import utils
import rospy
import pickle
from sensor_msgs.msg import JointState, PointCloud
from geometry_msgs.msg import Point32
import matplotlib.pyplot as plt
from matplotlib.offsetbox import (TextArea, DrawingArea, OffsetImage,
                                  AnnotationBbox)
import os
plt.rcParams["font.family"] = "Times New Roman"

plt.style.use('ggplot')

import os
transformer = tf.TransformerROS(True, rospy.Duration(0.1)) 



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
        self.gripperPoseIndex = 0


        self.keyPoints = []
        self.plotX = []
        self.plotY = []

    def step(self):


        while self.savedData.gripperPose[self.gripperPoseIndex].time < self.time:
            self.gripperPoseIndex += 1
        gripperPose = self.savedData.gripperPose[self.gripperPoseIndex].data
        #gripperPose[posRight, orientationRight, posLeft, orientationLeft]
   
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
                                            "fixture"+str(i+1),
                                            "yumi_base_link")


        while self.savedData.pathplannerState[self.pathplannerStateIndex].time < self.time:
            print(self.savedData.pathplannerState[self.pathplannerStateIndex].data, ' time, ', self.savedData.pathplannerState[self.pathplannerStateIndex].time-self.savedData.fixturesObj[0].time)
            time_ =  self.savedData.pathplannerState[self.pathplannerStateIndex].time-self.savedData.fixturesObj[0].time
   

            if self.savedData.pathplannerState[self.pathplannerStateIndex].data[0:len('Generate new trajectory')] == 'Generate new trajectory' and \
                 self.savedData.pathplannerState[self.pathplannerStateIndex+2].data[0:len('Trajectory goodindividual')] == 'Trajectory goodindividual':
                if len(self.plotY) > 0:
                    self.plotY.append(self.plotY[-1])
                    self.plotX.append(time_)

                self.keyPoints.append(len(self.plotY))
                self.plotY.append(1)
                self.plotX.append(time_)
                print('Individual start')
            
            if self.savedData.pathplannerState[self.pathplannerStateIndex].data[0:len('Generate new trajectory')] == 'Generate new trajectory' and \
                 self.savedData.pathplannerState[self.pathplannerStateIndex+2].data[0:len('Trajectory goodcombined')] == 'Trajectory goodcombined':
                if len(self.plotY) > 0:
                    self.plotY.append(self.plotY[-1])
                    self.plotX.append(time_)
                
                self.keyPoints.append(len(self.plotY))
                self.plotY.append(2)
                self.plotX.append(time_)
                print('Combined start')

            if self.savedData.pathplannerState[self.pathplannerStateIndex].data[0:len('Searching for solution')] == 'Searching for solution':
                if len(self.plotY) > 0:
                    self.plotY.append(self.plotY[-1])
                    self.plotX.append(time_)
                
                self.keyPoints.append(len(self.plotY))
                self.plotY.append(3)
                self.plotX.append(time_)
                print('GA Searching for solution')  

            if self.savedData.pathplannerState[self.pathplannerStateIndex-2].data[0:len('Searching for solution')] == 'Searching for solution' and \
                 self.savedData.pathplannerState[self.pathplannerStateIndex].data[0:len('Trajectory goodindividual')] == 'Trajectory goodindividual':
                if len(self.plotY) > 0:
                    self.plotY.append(self.plotY[-1])
                    self.plotX.append(time_)

                self.plotY.append(4)
                self.plotX.append(time_)
                print('execute GA start')

            if self.savedData.pathplannerState[self.pathplannerStateIndex].data[0:len('All tasks completed')] == 'All tasks completed':
                if self.plotY[-1] == 0:
                    pass
                else:
                    if len(self.plotY) > 0:
                        self.plotY.append(self.plotY[-1])
                        self.plotX.append(time_)
                    
                    self.keyPoints.append(len(self.plotY))
                    self.plotY.append(0)
                    self.plotX.append(time_)
                    print('End State')  

            self.pathplannerStateIndex += 1


        self.time += self.timeStep


    def publishDLO(self):
        cloudpoints = self.estimate
        cloud_msg = PointCloud()
        cloud_msg.header.stamp = rospy.Time.now()
        cloud_msg.header.frame_id = "yumi_base_link"

        for i in range(cloudpoints.shape[0]):
            cloud_msg.points.append(Point32(cloudpoints[i, 0], cloudpoints[i, 1], cloudpoints[i, 2])) 
        self.DLOPub.publish(cloud_msg)

def plot_img(plt, ax, x, y, scale_img, path, x_, y_, strr):
    arr_img = plt.imread(path, format='jpg')
    axin = ax.inset_axes([x, y, scale_img * 16, scale_img * 1],transform=ax.transData)    # create new inset axes in data coordinates
    axin.imshow(arr_img)
    axin.axis('off')
    axin.text(50, 800, strr, fontsize = 18)
    ax.plot([x+(scale_img * 16)/2,x_], [y+scale_img/2,y_], '--k')

def plot_plot(replay, folderPath, extension):
    fig, ax = plt.subplots(figsize =(12, 5.5))
    ax.plot(replay.plotX, replay.plotY, '-ob', linewidth=3, alpha=0.8)

    plt.yticks([-3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7], ['', '', '','End state', 'GrabDLO', 'ClipIntoFixture', 'GA search', 'GA execute', '', '', ''],rotation=0)  # Set text labels and properties.
    plt.subplots_adjust(top = 0.95, bottom = 0.1, right = 1, left = 0.08, hspace = 0, wspace = 0)
    
    scale_img = 3.3
    offset = 60
    yHighLow = [4.5, -3]

    numImg = len(replay.keyPoints)

    for i in range(numImg):
        try:
            path = folderPath + str(i+1) + extension
            # Draw image
            index_ = replay.keyPoints[i]
            if index_ > 0:
                x = (replay.plotX[index_-1]+ replay.plotX[index_])/2
                y = (replay.plotY[index_-1]+ replay.plotY[index_])/2  
            else:
                x = replay.plotX[index_]
                y = replay.plotY[index_]
            plot_img(plt, ax, (i//2) * offset, yHighLow[i%2], scale_img, path, x, y, str(i+1))
        
        except:
            continue

    plt.ylim([-3, 7.82])
    plt.xlabel('Time [s]')
    plt.show()

def main():
    # initilize ros node
    rospy.init_node('savedData', anonymous=True) 

    path = "/home/gabriel/zNewTestGA/Run8.obj" # path to obj
    ImgFolderPath = "/home/gabriel/Downloads/Vid/" # path to folder with images corresponding to key moments, named '1.jpg', '2.jpg' ... 
    ImgExtension = ".jpg" # image extension 

    playBackSpeed = 1

    file_load = open(path, 'rb')
    savedObj = pickle.load(file_load)
    file_load.close()

    replay = Replay(savedData=savedObj, timeStep=1/10)
    rate = rospy.Rate(playBackSpeed*10) 
    while not rospy.is_shutdown():        
        try:
            replay.step()
        except:
            print('done')
            plot_plot(replay, ImgFolderPath, ImgExtension)
            break
        
        rate.sleep()
    
    rospy.spin()
    print(savedObj.fixturesObj[0].data.getBasePosition())

if __name__ == '__main__':
    main()
