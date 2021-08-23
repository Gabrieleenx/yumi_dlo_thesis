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


def RotationError(currentQ, targetQ):

    if currentQ.dot(targetQ) < 0:
        currentQ = -currentQ

    skewTarget = np.array([[0, -targetQ[2], targetQ[1]],\
                            [targetQ[2],0,-targetQ[0]],\
                                [-targetQ[1],targetQ[0],0]])

    errorOrientation = currentQ[3]*targetQ[0:3] - targetQ[3]*currentQ[0:3] - skewTarget.dot(currentQ[0:3] )

    return errorOrientation


def calcAbsoluteAndRelative(translationRightArm, translationLeftArm, rotationRightArm, rotationLeftArm):

    tfMatrixRight = transformer.fromTranslationRotation(translation=translationRightArm, rotation=rotationRightArm)
    tfMatrixLeft = transformer.fromTranslationRotation(translation=translationLeftArm, rotation=rotationLeftArm)

    avgQ = np.vstack([rotationRightArm, rotationLeftArm])
    absoluteOrientation = utils.averageQuaternions(avgQ)  
    absolutePosition = 0.5*(translationRightArm + translationLeftArm)

    transformationAbsolute = transformer.fromTranslationRotation(translation=absolutePosition, rotation=absoluteOrientation)
    transformationAbsoluteInv = np.linalg.pinv(transformationAbsolute)

    transformationRightFromAbs = transformationAbsoluteInv.dot(tfMatrixRight)
    transformationLeftFromAbs = transformationAbsoluteInv.dot(tfMatrixLeft)
    quatRightAbs = tf.transformations.quaternion_from_matrix(transformationRightFromAbs)
    posRightAbs = tf.transformations.translation_from_matrix(transformationRightFromAbs)
    quatLeftAbs = tf.transformations.quaternion_from_matrix(transformationLeftFromAbs)
    posLeftAbs = tf.transformations.translation_from_matrix(transformationLeftFromAbs)

    relativeOrientation = tf.transformations.quaternion_multiply(quatRightAbs, tf.transformations.quaternion_conjugate(quatLeftAbs))
    realativPosition = posRightAbs - posLeftAbs

    return absolutePosition, absoluteOrientation, realativPosition, relativeOrientation

class TrajectoryPoint(object):
    def __init__(self,\
            positionLeft=np.array([0.4 ,0.2, 0.2]),\
            positionRight=np.array([0.4 ,-0.2, 0.2]),\
            orientationLeft=np.array([1,0,0,0]),\
            orientationRight=np.array([1,0,0,0]),\
            gripperLeft=np.array([0.0, 0.0]),\
            gripperRight=np.array([0.0, 0.0]),\
            pointTime=2.0):
        self.positionLeft = positionLeft
        self.positionRight = positionRight
        self.orientationLeft = orientationLeft
        self.orientationRight = orientationRight
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.pointTime = pointTime

def callbackTrajectory(data, gripperPose):
        # current pose as first point 
        #gripperPose[posRight, orientationRight, posLeft, orientationLeft]
        gripperLeft = np.array([0,0])
        gripperRight = gripperLeft
        if data.mode == 'combined':

            positionRight = np.asarray(gripperPose[0])
            positionLeft  = np.asarray(gripperPose[2])
            orientationRight = np.asarray(gripperPose[1])
            orientationLeft = np.asarray(gripperPose[3])
            positionRight, orientationRight,positionLeft,orientationLeft = \
                 calcAbsoluteAndRelative(positionRight, positionLeft, orientationRight, orientationLeft)

        elif data.mode == 'individual':
            positionRight = np.asarray(gripperPose[0])
            positionLeft  = np.asarray(gripperPose[2])
            orientationRight = np.asarray(gripperPose[1])
            orientationLeft = np.asarray(gripperPose[3])
        else:
            print('Error, mode not matching combined or individual')
            return
        # current gripper position # 

        currentPoint = TrajectoryPoint(positionRight=positionRight,\
                                                    positionLeft=positionLeft,\
                                                    orientationRight=orientationRight,\
                                                    orientationLeft=orientationLeft,\
                                                    gripperLeft=gripperLeft,\
                                                    gripperRight=gripperRight)
        trajectory = [currentPoint]
        # append trajectory points
        for i in range(len(data.trajectory)):
            if data.mode == 'combined':
                positionRight = np.asarray(data.trajectory[i].positionAbsolute)
                positionLeft  = np.asarray(data.trajectory[i].positionRelative)
                orientationRight = np.asarray(data.trajectory[i].orientationAbsolute)
                orientationLeft = np.asarray(data.trajectory[i].orientationRelative)
            else:
                positionRight = np.asarray(data.trajectory[i].positionRight)
                positionLeft  = np.asarray(data.trajectory[i].positionLeft)
                orientationRight = np.asarray(data.trajectory[i].orientationRight)
                orientationLeft = np.asarray(data.trajectory[i].orientationLeft)

            gripperLeft = np.asarray(data.trajectory[i].gripperLeft)
            gripperRight = np.asarray(data.trajectory[i].gripperRight)
            pointTime = np.asarray(data.trajectory[i].pointTime)
            trajectroyPoint = TrajectoryPoint(positionRight=positionRight,\
                                                    positionLeft=positionLeft,\
                                                    orientationRight=orientationRight,\
                                                    orientationLeft=orientationLeft,\
                                                    gripperLeft=gripperLeft,\
                                                    gripperRight=gripperRight,\
                                                    pointTime=pointTime)
            trajectory.append(trajectroyPoint)
        ''' # Ignoring, and not using any test sets where runtime problems ocur
        # use current velocity for smoother transitions, (not for orientaion)
        if self.controlInstructions.mode == data.mode:
            velLeftInit = np.copy(self.controlInstructions.velocities[6:9])
            velRightInit = np.copy(self.controlInstructions.velocities[0:3])
        elif self.controlInstructions.mode == 'individual':
            # simple solution, not fully accurate trasition 
            velLeftInit = np.zeros(3)
            velRightInit = 0.5*(np.copy(self.controlInstructions.velocities[0:3]) +\
                                     np.copy(self.controlInstructions.velocities[6:9]))
        elif self.controlInstructions.mode == 'combined':
            # simple solution, not fully accurate trasition 
            velLeftInit = np.copy(self.controlInstructions.velocities[0:3])
            velRightInit = np.copy(self.controlInstructions.velocities[0:3])
        else:
            print('Warning, Previous mode not matching, combined or individual')
        '''
        velLeftInit = np.zeros(3)
        velRightInit = np.zeros(3)
            

        # update the trajectroy 
        return trajectory, velLeftInit, velRightInit




class Trajectory(object):
    def __init__(self, deltaTime):
        self.trajectory = [] 
        self.trajectoryTime = 0
        self.deltaTime = deltaTime
        self.index = 1
        self.numberOfPoints = 0
        self.positionVelocitiesLeft = []
        self.positionVelocitiesRight = []
        self.roationMatrixLeft = []
        self.roationMatrixRight = []
        self.targetVelocity = np.zeros(12)
        self.targetPosition = np.zeros(6)
        self.targetOrientation = np.zeros(8)
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.timeOffset = 0

    def getTarget(self, timeThing):

        self.trajectoryTime = timeThing - self.timeOffset
        if self.trajectory[self.index].pointTime < self.trajectoryTime:
            if self.index < self.numberOfPoints - 1:
                #self.trajectoryTime = 0
                self.timeOffset = timeThing
                self.trajectoryTime = timeThing - self.timeOffset
                self.index += 1 
            else: # for last point 
                self.trajectoryTime = self.trajectory[self.index].pointTime
                self.index = self.numberOfPoints - 1

        # position
        q, dq = self.calcPosVel(qi=self.trajectory[self.index-1].positionRight,\
                                    dqi=self.positionVelocitiesRight[self.index-1],\
                                    qf=self.trajectory[self.index].positionRight,\
                                    dqf=self.positionVelocitiesRight[self.index],\
                                    tf=self.trajectory[self.index].pointTime,\
                                    t=self.trajectoryTime)
        self.targetPosition[0:3] = q
        self.targetVelocity[0:3] = dq

        q, dq = self.calcPosVel(qi=self.trajectory[self.index-1].positionLeft,\
                                    dqi=self.positionVelocitiesLeft[self.index-1],\
                                    qf=self.trajectory[self.index].positionLeft,\
                                    dqf=self.positionVelocitiesLeft[self.index],\
                                    tf=self.trajectory[self.index].pointTime,\
                                    t=self.trajectoryTime)
        self.targetPosition[3:6] = q
        self.targetVelocity[6:9] = dq

        # oritentaion 
        quat, we = self.calcOrientation(Ri=self.roationMatrixRight[self.index-1],\
                                Rf=self.roationMatrixRight[self.index])
        self.targetOrientation[0:4] = quat
        self.targetVelocity[3:6] = we

        quat, we = self.calcOrientation(Ri=self.roationMatrixLeft[self.index-1],\
                                Rf=self.roationMatrixLeft[self.index])
        self.targetOrientation[4:8] = quat
        self.targetVelocity[9:12] = we

        # update time 
        #self.trajectoryTime += self.deltaTime

        return self.targetPosition, self.targetOrientation, self.targetVelocity,\
             self.trajectory[self.index].gripperLeft, self.trajectory[self.index].gripperRight

    def calcPosVel(self, qi, dqi, qf, dqf, tf, t): # outputs target position and velocity 
        num = np.shape(qi)[0]
        q = np.zeros(num)
        dq = np.zeros(num)
        for k in range(num):
            a0 = qi[k]
            a1 = dqi[k]
            a2 = 3 * (qf[k] - (dqf[k]*tf)/3 - a1*tf*(2/3) - a0)/(tf*tf)
            a3 = (dqf[k] - (2*a2*tf + a1))/(3*tf*tf)
            q[k] = a3*t**3 + a2*t**2  + a1*t + a0
            dq[k] = 3*a3*t**2 + 2*a2*t + a1
        return q, dq

    def calcOrientation(self, Ri, Rf): # outputs target orientation and velocity
        R_i_f = np.transpose(Ri).dot(Rf)
        inCos = (R_i_f[0,0] + R_i_f[1,1] + R_i_f[2,2] - 1) / 2
        inCos = np.clip(inCos,-1,1)
        vf = np.arccos(inCos)
        if abs(vf) < 0.001: # sigularity for 180 degrees or 0 degrees
            rotMatrix = np.eye(4)
            rotMatrix[0:3,0:3] = Ri 
            quat = tf.transformations.quaternion_from_matrix(rotMatrix)
            return quat, np.zeros(3)

        r = (1/(2*np.sin(vf)))*np.array([[R_i_f[2,1] - R_i_f[1,2]],\
                                    [R_i_f[0,2] - R_i_f[2,0]],\
                                    [R_i_f[1,0] - R_i_f[0,1]]])

        v, dv = self.calcPosVel(qi=np.array([0]),\
                                    dqi=np.array([0]),\
                                    qf=np.array([vf]),\
                                    dqf=np.array([0]),\
                                    tf=self.trajectory[self.index].pointTime,\
                                    t=self.trajectoryTime)
        w_I = dv*r
        R_I = self.calcR_I(v, r)
        Re = Ri.dot(R_I)
        we = Ri.dot(w_I)
        rotMatrix = np.eye(4)
        rotMatrix[0:3,0:3] = Re 
        quat = tf.transformations.quaternion_from_matrix(rotMatrix)
        return quat, we.reshape((3,))

    def updatePoints(self, trajcetoryList, velLeftInit, velRightInit):
        self.positionVelocitiesLeft = [velLeftInit]
        self.positionVelocitiesRight = [velRightInit]
        self.roationMatrixLeft = []
        self.roationMatrixRight = []
        self.timeOffset = 0
        self.index = 1
        self.trajectoryTime = 0
        self.trajectory = trajcetoryList
        self.numberOfPoints = len(self.trajectory)

        # list of rotation matrices 
        for i in range(0,self.numberOfPoints):
            tfMatrixRight = self.transformer.fromTranslationRotation(translation=np.zeros(3), rotation=trajcetoryList[i].orientationRight)
            tfMatrixLeft = self.transformer.fromTranslationRotation(translation=np.zeros(3), rotation=trajcetoryList[i].orientationLeft)
            self.roationMatrixLeft.append(tfMatrixLeft[0:3,0:3])
            self.roationMatrixRight.append(tfMatrixRight[0:3,0:3])

        for i in range(1,self.numberOfPoints-1):
            vel = self.calcPointVel(trajcetoryList[i-1].positionRight, trajcetoryList[i].positionRight, trajcetoryList[i+1].positionRight, trajcetoryList[i].pointTime, trajcetoryList[i+1].pointTime)
            self.positionVelocitiesRight.append(vel)
            vel = self.calcPointVel(trajcetoryList[i-1].positionLeft, trajcetoryList[i].positionLeft, trajcetoryList[i+1].positionLeft, trajcetoryList[i].pointTime, trajcetoryList[i+1].pointTime)
            self.positionVelocitiesLeft.append(vel)

        #last point has velocity 0
        self.positionVelocitiesRight.append(np.zeros(3))
        self.positionVelocitiesLeft.append(np.zeros(3))

    def calcPointVel(self, v1, v2, v3, t2, t3): # velocity at point between first and last
        vel = np.zeros(3)
        vk = (v2 - v1)/t2
        vkk = (v3 - v2)/t3
        for i in range(3):
            if np.sign(vk[i]) == np.sign(vkk[i]):
                vel[i] = 0.5*(vk[i] + vkk[i]) 
            else:
                vel[i] = 0
        return vel

    def calcR_I(self, v, r): # convert back to rotation matrix
        cv = np.cos(v)[0]
        sv = np.sin(v)[0]
        rx = r[0,0]
        ry = r[1,0]
        rz = r[2,0]
    
        R_I = np.array([[(rx**2 * (1-cv)+cv), (rx*ry*(1-cv)- rz*sv), (rx*rz*(1-cv)+ry*sv)],\
                 [(rx*ry*(1-cv)+rz*sv),(ry**2 * (1-cv) + cv),(ry*rz*(1-cv)-rx*sv)],\
                  [(rx*rz*(1-cv)-ry*sv),(ry*rz*(1-cv)+rx*sv),(rz**2 * (1-cv)+cv)]])
        return R_I




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
        self.msgSentIndex = 0
        self.gripperPoseIndex = 0
        self.trajectory = Trajectory(timeStep)
        self.mode = 'individual'
        self.timeTrajOffset = 0

        self.positionSaveRLxyzIndividualTarget = []
        self.positionSaveRLxyzIndividualactual = []

        self.errorInividualPosRLxyz = []

        self.errorInividualRotR = []
        self.errorInividualRotL = []


        self.ttt = []
        self.plotIndividual = 0


        self.positionSaveRLxyzCombinedTarget = []
        self.positionSaveRLxyzCombinedactual = []

        self.errorCombinedPosRLxyz = []

        self.errorAbsrot = []
        self.errorRelrot = []
        self.tttCombined = []
        self.plotCombined = 0
        
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

                self.plotY.append(1)
                self.plotX.append(time_)
                print('Individual start')
            
            if self.savedData.pathplannerState[self.pathplannerStateIndex].data[0:len('Generate new trajectory')] == 'Generate new trajectory' and \
                 self.savedData.pathplannerState[self.pathplannerStateIndex+2].data[0:len('Trajectory goodcombined')] == 'Trajectory goodcombined':
                if len(self.plotY) > 0:
                    self.plotY.append(self.plotY[-1])
                    self.plotX.append(time_)

                self.plotY.append(2)
                self.plotX.append(time_)
                print('Combined start')

            if self.savedData.pathplannerState[self.pathplannerStateIndex].data[0:len('Searching for solution')] == 'Searching for solution':
                if len(self.plotY) > 0:
                    self.plotY.append(self.plotY[-1])
                    self.plotX.append(time_)

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
            # Change to camera frame
        self.DLOPub.publish(cloud_msg)

def plot_img(plt, ax, x, y, scale_img, path, x_, y_, strr):
    arr_img = plt.imread(path, format='jpg')
    axin = ax.inset_axes([x, y, scale_img * 16, scale_img * 1],transform=ax.transData)    # create new inset axes in data coordinates
    axin.imshow(arr_img)
    axin.axis('off')
    axin.text(50, 800, strr, fontsize = 18)

    ax.plot([x+(scale_img * 16)/2,x_], [y+scale_img/2,y_], '--k')

    
def main():
    # initilize ros node
    rospy.init_node('savedData', anonymous=True) 
    script_dir = os.path.dirname(__file__)
    rel_path = "SavedData/test2/test4.obj"
    abs_file_path = os.path.join(script_dir, rel_path)
    abs_file_path = "/home/gabriel/zNewTestGA/Run8.obj"
    print(abs_file_path)
        
    file_load = open(abs_file_path, 'rb')
    savedObj = pickle.load(file_load)
    file_load.close()

    replay = Replay(savedData=savedObj, timeStep=1/10)

    rate = rospy.Rate(1000) 
    print('hi')
    while not rospy.is_shutdown():
        #replay.step()
        
        try:
            replay.step()
        except:
            print('done')
            fig, ax = plt.subplots(figsize =(12, 6))
            ax.plot(replay.plotX, replay.plotY, '-ob', linewidth=3, alpha=0.8)

            plt.yticks([-3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7], ['', '', '','End state', 'GrabDLO', 'ClipIntoFixture', 'GA search', 'GA execute', '', '', ''],rotation=0)  # Set text labels and properties.
            plt.subplots_adjust(top = 0.95, bottom = 0.1, right = 1, left = 0.08, hspace = 0, wspace = 0)

            #n = get_sample_data(, asfileobj=False)
            path = "/home/gabriel/Downloads/Vid/1.jpg" #"/home/gabriel/catkin/src/yumi_dlo_thesis/path_planner/src/20210722_114545.jpg"
            #if we need find it first
  
            print(path)
            scale_img = 3.3
            offset = 60
            # Draw image
            x = replay.plotX[0]
            y = replay.plotY[0]
            plot_img(plt, ax, 0, 4.5, scale_img, path, x, y, '1')
            path = "/home/gabriel/Downloads/Vid/2.jpg"
            x = (replay.plotX[1]+ replay.plotX[2])/2
            y = (replay.plotY[1]+ replay.plotY[2])/2  
            plot_img(plt, ax, 0, -3, scale_img, path, x, y, '2')
            path = "/home/gabriel/Downloads/Vid/3.jpg"
            x = (replay.plotX[3]+ replay.plotX[4])/2
            y = (replay.plotY[3]+ replay.plotY[4])/2  
            plot_img(plt, ax, offset, 4.5, scale_img, path, x, y, '3')
            path = "/home/gabriel/Downloads/Vid/4.jpg"
            x = (replay.plotX[7]+ replay.plotX[8])/2
            y = (replay.plotY[7]+ replay.plotY[8])/2  
            plot_img(plt, ax, offset, -3, scale_img, path, x, y, '4')
            path = "/home/gabriel/Downloads/Vid/5.jpg"
            x = (replay.plotX[9]+ replay.plotX[10])/2
            y = (replay.plotY[9]+ replay.plotY[10])/2  
            plot_img(plt, ax, 2*offset, 4.5, scale_img, path, x, y, '5')
            path = "/home/gabriel/Downloads/Vid/6.jpg"
            x = (replay.plotX[11]+ replay.plotX[12])/2
            y = (replay.plotY[11]+ replay.plotY[12])/2  
            plot_img(plt, ax, 2*offset, -3, scale_img, path, x, y, '6')
            path = "/home/gabriel/Downloads/Vid/7.jpg"
            x = (replay.plotX[13]+ replay.plotX[14])/2
            y = (replay.plotY[13]+ replay.plotY[14])/2  
            plot_img(plt, ax, 3*offset, 4.5, scale_img, path, x, y, '7')
            path = "/home/gabriel/Downloads/Vid/8.jpg"
            x = (replay.plotX[15]+ replay.plotX[16])/2
            y = (replay.plotY[15]+ replay.plotY[16])/2  
            plot_img(plt, ax, 3*offset, -3, scale_img, path, x, y, '8')

            plt.ylim([-3, 7.82])
            plt.xlabel('Time [s]')
            plt.show()
            break
        
        rate.sleep()
    
    rospy.spin()
    print(savedObj.fixturesObj[0].data.getBasePosition())

if __name__ == '__main__':
    main()
