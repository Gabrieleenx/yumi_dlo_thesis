#!/usr/bin/env python3

import numpy as np
import tf
import utils
import rospy
import pickle
from sensor_msgs.msg import JointState, PointCloud
from geometry_msgs.msg import Point32
import matplotlib.pyplot as plt



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


    def step(self):


        while self.savedData.gripperPose[self.gripperPoseIndex].time < self.time:
            self.gripperPoseIndex += 1
        gripperPose = self.savedData.gripperPose[self.gripperPoseIndex].data
        #gripperPose[posRight, orientationRight, posLeft, orientationLeft]

        while self.savedData.msgSent[self.msgSentIndex].time < self.time:
            msgSent = self.savedData.msgSent[self.msgSentIndex].data
            self.mode = msgSent.mode
            trajectory, velLeftInit, velRightInit = callbackTrajectory(data=msgSent, gripperPose=gripperPose)
            self.trajectory.updatePoints(trajectory, velLeftInit, velRightInit)
            self.timeTrajOffset = self.savedData.msgSent[self.msgSentIndex].time# self.savedData.gripperPose[self.gripperPoseIndex].time
            if not self.msgSentIndex == len(self.savedData.msgSent) -1: 
                self.msgSentIndex += 1
            else:
                break
            if self.msgSentIndex == 2:
                self.plotIndividual = 1
            if self.msgSentIndex == 3:
                self.plotCombined = 1

        if self.msgSentIndex > 0:
            targetPosition, targetOrientation, _, _, _ = self.trajectory.getTarget(self.savedData.gripperPose[self.gripperPoseIndex].time-self.timeTrajOffset)
        
        if self.mode == 'individual' and self.msgSentIndex == 1:
            self.positionSaveRLxyzIndividualTarget.append(np.copy(targetPosition))
            #print(self.positionSaveRLxyzIndividualTarget[-1])
            gripperPosePos = [gripperPose[0], gripperPose[2]]
            flat_list = [item for sublist in gripperPosePos for item in sublist]
            flat_list = np.asarray(flat_list)
            self.positionSaveRLxyzIndividualactual.append(flat_list)
            error_ = flat_list - targetPosition
            self.errorInividualPosRLxyz.append(error_)
            self.ttt.append(self.savedData.gripperPose[self.gripperPoseIndex].time - self.timeTrajOffset)
            QrightG = np.asarray(gripperPose[1])
            QleftG = np.asarray(gripperPose[3])
            rotError = RotationError(currentQ=QrightG, targetQ=targetOrientation[0:4])
            self.errorInividualRotR.append(np.copy(rotError))

            rotError =RotationError(currentQ=QleftG, targetQ=targetOrientation[4:8])
            self.errorInividualRotL.append(np.copy(rotError))

        '''
        if self.mode == 'combined' and self.msgSentIndex == 2:
            self.positionSaveRLxyzCombinedTarget.append(np.copy(targetPosition))
            #print(self.positionSaveRLxyzIndividualTarget[-1])

            positionRight = np.asarray(gripperPose[0])
            positionLeft  = np.asarray(gripperPose[2])
            orientationRight = np.asarray(gripperPose[1])
            orientationLeft = np.asarray(gripperPose[3])
            positionRight, orientationRight,positionLeft,orientationLeft = \
                 calcAbsoluteAndRelative(positionRight, positionLeft, orientationRight, orientationLeft)

            gripperPosePos = [positionRight, positionLeft]

            flat_list = [item for sublist in gripperPosePos for item in sublist]
            flat_list = np.asarray(flat_list)
            self.positionSaveRLxyzCombinedactual.append(flat_list)
            error_ = flat_list - targetPosition
            self.errorCombinedPosRLxyz.append(error_)
            self.tttCombined.append(self.savedData.gripperPose[self.gripperPoseIndex].time - self.timeTrajOffset)


            rotError = RotationError(currentQ=orientationRight, targetQ=targetOrientation[0:4])
            self.errorAbsrot.append(np.copy(rotError))

            rotError =RotationError(currentQ=orientationLeft, targetQ=targetOrientation[4:8])
            self.errorRelrot.append(np.copy(rotError))

            #print(error_)
        
        if self.plotCombined == 1:
            self.plotCombined = 0
            print('hhhheeeellllooo')
            ttt = np.asarray(self.tttCombined)
            
            posAc = np.asarray(self.positionSaveRLxyzCombinedactual)
            postr = np.asarray(self.positionSaveRLxyzCombinedTarget)
            error_ = np.asarray(self.errorCombinedPosRLxyz)

            error_Rotabs = np.asarray(self.errorAbsrot)
            error_Rotrel = np.asarray(self.errorRelrot)

            plt.figure(1, figsize=(7,5))
            plt.subplot(211)
            #print(postr)
            l1 = plt.plot(ttt, posAc[:,0], 'r--', label='Position x-axis')
            l2 = plt.plot(ttt, postr[:,0], 'r-', label='Trajectory x-axis')
            l3 = plt.plot(ttt, posAc[:,1], 'g--' , label='Position y-axis')
            l4 = plt.plot(ttt, postr[:,1], 'g-', label='Trajectory y-axis')
            l5 = plt.plot(ttt, posAc[:,2], 'b--', label='Position z-axis')
            l6 = plt.plot(ttt, postr[:,2], 'b-', label='Trajectory z-axis')
            plt.legend(loc='upper right')
            plt.xlim(0, 21)
            plt.title('Absolute and relative position and trajectory')
            plt.xlabel('Time [s]')
            plt.ylabel('Position absolute [m]')
            plt.subplot(212)
            l1 = plt.plot(ttt, posAc[:,3], 'r--', label='Position x-axis')
            l2 = plt.plot(ttt, postr[:,3], 'r-', label='Trajectory x-axis')
            l3 = plt.plot(ttt, posAc[:,4], 'g--' , label='Position y-axis')
            l4 = plt.plot(ttt, postr[:,4], 'g-', label='Trajectory y-axis')
            l5 = plt.plot(ttt, posAc[:,5], 'b--', label='Position z-axis')
            l6 = plt.plot(ttt, postr[:,5], 'b-', label='Trajectory z-axis')
            plt.legend(loc='upper right')
            plt.xlabel('Time [s]')
            plt.xlim(0, 21)
            plt.ylabel('Position relative [m]')
            plt.tight_layout()

            plt.figure(2, figsize=(7,5))
            plt.subplot(211)
            #print(postr)
            l1 = plt.plot(ttt, error_[:,0], 'r-', label='Error x-axis')
            l4 = plt.plot(ttt, error_[:,1], 'g-', label='Error y-axis')
            l6 = plt.plot(ttt, error_[:,2], 'b-', label='Error z-axis')
            plt.legend(loc='upper right')
            plt.title('Error position')
            plt.xlim(0, 21)
            plt.xlabel('Time [s]')
            plt.ylabel('Error absolute [m]')
            plt.subplot(212)
            l1 = plt.plot(ttt, error_[:,3], 'r-', label='Error x-axis')
            l4 = plt.plot(ttt, error_[:,4], 'g-', label='Error y-axis')
            l6 = plt.plot(ttt, error_[:,5], 'b-', label='Error z-axis')
            plt.legend(loc='upper right')
            plt.xlabel('Time [s]')
            plt.xlim(0, 21)
            plt.ylabel('Error relative  [m]')
            plt.tight_layout()            

            print('mean pos', error_.mean())
            print('std pos', error_.std())
            print('rmse pos', np.sqrt((error_**2).mean()))


            plt.figure(3, figsize=(7,5))
            plt.subplot(211)
            #print(postr)
            plt.title('Error angular')

            l1 = plt.plot(ttt, error_Rotabs[:,0], 'r-', label='Error x-axis')
            l4 = plt.plot(ttt, error_Rotabs[:,1], 'g-', label='Error y-axis')
            l6 = plt.plot(ttt, error_Rotabs[:,2], 'b-', label='Error z-axis')
            plt.legend(loc='upper right')
            plt.xlabel('Time [s]')
            plt.xlim(0, 21)
            plt.ylabel('Angular error absolute [rad]')
            plt.subplot(212)
            l1 = plt.plot(ttt, error_Rotrel[:,0], 'r-', label='Error x-axis')
            l4 = plt.plot(ttt, error_Rotrel[:,1], 'g-', label='Error y-axis')
            l6 = plt.plot(ttt, error_Rotrel[:,2], 'b-', label='Error z-axis')
            plt.legend(loc='upper right')
            plt.xlabel('Time [s]')
            plt.xlim(0, 21)

            plt.ylabel('Angular error relative [rad]')
            plt.tight_layout()
            error_ = np.hstack([error_Rotabs, error_Rotrel])
            print('mean ang', error_.mean())
            print('std ang', error_.std())
            print('rmse ang', np.sqrt((error_**2).mean()))
            plt.show()
    

            plt.show()
            

        if self.plotIndividual == 1:
            self.plotIndividual = 0
            print('hhhheeeellllooo')
            ttt = np.asarray(self.ttt)
            
            posAc = np.asarray(self.positionSaveRLxyzIndividualactual)
            postr = np.asarray(self.positionSaveRLxyzIndividualTarget)
            error_ = np.asarray(self.errorInividualPosRLxyz)
            error_RotR = np.asarray(self.errorInividualRotR)
            error_RotL = np.asarray(self.errorInividualRotL)
            plt.figure(1, figsize=(7,5))
            plt.subplot(211)
            #print(postr)
            l1 = plt.plot(ttt, posAc[:,0], 'r--', label='Position x-axis')
            l2 = plt.plot(ttt, postr[:,0], 'r-', label='Trajectory x-axis')
            l3 = plt.plot(ttt, posAc[:,1], 'g--' , label='Position y-axis')
            l4 = plt.plot(ttt, postr[:,1], 'g-', label='Trajectory y-axis')
            l5 = plt.plot(ttt, posAc[:,2], 'b--', label='Position z-axis')
            l6 = plt.plot(ttt, postr[:,2], 'b-', label='Trajectory z-axis')
            plt.legend(loc='upper right')
            plt.xlim(0, 36)
            plt.title('Gripper position and trajectory')
            plt.xlabel('Time [s]')
            plt.ylabel('Position right gripper [m]')
            plt.subplot(212)
            l1 = plt.plot(ttt, posAc[:,3], 'r--', label='Position x-axis')
            l2 = plt.plot(ttt, postr[:,3], 'r-', label='Trajectory x-axis')
            l3 = plt.plot(ttt, posAc[:,4], 'g--' , label='Position y-axis')
            l4 = plt.plot(ttt, postr[:,4], 'g-', label='Trajectory y-axis')
            l5 = plt.plot(ttt, posAc[:,5], 'b--', label='Position z-axis')
            l6 = plt.plot(ttt, postr[:,5], 'b-', label='Trajectory z-axis')
            plt.legend(loc='upper right')
            plt.xlabel('Time [s]')
            plt.xlim(0, 36)
            plt.ylabel('Position left gripper  [m]')
            plt.tight_layout()
            plt.figure(2, figsize=(7,5))
            plt.subplot(211)
            #print(postr)
            l1 = plt.plot(ttt, error_[:,0], 'r-', label='Error x-axis')
            l4 = plt.plot(ttt, error_[:,1], 'g-', label='Error y-axis')
            l6 = plt.plot(ttt, error_[:,2], 'b-', label='Error z-axis')
            plt.legend(loc='upper right')
            plt.xlim(0, 36)
            plt.title('Error position')
            plt.xlabel('Time [s]')
            plt.ylabel('Error right gripper [m]')
            plt.subplot(212)
            l1 = plt.plot(ttt, error_[:,3], 'r-', label='Error x-axis')
            l4 = plt.plot(ttt, error_[:,4], 'g-', label='Error y-axis')
            l6 = plt.plot(ttt, error_[:,5], 'b-', label='Error z-axis')
            plt.legend(loc='upper right')
            plt.xlabel('Time [s]')
            plt.xlim(0, 36)
            plt.ylabel('Error left gripper  [m]')
            plt.tight_layout()
            print('mean pos', error_.mean())
            print('std pos', error_.std())
            print('rmse pos', np.sqrt((error_**2).mean()))

            plt.figure(3, figsize=(7,5))
            plt.subplot(211)
            #print(postr)
            plt.title('Error angular')

            l1 = plt.plot(ttt, error_RotR[:,0], 'r-', label='Error x-axis')
            l4 = plt.plot(ttt, error_RotR[:,1], 'g-', label='Error y-axis')
            l6 = plt.plot(ttt, error_RotR[:,2], 'b-', label='Error z-axis')
            plt.legend(loc='upper right')
            plt.xlabel('Time [s]')
            plt.xlim(0, 36)

            plt.ylabel('Angular error right gripper [rad]')
            plt.subplot(212)
            l1 = plt.plot(ttt, error_RotL[:,0], 'r-', label='Error x-axis')
            l4 = plt.plot(ttt, error_RotL[:,1], 'g-', label='Error y-axis')
            l6 = plt.plot(ttt, error_RotL[:,2], 'b-', label='Error z-axis')
            plt.legend(loc='upper right')
            plt.xlabel('Time [s]')
            plt.xlim(0, 36)

            plt.ylabel('Angular error left gripper [rad]')
            plt.tight_layout()
            error_ = np.hstack([error_RotR, error_RotL])
            print('mean ang', error_.mean())
            print('std ang', error_.std())
            print('rmse ang', np.sqrt((error_**2).mean()))
            print('DLO Hz', self.DLOPointsIndex/ttt[-1])
            plt.show()
        '''  


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


    
def main():
    # initilize ros node
    rospy.init_node('savedData', anonymous=True) 
    script_dir = os.path.dirname(__file__)
    rel_path = "SavedData/test2/test4.obj"
    abs_file_path = os.path.join(script_dir, rel_path)
    print(abs_file_path)
        
    file_load = open(abs_file_path, 'rb')
    savedObj = pickle.load(file_load)
    file_load.close()

    replay = Replay(savedData=savedObj, timeStep=1/10)

    rate = rospy.Rate(10) 
    print('hi')
    while not rospy.is_shutdown():
        #replay.step()
        
        try:
            replay.step()
        except:
            print('done')
            break
        
        rate.sleep()
    
    rospy.spin()
    print(savedObj.fixturesObj[0].data.getBasePosition())

if __name__ == '__main__':
    main()
