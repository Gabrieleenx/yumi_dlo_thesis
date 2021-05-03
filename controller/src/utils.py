#!/usr/bin/env python3

import numpy as np
import rospy
import tf

class JointState(object):
    def __init__(self,\
            jointPosition=np.array([1.0, -2.0, -1.2, 0.6, -2.0, 1.0, 0.0, -1.0, -2.0, 1.2, 0.6, 2.0, 1.0, 0.0]),\
            jointVelocity=np.zeros(14)):
        self.jointPosition = jointPosition # only arm not gripper
        self.jointVelocity = jointVelocity # only arm not gripper

    
    def GetJointVelocity(self):
        return np.hstack([self.jointVelocity])
    
    def GetJointPosition(self):
        return np.hstack([self.jointPosition])

    def UpdatePose(self, pose):
        self.jointPosition = pose[0:14]

    def UpdateVelocity(self, vel):
        self.jointVelocity = vel[0:14]


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

    def getTarget(self):
        if self.trajectory[self.index].pointTime < self.trajectoryTime:
            if self.index < self.numberOfPoints - 1:
                self.trajectoryTime = 0
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
        self.trajectoryTime += self.deltaTime

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


class ControlInstructions(object): # generates target velocity in task space
    def __init__(self, deltaTime):
        self.mode = 'individual'
        self.trajectory = Trajectory(deltaTime)
        self.velocities = np.zeros(12)
        self.error = np.zeros(12)
        self.maxDeviation = np.array([0.01,0.01,0.01, 0.05,0.05,0.05, 0.01,0.01,0.01, 0.05,0.05,0.05])
        self.gripperLeft = np.array([0.01,0.01])
        self.gripperRight = np.array([0.01,0.01])
        self.lastGripperLeft = np.array([-1,-1])
        self.lastGripperRight = np.array([-1,-1])
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.trajIndex = 0

    def getIndividualTargetVelocity(self, k):

        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)

        self.error[0:3] = PositionError(self.translationRightArm,\
            self.targetPosition[0:3], 0.1)
        self.error[6:9] = PositionError(self.translationLeftArm,\
            self.targetPosition[3:6], 0.1)
        self.error[3:6]= RotationError(self.rotationRightArm, \
            self.targetOrientation[0:4], 0.2)
        self.error[9:12]= RotationError(self.rotationLeftArm, \
            self.targetOrientation[4:8], 0.2)

        self.velocities = self.targetVelocity + k*self.error

        return self.velocities

    def getAbsoluteTargetVelocity(self, k):

        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)
        self.error[0:3] = PositionError(self.absolutePosition,\
            self.targetPosition[0:3], 0.1)
        self.error[3:6]= RotationError(self.absoluteOrientation, \
            self.targetOrientation[0:4], 0.2)

        self.velocities[0:6] = self.targetVelocity[0:6] + k*self.error[0:6]

        return self.velocities[0:6]

    def getRelativeTargetVelocity(self, k):
        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)
        
        relativePosTarget = self.targetPosition[3:6]

        self.error[6:9] = PositionError(self.realativPosition ,\
            relativePosTarget, 0.1)

        self.error[9:12]= RotationError(self.rotationRelative, \
            self.targetOrientation[4:8], 0.2)

        self.velocities[6:12] = self.targetVelocity[6:12]  + k*self.error[6:12]

        return self.velocities[6:12], self.translationRightArm, self.translationLeftArm, self.absoluteOrientation

    def newIndex(self):
        if self.trajIndex != self.trajectory.index:
            self.trajIndex = self.trajectory.index
            return True
        else:
            return False

    def updateTarget(self):
        if len(self.trajectory.trajectory) < 2: 
            return
        self.targetPosition, self.targetOrientation, self.targetVelocity, self.gripperLeft, self.gripperRight = self.trajectory.getTarget()

    def updateTransform(self, yumiGrippPoseR, yumiGrippPoseL):

        self.translationRightArm = yumiGrippPoseR.getPosition()
        self.translationLeftArm = yumiGrippPoseL.getPosition()
        self.rotationRightArm = yumiGrippPoseR.getQuaternion()
        self.rotationLeftArm = yumiGrippPoseL.getQuaternion()

        tfMatrixRight = self.transformer.fromTranslationRotation(translation=self.translationRightArm, rotation=self.rotationRightArm)
        tfMatrixLeft = self.transformer.fromTranslationRotation(translation=self.translationLeftArm, rotation=self.rotationLeftArm)
        tfMatrixLeftInv = np.linalg.pinv(tfMatrixLeft)

        self.translationRelativeLeftRight = tfMatrixLeftInv.dot(np.hstack([self.translationRightArm, 1]))[0:3]
        rotLeftRight = tfMatrixLeftInv.dot(tfMatrixRight)
        self.rotationRelative = tf.transformations.quaternion_from_matrix(rotLeftRight)

        avgQ = np.vstack([self.rotationRightArm, self.rotationLeftArm])
        self.absoluteOrientation = averageQuaternions(avgQ)  
        self.absolutePosition = 0.5*(self.translationRightArm + self.translationLeftArm)

        transformation1 = self.transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=self.absoluteOrientation)
        transformationInv1 = np.linalg.pinv(transformation1)
        transformation2 = self.transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=self.rotationLeftArm)

        leftToAbsoluteFrameRot = transformationInv1.dot(transformation2)
        homogeneousLeftRelative = np.hstack([self.translationRelativeLeftRight,1])
        self.homogeneousAbsouluteRelative = leftToAbsoluteFrameRot.dot(homogeneousLeftRelative)
        self.realativPosition = self.homogeneousAbsouluteRelative[0:3]

    def checkDevation(self):
        devation = np.max(np.abs(self.error[0:3]) - self.maxDeviation[0:3])
        if devation > 0:
            return False
        return True

# used to calculate error
def PositionError(currentPositionXYZ, targetPositionXYZ, maxVelocity):
    positionDiff = (targetPositionXYZ - currentPositionXYZ) # /10
    norm = np.linalg.norm(positionDiff)
    positionDiffNormalized = normalize(positionDiff)        
    return positionDiffNormalized*min([maxVelocity, norm])

# used to calculate error
def RotationError(currentQ, targetQ, maxRotVel):

    if currentQ.dot(targetQ) < 0:
        currentQ = -currentQ

    skewTarget = np.array([[0, -targetQ[2], targetQ[1]],\
                            [targetQ[2],0,-targetQ[0]],\
                                [-targetQ[1],targetQ[0],0]])

    errorOrientation = currentQ[3]*targetQ[0:3] - targetQ[3]*currentQ[0:3] - skewTarget.dot(currentQ[0:3] )
    norm = np.linalg.norm(errorOrientation)

    errorOrientationNormalized = normalize(errorOrientation)     
    #errorOrientationNormalized*min([maxRotVel, norm])   
    return errorOrientationNormalized*min([maxRotVel, norm])


def CalcJacobianCombined(data, gripperLengthRight, gripperLengthLeft, transformer, yumiGrippPoseR, yumiGrippPoseL):
    jacobianRightArm = np.zeros((6,7))
    jacobianLeftArm = np.zeros((6,7))

    dataNP = np.asarray(data.data)

    jacobianRightArm = dataNP[0::2].reshape((6,7))
    jacobianLeftArm = dataNP[1::2].reshape((6,7))
    
    # change endeffector frame 
    translationRightArm = yumiGrippPoseR.getPosition()
    translationLeftArm = yumiGrippPoseL.getPosition()
    rotationRightArm = yumiGrippPoseR.getQuaternion()
    rotationLeftArm = yumiGrippPoseL.getQuaternion()

    jacobianRightArm = changeFrameJacobian(jacobianRightArm, gripperLengthRight, rotationRightArm, transformer)
    jacobianLeftArm = changeFrameJacobian(jacobianLeftArm, gripperLengthLeft, rotationLeftArm, transformer)
    
    return np.asarray(np.bmat([[jacobianRightArm,np.zeros((6,7))],[np.zeros((6,7)),jacobianLeftArm]]))


def changeFrameJacobian(jacobian, gripperLenght, rotation, transformer):
    # change end effector for jacobian 
    transformationMatrix = transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=rotation)
    gripperLenght = np.asarray(gripperLenght)
    velocityXYZ = transformationMatrix[0:3,0:3].dot(gripperLenght.reshape((3,1)))
    eye3 = np.eye(3)
    zeros3 = np.zeros((3,3))
    linkRotation = np.array([[0, velocityXYZ[2,0], -velocityXYZ[1,0]],[-velocityXYZ[2,0],0,velocityXYZ[0,0]],\
        [velocityXYZ[1,0],-velocityXYZ[0,0],0]])
    linkingMatrix = np.asarray(np.bmat([[eye3,linkRotation],[zeros3,eye3]]))
    return linkingMatrix.dot(jacobian)


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm


# taken (Modified) from https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (x,y,z,w), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    # from (x,y,z,w) to (w,x,y,z)
    if Q[0].dot(Q[1]) < 0:
        Q[0] = -Q[0]

    Q = np.roll(Q,1,axis=1)
    # Number of quaternions to average
    M = Q.shape[0]
    A = np.zeros(shape=(4,4))
    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A
    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    avgQ = np.real(eigenVectors[:,0])
    # from (w,x,y,z) to (x,y,z,w) 
    avgQ = np.roll(avgQ,-1)

    return avgQ

class FramePose(object):
    def __init__(self):
        self.tempPosition = np.zeros(3)
        self.position = np.zeros(4)
        self.quaternion = np.zeros(4)

    def getQuaternion(self):
        return self.quaternion

    def getPosition(self):
        return self.position[0:3]

    def update(self, pose, transfromer, gripperLength):
        self.tempPosition[0] = pose.position.x
        self.tempPosition[1] = pose.position.y
        self.tempPosition[2] = pose.position.z

        self.quaternion[0] = pose.orientation.x
        self.quaternion[1] = pose.orientation.y
        self.quaternion[2] = pose.orientation.z
        self.quaternion[3] = pose.orientation.w

        tfMatrix = transfromer.fromTranslationRotation(translation=self.tempPosition, rotation=self.quaternion)
        self.position = tfMatrix.dot(np.hstack([gripperLength, 1]))



