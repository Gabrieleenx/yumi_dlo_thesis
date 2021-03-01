#!/usr/bin/env python3

import numpy as np
import rospy
import tf

class JointState(object):
    def __init__(self,\
            jointPosition=np.array([-0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),\
            jointVelocity=np.zeros(14),\
            gripperRightPosition=np.zeros(2),\
            gripperLeftPosition=np.zeros(2),\
            gripperRightVelocity=np.zeros(2),\
            gripperLeftVelocity=np.zeros(2)):

        self.jointPosition = jointPosition # only arm not gripper
        self.jointVelocity = jointVelocity # only arm not gripper
        self.gripperRightPosition = gripperRightPosition
        self.gripperLeftPosition = gripperLeftPosition
        self.gripperRightVelocity = gripperRightVelocity
        self.gripperLeftVelocity = gripperLeftVelocity
    
    def GetJointVelocity(self):
        return np.hstack([self.jointVelocity, \
            self.gripperRightVelocity, self.gripperLeftVelocity])
    
    def GetJointPosition(self):
        return np.hstack([self.jointPosition, \
            self.gripperRightPosition, self.gripperLeftPosition])

    def UpdatePose(self, pose):
        self.jointPosition = pose[0:14]
        self.gripperRightPosition = pose[14:16]
        self.gripperLeftPosition = pose[16:18]


class TrajcetoryPoint(object):
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
        self.trajectory = [] # first element should be initial position for when trajectory is updated (pointTime = 0). 
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

        # update time (maybe move up?)
        self.trajectoryTime += self.deltaTime

        return self.targetPosition, self.targetOrientation, self.targetVelocity

    def calcPosVel(self, qi, dqi, qf, dqf, tf, t):
        num = np.shape(qi)[0]
        q = np.zeros(num)
        dq = np.zeros(num)
        for k in range(num):
            a0 = qi[k]
            a1 = dqi[k]
            a2 = 3 * (qf[k] - (dq[k]*tf)/3 - a1*tf*(2/3) - a0)/(tf*tf)
            a3 = (dqf[k] - (2*a2*tf + a1))/(3*tf*tf)
            q[k] = a3*t**3 + a2*t**2  + a1*t + a0
            dq[k] = 3*a3*t**2 + 2*a2*t + a1
        return q, dq

    def calcOrientation(self, Ri, Rf):
        R_i_f = np.transpose(Ri).dot(Rf)
        inCos = (R_i_f[0,0] + R_i_f[1,1] + R_i_f[2,2] - 1) / 2
        inCos = np.clip(inCos,-1,1)
        vf = np.arccos(inCos)
        if abs(vf) < 0.001:
            rotMatrix = np.eye(4)
            rotMatrix[0:3,0:3] = Ri 
            quat = tf.transformations.quaternion_from_matrix(rotMatrix)
            return quat, np.zeros(3)

        r = 1/(2*np.sin(vf))*np.array([[R_i_f[2,1] - R_i_f[1,2]],\
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

        for i in range(0,self.numberOfPoints):
            tfMatrixRight = self.transformer.fromTranslationRotation(translation=np.zeros(3), rotation=trajcetoryList[i].orientationRight)
            tfMatrixLeft = self.transformer.fromTranslationRotation(translation=np.zeros(3), rotation=trajcetoryList[i].orientationLeft)
            self.roationMatrixLeft.append(tfMatrixLeft[0:3,0:3])
            self.roationMatrixRight.append(tfMatrixRight[0:3,0:3])

        for i in range(1,self.numberOfPoints-1):

            vel = self.calcPointVel(trajcetoryList[i-1].positionRight, trajcetoryList[i].positionRight, trajcetoryList[i+1].positionRight)
            self.positionVelocitiesRight.append(vel)

            vel = self.calcPointVel(trajcetoryList[i-1].positionLeft, trajcetoryList[i].positionLeft, trajcetoryList[i+1].positionLeft)
            self.positionVelocitiesLeft.append(vel)

        self.positionVelocitiesRight.append(np.zeros(3))
        self.positionVelocitiesLeft.append(np.zeros(3))

    def calcPointVel(self, v1, v2, v3):
        vel = np.zeros(3)
        vk = v2 - v1
        vkk = v3 - v2
        for i in range(3):
            if np.sign(vk[i]) == np.sign(vkk[i]):
                vel[i] = 0.5*(vk[i] + vkk[i])
            else:
                vel[i] = 0
        return vel

    def calcR_I(self, v, r):
        cv = np.cos(v)[0]
        sv = np.sin(v)[0]
        rx = r[0,0]
        ry = r[1,0]
        rz = r[2,0]
    
        R_I = np.array([[(rx**2 * (1-cv)+cv), (rx*ry*(1-cv)- rz*sv), (rx*rz*(1-cv)+ry*sv)],\
                 [(rx*ry*(1-cv)+rz*sv),(ry**2 * (1-cv) + cv),(ry*rz*(1-cv)-rx*sv)],\
                  [(rx*rz*(1-cv)-ry*sv),(ry*rz*(1-cv)+rx*sv),(rz**2 * (1-cv)+cv)]])
        return R_I


class ControlInstructions(object):
    def __init__(self, deltaTime):
        self.mode = 'individual'
        self.trajectory = Trajectory(deltaTime)
        self.velocities = np.zeros(12)
        self.errorVelocities = np.zeros(12)

        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.ifForceControl = 0

        self.maxRelativeNorm = 2 #max distans init value  
        self.targetRelativeNorm = 0.2 # Initial target norm, if 0 collision could happen 


    def getIndividualTargetVelocity(self, k):

        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)

        self.errorVelocities[0:3] = PositionToVelocity(self.translationRightArm,\
            self.targetPosition[0:3], 0.1)

        self.errorVelocities[6:9] = PositionToVelocity(self.translationLeftArm,\
            self.targetPosition[3:6], 0.1)

        self.errorVelocities[3:6]= QuaternionToRotVel(self.rotationRightArm, \
            self.targetOrientation[0:4], 0.2)
        self.errorVelocities[9:12]= QuaternionToRotVel(self.rotationLeftArm, \
            self.targetOrientation[4:8], 0.2)

        self.velocities = self.targetVelocity + k*self.errorVelocities
        return self.velocities

    def getAbsoluteTargetVelocity(self, k):

        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)

        absolutePosition = 0.5*(self.translationRightArm + self.translationLeftArm)
        avgQ = np.vstack([self.rotationRightArm, self.rotationLeftArm])
        absoluteOrientation = averageQuaternions(avgQ) 

        self.errorVelocities[0:3] = PositionToVelocity(absolutePosition,\
            self.targetPosition[0:3], 0.1)
        self.errorVelocities[3:6]= QuaternionToRotVel(absoluteOrientation, \
            self.targetOrientation[0:4], 0.2)

        self.velocities[0:6] = self.targetVelocity[0:6] + k*self.errorVelocities[0:6]

        return self.velocities[0:6]

    def getRelativeTargetVelocity(self, k):
        if len(self.trajectory.trajectory) < 2: 
            return np.zeros(12)

        '''
        relativePosNorm = np.linalg.norm(self.translationRelativeLeftRight)
        print('distance', relativePosNorm)
        force = 0
        if relativePosNorm > 0.2 and self.trajectoryIndex>0:
            self.ifForceControl = 1
            force = (relativePosNorm-0.2)*200
            print('force' , force)
        self.updateForceControl(force=force, maxForce=4, deltaT=0.01, K=0.01)
        '''

        relativePos = self.targetPosition[3:6]
        '''
        print('relativePosOriginal', relativePos)
        if self.ifForceControl == 1:    
            relativePosNorm = np.linalg.norm(relativePos)
            if relativePosNorm > self.maxRelativeNorm: 
                relativePosNormalized = normalize(relativePos)
                relativePos = relativePosNormalized * self.maxRelativeNorm
        print('relativePosAfter', relativePos)
        '''
        self.errorVelocities[6:9] = PositionToVelocity(self.realativPosition ,\
            relativePos, 0.1)

        self.errorVelocities[9:12]= QuaternionToRotVel(self.rotationRelative, \
            self.targetOrientation[4:8], 0.2)

        self.velocities[6:12] = self.targetVelocity[6:12]  + k*self.errorVelocities[6:12]

        return self.velocities[6:12], self.translationRightArm, self.translationLeftArm, self.absoluteOrientation

    def updateTarget(self):
        if len(self.trajectory.trajectory) < 2: 
            return
        self.targetPosition, self.targetOrientation, self.targetVelocity = self.trajectory.getTarget()

    def updateForceControl(self, force, maxForce, deltaT, K):
        #TODO fix problems...
        forceDifferance = maxForce - force
        relativePosNorm = np.linalg.norm(self.trajectory[self.trajectoryIndex].positionLeft)
        if forceDifferance < 0 and self.maxRelativeNorm > relativePosNorm:
            self.maxRelativeNorm = relativePosNorm
        elif force == 0:
            self.maxRelativeNorm = relativePosNorm + 0.01

        forceDifferance = np.clip(forceDifferance, -10, 10) # for safty ... 

        self.maxRelativeNorm = self.maxRelativeNorm  + deltaT * K * forceDifferance
        self.maxRelativeNorm = np.clip(self.maxRelativeNorm, 0.05, 2)
        print('maxNorm', self.maxRelativeNorm, ' diff ', deltaT * K * forceDifferance)


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
        

def PositionToVelocity(currentPositionXYZ, targetPositionXYZ, maxVelocity):
    positionDiff = targetPositionXYZ - currentPositionXYZ
    norm = np.linalg.norm(positionDiff)
    positionDiffNormalized = normalize(positionDiff)        
    return positionDiffNormalized*min([maxVelocity, norm])

def QuaternionToRotVel(currentQ, targetQ, maxRotVel):

    if currentQ.dot(targetQ) < 0:
        currentQ = -currentQ

    skewTarget = np.array([[0, -targetQ[2], targetQ[1]],\
                            [targetQ[2],0,-targetQ[0]],\
                                [-targetQ[1],targetQ[0],0]])

    errorOrientation = currentQ[3]*targetQ[0:3] - targetQ[3]*currentQ[0:3] - skewTarget.dot(currentQ[0:3] )
    norm = np.linalg.norm(errorOrientation)

    errorOrientationNormalized = normalize(errorOrientation)     
    errorOrientationNormalized*min([maxRotVel, norm])   
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


def closest_ang(target, currentRotation):
    rotation = target - currentRotation
    if abs(rotation) > np.pi:
        rotation = rotation - np.sign(rotation)*2*np.pi
    return rotation


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


