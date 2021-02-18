#!/usr/bin/env python3

import numpy as np
import rospy

class JointState(object):
    def __init__(self,\
            jointPosition=np.array([-0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),\
            jointVelocity=np.zeros(14),\
            gripperRightPosition=np.zeros(2),\
            gripperLeftPosition=np.zeros(2),\
            gripperRightVelocity=np.zeros(2),\
            gripperLeftVelocity=np.zeros(2)):

        self.jointPosition = jointPosition
        self.jointVelocity = jointVelocity
        self.gripperRightPosition = gripperRightPosition
        self.gripperLeftPosition = gripperLeftPosition
        self.gripperRightVelocity = gripperRightVelocity
        self.gripperLeftVelocity = gripperLeftVelocity
    
    def GetJointVelocity(self):
        return np.hstack([self.armJointVelocity, \
            self.gripperRightVelocity, self.gripperLeftVelocity])
    
    def GetJointPosition(self):
        return np.hstack([self.armJointPosition, \
            self.gripperRightPosition, self.gripperLeftPosition])

    def UpdatePose(self, pose):
        self.jointPosition = pose[0:14]
        self.gripperRightPosition = pose[14:16]
        self.gripperLeftPosition = pose[16:18]


class Trajectory(object):
    def __init__(self, \
            jointPosition=np.array([[0.3],[-0.2],[0.0],[-3.14],[0],[0], [0.3],[0.2],[0.0],[-3.14],[0],[0]]),\
            gripperLeft=np.array([0.0, 0.0]),\
            gripperRight=np.array([0.0, 0.0]),\
            absVelocity=0.05,\
            relVelocity=0.05,\
            absRotVelocity=0.2,\
            relRotVelocity=0,2,\
            grippVelocity=0.03,\
            maxRelForce=4,\
            maxAbsForce=4,\
            targetTreshold=0.01):

        self.jointPosition = jointPosition
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.absVelocity = absVelocity
        self.relVelocity = relVelocity
        self.absRotVelocity = absRotVelocity
        self.relRotVelocity = relRotVelocit
        self.grippVelocity = grippVelocity
        self.maxRelForce = maxRelForce
        self.maxAbsForce = maxAbsForce
        self.targetTreshold = targetTreshold


class ControlInstructions(object):
    def __init__(self):
        self.mode = 'individual'
        self.trajectory = []
        self.trajectoryIndex = 0

    def getTargetVelocity(self):
        pass

    def computeTrajectoryIntex(self):
        pass

    def getNumTrajectryPoints(self):
        return len(self.trajectory)


def CalcJacobianCombined(data, tfListener):
    jacobianRightArm = np.zeros((6,7))
    jacobianLeftArm = np.zeros((6,7))

    dataNP = np.asarray(data.data)

    jacobianRightArm = dataNP[0::2].reshape((6,7))
    jacobianLeftArm = dataNP[1::2].reshape((6,7))
    
    # change endeffector frame 

    (translationRightArm, rotationRightArm) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
    (translationLeftArm, rotationLeftArm) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))

    (gripperLengthRight, _) = self.tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
    (gripperLengthLeft, _) = self.tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))

    jacobianRightArm = self.changeFrameJacobian(jacobianRightArm, gripperLengthRight, rotationRightArm)
    jacobianLeftArm = self.changeFrameJacobian(jacobianLeftArm, gripperLengthLeft, rotationLeftArm)
    
    return np.asarray(np.bmat([[jacobianRightArm,np.zeros((6,7))],[np.zeros((6,7)),jacobianLeftArm]]))


def changeFrameJacobian(jacobian, gripperLenght, rotation):
    # change end effector for jacobian 
    transformationMatrix = self.transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=rotation)
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


# taken from https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
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
    return np.real(eigenVectors[:,0])
