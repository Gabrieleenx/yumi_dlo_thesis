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

# change jointPosition to positionLeft, positionRight, orientationLeft, orientationRight
# quaternion as x y z w 
class Trajectory(object):
    def __init__(self, \
            positionLeft=np.array([0.4 ,0.2, 0.2]),\
            positionRight=np.array([0.4 ,-0.2, 0.2]),\
            orientationLeft=np.array([-1,0,0,0]),\
            orientationRight=np.array([-1,0,0,0]),\
            gripperLeft=np.array([0.0, 0.0]),\
            gripperRight=np.array([0.0, 0.0]),\
            absVelocity=0.05,\
            relVelocity=0.05,\
            absRotVelocity=0.3,\
            relRotVelocity=0.3,\
            grippVelocity=0.03,\
            maxRelForce=4,\
            maxAbsForce=4,\
            targetTreshold=0.01):

        self.positionLeft = positionLeft
        self.positionRight = positionRight
        self.orientationLeft = orientationLeft
        self.orientationRight = orientationRight
        self.gripperLeft = gripperLeft
        self.gripperRight = gripperRight
        self.absVelocity = absVelocity
        self.relVelocity = relVelocity
        self.absRotVelocity = absRotVelocity
        self.relRotVelocity = relRotVelocity
        self.grippVelocity = grippVelocity
        self.maxRelForce = maxRelForce
        self.maxAbsForce = maxAbsForce
        self.targetTreshold = targetTreshold


class ControlInstructions(object):
    def __init__(self):
        self.mode = 'individual'
        self.trajectory = []
        self.trajectoryIndex = 0
        self.tfListener = tf.TransformListener()
        self.effectorVelocities = np.zeros(12)

    def getTargetVelocity(self):
        
        if self.mode == 'individual':
            (translationRightArm, rotationRightArm) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
            (translationLeftArm, rotationLeftArm) = self.tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))
        elif self.mode == 'combined':
            # to be implemented
            pass
        else:
            print('Control mode not valid')

        # Right arm position (in realtive mode it's the absolute position)
        self.effectorVelocities[0:3] = PositionToVelocity(np.asarray(translationRightArm),\
             self.trajectory[self.trajectoryIndex].positionRight, self.trajectory[self.trajectoryIndex].absVelocity)
        # Left arm position (in realtive mode it's the relative position)
        self.effectorVelocities[6:9] = PositionToVelocity(np.asarray(translationLeftArm),\
             self.trajectory[self.trajectoryIndex].positionLeft, self.trajectory[self.trajectoryIndex].absVelocity)
        # Right arm orientation (in realtive mode it's the absolute orientation)
        self.effectorVelocities[3:6]= QuaternionToRotVel(np.array(rotationRightArm), \
            self.trajectory[self.trajectoryIndex].orientationRight, self.trajectory[self.trajectoryIndex].absRotVelocity)
        # Left arm orientation (in realtive mode it's the relative orientation)
        self.effectorVelocities[9:12]= QuaternionToRotVel(np.array(rotationLeftArm), \
            self.trajectory[self.trajectoryIndex].orientationLeft, self.trajectory[self.trajectoryIndex].absRotVelocity)
        
        # check target point 
        self.computeTrajectoryIntex()

        return self.effectorVelocities

    def computeTrajectoryIntex(self):
        if self.trajectoryIndex < len(self.trajectory)-1:
            if np.linalg.norm(self.effectorVelocities) <= self.trajectory[self.trajectoryIndex].targetTreshold:
                self.trajectoryIndex += 1

    def getNumTrajectryPoints(self):
        return len(self.trajectory)


def PositionToVelocity(currentPositionXYZ, targetPositionXYZ, maxVelocity):
    positionDiff = targetPositionXYZ - currentPositionXYZ
    norm = np.linalg.norm(positionDiff)
    positionDiffNormalized = normalize(positionDiff)        
    return positionDiffNormalized*min([maxVelocity, norm])

def QuaternionToRotVel(currentQ, targetQ, maxRotVel):
    skewTarget = np.array([[0, -targetQ[2], targetQ[1]],\
                            [targetQ[2],0,-targetQ[0]],\
                                [-targetQ[1],targetQ[0],0]])
    errorOrientation = currentQ[3]*targetQ[0:3] - targetQ[3]*currentQ[0:3] - skewTarget.dot(currentQ[0:3] )
    norm = np.linalg.norm(errorOrientation)
    errorOrientationNormalized = normalize(errorOrientation)        
    return errorOrientationNormalized*min([maxRotVel, 2*norm])


def CalcJacobianCombined(data, tfListener, transformer):
    jacobianRightArm = np.zeros((6,7))
    jacobianLeftArm = np.zeros((6,7))

    dataNP = np.asarray(data.data)

    jacobianRightArm = dataNP[0::2].reshape((6,7))
    jacobianLeftArm = dataNP[1::2].reshape((6,7))
    
    # change endeffector frame 

    (translationRightArm, rotationRightArm) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
    (translationLeftArm, rotationLeftArm) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))

    (gripperLengthRight, _) = tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
    (gripperLengthLeft, _) = tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))

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
