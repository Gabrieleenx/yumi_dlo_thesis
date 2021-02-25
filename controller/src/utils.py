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
            orientationLeft=np.array([1,0,0,0]),\
            orientationRight=np.array([1,0,0,0]),\
            gripperLeft=np.array([0.0, 0.0]),\
            gripperRight=np.array([0.0, 0.0]),\
            absVelocity=0.05,\
            relVelocity=0.05,\
            absRotVelocity=0.2,\
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
        self.velocities = np.zeros(12)
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
        self.ifForceControl = 0
        self.maxRelativeNorm = 2
        self.targetRelativeNorm = 0.2 # Initial target norm, if 0 collision could happen 
        #self.oldNorm = None

    def getIndividualTargetVelocity(self):

        self.velocities[0:3] = PositionToVelocity(self.translationRightArm,\
             self.trajectory[self.trajectoryIndex].positionRight, self.trajectory[self.trajectoryIndex].absVelocity)
        self.velocities[6:9] = PositionToVelocity(self.translationLeftArm,\
             self.trajectory[self.trajectoryIndex].positionLeft, self.trajectory[self.trajectoryIndex].absVelocity)
        self.velocities[3:6]= QuaternionToRotVel(self.rotationRightArm, \
            self.trajectory[self.trajectoryIndex].orientationRight, self.trajectory[self.trajectoryIndex].absRotVelocity)
        self.velocities[9:12]= QuaternionToRotVel(self.rotationLeftArm, \
            self.trajectory[self.trajectoryIndex].orientationLeft, self.trajectory[self.trajectoryIndex].absRotVelocity)
        self.computeTrajectoryIntex()

        return self.velocities

    def getAbsoluteTargetVelocity(self):

        absolutePosition = 0.5*(self.translationRightArm + self.translationLeftArm)
        avgQ = np.vstack([self.rotationRightArm, self.rotationLeftArm])
        absoluteOrientation = averageQuaternions(avgQ) 

        self.velocities[0:3] = PositionToVelocity(absolutePosition,\
            self.trajectory[self.trajectoryIndex].positionRight, self.trajectory[self.trajectoryIndex].absVelocity)

        self.velocities[3:6]= QuaternionToRotVel(absoluteOrientation, \
            self.trajectory[self.trajectoryIndex].orientationRight, self.trajectory[self.trajectoryIndex].absRotVelocity)
        return self.velocities[0:6]

    def getRelativeTargetVelocity(self):
        relativePosNorm = np.linalg.norm(self.translationRelativeLeftRight)
        print('distance', relativePosNorm)
        force = 0
        if relativePosNorm > 0.2 and self.trajectoryIndex>0:
            self.ifForceControl = 1
            force = (relativePosNorm-0.2)*200
            print('force' , force)
        self.updateForceControl(force=force, maxForce=4, deltaT=0.01, K=0.01)

        avgQ = np.vstack([self.rotationRightArm, self.rotationLeftArm])
        absoluteOrientation = averageQuaternions(avgQ)  

        transformation1 = self.transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=absoluteOrientation)
        transformationInv1 = np.linalg.pinv(transformation1)
        transformation2 = self.transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=self.rotationLeftArm)

        leftToAbsoluteFrameRot = transformationInv1.dot(transformation2)
        homogeneousLeftRelative = np.hstack([self.translationRelativeLeftRight,1])
        homogeneousAbsouluteRelative = leftToAbsoluteFrameRot.dot(homogeneousLeftRelative)

        relativePos = self.trajectory[self.trajectoryIndex].positionLeft
        print('relativePosOriginal', relativePos)
        if self.ifForceControl == 1:    
            relativePosNorm = np.linalg.norm(relativePos)
            if relativePosNorm > self.maxRelativeNorm: 
                relativePosNormalized = normalize(relativePos)
                relativePos = relativePosNormalized * self.maxRelativeNorm
        print('relativePosAfter', relativePos)

        self.velocities[6:9] = PositionToVelocity(homogeneousAbsouluteRelative[0:3],\
            relativePos, self.trajectory[self.trajectoryIndex].relVelocity)

        self.velocities[9:12]= QuaternionToRotVel(self.rotationRelative, \
            self.trajectory[self.trajectoryIndex].orientationLeft, self.trajectory[self.trajectoryIndex].relRotVelocity)

        return self.velocities[6:12], self.translationRightArm, self.translationLeftArm, absoluteOrientation

    def computeTrajectoryIntex(self):
        if self.trajectoryIndex < len(self.trajectory)-1:
            if np.linalg.norm(self.velocities) <= self.trajectory[self.trajectoryIndex].targetTreshold:
                self.trajectoryIndex += 1
    
    def updateForceControl(self, force, maxForce, deltaT, K):

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
        rotMatrixRight = tfMatrixRight[0:3,0:3]
        rotMatrixLeft = tfMatrixLeft[0:3,0:3]
        tfMatrixLeftInv = np.linalg.pinv(tfMatrixLeft)

        self.translationRelativeLeftRight = tfMatrixLeftInv.dot(np.hstack([self.translationRightArm, 1]))[0:3]
        rotLeftRight = tfMatrixLeftInv.dot(tfMatrixRight)
        self.rotationRelative = tf.transformations.quaternion_from_matrix(rotLeftRight)


    def resetVelocity(self):
        self.velocities = np.zeros(12)

    def getNumTrajectryPoints(self):
        return len(self.trajectory)


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
    errorOrientationNormalized*min([maxRotVel, 2*norm])   
    return errorOrientationNormalized*min([maxRotVel, 8*norm])


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


