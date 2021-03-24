#!/usr/bin/env python3

# based on https://github.com/ritalaezza/sot-myo/blob/akos_re/src/Task.py
import numpy as np
import tf
import rospy

class Task(object):
    """
    Base abstract class for all tasks.
    """
    def __init__(self, Dof, slackRatio=1e4):
        self.Dof = Dof
        self.constraintMatrix = np.array([])
        self.constraintVector = np.array([])
        self.constraintType = None  # 0 for equality, 1 for Gx <= h, -1 for Gx >= h
        self.slackRatio = slackRatio         # Between qdot and w cost

    def ndim(self):
        """
        Returns number of joint variables i.e. degrees of freedom in the robotic structure.
        """
        return self.Dof

    def mdim(self):
        """
        Returns the number of constraint equations defining the task.
        """
        return self.constraintVector.size

    def slack_ratio(self):
        return self.slackRatio

    def append_slack_locked(self, m, w):
        """
        Returns constraintMatrix and constraintVector for previously solved tasks including optimal slack variables,
        thus defining their nullspaces.
        """
        
        if self.constraintType == 0:
            A = np.hstack((self.constraintMatrix, np.zeros((self.mdim(), m))))
            b = self.constraintVector + w
            G = None
            h = None
        elif self.constraintType == 1:
            A = None
            b = None
            G = np.hstack((self.constraintMatrix, np.zeros((self.mdim(), m))))
            h = self.constraintVector + np.maximum(w, 0)
        elif self.constraintType == -1:
            A = None
            b = None
            G = np.hstack((self.constraintMatrix, np.zeros((self.mdim(), m))))
            h = self.constraintVector - np.maximum(w, 0)
           
        return A, b, G, h

    def append_slack(self, m):
        """
        Returns constraintMatrix and constraintVector with m added slack variables, i.e. one for each row of the task.
        """
        if self.constraintType == 0:
            A = np.hstack((self.constraintMatrix, -np.eye(m)))
            b = self.constraintVector
            G = np.zeros((1, A.shape[1]))       # Trivial constraint. Makes adding previous stages easier, and solver needs it in the case of 1 task.
            h = np.zeros((1, ))                 # Does not do anything otherwise.
        elif self.constraintType == 1:
            G = np.hstack((self.constraintMatrix, -np.eye(m)))
            h = self.constraintVector
            A = np.zeros((1, G.shape[1]))       # Trivial constraint. Makes adding previous stages easier, and solver needs it in the case of 1 task.
            b = np.zeros((1, ))                 # Does not do anything otherwise.
        elif self.constraintType == -1:
            G = np.hstack((self.constraintMatrix, np.eye(m)))
            h = self.constraintVector
            A = np.zeros((1, G.shape[1]))       # Trivial constraint. Makes adding previous stages easier, and solver needs it in the case of 1 task.
            b = np.zeros((1, ))                 # Does not do anything otherwise.
        return A, b, G, h
    

class JointPositionBoundsTask(Task):
    def __init__(self, Dof, bounds, timestep, ctype):
        super(JointPositionBoundsTask, self).__init__(Dof, 1e3)
        self.timestep = timestep
        self.bounds = bounds
        self.constraintType = ctype

    def compute(self, jointState):
        if self.constraintType == 1:
            self.constraintMatrix = self.timestep * np.eye(self.ndim())
            self.constraintVector = self.bounds - jointState.jointPosition
        elif self.constraintType == -1:
            self.constraintMatrix = -self.timestep * np.eye(self.ndim())
            self.constraintVector = -self.bounds + jointState.jointPosition  


class JointVelocityBoundsTask(Task):
    def __init__(self, Dof, bounds, ctype):
        super(JointVelocityBoundsTask, self).__init__(Dof, 1e3)
        self.bounds = bounds
        self.constraintType = ctype

    def compute(self):
        if self.constraintType == 1:
            self.constraintMatrix = np.eye(self.ndim())
            self.constraintVector = self.bounds
        elif self.constraintType == -1:
            self.constraintMatrix = -np.eye(self.ndim())
            self.constraintVector = -self.bounds


class IndividualControl(Task):
    def __init__(self, Dof):
        super(IndividualControl, self).__init__(Dof)
        self.constraintType = 0

    def compute(self, controlInstructions, jacobian):
        effectorVelocities = controlInstructions.getIndividualTargetVelocity(k=1) # k is the gain for vel = vel + k*error
        self.constraintMatrix = jacobian
        self.constraintVector = effectorVelocities


class RelativeControl(Task):
    def __init__(self, Dof):
        super(RelativeControl, self).__init__(Dof)
        self.constraintType = 0
    
    def compute(self, controlInstructions, jacobian, transformer):
        velocities, tfRightArm, tfLeftArm, absoluteOrientation = controlInstructions.getRelativeTargetVelocity(k=1) # k is the gain for vel = vel + k*error

        tfMatrix = transformer.fromTranslationRotation(translation=np.array([0,0,0]), rotation=absoluteOrientation)

        rotaionMatrix = np.linalg.pinv(tfMatrix[0:3,0:3])

        diffXYZ = tfRightArm - tfLeftArm
        skewMatrixDiff = np.array([[0,-diffXYZ[2],diffXYZ[1]],\
                                    [diffXYZ[2],0,-diffXYZ[0]],\
                                    [-diffXYZ[1],diffXYZ[0],0]])

        rotationSkew = 0.5*rotaionMatrix.dot(skewMatrixDiff)
        linkJ  = np.asarray(np.bmat([[rotaionMatrix, rotationSkew, -rotaionMatrix, rotationSkew],\
                            [np.zeros((3,3)), rotaionMatrix, np.zeros((3,3)), -rotaionMatrix]]))
        
        relativeJacobian = linkJ.dot(jacobian)
        self.constraintMatrix = np.vstack([relativeJacobian])
        self.constraintVector = np.hstack([velocities])


class AbsoluteControl(Task):
    def __init__(self, Dof):
        super(AbsoluteControl, self).__init__(Dof)
        self.constraintType = 0
    
    def compute(self, controlInstructions, jacobian):
        velocities = controlInstructions.getAbsoluteTargetVelocity(k=1) # k is the gain for vel = vel + k*error

        linkJ = np.hstack([0.5*np.eye(6), 0.5*np.eye(6)])
        absoluteJacobian = linkJ.dot(jacobian)
        self.constraintMatrix = np.vstack([absoluteJacobian])
        self.constraintVector = np.hstack([velocities])


class ElbowCollision(Task):
    def __init__(self, Dof, arm, minDistance, timestep):
        super(ElbowCollision, self).__init__(Dof, 1e3)
        self.arm = arm
        if arm == 'right':
            self.constraintType = 1
        else:            
            self.constraintType = -1
        self.timestep = timestep

        self.minDistance = minDistance

    def compute(self, jacobian, yumiElbowPoseR, yumiElbowPoseL):
       
        translationRightElbow = yumiElbowPoseR.getPosition()
        translationLeftElbow = yumiElbowPoseL.getPosition()

        difference = translationLeftElbow[1] - translationRightElbow[1]  

        self.safetyMargin = 4

        if self.arm == 'right':
            boundPoint = translationLeftElbow[1] - self.minDistance
            jacobianNew = np.zeros((1, self.Dof))

            jacobianNew[0, 0:4] = jacobian[1,0:4]
            self.constraintMatrix = self.timestep*self.safetyMargin * jacobianNew

            self.constraintVector = np.array([(boundPoint - translationRightElbow[1])])

        elif self.arm == 'left':
            boundPoint = translationRightElbow[1] + self.minDistance
            jacobianNew = np.zeros((1, self.Dof))
            jacobianNew[0, 7:11] = jacobian[1,0:4]
            self.constraintMatrix = -self.timestep*self.safetyMargin * jacobianNew
            self.constraintVector = -np.array([(boundPoint - translationLeftElbow[1])])


class JointPositionPotential(Task):
    def __init__(self, Dof, defaultPose, timestep):
        super(JointPositionPotential, self).__init__(Dof, 2e2)
        self.timestep = timestep
        self.defaultPose = defaultPose
        self.constraintType = 0

    def compute(self, jointState):
        
        self.constraintMatrix =  self.timestep * np.eye(self.ndim())
        self.constraintMatrix[0,0] = self.constraintMatrix[0,0]*6
        self.constraintMatrix[7,7] = self.constraintMatrix[7,7]*6
        self.constraintVector = (self.defaultPose - jointState.jointPosition)/150
