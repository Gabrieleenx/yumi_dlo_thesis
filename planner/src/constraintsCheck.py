import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils, utilsSolve, collisionCheck

class Evaluate(object):
    def __init__(self, collisionCheckObj):
        
        self.objectsR, self.objectsL = collisionCheck.calcGripperObjects()

        self.collisionCheckObj = collisionCheckObj


    def check(self, task, logger):
        instruction = 0
        mode = task.mode
        logger.appendPathplannerState(data='Checking trajectory, mode = ' + mode)

        within = utilsSolve.checkTaskWithinReach(task=task)
        if not within:
            print("trajectories not in reach")
            logger.appendPathplannerState(data='Problem detected, trajectories are outside of reach')
            instruction = 1

        if mode == 'individual': # assumption, individual used for picking up cable 
            if self.checkCollision(task):
                print(" collision predicted")
                logger.appendPathplannerState(data='Problem detected, collision predicted')
                instruction = 1
            '''
            trajClear = utilsSolve.checkCloseToFixtures(task=task)
            if not trajClear:
                print("trajectories to close to fixture")
                logger.appendPathplannerState(data='Problem detected, trajectories are to close to fixture')
                instruction = 1
            '''
            trajClear = utilsSolve.checkIfTrajectoriesPassToClose(task=task)
            if not trajClear:
                print("trajectories pass to close ")
                logger.appendPathplannerState(data='Problem detected, trajectories pass to close to each other')
                instruction = 1
            
            armNotCross = utilsSolve.checkIfNotLeftRightArmCross(task=task)
            if not armNotCross:
                print('Crossing individual ')
                logger.appendPathplannerState(data='Problem detected, trajectories cross to far on y axis')
                instruction = 1

            overRotation = utilsSolve.checkOverRotation(task=task)
            if not overRotation:
                print('Overrotation individual ')
                logger.appendPathplannerState(data='Problem detected, sqeuence of orientations are outside of limits')
                instruction = 3

        elif mode == 'coordinated':

            armNotCross = utilsSolve.checkIfNotLeftRightArmCross(task=task)
            if not armNotCross:
                print('Crossing combined ')
                logger.appendPathplannerState(data='Problem detected, trajectories cross to far on y axis')
                instruction = 1

            if self.checkCollision(task):
                print(" collision predicted")
                logger.appendPathplannerState(data='Problem detected, collision predicted')
                instruction = 2    

        if instruction == 0:
            logger.appendPathplannerState(data='Trajectory good' + mode)

        return instruction

    def checkCollision(self, task):
        traj = task.trajectory
        
        for j in range(len(traj)):
            if task.mode == 'individual':
                leftPos = np.asarray(traj[j].positionLeft)
                rightPos = np.asarray(traj[j].positionRight)

                leftQuat = np.asarray(traj[j].orientationLeft)
                rightQuat = np.asarray(traj[j].orientationRight)
            else:
                absolutePos = np.asarray(traj[j].positionAbsolute)
                relativePos = np.asarray(traj[j].positionRelative)

                absoluteQuat = np.asarray(traj[j].orientationAbsolute)
                relativeQuat = np.asarray(traj[j].orientationRelative)

                rightPos, leftPos = utilsSolve.calcAbsolutToIndividualPos(posAbsolut=absolutePos,\
                                            quatAbsolute=absoluteQuat, posRelative=relativePos)
                leftQuat = absoluteQuat
                rightQuat = leftQuat

            self.updateGrippers(leftPos, leftQuat, rightPos, rightQuat)

            if self.collisionCheckObj.checkCollision():
                return True

        return False
        
    def updateGrippers(self, leftPos, leftQuat, rightPos, rightQuat):
        for i in range(len(self.objectsR)):
            self.objectsR[i].updatePose(rightPos, rightQuat)
        for i in range(len(self.objectsL)):
            self.objectsL[i].updatePose(leftPos, leftQuat)
        objects = []
        objects.extend(self.objectsR)
        objects.extend(self.objectsL)

        self.collisionCheckObj.updateDynamicObjects(objects)

