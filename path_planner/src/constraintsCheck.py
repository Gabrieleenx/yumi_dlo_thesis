import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils, utilsSolve


def check(task, logger):

    instruction = 0
    mode = task.mode
    logger.appendPathplannerState(data='Checking trajectory, mode = ' + mode)

    within = utilsSolve.checkTaskWithinReach(task=task)
    if not within:
        print("trajectories not in reach")
        logger.appendPathplannerState(data='Problem detected, trajectories are outside of reach')
        instruction = 1

    if mode == 'individual': # assumption, individual used for picking up cable 

        trajClear = utilsSolve.checkCloseToFixtures(task=task)
        if not trajClear:
            print("trajectories to close to fixture")
            logger.appendPathplannerState(data='Problem detected, trajectories are to close to fixture')
            instruction = 1

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

    elif mode == 'combined':

        armNotCross = utilsSolve.checkIfNotLeftRightArmCross(task=task)
        if not armNotCross:
            print('Crossing combined ')
            logger.appendPathplannerState(data='Problem detected, trajectories cross to far on y axis')
            instruction = 1
        
    return instruction

