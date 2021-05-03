import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils, utilsSolve


def check(task):
    instruction = 0
    mode = task.mode
    within = utilsSolve.checkTaskWithinReach(task=task)
    if not within:
        print("trajectories not in reach")
        instruction = 1

    if mode == 'individual': # assumption, individual used for picking up cable 

        trajClear = utilsSolve.checkCloseToFixtures(task=task)
        if not trajClear:
            print("trajectories to close to fixture")
            instruction = 1

        trajClear = utilsSolve.checkIfTrajectoriesPassToClose(task=task)
        if not trajClear:
            print("trajectories pass to close ")
            instruction = 1
        
        armNotCross = utilsSolve.checkIfNotLeftRightArmCross(task=task)
        if not armNotCross:
            print('Crossing individual ')
            instruction = 1

        overRotation = utilsSolve.checkOverRotation(task=task)
        if not overRotation:
            print('Overrotation individual ')
            instruction = 3

    elif mode == 'combined':

        armNotCross = utilsSolve.checkIfNotLeftRightArmCross(task=task)
        if not armNotCross:
            print('Crossing combined ')
            instruction = 1
        
    return instruction

