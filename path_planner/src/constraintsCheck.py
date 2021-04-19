import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils


class checkConstraints(object):
    def __init__(self):
        pass

    def check(self, input_):
        map_ = input_[0]
        traj = input_[1]
        DLO = input_[2]
        mode = input_[3]

        if mode == 'individual':
            clipPoint = utils.calcClipPoint(targetFixture, previousFixture, map_, cableSlack, DLO)
            leftGrippPoint, rightGrippPoint = utils.calcGrippPoints(targetFixture, map_, DLO, grippWidth, clipPoint)


        for i in range(len(traj)):
            pass

            # if individal
            # is pickup point outside of reach
            # is pickup point to close to fixture
            # is pikup points to close to each other
            # do the trajectories go to close to each other or cross


            # if combined
            # do trajectories cross?
            # is any of the effectors out of reach 
             


        for i in range(len(traj-1)) 


