import numpy as np
import rospy
from controller.msg import Trajectory_point
import tf
import utils


class CheckConstraints(object):
    def __init__(self):
        self.reach = 0.559 - 0.03 #value from https://search.abb.com/library/Download.aspx?DocumentID=9AKK106354A3254&LanguageCode=en&DocumentPartId=&Action=Launch
        # with some margin
        self.minGripperDistance = 0.1
        self.fixtureRadius = 0.06


    def check(self, map_, traj, DLO, mode, tfListener, targetFixture, previousFixture, cableSlack, grippWidth=0.15):
        instruction = 0

        reachLeftCentrum = np.array([0.138, 0.106, 0.462])
        reachRightCentrum = np.array([0.138, -0.106, 0.462])

        (baseToGripperRight, _) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
        (Link7ToGripperRight, _) = tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        baseToGripperRight = np.asarray(baseToGripperRight)
        Link7ToGripperRight = np.asarray(Link7ToGripperRight)

        (baseToGripperLeft, _) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))
        (Link7ToGripperLeft, _) = tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))
        baseToGripperLeft = np.asarray(baseToGripperLeft)
        Link7ToGripperLeft = np.asarray(Link7ToGripperLeft)

        if mode == 'individual': # assumption, individual used for picking up cable 
            clipPoint = utils.calcClipPoint(targetFixture, previousFixture, map_, cableSlack, DLO)
            leftGrippPoint, rightGrippPoint = utils.calcGrippPoints(targetFixture, map_, DLO, grippWidth, clipPoint)
            leftGrippPointCoord = DLO.getCoord(leftGrippPoint)
            rightGrippPointCoord = DLO.getCoord(rightGrippPoint)
            leftGrippPointCoordHigh = leftGrippPointCoord + Link7ToGripperLeft
            rightGrippPointCoordHigh = rightGrippPointCoord + Link7ToGripperRight

            if np.linalg.norm(rightGrippPointCoordHigh - reachRightCentrum) >= self.reach:
                print(' pickup point for right arm is out of reach, dist = ', np.linalg.norm(rightGrippPointCoordHigh - reachRightCentrum))
                instruction = 1
            if np.linalg.norm(leftGrippPointCoordHigh - reachLeftCentrum) >= self.reach:
                print(' pickup point for left arm is out of reach, dist = ',  np.linalg.norm(leftGrippPointCoordHigh - reachLeftCentrum))
                instruction = 1

            if np.linalg.norm(leftGrippPointCoord - rightGrippPointCoord) < self.minGripperDistance:
                print(' Pickup points too close, dist = ', np.linalg.norm(leftGrippPointCoord - rightGrippPointCoord))

            for i in range(len(map_)):
                fixturePos = map_[i].getBasePosition()
                if np.linalg.norm(fixturePos - leftGrippPointCoord) <= self.fixtureRadius:
                    print('pickup point Left to close to fixture, dist= ', np.linalg.norm(fixturePos - leftGrippPointCoord))
                
                if np.linalg.norm(fixturePos - rightGrippPointCoord) <= self.fixtureRadius:
                    print('pickup point Right to close to fixture, dist= ', np.linalg.norm(fixturePos - rightGrippPointCoord))
                
            for i in range(len(traj)):
                if i == 0:
                    pointA0 = baseToGripperRight
                    pointA1 = np.asarray(traj[i].positionRight)
                    pointB0 = baseToGripperLeft
                    pointB1 = np.asarray(traj[i].positionLeft)
                else:
                    pointA0 = np.asarray(traj[i-1].positionRight)
                    pointA1 = np.asarray(traj[i].positionRight)
                    pointB0 = np.asarray(traj[i-1].positionLeft)
                    pointB1 = np.asarray(traj[i].positionLeft)
                closestDist = utils.closestDistLineToLineSegment(pointA0, pointA1, pointB0, pointB1)
                if closestDist <= self.minGripperDistance:
                    print("trajectories pass to close, dist ", closestDist)

                diff = pointB1 - pointA1

                angle = np.arctan2(diff[1], diff[0])

                if angle <= 0 - 20 * np.pi /180:
                    print('Crossing, angle ', np.arctan2(diff[1], diff[0])*180/np.pi)
                elif angle >= np.pi + 20 * np.pi /180:
                    print('Crossing, angle ', np.arctan2(diff[1], diff[0])*180/np.pi)
                    
        elif mode == 'combined':
            for i in range(len(traj)):
                quat = np.asarray(traj[i].orientationRight)
                absoluteEuler = tf.transformations.euler_from_quaternion(quat, 'sxyz')
                if absoluteEuler[2] < -np.pi/2 - 20 * np.pi /180 or absoluteEuler[2] > np.pi/2 + 20 * np.pi /180:
                    print('cross over, angle = ',absoluteEuler[2]*180/np.pi)

        return instruction


        # if individal
        # is pickup point outside of reach, identifyed
        # is pickup point to close to fixture, identified
        # is pikup points to close to each other, identifyed
        # do the trajectories go to close to each other or cross, identifyed


        # if combined
        # do trajectories cross?
        # is any of the effectors out of reach 
            


