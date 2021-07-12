#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg
import tf
import numpy as np
import utils


def main():
    # initilize ros node
    rospy.init_node('CheckFixturePostition', anonymous=True) 
    tfListener = tf.TransformListener()
    # sleep for listener to have time to pick up the transformes 
    rospy.sleep(0.5)
    

    # add existing fixtures in list
    fixtureList = ['/Fixture1','/Fixture2','/Fixture3','/Fixture4']
    listOfObjects = []
    for i in range(len(fixtureList)):
        try:
            (pos, rot) = tfListener.lookupTransform('/yumi_base_link', fixtureList[i], rospy.Time(0))
            pos = np.asarray(pos)
            rot = np.asarray(rot)
            obj = utils.FixtureObject(position=pos,\
                                        orientation=rot,\
                                        fixtureHeight=0.06,\
                                        fixtureRadius=0.06)
            listOfObjects.extend([obj])

        except:
            break

    pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
    rospy.sleep(0.1)

    # --------------------------------------------------
    
    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'individual'
    trajectory = []

    for obj in listOfObjects:
        pos = obj.getBasePosition()
        trajectoryPoint = Trajectory_point()
        trajectoryPoint.positionRight = [pos[0], pos[1], 0.08]
        trajectoryPoint.positionLeft = [0.05, 0.3, 0.15]
        trajectoryPoint.orientationLeft = [1,0,0,0]
        trajectoryPoint.orientationRight = [1,0,0,0]
        trajectoryPoint.gripperLeft = [0,0]
        trajectoryPoint.gripperRight = [0.0,0.0]
        trajectoryPoint.pointTime = 8.0

        trajectory.append(trajectoryPoint)
        # add same point twice for the gripper to stop above the fixture 
        trajectory.append(trajectoryPoint)

    pos = obj.getBasePosition()
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.3, -0.3, 0.1]
    trajectoryPoint.positionLeft = [0.3, 0.3, 0.1]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0
    trajectory.append(trajectoryPoint)

    for obj in listOfObjects:
        pos = obj.getBasePosition()
        trajectoryPoint = Trajectory_point()
        trajectoryPoint.positionRight = [0.05, -0.3, 0.15]
        trajectoryPoint.positionLeft = [pos[0], pos[1], 0.08]
        trajectoryPoint.orientationLeft = [1,0,0,0]
        trajectoryPoint.orientationRight = [1,0,0,0]
        trajectoryPoint.gripperLeft = [0,0]
        trajectoryPoint.gripperRight = [0.0,0.0]
        trajectoryPoint.pointTime = 8.0

        trajectory.append(trajectoryPoint)
        # add same point twice for the gripper to stop above the fixture 
        trajectory.append(trajectoryPoint)

    pos = obj.getBasePosition()
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.2, -0.3, 0.1]
    trajectoryPoint.positionLeft = [0.2, 0.3, 0.1]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0


    trajectory.append(trajectoryPoint)

    msg.trajectory = trajectory
    
    pub.publish(msg)
    rospy.sleep(0.1)
    pub.publish(msg)
    rospy.sleep(0.1)

    print('sent individual' )
    rospy.spin()


if __name__ == '__main__':
    main()
