#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg


def main():

    # starting ROS node and subscribers
    rospy.init_node('trajectory_test', anonymous=True) 
    pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
    
    # --------------------------------------------------

    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'individual'
    msg.forceControl = 0
    msg.maxForce = 4.0

    # ---------------
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.3, -0.2, 0.2]
    trajectoryPoint.positionLeft = [0.3, 0.2, 0.2]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.02,0.02]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 4.0

    trajectory = [trajectoryPoint]
    
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.3, -0.1, 0.2]
    trajectoryPoint.positionLeft = [0.3, 0.15, 0.0]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.02,0.02]
    trajectoryPoint.gripperRight = [0.02,0.02]
    trajectoryPoint.pointTime = 3.0

    trajectory.append(trajectoryPoint)
    # ---------------
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.3, -0.1, 0.0]
    trajectoryPoint.positionLeft = [0.3, 0.15, 0.0]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.02,0.02]
    trajectoryPoint.gripperRight = [0.02,0.02]
    trajectoryPoint.pointTime = 1.0

    trajectory.append(trajectoryPoint)
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.3, -0.1, 0.0]
    trajectoryPoint.positionLeft = [0.3, 0.15, 0.0]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 2.0

    trajectory.append(trajectoryPoint)
    # ---------------
    msg.trajectory = trajectory
    pub.publish(msg)
    rospy.sleep(0.1)
    pub.publish(msg)

    print('sent individual' )
    rospy.sleep(12)

    # --------------------------------------------
    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'combined'
    msg.forceControl = 1
    msg.maxForce = 4.0

     # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.3, 0.0, 0.2]
    trajectoryPoint.positionLeft = [0, 0.25, 0]
    trajectoryPoint.orientationLeft = [0,0,0,1]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 4.0

    trajectory = [trajectoryPoint]

    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.4, 0.1, 0.2]
    trajectoryPoint.positionLeft = [0, 0.25, 0]
    trajectoryPoint.orientationLeft = [0,0,0,1]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 4.0

    trajectory.append(trajectoryPoint)


    # ---------------
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.4, 0.1, 0.4]
    trajectoryPoint.positionLeft = [0, 0.35, 0]
    trajectoryPoint.orientationLeft = [0,0,0,1]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 4.0

    trajectory.append(trajectoryPoint)
    # ---------------

    msg.trajectory = trajectory

    pub.publish(msg)



   
    print('sent')



   






    rospy.spin()


if __name__ == '__main__':
    main()

