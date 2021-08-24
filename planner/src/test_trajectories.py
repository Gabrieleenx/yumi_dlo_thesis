#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg
import tf
import numpy as np

def main():

    # starting ROS node and subscribers
    rospy.init_node('trajectory_test', anonymous=True) 
    pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
    rospy.sleep(0.1)

    # --------------------------------------------------
    
    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'individual'
    trajectoryPoint = Trajectory_point()
    trajectory = []
    

    # ---------------
    trajectoryPoint.positionRight = [0.35, -0.25, 0.05]
    trajectoryPoint.positionLeft = [0.35, 0.25, 0.05]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 8.0

    trajectory = [trajectoryPoint]
    
    # ---------------
    rotR = tf.transformations.quaternion_from_euler(0*np.pi/180, -0*np.pi/180, 180*np.pi/180, 'rzyx')
    rotL = tf.transformations.quaternion_from_euler(0*np.pi/180, -0*np.pi/180, 180*np.pi/180, 'rzyx')


    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.25, 0.015]
    trajectoryPoint.positionLeft = [0.35, 0.25, 0.015]
    trajectoryPoint.orientationLeft = rotL
    trajectoryPoint.orientationRight = rotR
    trajectoryPoint.gripperLeft = 0.0
    trajectoryPoint.gripperRight = 0.0
    trajectoryPoint.pointTime = 3.0

    trajectory.append(trajectoryPoint)
    '''
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.2, 0.002]
    trajectoryPoint.positionLeft = [0.05, 0.3, 0.1]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
 
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, 0.2, 0.002]
    trajectoryPoint.positionLeft = [0.05, 0.3, 0.1]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 20.0

    trajectory.append(trajectoryPoint)
 
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.2, 0.002]
    trajectoryPoint.positionLeft = [0.05, 0.3, 0.1]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 20.0

    trajectory.append(trajectoryPoint)
 

        # ---------------
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.3, 0.2]
    trajectoryPoint.positionLeft = [0.35, 0.3, 0.2]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [20.0,20.0]
    trajectoryPoint.gripperRight = [20.0,20.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    

    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.3, 0.2]
    trajectoryPoint.positionLeft = [0.35, 0.2, 0.002]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
 
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.05, -0.3, 0.2]
    trajectoryPoint.positionLeft = [0.35, -0.2, 0.002]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 20.0

    trajectory.append(trajectoryPoint)
 
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.3, 0.2]
    trajectoryPoint.positionLeft = [0.35, 0.2, 0.002]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 20.0

    trajectory.append(trajectoryPoint)

        # ---------------
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.3, 0.2]
    trajectoryPoint.positionLeft = [0.35, 0.3, 0.2]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [20.0,20.0]
    trajectoryPoint.gripperRight = [20.0,20.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    '''
    msg.trajectory = trajectory
    
    pub.publish(msg)
    rospy.sleep(0.1)
    pub.publish(msg)
    rospy.sleep(0.1)

    
    print('sent individual' )

  

    '''

    rot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 180*np.pi/180, 'rzyx')
    rotrel = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 0*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = rotrel
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 5.0

    trajectory.append(trajectoryPoint)
    # ---------------
    msg.trajectory = trajectory

    pub.publish(msg)
    print('sent msg 2, combined ')
    '''

    



    rospy.spin()


if __name__ == '__main__':
    main()

