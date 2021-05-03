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


    # ---------------
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.2, 0.2]
    trajectoryPoint.positionLeft = [0.35, 0.2, 0.2]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [20,20]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 12.0

    trajectory = [trajectoryPoint]
    
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.1, 0.03]
    trajectoryPoint.positionLeft = [0.35, 0.1, 0.03]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0,0]
    trajectoryPoint.gripperRight = [0,0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    
    # ---------------
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.1, 0.03]
    trajectoryPoint.positionLeft = [0.35, 0.15, 0.03]
    trajectoryPoint.orientationLeft = [1,0,0,0]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [20,20]
    trajectoryPoint.gripperRight = [20,20]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.35, -0.1, 0.03]
    trajectoryPoint.positionLeft = [0.35, 0.15, 0.03]
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
    rospy.sleep(31)

    # --------------------------------------------
    
    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'combined'


     # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.35, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory = [trajectoryPoint]

    # ---------------

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.45, 0.1, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)


    # ---------------
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.45, 0.1, 0.4]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------
    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.35, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------

    msg.trajectory = trajectory

    pub.publish(msg)
    print('sent msg 2, combined ')
    rospy.sleep(33)


    # --------------------------------------------


    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'combined'

     # ---------------
    rot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 210*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory = [trajectoryPoint]

    # ---------------
    rot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 150*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)

    # ---------------
    rot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 180*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------
    rot = tf.transformations.quaternion_from_euler(40*np.pi/180, 0, 180*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------

    rot = tf.transformations.quaternion_from_euler(-40*np.pi/180, 0, 180*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------
    rot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 180*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------

    rot = tf.transformations.quaternion_from_euler(0,40*np.pi/180,  180*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------

    rot = tf.transformations.quaternion_from_euler( 0,-40*np.pi/180, 180*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 16.0

    trajectory.append(trajectoryPoint)
    # ---------------
    rot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 180*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = [0,0,0,1]
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------

    rot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 180*np.pi/180, 'rzyx')
    rotrel = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 60*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = rotrel
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------

    rot = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 180*np.pi/180, 'rzyx')
    rotrel = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 0*np.pi/180, 'rzyx')

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionAbsolute = [0.4, 0.0, 0.2]
    trajectoryPoint.positionRelative = [0, 0.25, 0]
    trajectoryPoint.orientationRelative = rotrel
    trajectoryPoint.orientationAbsolute = rot
    trajectoryPoint.gripperLeft = [0.0,0.0]
    trajectoryPoint.gripperRight = [0.0,0.0]
    trajectoryPoint.pointTime = 8.0

    trajectory.append(trajectoryPoint)
    # ---------------
    msg.trajectory = trajectory

    pub.publish(msg)
    print('sent msg 2, combined ')
    '''

    '''
    rospy.sleep(30)

   
    





    rospy.spin()


if __name__ == '__main__':
    main()

