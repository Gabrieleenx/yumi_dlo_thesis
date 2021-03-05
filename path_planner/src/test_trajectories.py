#!/usr/bin/env python3

import rospy
from controller.msg import Trajectory_point, Trajectory_msg


def main():

    # starting ROS node and subscribers
    rospy.init_node('trajectory_test', anonymous=True) 
    pub = rospy.Publisher('/Trajectroy', Trajectory_msg, queue_size=1)
    msg = Trajectory_msg()
    msg.header.stamp = rospy.Time.now()
    msg.mode = 'combined'

    trajectoryPoint = Trajectory_point()
    trajectoryPoint.positionRight = [0.3, 0.0, 0.2]
    trajectoryPoint.positionLeft = [0, 0.3, 0]
    trajectoryPoint.orientationLeft = [0,0,0,1]
    trajectoryPoint.orientationRight = [1,0,0,0]
    trajectoryPoint.gripperLeft = [0.01,0.01]
    trajectoryPoint.gripperRight = [0.01,0.01]
    trajectoryPoint.pointTime = 3.0

    msg.trajectory = [trajectoryPoint]
    msg.forceControl = 0

    rate = rospy.Rate(2) 
    '''
    while not rospy.is_shutdown():
        print('sent')
        pub.publish(msg)
        rate.sleep()
    '''
    print('sent')
    pub.publish(msg)
    rate.sleep()
    print('sent')
    pub.publish(msg)
    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()

