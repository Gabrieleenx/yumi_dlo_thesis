#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState



def main():

    # starting ROS node and subscribers
    rospy.init_node('pub_joint_pos', anonymous=True) 
    pub = rospy.Publisher('/joint_states', JointState, queue_size=5)
    rate = rospy.Rate(10) # 10hz

    msg = JointState()
    

    seq = 1
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = seq
        msg.name = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', \
            'yumi_joint_6_r', 'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', \
                'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l', 'gripper_r_joint', 'gripper_r_joint_m',\
                    'gripper_l_joint', 'gripper_l_joint_m']
        msg.position = [-0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
             0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                  0.0, 0.0]
        pub.publish(msg)
        rate.sleep()
        seq += 1


if __name__ == '__main__':
    main()