#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf

class Ymui_contoller(object):
    def __init__(self):
        self.update_rate = 20
        self.dT = 1/self.update_rate
        self.pose = np.array([-0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
             0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                  0.0, 0.0])   
        
        # object that listen to transformation tree. 
        self.tf_listener = tf.TransformListener()

    def update_position(self):
        pass 

    def callback(self, data):
        jacobian_R_arm = np.zeros((6,7))
        jacobian_L_arm = np.zeros((6,7))

        data_np = np.asarray(data.data)
        data_R_arm = data_np[0::2]
        data_L_arm = data_np[1::2]

        jacobian_R_arm = data_R_arm.reshape((6,7))
        jacobian_L_arm = data_L_arm.reshape((6,7))

        (trans, rot) = self.tf_listener.lookupTransform('/world', '/gripper_r_base', rospy.Time(0))
        
        print(np.linalg.pinv(jacobian_R_arm))

def main():

    # starting ROS node and subscribers
    rospy.init_node('pub_joint_pos', anonymous=True) 
    pub = rospy.Publisher('/joint_states', JointState, queue_size=5)

    ymui_contoller = Ymui_contoller()

    rospy.Subscriber("/Jacobian_R_L", Float64MultiArray, ymui_contoller.callback)

    
    rate = rospy.Rate(ymui_contoller.update_rate) 

    msg = JointState()
    

    seq = 1
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = seq
        msg.name = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', \
            'yumi_joint_6_r', 'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', \
                'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l', 'gripper_r_joint', 'gripper_r_joint_m',\
                    'gripper_l_joint', 'gripper_l_joint_m']
        msg.position = ymui_contoller.pose.tolist()
        pub.publish(msg)
        rate.sleep()
        seq += 1


if __name__ == '__main__':
    main()