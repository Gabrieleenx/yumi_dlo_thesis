#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R_scipy

class Ymui_contoller(object):
    def __init__(self):
        self.update_rate = 20
        self.dT = 1/self.update_rate
        self.joint_pose = np.array([-0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
             0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                  0.0, 0.0])   
        self.joint_pose_dT = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                  0.0, 0.0])  

        self.right_arm_V = np.array([[0],[0],[0.0],[0.0],[0.0],[0.0]])

        self.right_arm_target = np.array([[0.3],[0],[0.2],[-3.14],[0],[0]])

        # object that listen to transformation tree. 
        self.tf_listener = tf.TransformListener()
        self.transformer_ = tf.TransformerROS(True, rospy.Duration(1.0))

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

        (trans, rot) = self.tf_listener.lookupTransform('/yumi_base_link', '/yumi_link_7_r', rospy.Time(0))
        self.update_vel(trans, rot)
        pinv_jac_right_arm = np.linalg.pinv(jacobian_R_arm)

        self.joint_pose_dT[0:7] = pinv_jac_right_arm.dot(self.right_arm_V).reshape(7)
        print('vel ', self.joint_pose_dT[0:7])
        self.joint_pose_dT[0:7] = self.joint_pose_dT[0:7].clip(-0.3,0.3)

        self.joint_pose[0:7] = self.joint_pose[0:7] + self.joint_pose_dT[0:7]*self.dT

    def update_vel(self, T_r, R_r):

        # position
        vec_T = self.right_arm_target[0:3].reshape(3) - T_r
        norm_T = np.linalg.norm(vec_T)
        vec_T = normalize(vec_T)        
        new_v = vec_T.reshape((3,1))*min([0.05, norm_T])

        # rotation
        r = R_scipy.from_quat(R_r)
        rot_C = r.as_euler('xyz', degrees=False)
        
        rot_x = self.right_arm_target[3,0] - rot_C[0]
        if abs(rot_x) > np.pi:
            rot_x = rot_x - np.sign(rot_x)*2*np.pi


        rot_y = self.right_arm_target[4,0] - rot_C[1]
        if abs(rot_y) > np.pi:
            rot_y = rot_y - np.sign(rot_y)*2*np.pi
        rot_z = self.right_arm_target[5,0] - rot_C[2]
        if abs(rot_z) > np.pi:
            rot_z = rot_z - np.sign(rot_z)*2*np.pi


        vec_R = np.array([rot_x, rot_y, rot_z])
        norm_R = np.linalg.norm(vec_R)
        vec_R = normalize(vec_R) 
        vec_R = vec_R.reshape((3,1))*min([0.15, norm_R])


        self.right_arm_V[0:3] = new_v
        self.right_arm_V[3:6] = vec_R


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm




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
        msg.position = ymui_contoller.joint_pose.tolist()
        pub.publish(msg)
        rate.sleep()
        seq += 1


if __name__ == '__main__':
    main()