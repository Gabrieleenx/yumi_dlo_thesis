#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState, PointCloud
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R_scipy
import threading
import time

class Ymui_contoller(object):
    def __init__(self):
        self.update_rate = 20
        self.dT = 1/self.update_rate
        self.joint_pose = np.array([-0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
             0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                  0.0, 0.0, 0.0, 0.0])   
        self.joint_pose_dT = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                  0.0, 0.0])  

        self.right_gripper_target = np.array([0.0, 0.0])
        self.left_gripper_target = np.array([0.0, 0.0])

        self.absolute_v = np.array([[-0.0],[0.0],[0.0],[0.0],[-0.0],[0.0]])
        self.relative_v = np.array([[0.00],[0.0],[0.0],[0.0],[0.0],[0.0]])
        
        self.absolute_target = np.array([[0.45],[-0.3],[0.4],[-3.14],[0],[0]])
        self.relative_target = np.array([[0],[0.3],[0],[0],[0],[0]])      

        #self.gripp_pos_RL = np.array([0.1, 0.4]) # meters from rigth side of cable 

        self.effector_max_vel = 0.05
        self.effector_max_rot_vel = 0.20
        self.effector_max_gripp_vel = 0.03

        # object that listen to transformation tree. 
        self.tf_listener = tf.TransformListener()
        self.transformer_ = tf.TransformerROS(True, rospy.Duration(4.0))

        # mutex
        self.dlo_mtx = threading.Lock()

        # if the SPR algorithm has started
        self.recive_dlo = 0

        # state
        self.state_seq = 0

        # time for demo
        self.last_time = time.time()



    def callback(self, data):
        self.update_vel()
        jacobian_R_arm = np.zeros((6,7))
        jacobian_L_arm = np.zeros((6,7))

        data_np = np.asarray(data.data)
        data_R_arm = data_np[0::2]
        data_L_arm = data_np[1::2]

        jacobian_R_arm = data_R_arm.reshape((6,7))
        jacobian_L_arm = data_L_arm.reshape((6,7))

        (trans_r, rot_r) = self.tf_listener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
        (trans_l, rot_l) = self.tf_listener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))

        (gripper_r_d, _) = self.tf_listener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        (gripper_l_d, _) = self.tf_listener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))

        jacobian_R_arm = self.change_frame_jacobian(jacobian_R_arm, gripper_r_d, rot_r)
        jacobian_L_arm = self.change_frame_jacobian(jacobian_L_arm, gripper_l_d, rot_l)        

        J_LR = np.asarray(np.bmat([[jacobian_R_arm,np.zeros((6,7))],[np.zeros((6,7)),jacobian_L_arm]]))
        
        P_R = np.asarray(trans_r).reshape((3,1))
        P_L = np.asarray(trans_l).reshape((3,1))
        # might need other direction ... 
        R_R = np.asarray(rot_r)
        R_L = np.asarray(rot_l)

        # naiv avg of quaternions 
        #R_A = 0.5*(R_R + R_L)
        #R_A = R_A/np.linalg.norm(R_A)

        Q = np.array([[R_R[3], R_R[0],R_R[1],R_R[2]],[R_L[3], R_L[0],R_L[1],R_L[2]]])
        R_A = averageQuaternions(Q)
        #print('rot r ', rot_r, 'rot_l ', rot_l, 'R_A', R_A)
        #R_A = np.array([[0.9607],[0],[0.277],[0]])
        R_A = np.array([R_A[1],R_A[2],R_A[3],R_A[0]])
        P_A = 0.5*(P_R + P_L)          

        tf_matrix = self.transformer_.fromTranslationRotation(translation=np.array([0,0,0]), rotation=R_A)

        R_B_A = np.linalg.inv(tf_matrix[0:3,0:3])
        #R_B_A = tf_matrix[0:3,0:3]
        vel_xyz = P_R - P_A
        L_R = np.array([[0, vel_xyz[2,0], -vel_xyz[1,0]],[-vel_xyz[2,0],0,vel_xyz[0,0]],[vel_xyz[1,0],-vel_xyz[0,0],0]])
        
        vel_xyz = P_L - P_A
        L_L = np.array([[0, vel_xyz[2,0], -vel_xyz[1,0]],[-vel_xyz[2,0],0,vel_xyz[0,0]],[vel_xyz[1,0],-vel_xyz[0,0],0]])

        R_vel_R = 0.5*R_B_A.dot(L_R)
        R_vel_L = 0.5*R_B_A.dot(L_L)

        link_rel = np.asarray(np.bmat([[R_B_A, R_vel_L-R_vel_R, -R_B_A, R_vel_L-R_vel_R],\
                            [np.zeros((3,3)), R_B_A, np.zeros((3,3)), -R_B_A]]))

        #Link_J =  np.asarray(np.bmat([[0.5*np.eye(6), 0.5*np.eye(6)],\
        #    [-np.eye(6),np.eye(6)]]))

        Link_J =  np.asarray(np.bmat([[0.5*np.eye(6), 0.5*np.eye(6)],\
            [link_rel]]))
        J = Link_J.dot(J_LR)

        J_pinv = np.linalg.pinv(J)
        va_vr = np.vstack((self.absolute_v, self.relative_v))
        self.joint_pose_dT[0:14] = J_pinv.dot(va_vr).reshape(14)
        #self.joint_pose_dT[0:7] = pinv_jac_right_arm.dot(self.right_arm_effector_vel).reshape(7)
        #self.joint_pose_dT[7:14] = pinv_jac_left_arm.dot(self.left_arm_effector_vel).reshape(7)
        scale_ =1.0
        if  np.abs(self.joint_pose_dT[0:14]).max() > 0.5:
            scale_ = np.abs(self.joint_pose_dT[0:14]).max()/0.5
            if scale_ > 5:
                scale_ = 5
        self.joint_pose_dT[0:14] = self.joint_pose_dT[0:14]/scale_
        self.joint_pose_dT[0:14] = self.joint_pose_dT[0:14].clip(-0.5,0.5)
        self.joint_pose = self.joint_pose + self.joint_pose_dT*self.dT

        
    def callback_dlo(self, data):
        self.dlo_mtx.acquire()

        self.data_np = np.asarray(data.points)
        self.num_of_points = np.shape(self.data_np)[0]
        self.dist_to_p = np.zeros(self.num_of_points)

        for i in range(1, self.num_of_points):
            vec = np.array([[self.data_np[i].x-self.data_np[i-1].x], \
                [self.data_np[i].y-self.data_np[i-1].y], \
                    [self.data_np[i].z-self.data_np[i-1].z] ])
            self.dist_to_p[i] = np.linalg.norm(vec)
            
        self.dlo_len = np.sum(self.dist_to_p)
        self.recive_dlo = 1
        self.dlo_mtx.release()

        print('dlo length ', self.dlo_len)

    def change_frame_jacobian(self, jacobian, gripper_d, rot):

        # change end effector for each jacobian 
        tf_matrix = self.transformer_.fromTranslationRotation(translation=np.array([0,0,0]), rotation=rot)
        gripper_d = np.asarray(gripper_d)
        vel_xyz = tf_matrix[0:3,0:3].dot( gripper_d.reshape((3,1)) )
        eye3 = np.eye(3)
        zeros3 = np.zeros((3,3))
        L = np.array([[0, vel_xyz[2,0], -vel_xyz[1,0]],[-vel_xyz[2,0],0,vel_xyz[0,0]],[vel_xyz[1,0],-vel_xyz[0,0],0]])
        j_t = np.asarray(np.bmat([[eye3,L],[zeros3,eye3]]))

        return j_t.dot(jacobian)

    def update_vel(self):

        if time.time() - self.last_time > 3:
            self.state_seq += 1
            self.last_time = time.time()

        if self.state_seq == 1:
            self.absolute_v = np.array([[-0.05],[0.0],[0.0],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 2:
            self.absolute_v = np.array([[0.03],[0.0],[0.0],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 3:
            self.absolute_v = np.array([[0.0],[0.05],[0.0],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 4:
            self.absolute_v = np.array([[0.0],[-0.05],[0.0],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 5:
            self.absolute_v = np.array([[0.0],[-0.0],[0.05],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 6:
            self.absolute_v = np.array([[0.0],[-0.0],[-0.05],[0.0],[-0.0],[0.0]])   
        elif self.state_seq == 7:
            self.absolute_v = np.array([[0.0],[0.0],[0.0],[0.15],[-0.0],[0.0]])
        elif self.state_seq == 8:
            self.absolute_v = np.array([[0.0],[0.0],[0.0],[-0.15],[-0.0],[0.0]])
        elif self.state_seq == 9:
            self.absolute_v = np.array([[0.0],[-0.0],[0.0],[0.0],[0.15],[0.0]])
        elif self.state_seq == 10:
            self.absolute_v = np.array([[0.0],[-0.0],[0.0],[0.0],[-0.15],[0.0]])
        elif self.state_seq == 11:
            self.absolute_v = np.array([[0.0],[-0.0],[-0.0],[0.0],[-0.0],[0.15]])
        elif self.state_seq == 12:
            self.absolute_v = np.array([[0.0],[-0.0],[-0.0],[0.0],[-0.0],[-0.15]])

        elif self.state_seq == 13:
            self.absolute_v = np.array([[0.0],[-0.0],[-0.0],[0.0],[-0.0],[-0.0]])

            self.relative_v = np.array([[-0.03],[0.0],[0.0],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 14:
            self.relative_v = np.array([[0.03],[0.0],[0.0],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 15:
            self.relative_v = np.array([[0.0],[0.03],[0.0],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 16:
            self.relative_v = np.array([[0.0],[-0.03],[0.0],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 17:
            self.relative_v = np.array([[0.0],[-0.0],[0.03],[0.0],[-0.0],[0.0]])
        elif self.state_seq == 18:
            self.relative_v = np.array([[0.0],[-0.0],[-0.03],[0.0],[-0.0],[0.0]])   
        elif self.state_seq == 19:
            self.relative_v = np.array([[0.0],[0.0],[0.0],[0.15],[-0.0],[0.0]])
        elif self.state_seq == 20:
            self.relative_v = np.array([[0.0],[0.0],[0.0],[-0.15],[-0.0],[0.0]])
        elif self.state_seq == 21:
            self.relative_v = np.array([[0.0],[-0.0],[0.0],[0.0],[0.15],[0.0]])
        elif self.state_seq == 22:
            self.relative_v = np.array([[0.0],[-0.0],[0.0],[0.0],[-0.15],[0.0]])
        elif self.state_seq == 23:
            self.relative_v = np.array([[0.0],[-0.0],[-0.0],[0.0],[-0.0],[0.2]])
        elif self.state_seq == 24:
            self.relative_v = np.array([[0.0],[-0.0],[-0.0],[0.0],[-0.0],[-0.2]])

        elif self.state_seq == 25:
            self.relative_v = np.array([[0.0],[-0.0],[-0.0],[0.0],[-0.0],[-0.0]])




def closest_ang(target, current_rot):
    rot = target - current_rot
    if abs(rot) > np.pi:
        rot = rot - np.sign(rot)*2*np.pi
    return rot

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

# taken from https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py
# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = np.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = np.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = np.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return np.real(eigenVectors[:,0])

def main():

    # starting ROS node and subscribers
    rospy.init_node('pub_joint_pos', anonymous=True) 
    pub = rospy.Publisher('/joint_states', JointState, queue_size=5)

    ymui_contoller = Ymui_contoller()
    rospy.sleep(0.5)

    rospy.Subscriber("/Jacobian_R_L", Float64MultiArray, ymui_contoller.callback)
    rospy.Subscriber("/spr/dlo_estimation", PointCloud, ymui_contoller.callback_dlo)
    
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