#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState, PointCloud
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R_scipy
import threading

class Ymui_contoller(object):
    def __init__(self):
        self.update_rate = 20
        self.dT = 1/self.update_rate
        self.joint_pose = np.array([-0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
             0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                  0.0, 0.0, 0.0, 0.0])   
        self.joint_pose_dT = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,\
                  0.0, 0.0])  

        self.right_arm_V = np.array([[0],[0],[0.0],[0.0],[0.0],[0.0]])

        self.right_arm_target = np.array([[0.45],[-0.3],[0.4],[-3.14],[0],[0]])

        self.right_gripper_target = np.array([0.0, 0.0])


        self.right_arm_gripp = 0.1 # meters from rigth side of cable 

        # object that listen to transformation tree. 
        self.tf_listener = tf.TransformListener()
        self.transformer_ = tf.TransformerROS(True, rospy.Duration(4.0))

        # mutex
        self.dlo_mtx = threading.Lock()

        # if the SPR algorithm has started
        self.recive_dlo = 0

        # state
        self.state_seq = 1 # 0 nothing, 1 get above cable, 2 open grippers, 3 lower down, 4 close grippers 


    def callback(self, data):
        jacobian_R_arm = np.zeros((6,7))
        jacobian_L_arm = np.zeros((6,7))

        data_np = np.asarray(data.data)
        data_R_arm = data_np[0::2]
        data_L_arm = data_np[1::2]

        jacobian_R_arm = data_R_arm.reshape((6,7))
        jacobian_L_arm = data_L_arm.reshape((6,7))

        self.dlo_mtx.acquire()


        if self.recive_dlo == 1:
            (trans, rot) = self.tf_listener.lookupTransform('/yumi_base_link', '/yumi_link_7_r', rospy.Time(0))
            self.update_target()
            self.update_vel(trans, rot)
            pinv_jac_right_arm = np.linalg.pinv(jacobian_R_arm)

            self.joint_pose_dT[0:7] = pinv_jac_right_arm.dot(self.right_arm_V).reshape(7)
            print('vel ', self.joint_pose_dT[0:7])
            self.joint_pose_dT[0:7] = self.joint_pose_dT[0:7].clip(-0.3,0.3)

            self.joint_pose = self.joint_pose + self.joint_pose_dT*self.dT

        self.dlo_mtx.release()

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

    def update_target(self):

        self.right_arm_gripp
        dist_r = 0
        for i in range(1, self.num_of_points):
            dist_r += self.dist_to_p[i]
            if dist_r > self.right_arm_gripp:
                point_r = self.data_np[i]
                dy = self.data_np[i+1].y-self.data_np[i-1].y
                dx = self.data_np[i+1].x-self.data_np[i-1].x 
                rot_z = np.arctan2(dy,dx)
                break
        if self.state_seq == 1:
            self.right_arm_target = np.array([[point_r.x],[point_r.y],[point_r.z+0.2],[-3.14],[0],[rot_z-np.pi/2]])
        elif self.state_seq == 2:
            self.right_gripper_target = np.array([0.025, 0.025])
        elif self.state_seq == 3:
            self.right_arm_target = np.array([[point_r.x],[point_r.y],[point_r.z+0.13],[-3.14],[0],[rot_z-np.pi/2]])
        elif self.state_seq == 4:
            self.right_gripper_target = np.array([0.005, 0.005])
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

        #gripper 
        vec_G = self.right_gripper_target - self.joint_pose[14:16]
        norm_G = np.linalg.norm(vec_G)
        vec_G = normalize(vec_G) 
        vec_G = vec_G*min([0.03, norm_G])
        self.joint_pose_dT[14:16] = vec_G

        if self.state_seq == 1:
            if norm_T < 0.01:
                self.state_seq = 2
        elif self.state_seq == 2 and norm_G < 0.002:
            self.state_seq = 3
        elif self.state_seq == 3:
            if norm_T < 0.01:
                self.state_seq = 4
            
                
            



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