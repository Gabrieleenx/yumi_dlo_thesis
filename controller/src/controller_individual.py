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

        self.right_arm_effector_vel = np.array([[0],[0],[0.0],[0.0],[0.0],[0.0]])
        self.right_arm_target = np.array([[0.3],[-0.2],[0.0],[-3.14],[0],[0]])
        self.right_gripper_target = np.array([0.0, 0.0])

        self.left_arm_effector_vel = np.array([[0],[0],[0.0],[0.0],[0.0],[0.0]])
        self.left_arm_target = np.array([[0.3],[0.2],[0.0],[-3.14],[0],[0]])
        self.left_gripper_target = np.array([0.0, 0.0])

        self.gripp_pos_RL = np.array([0.1, 0.4]) # meters from rigth side of cable 

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
        self.state_seq = 1 # 0 nothing, 1 get above cable, 2 open grippers, 3 lower down, 4 close grippers 


    def callback(self, data):
        jacobian_R_arm = np.zeros((6,7))
        jacobian_L_arm = np.zeros((6,7))

        data_np = np.asarray(data.data)
        data_R_arm = data_np[0::2]
        data_L_arm = data_np[1::2]

        jacobian_R_arm = data_R_arm.reshape((6,7))
        jacobian_L_arm = data_L_arm.reshape((6,7))
        
        # change endeffector frame 

        (trans_r, rot_r) = self.tf_listener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
        (trans_l, rot_l) = self.tf_listener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))

        (gripper_r_d, _) = self.tf_listener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        (gripper_l_d, _) = self.tf_listener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))

        jacobian_R_arm = self.change_frame_jacobian(jacobian_R_arm, gripper_r_d, rot_r)
        jacobian_L_arm = self.change_frame_jacobian(jacobian_L_arm, gripper_l_d, rot_l)

        self.dlo_mtx.acquire()

        if self.recive_dlo == 1:
            #self.update_target()
            self.update_vel(trans_r, rot_r, trans_l, rot_l) # -------------------------
            pinv_jac_right_arm = np.linalg.pinv(jacobian_R_arm)
            pinv_jac_left_arm = np.linalg.pinv(jacobian_L_arm)
            #print('right speed ', self.right_arm_effector_vel, 'left speed ', self.left_arm_effector_vel)
            self.joint_pose_dT[0:7] = pinv_jac_right_arm.dot(self.right_arm_effector_vel).reshape(7)
            self.joint_pose_dT[7:14] = pinv_jac_left_arm.dot(self.left_arm_effector_vel).reshape(7)

            self.joint_pose_dT[0:14] = self.joint_pose_dT[0:14].clip(-0.3,0.3)

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

    def update_target(self):

        dist_r = 0
        for i in range(1, self.num_of_points):
            dist_r += self.dist_to_p[i]
            if dist_r > self.gripp_pos_RL[0]:
                point_r = self.data_np[i]
                dy = self.data_np[i+1].y-self.data_np[i-1].y
                dx = self.data_np[i+1].x-self.data_np[i-1].x 
                rot_z_r = np.arctan2(dy,dx)
                break
        dist_l = 0
        for i in range(1, self.num_of_points):
            dist_l += self.dist_to_p[i]
            if dist_l > self.gripp_pos_RL[1]:
                point_l = self.data_np[i]
                dy = self.data_np[i+1].y-self.data_np[i-1].y
                dx = self.data_np[i+1].x-self.data_np[i-1].x 
                rot_z_l = np.arctan2(dy,dx)
                break

        if self.state_seq == 1:
            self.right_arm_target = np.array([[point_r.x],[point_r.y],[point_r.z+0.05],[-3.14],[0],[rot_z_r-np.pi/2]])
            self.left_arm_target = np.array([[point_l.x],[point_l.y],[point_l.z+0.05],[-3.14],[0],[rot_z_l-np.pi/2]])

        elif self.state_seq == 2:
            self.right_gripper_target = np.array([0.025, 0.025])
            self.left_gripper_target = np.array([0.025, 0.025])

        elif self.state_seq == 3:
            self.right_arm_target = np.array([[point_r.x],[point_r.y],[point_r.z-0.01],[-3.14],[0],[rot_z_r-np.pi/2]])
            self.left_arm_target = np.array([[point_l.x],[point_l.y],[point_l.z-0.01],[-3.14],[0],[rot_z_l-np.pi/2]])

        elif self.state_seq == 4:
            self.right_gripper_target = np.array([0.005, 0.005])
            self.left_gripper_target = np.array([0.005, 0.005])


    def update_vel(self, T_r, R_r, T_l, R_l):

        # position
        d_pos_r = self.right_arm_target[0:3].reshape(3) - T_r
        norm_pos_r = np.linalg.norm(d_pos_r)
        d_pos_r = normalize(d_pos_r)        
        self.right_arm_effector_vel[0:3] = d_pos_r.reshape((3,1))*min([self.effector_max_vel, norm_pos_r])

        d_pos_l = self.left_arm_target[0:3].reshape(3) - T_l
        norm_pos_l = np.linalg.norm(d_pos_l)
        d_pos_l = normalize(d_pos_l)        
        self.left_arm_effector_vel[0:3] = d_pos_l.reshape((3,1))*min([self.effector_max_vel, norm_pos_l])

        # rotation
        r_r = R_scipy.from_quat(R_r)
        r_l = R_scipy.from_quat(R_l)

        rot_xyz_r = r_r.as_euler('xyz', degrees=False)
        rot_xyz_l = r_l.as_euler('xyz', degrees=False)

        rot_x_r = closest_ang( self.right_arm_target[3,0], rot_xyz_r[0])
        rot_y_r = closest_ang( self.right_arm_target[4,0], rot_xyz_r[1])
        rot_z_r = closest_ang( self.right_arm_target[5,0], rot_xyz_r[2])

        rot_x_l = closest_ang( self.left_arm_target[3,0], rot_xyz_l[0])
        rot_y_l = closest_ang( self.left_arm_target[4,0], rot_xyz_l[1])
        rot_z_l = closest_ang( self.left_arm_target[5,0], rot_xyz_l[2])

        vec_R = np.array([rot_x_r, rot_y_r, rot_z_r])
        norm_rot_R = np.linalg.norm(vec_R)
        vec_R = normalize(vec_R) 
        self.right_arm_effector_vel[3:6] = vec_R.reshape((3,1))*min([self.effector_max_rot_vel, norm_rot_R])

        vec_L = np.array([rot_x_l, rot_y_l, rot_z_l])
        norm_rot_L = np.linalg.norm(vec_L)
        vec_L = normalize(vec_L) 
        self.left_arm_effector_vel[3:6] = vec_L.reshape((3,1))*min([self.effector_max_rot_vel, norm_rot_L])

        #gripper 
        gripp_r = self.right_gripper_target - self.joint_pose[14:16]
        norm_gripp_r = np.linalg.norm(gripp_r)
        gripp_r = normalize(gripp_r) 
        self.joint_pose_dT[14:16] = gripp_r*min([self.effector_max_gripp_vel, norm_gripp_r])

        gripp_l = self.left_gripper_target - self.joint_pose[16:18]
        norm_gripp_l = np.linalg.norm(gripp_l)
        gripp_l = normalize(gripp_l) 
        self.joint_pose_dT[16:18] = gripp_l*min([self.effector_max_gripp_vel, norm_gripp_l])
        
        # update state
        norm_sum = norm_pos_r +norm_pos_l + norm_rot_R + norm_rot_L

        if self.state_seq == 1:
            if norm_sum < 0.03:
                self.state_seq = 2
        elif self.state_seq == 2 and norm_gripp_r+norm_gripp_l < 0.005:
            self.state_seq = 3
        elif self.state_seq == 3:
            if norm_sum < 0.03:
                self.state_seq = 4
            
                
            
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