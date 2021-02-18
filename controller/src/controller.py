#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState, PointCloud
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R_scipy
import threading

import HQPSolver
import utils 

class YmuiContoller(object):
    def __init__(self):

        self.updateRate = 20 #Hz
        self.dT = 1/self.updateRate

        self.jointState = utils.JointState()

        self.maxJointVelocity = 0.5
        self.maxScaleJointVelocity = 5

        self.controlArmVelocity = mp.zeros((14,1))
        
        trajectory = utils.Trajectory()
        
        self.controlInstructions = utils.ControlInstructions()
        self.controlInstructions.trajectory = trajectory

        # object that listen to transformation tree. 
        self.tfListener = tf.TransformListener()
        self.transformer = tf.TransformerROS(True, rospy.Duration(4.0))

        # solver 
        self.HQP = HQPSolver.HQPSolver()

        # Task objects 

        
    def callback(self, data):

        jacobianCombined = utils.CalcJacobianCombined(data=data, tfListener=self.tfListener)
        
        # ----------------------
        # stack of tasks, in decending hierarchy
        SoT = []
        SoT.append()
         

        # ----------------------

        self.jointState.jointVelocity = self.HQP.solve(SoT=SoT)
        # -------------------------------------------------
        pose = self.jointState.GetJointPosition + self.jointState.GetJointVelocity*self.dT
        self.jointState.UpdatePose(pose=pose)




def main():

    # starting ROS node and subscribers
    rospy.init_node('pub_joint_pos', anonymous=True) 
    pub = rospy.Publisher('/joint_states', JointState, queue_size=5)

    ymuiContoller = YmuiContoller()
    rospy.sleep(0.5)

    rospy.Subscriber("/Jacobian_R_L", Float64MultiArray, ymuiContoller.callback)
    rospy.Subscriber("/spr/dlo_estimation", PointCloud, ymuiContoller.callback_dlo)
    
    rate = rospy.Rate(ymuiContoller.updateRate) 

    msg = JointState()
    

    seq = 1
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = seq
        msg.name = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', \
            'yumi_joint_6_r', 'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', \
                'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l', 'gripper_r_joint', 'gripper_r_joint_m',\
                    'gripper_l_joint', 'gripper_l_joint_m']
        msg.position = ymuiContoller.jointPose.tolist()
        pub.publish(msg)
        rate.sleep()
        seq += 1


if __name__ == '__main__':
    main()