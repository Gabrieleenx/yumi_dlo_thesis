#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState, PointCloud
from std_msgs.msg import Float64MultiArray
from controller.msg import Jacobian_msg
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R_scipy
import threading

import HQPSolver
import utils 
import Task

class YmuiContoller(object):
    def __init__(self):

        self.updateRate = 100 #Hz
        self.dT = 1/self.updateRate

        self.jointState = utils.JointState()

        self.maxJointVelocity = 0.5
        self.maxScaleJointVelocity = 5

        self.controlArmVelocity = np.zeros((14,1))

        # Trajectory, temporary for testing 
        rotabstest1 = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 160*np.pi/180, 'rzyx')
        rotabstest2 = tf.transformations.quaternion_from_euler(-0*np.pi/180, 0, 200*np.pi/180, 'rzyx')
        rotabstest3 = tf.transformations.quaternion_from_euler(0*np.pi/180, 0, 160*np.pi/180, 'rzyx')

        localrottest = tf.transformations.quaternion_from_euler(80*np.pi/180, 0, 0*np.pi/180, 'rzyx')

        trajectory = utils.Trajectory(positionRight=np.array([0.4 ,0, 0.2]), positionLeft=np.array([0 ,0.2, 0.0]), orientationLeft=np.array([0,0,0,1]))
        trajectory1 = utils.Trajectory(positionRight=np.array([0.4 ,0, 0.2]), positionLeft=np.array([0 ,0.2, 0.0]), orientationLeft=np.array([0,0,0,1]), orientationRight=rotabstest1)
        trajectory2 = utils.Trajectory(positionRight=np.array([0.4 ,0, 0.2]), positionLeft=np.array([0 ,0.2, 0.0]), orientationLeft=np.array([0,0,0,1]), orientationRight=rotabstest2)
        trajectory3 = utils.Trajectory(positionRight=np.array([0.4 ,0, 0.2]), positionLeft=np.array([0 ,0.2, 0.0]), orientationLeft=np.array([0,0,0,1]), orientationRight=rotabstest3)
        trajectory4 = utils.Trajectory(positionRight=np.array([0.4 ,0, 0.2]), positionLeft=np.array([0 ,0.2, 0.0]), orientationLeft=localrottest)
       
        self.controlInstructions = utils.ControlInstructions()
        self.controlInstructions.mode = 'combined'
        self.controlInstructions.trajectory = [trajectory, trajectory1, trajectory, trajectory2, trajectory, trajectory3, trajectory, trajectory4, trajectory]

        # object that listen to transformation tree. 
        self.tfListener = tf.TransformListener()
        self.transformer = tf.TransformerROS(True, rospy.Duration(4.0))

        # solver 
        self.HQP = HQPSolver.HQPSolver()

        # Task objects 
        # ctype 0 = equality, 1 = upper, -1 = lower

        #values from https://search.abb.com/library/Download.aspx?DocumentID=3HAC052982-001&LanguageCode=en&DocumentPartId=&Action=Launch
        jointPoistionBoundUpper = np.array([168.5, 43.5, 168.5, 80, 290, 138, 229])*np.pi/(180) # in radians 
        jointPoistionBoundUpper = np.hstack([jointPoistionBoundUpper, jointPoistionBoundUpper]) # two arms
        self.jointPositionBoundUpper = Task.JointPositionBoundsTask(Dof=14,\
                     bounds=jointPoistionBoundUpper, timestep=self.dT, ctype=1)
        
        jointPoistionBoundLower = np.array([-168.5, -143.5, -168.5, -123.5, -290, -88, -229])*np.pi/(180) # in radians 
        jointPoistionBoundLower = np.hstack([jointPoistionBoundLower, jointPoistionBoundLower]) # two arms
        self.jointPositionBoundLower = Task.JointPositionBoundsTask(Dof=14,\
                     bounds=jointPoistionBoundLower, timestep=self.dT, ctype=-1)

        velocityDownScaling = 4
        jointVelocityBound = np.array([180, 180, 180, 180, 400, 400, 400])*np.pi/(180) / velocityDownScaling # in radians 
        jointVelocityBound = np.hstack([jointVelocityBound, jointVelocityBound]) # two arms

        self.jointVelocityBoundUpper = Task.JointVelocityBoundsTask(Dof=14,\
                     bounds=jointVelocityBound, ctype=1)
        self.jointVelocityBoundUpper.compute() # constant

        self.jointVelocityBoundLower = Task.JointVelocityBoundsTask(Dof=14,\
                     bounds=-jointVelocityBound, ctype=-1)
        self.jointVelocityBoundLower.compute() # constant

        self.indiviualControl = Task.IndividualControl(Dof=14)

        self.absoluteControl = Task.AbsoluteControl(Dof=14)

        self.relativeControl = Task.RelativeControl(Dof=14)

    def callback(self, data):

        jacobianCombined = utils.CalcJacobianCombined(data=data.jacobian[0], tfListener=self.tfListener, transformer=self.transformer)
        
        # stack of tasks, in decending hierarchy
        # ----------------------
        SoT = [self.jointVelocityBoundUpper, self.jointVelocityBoundLower]
        #SoT = []
        self.jointPositionBoundUpper.compute(jointState=self.jointState)
        SoT.append(self.jointPositionBoundUpper)
        self.jointPositionBoundLower.compute(jointState=self.jointState)
        SoT.append(self.jointPositionBoundLower)

        if self.controlInstructions.mode == 'individual':
            # indvidual task update 
            self.indiviualControl.compute(controlInstructions=self.controlInstructions, jacobian=jacobianCombined)
            SoT.append(self.indiviualControl)

        elif self.controlInstructions.mode == 'combined':
            self.controlInstructions.updateTransform()
            self.relativeControl.compute(controlInstructions=self.controlInstructions,\
                                        jacobian=jacobianCombined,\
                                        transformer=self.transformer)
            SoT.append(self.relativeControl) 

            self.absoluteControl.compute(controlInstructions=self.controlInstructions,\
                                        jacobian=jacobianCombined)
            SoT.append(self.absoluteControl) 
            self.controlInstructions.computeTrajectoryIntex()

        else:
            print('Non valid control mode, stopping')
            self.jointState.jointVelocity = np.zeros(14)
            self.jointState.gripperLeftVelocity = np.zeros(2)
            self.jointState.gripperRightVelocity = np.zeros(2)
            return
        # solve HQP
        # ----------------------
        self.jointState.jointVelocity = self.HQP.solve(SoT=SoT)
        
        # gripper control
        # ----------------------

        
        # temporary update 
        # ----------------------
        pose = self.jointState.GetJointPosition() + self.jointState.GetJointVelocity()*self.dT

        self.jointState.UpdatePose(pose=pose)




def main():

    # starting ROS node and subscribers
    rospy.init_node('pub_joint_pos', anonymous=True) 
    pub = rospy.Publisher('/joint_states', JointState, queue_size=5)

    ymuiContoller = YmuiContoller()
    rospy.sleep(0.5)

    rospy.Subscriber("/Jacobian_R_L", Jacobian_msg, ymuiContoller.callback)
    #rospy.Subscriber("/spr/dlo_estimation", PointCloud, ymuiContoller.callback_dlo)
    
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
        msg.position = ymuiContoller.jointState.GetJointPosition().tolist()
        pub.publish(msg)
        rate.sleep()
        seq += 1


if __name__ == '__main__':
    main()