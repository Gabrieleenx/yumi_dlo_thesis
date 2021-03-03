#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState, PointCloud
from std_msgs.msg import Float64MultiArray
from controller.msg import Jacobian_msg, Trajectory_msg
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R_scipy
import threading

import message_filters
import time
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

        # Trajectory
        self.controlInstructions = utils.ControlInstructions(self.dT)


        # Forward kinematics and transformers 
        # for critical updates
        self.yumiGrippPoseR = utils.FramePose()
        self.yumiGrippPoseL = utils.FramePose()
        self.yumiElbowPoseR = utils.FramePose()
        self.yumiElbowPoseL = utils.FramePose()
        self.yumiElbowPoseL = utils.FramePose()

        self.tfListener = tf.TransformListener() # for non critical updates, no guarantee for synch
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))

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

        self.selfCollisionRightElbow = Task.ElbowCollision(Dof=14, arm='right', minDistance=0.3, timestep=self.dT)
        self.selfCollisionLeftElbow = Task.ElbowCollision(Dof=14, arm='left', minDistance=0.3, timestep=self.dT)

        self.lock = threading.Lock()

    def callback(self, data):
        time_start = time.time()
        (gripperLengthRight, _) = self.tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        (gripperLengthLeft, _) = self.tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))

        self.gripperLengthRight = np.asarray(gripperLengthRight)
        self.gripperLengthLeft = np.asarray(gripperLengthLeft)

        jacobianCombined = utils.CalcJacobianCombined(data=data.jacobian[0], \
                                                    gripperLengthRight=gripperLengthRight,\
                                                    gripperLengthLeft=gripperLengthLeft,\
                                                    transformer=self.transformer,\
                                                    yumiGrippPoseR=self.yumiGrippPoseR,\
                                                    yumiGrippPoseL=self.yumiGrippPoseL)
        self.yumiGrippPoseR.update(data.forwardKinematics[0], self.transformer, self.gripperLengthRight)
        self.yumiGrippPoseL.update(data.forwardKinematics[1], self.transformer, self.gripperLengthLeft)
        self.yumiElbowPoseR.update(data.forwardKinematics[2],  self.transformer, np.zeros(3))
        self.yumiElbowPoseL.update(data.forwardKinematics[3], self.transformer, np.zeros(3))
        
        # stack of tasks, in decending hierarchy
        # ----------------------
        # velocity bound
        SoT = [self.jointVelocityBoundUpper, self.jointVelocityBoundLower]

        #position bound
        self.jointPositionBoundUpper.compute(jointState=self.jointState)
        SoT.append(self.jointPositionBoundUpper)
        self.jointPositionBoundLower.compute(jointState=self.jointState)
        SoT.append(self.jointPositionBoundLower)

        # Self collision elbow
        jacobianRightElbow = np.zeros((6,4))
        jacobianLeftElbow = np.zeros((6,4))

        dataNP = np.asarray(data.jacobian[1].data)

        jacobianRightElbow = dataNP[0::2].reshape((6,4))
        jacobianLeftElbow = dataNP[1::2].reshape((6,4))
        self.selfCollisionRightElbow.compute(jacobian=jacobianRightElbow,\
                                            yumiElbowPoseR=self.yumiElbowPoseR,\
                                            yumiElbowPoseL=self.yumiElbowPoseL)
        self.selfCollisionLeftElbow.compute(jacobian=jacobianLeftElbow,\
                                            yumiElbowPoseR=self.yumiElbowPoseR,\
                                            yumiElbowPoseL=self.yumiElbowPoseL)
        SoT.append(self.selfCollisionRightElbow)
        SoT.append(self.selfCollisionLeftElbow)
        
        # trajcetory Control 
        self.controlInstructions.updateTransform(yumiGrippPoseR=self.yumiGrippPoseR,\
                                                 yumiGrippPoseL=self.yumiGrippPoseL)
        
        self.controlInstructions.updateTarget()

        if self.controlInstructions.mode == 'individual':
            # indvidual task update 
            self.indiviualControl.compute(controlInstructions=self.controlInstructions, jacobian=jacobianCombined)
            SoT.append(self.indiviualControl)

        elif self.controlInstructions.mode == 'combined':

            self.relativeControl.compute(controlInstructions=self.controlInstructions,\
                                        jacobian=jacobianCombined,\
                                        transformer=self.transformer)
            SoT.append(self.relativeControl) 

            self.absoluteControl.compute(controlInstructions=self.controlInstructions,\
                                        jacobian=jacobianCombined)
            SoT.append(self.absoluteControl) 

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
        self.lock.acquire()
        self.jointState.UpdatePose(pose=pose)
        self.lock.release()
        #print('Hz', 1/(time.time() - time_start))

    def callbackTrajectory(self, data):
        # set mode
        self.controlInstructions.mode = data.mode
        # current point as first point 
        #TODO add grippers
        if data.mode == 'combined':
            positionRight = self.controlInstructions.absolutePosition
            positionLeft  = self.controlInstructions.realativPosition 
            orientationRight = self.controlInstructions.absoluteOrientation
            orientationLeft = self.controlInstructions.rotationRelative
        elif data.mode == 'individual':
            positionRight = self.controlInstructions.translationRightArm
            positionLeft  = self.controlInstructions.translationLeftArm 
            orientationRight = self.controlInstructions.rotationRightArm
            orientationLeft = self.controlInstructions.rotationLeftArm
        else:
            print('Error, mode not matching combined or individual')
            return

        currentPoint = utils.TrajcetoryPoint(positionRight=positionRight, positionLeft=positionLeft,orientationRight=orientationRight, orientationLeft=orientationLeft)
        trajectory = [currentPoint]
        
        for i in range(len(data.trajcetory)):
            positionRight = np.asarray(data.trajcetory[i].positionRight)
            positionLeft  = np.asarray(data.trajcetory[i].positionLeft)
            orientationRight = np.asarray(data.trajcetory[i].orientationRight)
            orientationLeft = np.asarray(data.trajcetory[i].orientationLeft)
            gripperLeft = np.asarray(data.trajcetory[i].gripperLeft)
            gripperRight = np.asarray(data.trajcetory[i].gripperRight)
            pointTime = np.asarray(data.trajcetory[i].pointTime)
            trajectroyPoint = utils.TrajcetoryPoint(positionRight=positionRight,\
                                                    positionLeft=positionLeft,\
                                                    orientationRight=orientationRight,\
                                                    orientationLeft=orientationLeft,\
                                                    gripperLeft=gripperLeft,\
                                                    gripperRight=gripperRight,\
                                                    pointTime=pointTime)
            trajectory.append(trajectroyPoint)

        self.controlInstructions.trajectory.updatePoints(trajectory, np.zeros(3), np.zeros(3))


def main():

    # starting ROS node and subscribers
    rospy.init_node('pub_joint_pos', anonymous=True) 
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

    ymuiContoller = YmuiContoller()
    rospy.sleep(0.5)

    rospy.Subscriber("/Jacobian_R_L", Jacobian_msg, ymuiContoller.callback, queue_size=1)
    #rospy.Subscriber("/spr/dlo_estimation", PointCloud, ymuiContoller.callback_dlo)
    rospy.Subscriber("/Trajectroy", Trajectory_msg, ymuiContoller.callbackTrajectory, queue_size=1)


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
        ymuiContoller.lock.acquire()
        msg.position = ymuiContoller.jointState.GetJointPosition().tolist()
        ymuiContoller.lock.release()
        pub.publish(msg)
        rate.sleep()
        seq += 1


if __name__ == '__main__':
    main()