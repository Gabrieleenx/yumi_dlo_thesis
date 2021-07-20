#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64, Int64
from controller.msg import Jacobian_msg, Trajectory_msg
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R_scipy
import threading
import rosservice
from abb_rapid_sm_addin_msgs.srv import SetSGCommand
from abb_robot_msgs.srv import TriggerWithResultCode

import message_filters
import time
import HQPSolver
import utils 
import Task

class YmuiContoller(object):
    def __init__(self):

        self.updateRate = 50 #Hz also defined in kdl_jacobian 
        self.dT = 1/self.updateRate

        self.jointState = utils.JointState()

        # class for generating target velocities
        self.controlInstructions = utils.ControlInstructions(self.dT)

        # Forward kinematics and transformers 
        # for critical updates
        self.yumiGrippPoseR = utils.FramePose()
        self.yumiGrippPoseL = utils.FramePose()
        self.yumiElbowPoseR = utils.FramePose()
        self.yumiElbowPoseL = utils.FramePose()

        self.tfListener = tf.TransformListener() # for non critical updates, no guarantee for synch
        self.transformer = tf.TransformerROS(True, rospy.Duration(1.0))
                
        #rosservice, for control over grippers
        self.SetSGCommand = rospy.ServiceProxy('/yumi/rws/sm_addin/set_sg_command', SetSGCommand)
        self.RunSGRoutine = rospy.ServiceProxy('/yumi/rws/sm_addin/run_sg_routine', TriggerWithResultCode)
        stopEGM = rospy.ServiceProxy('/yumi/rws/sm_addin/stop_egm', TriggerWithResultCode)
        
        # HQP solver 
        self.HQP = HQPSolver.HQPSolver(stopEGM)

        # Task objects -------------
        # ctype 0 = equality, 1 = upper, -1 = lower

        #values from https://search.abb.com/library/Download.aspx?DocumentID=3HAC052982-001&LanguageCode=en&DocumentPartId=&Action=Launch
        jointPoistionBoundUpper = np.array([168.5, 43.5, 168.5, 80, 290, 138, 229])*np.pi/(180) *0.99 # in radians 
        jointPoistionBoundUpper = np.hstack([jointPoistionBoundUpper, jointPoistionBoundUpper]) # two arms
        self.jointPositionBoundUpper = Task.JointPositionBoundsTask(Dof=14,\
                     bounds=jointPoistionBoundUpper, timestep=self.dT, ctype=1)
        
        jointPoistionBoundLower = np.array([-168.5, -143.5, -168.5, -123.5, -290, -88, -229])*np.pi/(180) *0.99 # in radians 
        jointPoistionBoundLower = np.hstack([jointPoistionBoundLower, jointPoistionBoundLower]) # two arms
        self.jointPositionBoundLower = Task.JointPositionBoundsTask(Dof=14,\
                     bounds=jointPoistionBoundLower, timestep=self.dT, ctype=-1)

        # joint velocity limit 
        jointVelocityBound = np.array([1, 1, 1, 1, 1, 1, 1])
        jointVelocityBound = np.hstack([jointVelocityBound, jointVelocityBound]) # two arms

        self.jointVelocityBoundUpper = Task.JointVelocityBoundsTask(Dof=14,\
                     bounds=jointVelocityBound, ctype=1)
        self.jointVelocityBoundUpper.compute() # constant

        self.jointVelocityBoundLower = Task.JointVelocityBoundsTask(Dof=14,\
                     bounds=-jointVelocityBound, ctype=-1)
        self.jointVelocityBoundLower.compute() # constant

        # Control objective
        
        self.indiviualControl = Task.IndividualControl(Dof=14)

        self.absoluteControl = Task.AbsoluteControl(Dof=14)

        self.relativeControl = Task.RelativeControl(Dof=14)

        # elbow colision
        self.selfCollisionRightElbow = Task.ElbowCollision(Dof=14, arm='right', minDistance=0.2, timestep=self.dT)
        self.selfCollisionLeftElbow = Task.ElbowCollision(Dof=14, arm='left', minDistance=0.2, timestep=self.dT)

        # joint potential 
        defaultPose = np.array([0.7, -1.7, -0.8, 1.0, -2.2, 1.0, 0.0, -0.7, -1.7, 0.8, 1.0, 2.2, 1.0, 0.0])
        self.jointPositionPotential = Task.JointPositionPotential(Dof=14, defaultPose=defaultPose, timestep=self.dT)
        
        # mutex
        self.lock = threading.Lock()
        
        # publish velocity comands
        self.pub = rospy.Publisher('/yumi/egm/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1)
        self.pubGripperSim = rospy.Publisher('/sim/grippers', Float64MultiArray, queue_size=1) # only for sim

        self.pubSubTask = rospy.Publisher('/controller/sub_task', Int64, queue_size=1)


    def callback(self, data):
        # update joint position
        self.jointState.UpdatePose(pose=np.asarray(data.jointPosition))
        # distance from wrist to tip of grippers, ussually constant. changed in robot_setup_tf 
        (gripperLengthRight, _) = self.tfListener.lookupTransform('/yumi_link_7_r', '/yumi_gripp_r', rospy.Time(0))
        (gripperLengthLeft, _) = self.tfListener.lookupTransform('/yumi_link_7_l', '/yumi_gripp_l', rospy.Time(0))

        self.gripperLengthRight = np.asarray(gripperLengthRight)
        self.gripperLengthLeft = np.asarray(gripperLengthLeft)
        # calculated the combined geometric jacobian from base link to tip of gripper for both arms
        jacobianCombined = utils.CalcJacobianCombined(data=data.jacobian[0], \
                                                    gripperLengthRight=gripperLengthRight,\
                                                    gripperLengthLeft=gripperLengthLeft,\
                                                    transformer=self.transformer,\
                                                    yumiGrippPoseR=self.yumiGrippPoseR,\
                                                    yumiGrippPoseL=self.yumiGrippPoseL)
        # forward kinematics information 
        self.yumiGrippPoseR.update(data.forwardKinematics[0], self.transformer, self.gripperLengthRight)
        self.yumiGrippPoseL.update(data.forwardKinematics[1], self.transformer, self.gripperLengthLeft)
        self.yumiElbowPoseR.update(data.forwardKinematics[2], self.transformer, np.zeros(3))
        self.yumiElbowPoseL.update(data.forwardKinematics[3], self.transformer, np.zeros(3))
        
        # stack of tasks, in decending hierarchy
        # ----------------------
        SoT = []
        # velocity bound
        SoT = [self.jointVelocityBoundUpper, self.jointVelocityBoundLower]
        
        # position bound
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
        self.lock.acquire()
        # calculates target velocities and positions
        self.controlInstructions.updateTarget()

        if self.controlInstructions.mode == 'individual':
            # indvidual task update 
            self.indiviualControl.compute(controlInstructions=self.controlInstructions, jacobian=jacobianCombined)
            SoT.append(self.indiviualControl)

        elif self.controlInstructions.mode == 'combined':
            # combined task update
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
            self.lock.release()
            # publish velocity comands
            self.publishVelocity()
            return

        self.lock.release()

        # check so devation is not to big, stop if it is 
        if not self.controlInstructions.checkDevation():
            print('Deveation from trajectory too large, stopping')
            self.jointState.jointVelocity = np.zeros(14)
            self.publishVelocity()
            return
     
        self.jointPositionPotential.compute(jointState=self.jointState)
        SoT.append(self.jointPositionPotential)
        
        # solve HQP
        # ----------------------
        self.jointState.jointVelocity = self.HQP.solve(SoT=SoT)

        # gripper control
        # ----------------------
        
        if self.controlInstructions.newIndex():
            tol = 1e-5
            try:
                # not to sent same command twice. As the grippers might momentarly regripps if the same command is sent twice. 
                if abs(self.controlInstructions.lastGripperLeft[0] - self.controlInstructions.gripperLeft[0]) >= tol:
                    if self.controlInstructions.gripperLeft[0] <= 0.1:
                        self.SetSGCommand(task="T_ROB_L", command=6)
                    else:
                        self.SetSGCommand(task="T_ROB_L", command=5, target_position=self.controlInstructions.gripperLeft[0])

                    self.controlInstructions.lastGripperLeft[0] = self.controlInstructions.gripperLeft[0]

                if abs(self.controlInstructions.lastGripperRight[0] - self.controlInstructions.gripperRight[0]) >= tol:
                    if self.controlInstructions.gripperRight[0] <= 0.1:
                        self.SetSGCommand(task="T_ROB_R", command=6)
                    else:
                        self.SetSGCommand(task="T_ROB_R", command=5, target_position=self.controlInstructions.gripperRight[0])

                    self.controlInstructions.lastGripperRight[0] = self.controlInstructions.gripperRight[0]
                # sends of the commandes to the robot
                self.RunSGRoutine()
            except:
                print('smart gripper error or running simulation')
                msg = Float64MultiArray()
                msg.data = np.hstack([self.controlInstructions.gripperRight[0], self.controlInstructions.gripperLeft[0]]).tolist()
                self.pubGripperSim.publish(msg)
        
        # publish velocity comands
        self.publishVelocity()


    def publishVelocity(self):
        msg = Float64MultiArray()
        # Left Arm, Right Arm
        msg.data = np.hstack([self.jointState.GetJointVelocity()[7:14], self.jointState.GetJointVelocity()[0:7]]).tolist()
        self.pub.publish(msg)
        msgSubTask = Int64()
        msgSubTask.data = self.controlInstructions.trajectory.index - 1
        self.pubSubTask.publish(msgSubTask)


    def callbackTrajectory(self, data):
        # current pose as first point 
        if data.mode == 'combined':
            positionRight = np.copy(self.controlInstructions.absolutePosition)
            positionLeft  = np.copy(self.controlInstructions.realativPosition) 
            orientationRight = np.copy(self.controlInstructions.absoluteOrientation)
            orientationLeft = np.copy(self.controlInstructions.rotationRelative)
        elif data.mode == 'individual':
            positionRight = np.copy(self.controlInstructions.translationRightArm)
            positionLeft  = np.copy(self.controlInstructions.translationLeftArm)
            orientationRight = np.copy(self.controlInstructions.rotationRightArm)
            orientationLeft = np.copy(self.controlInstructions.rotationLeftArm)
        else:
            print('Error, mode not matching combined or individual')
            return
        # current gripper position # 
        gripperLeft = self.controlInstructions.lastGripperLeft
        gripperRight = self.controlInstructions.lastGripperRight
        currentPoint =utils.TrajectoryPoint(positionRight=positionRight,\
                                                    positionLeft=positionLeft,\
                                                    orientationRight=orientationRight,\
                                                    orientationLeft=orientationLeft,\
                                                    gripperLeft=gripperLeft,\
                                                    gripperRight=gripperRight)
        trajectory = [currentPoint]
        # append trajectory points
        for i in range(len(data.trajectory)):
            if data.mode == 'combined':
                positionRight = np.asarray(data.trajectory[i].positionAbsolute)
                positionLeft  = np.asarray(data.trajectory[i].positionRelative)
                orientationRight = np.asarray(data.trajectory[i].orientationAbsolute)
                orientationLeft = np.asarray(data.trajectory[i].orientationRelative)
            else:
                positionRight = np.asarray(data.trajectory[i].positionRight)
                positionLeft  = np.asarray(data.trajectory[i].positionLeft)
                orientationRight = np.asarray(data.trajectory[i].orientationRight)
                orientationLeft = np.asarray(data.trajectory[i].orientationLeft)

            gripperLeft = np.asarray(data.trajectory[i].gripperLeft)
            gripperRight = np.asarray(data.trajectory[i].gripperRight)
            pointTime = np.asarray(data.trajectory[i].pointTime)
            trajectroyPoint = utils.TrajectoryPoint(positionRight=positionRight,\
                                                    positionLeft=positionLeft,\
                                                    orientationRight=orientationRight,\
                                                    orientationLeft=orientationLeft,\
                                                    gripperLeft=gripperLeft,\
                                                    gripperRight=gripperRight,\
                                                    pointTime=pointTime)
            trajectory.append(trajectroyPoint)
        self.lock.acquire()       

        # use current velocity for smoother transitions, (not for orientaion)
        if self.controlInstructions.mode == data.mode:
            velLeftInit = np.copy(self.controlInstructions.velocities[6:9])
            velRightInit = np.copy(self.controlInstructions.velocities[0:3])
        elif self.controlInstructions.mode == 'individual':
            # simple solution, not fully accurate trasition 
            velLeftInit = np.zeros(3)
            velRightInit = 0.5*(np.copy(self.controlInstructions.velocities[0:3]) +\
                                     np.copy(self.controlInstructions.velocities[6:9]))
        elif self.controlInstructions.mode == 'combined':
            # simple solution, not fully accurate trasition 
            velLeftInit = np.copy(self.controlInstructions.velocities[0:3])
            velRightInit = np.copy(self.controlInstructions.velocities[0:3])
        else:
            print('Warning, Previous mode not matching, combined or individual')
            velLeftInit = np.zeros(3)
            velRightInit = np.zeros(3)
            
        # set mode
        self.controlInstructions.mode = data.mode

        # update the trajectroy 
        self.controlInstructions.trajectory.updatePoints(trajectory, velLeftInit, velRightInit)
        self.controlInstructions.trajIndex = 0
        self.lock.release()


def main():

    # starting ROS node and subscribers
    rospy.init_node('controller', anonymous=True) 

    ymuiContoller = YmuiContoller()
    rospy.sleep(0.05)
    rospy.Subscriber("/Jacobian_R_L", Jacobian_msg, ymuiContoller.callback, queue_size=3)
    rospy.Subscriber("/Trajectroy", Trajectory_msg, ymuiContoller.callbackTrajectory, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    main()