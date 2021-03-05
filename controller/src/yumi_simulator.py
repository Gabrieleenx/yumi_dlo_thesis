#!/usr/bin/env python3
# This is a super basic "simulator" or more like it integrates the velocity commands at 250 hz
import rospy
import utils
import threading
import numpy as np
from sensor_msgs.msg import JointState, PointCloud

class Simulator(object):
    def __init__(self):
        self.updateRate = 250 #hz
        self.dT = 1/self.updateRate
        self.jointState = utils.JointState()
        self.lock = threading.Lock()
        upperArmLimit = np.array([168.5, 43.5, 168.5, 80, 290, 138, 229])*np.pi/(180)
        lowerArmLimit = np.array([-168.5, -143.5, -168.5, -123.5, -290, -88, -229])*np.pi/(180)
        self.jointPoistionBoundUpper = np.hstack([upperArmLimit, upperArmLimit, np.array([0.3,0.3,0.3,0.3])]) # in radians 
        self.jointPoistionBoundLower = np.hstack([lowerArmLimit, lowerArmLimit, np.array([-0.3,-0.3,-0.3,-0.3])]) # in radians 

    def callback(self, data):
        vel = np.asarray(data.velocity)
        self.lock.acquire()
        self.jointState.UpdateVelocity(vel)
        self.lock.release()

    def update(self):
        # updates the pose
        self.lock.acquire()
        pose = self.jointState.GetJointPosition() + self.jointState.GetJointVelocity()*self.dT
        self.lock.release()
        # hard joint limits 
        pose = np.clip(pose, self.jointPoistionBoundLower, self.jointPoistionBoundUpper)
        self.jointState.UpdatePose(pose=pose)


def main():

    # starting ROS node and subscribers
    rospy.init_node('yumi_simulator', anonymous=True) 
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    simulator = Simulator()

    rospy.Subscriber("/joint_velocity", JointState, simulator.callback, queue_size=1)
   
    rate = rospy.Rate(simulator.updateRate) 

    msg = JointState()
    
    seq = 1
    while not rospy.is_shutdown():
        simulator.update()
        msg.header.stamp = rospy.Time.now()
        msg.header.seq = seq
        msg.name = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', \
            'yumi_joint_6_r', 'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', \
                'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l', 'gripper_r_joint', 'gripper_r_joint_m',\
                    'gripper_l_joint', 'gripper_l_joint_m']
        msg.position = simulator.jointState.GetJointPosition().tolist()
        pub.publish(msg)
        rate.sleep()
        seq += 1


if __name__ == '__main__':
    main()




