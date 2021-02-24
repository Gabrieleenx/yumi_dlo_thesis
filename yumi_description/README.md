This code is based on urdf_tutorial, see the tutorials over at http://wiki.ros.org/urdf_tutorial

The urdf file is from https://github.com/kth-ros-pkg/yumi/tree/egm_modifications/yumi_description. 

To run
```
roslaunch yumi_description display.launch model:='$(find yumi_description)/urdf/yumi.urdf'
```
visualizes in rviz ans subcribes to joint positions. (change fixed frame to yumi_base_link in rviz, and arms are not visulized until first jointposition is sent)

Joint state msg can be created by
    
msg = JointState()

msg.header.stamp = rospy.Time.now()

msg.name = ['yumi_joint_1_r', 'yumi_joint_2_r', 'yumi_joint_7_r', 'yumi_joint_3_r', 'yumi_joint_4_r', 'yumi_joint_5_r', \
            'yumi_joint_6_r', 'yumi_joint_1_l', 'yumi_joint_2_l', 'yumi_joint_7_l', 'yumi_joint_3_l', \
                'yumi_joint_4_l', 'yumi_joint_5_l', 'yumi_joint_6_l', 'gripper_r_joint', 'gripper_r_joint_m',\
                    'gripper_l_joint', 'gripper_l_joint_m']
                    
msg.position = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

The position is given in radians for each joint in the same sequence as the joint names are defined in msg.name 

Alternitavly for just visualization with GUI controls, uncomment line 9 and 10 in launch/display.launch (then no jointstate msg is needed)

depends on robot_state_publisher