#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import numpy as np
import tf


def main():

    # starting ROS node and subscribers
    rospy.init_node('pub_force_sim', anonymous=True) 
    pub = rospy.Publisher('/CableForce', Float64, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    force = 0
    cableLength = 0.25
    tfListener = tf.TransformListener() 
    rospy.sleep(0.5)
    while not rospy.is_shutdown():
        (Right, _) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_r', rospy.Time(0))
        (Left, _) = tfListener.lookupTransform('/yumi_base_link', '/yumi_gripp_l', rospy.Time(0))
        Right = np.asarray(Right)
        Left = np.asarray(Left)
        diff = Right - Left
        dist = np.linalg.norm(diff)
        if dist > cableLength:
            force = (dist-cableLength)*1000
        print('dist ', dist, ' force ', force)
        msg = Float64()
        msg.data = force
        pub.publish(msg)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()