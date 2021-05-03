#!/usr/bin/env python3

import numpy as np
import cv2
import open3d as o3d
import time

import SPR_local_gpu
import rospy
from sensor_msgs.msg import Image, PointCloud, CameraInfo
from geometry_msgs.msg import Point32
import roslib
import std_msgs.msg
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import tf

spr_local = SPR_local_gpu.SPR()

class ObjectTracking(object):
    def __init__(self, SPR_opt, init_estimate):
        self.SPR_opt = SPR_opt
        self.init_estimate = init_estimate
        self.estimate = init_estimate
        self.target = init_estimate
        self.pub = rospy.Publisher('/spr/dlo_estimation', PointCloud, queue_size=3)
        # object that listen to transformation tree. 
        self.tf_listener = tf.TransformListener()
        self.target_num_points = 300
        self.min_num_points = self.target_num_points/3
        self.voxel_size = 15
        self.offset_xyz = np.array([-0.045,0,0]) # offset in x,y,z in meters
        # camera properites
        self.camera_properties = {
            'res_color_height': 720,
            'res_color_width': 1280,
            'res_depth_height': 480,
            'res_depth_width': 640,
            'color_K': np.array([[919.5044555664062, 0.0, 645.03076171875,],\
                [0.0, 919.5109252929688, 357.19921875],[0.0, 0.0, 1.0]]),
            'depth_K': np.array([[385.37188720703125, 0.0, 320.0655517578125],\
            [0.0, 385.37188720703125, 241.86325073242188],[0.0, 0.0, 1.0]]),
            'depth_reshape': np.array([[1],[640]])
        }
        # from ros image to opencv image
        self.bridge = CvBridge()
        self.pcd = o3d.geometry.PointCloud() #creating point cloud

        # set to something else if apriltags are not used and manual calibration is desierd. 
        self.tag_quaternion_xyzw = np.zeros(4)
        self.tag_translation_xyz = np.zeros(3)
        self.rotation_matrix = np.eye(3)
        self.num_calibration_itter = 0

        # filter table
        self.height_off = -0.086 + 0.003 # m offest for which all points under get filtered away

    def callback(self, depth_data, rgb_data):
        time_start = time.time()

        try:
            color_img = self.bridge.imgmsg_to_cv2(rgb_data, rgb_data.encoding)
        except CvBridgeError as e:
            print(e)
        
        # will change
        self.stamp = rgb_data.header.stamp
        self.frame_id = rgb_data.header.frame_id
        # keep the 16 bit information 
        depth_vec = np.frombuffer(depth_data.data, dtype=np.uint16)

        depth_img = depth_vec.reshape((self.camera_properties['res_depth_height'], \
             self.camera_properties['res_depth_width']))
        
        depth_img = depth_img/1000 # from milimeter int to float meter. 

        # mask cable
        black_lower = (0, 0, 0)
        black_upper = (180, 255, 25)

        blue_lower = (100, 235, 100)
        blue_upper = (120, 255, 255)

        hsv_image = cv2.cvtColor(color_img, cv2.COLOR_RGB2HSV)
        mask = cv2.inRange(hsv_image, blue_lower, blue_upper)
      
        # get coord
        [row,col]=np.nonzero(mask)

        p_color = np.transpose(np.array([col, row, np.ones(np.size(row))]))

        # down sample
        self.pcd.points = o3d.utility.Vector3dVector(p_color)
        self.pcd = self.pcd.voxel_down_sample(voxel_size=self.voxel_size)

        #homogeneous coordinates 
        K_inv = np.linalg.pinv(self.camera_properties['color_K'])
        p_homogeneous = np.transpose(K_inv.dot(np.asarray(self.pcd.points).T))
        p_homogeneous_offset = p_homogeneous + self.offset_xyz

        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', depth_img)
        #cv2.imshow('RealSense', mask)
        #cv2.waitKey(1)
        if np.shape(p_homogeneous)[0] > self.min_num_points: 
            # dynamic downsample
            down_samlpe = np.shape(p_homogeneous)[0] / self.target_num_points
            if down_samlpe < 0.95:
                self.voxel_size -= 0.2    
            elif down_samlpe > 1.05:
                self.voxel_size += 0.2    
        
            # to depth camera 
            p_depth = np.transpose(self.camera_properties['depth_K'].dot(p_homogeneous_offset.T)) 

            # round to nearest int
            p_depth_idx = np.rint(p_depth)
            indices = p_depth_idx[:,0:2].astype(int).dot(self.camera_properties['depth_reshape'])
            
            # clip indecies within range, maybe implement a better version in future
            indices = indices.clip(0, \
                (self.camera_properties['res_depth_height']*self.camera_properties['res_depth_width']-1))
            depth_z = depth_img.reshape(-1)[indices]

            # calculate point cloud            
            point_cloud = p_homogeneous*depth_z

            # remove non vaild points
            point_cloud = point_cloud[~np.all(point_cloud == 0, axis=1)]
            
            # get transformation matrix
            (trans, rot) = self.tf_listener.lookupTransform('/yumi_base_link', '/camera_link', rospy.Time(0))
            transformer_ = tf.TransformerROS(True, rospy.Duration(1.0))
            tf_matrix = transformer_.fromTranslationRotation(translation=trans, rotation=rot)

            # homogenious coordinates 
            col_ones = np.ones((np.shape(point_cloud)[0],1))
            point_cloud = np.hstack((point_cloud,col_ones))

            # Transform
            point_cloud = np.transpose(tf_matrix.dot(point_cloud.T))
            
            # flattend 
            point_cloud = point_cloud[:,0:3]/point_cloud[:,3:4]

            # remove point that are under certatin hight from the worktable 
            index_ = point_cloud[:,2] >= self.height_off
            point_cloud = point_cloud[index_,:]

            self.target = point_cloud

            # plot depthimage and slected points for calibration 
            #depth_img.reshape(-1)[indices] = 255
            #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            #cv2.imshow('RealSense', depth_img)
            #cv2.imshow('RealSense', mask)
            #cv2.waitKey(1)
            
            # call SPR 
            SPR_Transform = spr_local.SPR_register(self.target, self.estimate, self.SPR_opt)  # CPD warp Y to X, fliped! X_warp = SPR_Transform.Y;
            self.estimate = SPR_Transform.get('Y')
            time_update = time.time() - time_start 
            print('time_update ', time_update)
            self.publish_msg()

    def publish_msg(self):
        cloudpoints = self.estimate
        cloud_msg = PointCloud()
        cloud_msg.header.stamp = self.stamp
        cloud_msg.header.frame_id = "yumi_base_link"

        for i in range(cloudpoints.shape[0]):
            cloud_msg.points.append(Point32(cloudpoints[i, 0], cloudpoints[i, 1], cloudpoints[i, 2])) 
            # Change to camera frame
        self.pub.publish(cloud_msg)

    def update_transformation(self):
        pass

    def update_calibration_color(self, camera_info): 
        K = np.array(camera_info.K)
        self.camera_properties['color_K'] = K.reshape((3, 3))
        self.camera_properties['res_color_width'] = camera_info.width
        self.camera_properties['res_color_height'] = camera_info.height


    def update_calibration_depth(self, camera_info):
        K = np.array(camera_info.K)
        self.camera_properties['depth_K'] = K.reshape((3, 3))
        self.camera_properties['res_depth_width'] = camera_info.width
        self.camera_properties['res_depth_height'] = camera_info.height
        self.camera_properties['depth_reshape'] = np.array([[1],[camera_info.width]])



def main():

    SPR_opt = {
        'method': 'nonrigid',
        'viz': 'True',
        'max_it': 20,
        'tol': -1,
        'outliers': 0,
        'normalize': True,
        'knn': 15,
        'tau': 500,
        'beta': 2,
        'lambda': 3,
        'tau_annealing_handle': lambda iter, max_it:  0.95 ** iter,
        'lambda_annealing_handle': lambda iter, max_it: 0.95 ** iter
    }


    # initial estimate 
    cable_length = 1
    num_points = 100
    distance = 0.6

    x = np.ones(num_points)*distance
    y = cable_length*(np.arange(num_points)/num_points)-cable_length/2
    z = np.zeros((num_points))

    init_estimate = np.array([x,y,z]).T
    
   

    # starting ROS node and subscribers
    rospy.init_node('CV_SPR', anonymous=True) 

     # inititate class
    objectTracking = ObjectTracking(SPR_opt, init_estimate)

    rospy.Subscriber('/camera/color/camera_info', CameraInfo, objectTracking.update_calibration_color)
    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, objectTracking.update_calibration_depth)

    depth_camera = message_filters.Subscriber("/camera/depth/image_rect_raw", Image)
    rgb_camera = message_filters.Subscriber("/camera/color/image_raw", Image)

    ts = message_filters.ApproximateTimeSynchronizer([depth_camera, rgb_camera], 5, 0.05, allow_headerless=False)
    ts.registerCallback(objectTracking.callback)

    #rospy.sleep(3)
    #while not rospy.is_shutdown():
    #    objectTracking.publish_msg()
    #    rospy.sleep(0.1)
    rospy.spin()


if __name__ == '__main__':
    main()