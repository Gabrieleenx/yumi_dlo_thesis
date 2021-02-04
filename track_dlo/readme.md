## SPR for thesis work with YuMi 

## Maintainer 
* Gabriel Arslan Waltersson

## Table of Contents
* [General](#general)
* [ROScommunication](#ros_communication)
* [Dependencies](#dependencies)
* [Usage](#usage)

## General
This package tracks defomable linear objects (DLOs) using a RGB-D camera (intel realsense d435). The SPR algorithim is converted from the original creators https://github.com/thomastangucb/SPR matlab code to python, done in a design project course (by other students - \ref) and has been modified for this thesis. Manily to keep the local regularization constant in order to improve stability. The package uses ros tf tree to get transformation from camera frame to world frame. Currently the cable is indentifyed by filtering for black, but future  implementation will filter for color.      

## ROS_communication
* Input
```
/camera/depth/image_rect_raw
/camera/color/image_raw
/camera/color/camera_info
/camera/depth/camera_info
tf.TransformListener()
```
* Output (point cloud)
```
/spr/dlo_estimation
```


## Dependencies
This package uses Ros melodic and python3, ROS and catkin has to be reconfigured for python3
* for pythpn3 ros packages 
```
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy
```
* for tf package to be compiled for python3, (if(NOT CATKIN_ENABLE_TESTING) in cmake list in test_tf2 may need to be commented out)
```
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v 0.6.5
wstool up
rosdep install --from-paths src --ignore-src -y -r
```
* build opencv for python3 and melodic from source in catkin/src
```
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
```
* setup tf tree node
```
https://github.com/Gabrieleenx/robot_setup_tf
```

* python pakages
``` 
    numpy
    scipy
```

## Usage
* compile, as some packages are not standard catkin and compile with only cmake
``` 
catkin_make_isolated -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
``` 

* start the camera, depth resolution running as standard (640, 480)
``` 
roslaunch realsense2_camera rs_camera.launch color_width:=1280 color_height:=720
``` 
* start tf broadcaster, for seting up transformation tree between the camera and world frame
``` 
rosrun robot_setup_tf tf_broadcaster
``` 

* start apriltag
``` 
roslaunch apriltag_ros continuous_detection.launch
``` 

* If camera is fixed, then to save computaion, close apriltag node. tf broadcaster will use last known position of tag. 

* to run the node
``` 
rosrun track_dlo main_spr.py
``` 

* visulize in rviz
```
rivz 
``` 
