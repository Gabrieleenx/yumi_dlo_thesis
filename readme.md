# yumi, planner and DLO tracking
ROS implementation of DLO planner and tracking

## Maintainer 
* Gabriel Arslan Waltersson

## Table of Contents
* [General](#general)
* [Dependencies](#dependencies)
* [Usage](#usage)

## General
This package contains a planner for generating trajectory paramters and a DLO tracker. It uses a controller besed on HQP, see https://github.com/CRIS-Chalmers/yumi.git. It aslo contains a very simple DLO simulator, not physically accurate. 

### Tracking 
This package tracks defomable linear objects (DLOs) using a RGB-D camera (intel realsense d435). The SPR algorithim is converted from the original creators https://github.com/thomastangucb/SPR matlab code to python, done in a design project course (by other students - \ref) and has been modified for this thesis. Manily to keep the local regularization constant in order to improve stability. The package uses ros tf tree to get transformation from camera frame to world frame. Currently the cable is sperated by color from the background.      



## Dependencies
First follow the dependencies for  https://github.com/CRIS-Chalmers/yumi.git

* extra python pakages (not in https://github.com/CRIS-Chalmers/yumi.git )
``` 
    open3d
    cupy (optional for gpu)
```
* follow instructions to setup realsense 
```
https://github.com/IntelRealSense/realsense-ros
```

* build opencv for python3 and melodic from source in catkin/src
```
git clone -b melodic https://github.com/ros-perception/vision_opencv.git
```
* Follow instructions for april tags ros, also set up correct tags. (tag_family: 'tag36h11', and id:0 is for world, rest is for fixtures)
```
https://github.com/AprilRobotics/apriltag_ros
```

* Comand to complie for \catkin folder
``` 
catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
``` 

## Usage


### Simulation:

To start the controller through the interface script. See more at https://github.com/CRIS-Chalmers/yumi.git

* open a terminal and start the roscore
``` 
roscore
``` 

* open a second terminal
``` 
rosrun robot_setup_tf Interface.py
``` 
Then start the simulation and the controller through the interface

* in a new terminal, start rope simulation
``` 
rosrun simulation_rope ropeSim
``` 

* in a new terminal, start the planner
``` 
rosrun planner plannerMaster.py
``` 

### Real-world:
Read through wiki at https://github.com/CRIS-Chalmers/yumi.git
* open a terminal and start the roscore
``` 
roscore
``` 

* open a second terminal
``` 
rosrun robot_setup_tf Interface.py
``` 
Connect to the robot, start egm and the controller through the interface

* start the camera, depth resolution running as standard (640, 480)
``` 
roslaunch realsense2_camera rs_camera.launch color_width:=1280 color_height:=720 color_fps:=15 depth_fps:=15 align_depth:=true
``` 
* tf_camera
``` 
rosrun track_dlo tf_camera
``` 
* start apriltag
``` 
roslaunch apriltag_ros continuous_detection.launch
``` 

* If camera is fixed, then to save computaion, close apriltag node. tf broadcaster will use last known position of tag. Then to start the DLO tracking 
``` 
rosrun track_dlo main_spr.py
``` 

Once the tracking is running, start the planner
``` 
rosrun planner plannerMaster.py
``` 
