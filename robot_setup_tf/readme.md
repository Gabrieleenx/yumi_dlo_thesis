## Node to setup tranfromation tree in ROS

## Maintainer 
- Gabriel Arslan Waltersson

## Table of Contents
* [General](#general)
* [ROScommunication](#ros_communication)
* [Dependencies](#dependencies)
* [Usage](#usage)

## General
Creates a world frame and connecting it with the realsense camera frame. The world origin is defined by a arpiltag of the family tag36h11 with ID 0. Future work will also connect the YuMi to the tree. If the apriltag system is shut down, then it uses the last know detections as a fixed origin. 

## ROS_communication
Subscribed to "/tag_detections" and broadcasts the updates. 
## Dependencies
* ROS melodic 
* catkin
* geometry mesges
* roscpp
* tf
* apriltag, folow instruction and install from source 
```
https://github.com/AprilRobotics/apriltag_ros
```

## Usage

rosrun robot_setup_tf tf_broadcaster






