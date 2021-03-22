## Controller for thesis work with YuMi 

## Maintainer 
* Gabriel Arslan Waltersson

## Table of Contents
* [General](#general)
* [ROScommunication](#ros_communication)
* [Dependencies](#dependencies)
* [Usage](#usage)

## General
This package takes in the comtrol comands as individual or combined (absolute and relative) trajectories and calulates the inverse kinematics and force control to output joint velocity comands for the YuMi robot. (force controll still needs work)      

## ROS_communication
* Input - kdl_jacobian
```
/yumi/egm/joint_states
/yumi/egm/egm_states
```
* Output  - kdl_jacobian
```
/Jacobian_R_L
/joint_states 
/yumi/egm/joint_group_velocity_controller/command
```
* Input - controllerMaster
```
/Jacobian_R_L
/Trajectroy_msg
/TransformListener()
/CableForce
```
* Output  - controllerMaster
```
/yumi/egm/joint_group_velocity_controller/command
/yumi/rws/sm_addin/set_sg_command (rosservice)
/yumi/rws/sm_addin/run_sg_routine (rosservice)
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

* build orocos_kinematics_dynamics from source in catkin/src
```
https://github.com/orocos/orocos_kinematics_dynamics
```

* setup tf tree node
```
https://github.com/Gabrieleenx/yumi_dlo_thesis/tree/master/robot_setup_tf
```

* python pakages
``` 
    numpy
    scipy
    quadprog
```
* abb_robot_driver
```
https://github.com/ros-industrial/abb_robot_driver
```

## Usage
* compile, as some packages are not standard catkin and compile with only cmake
``` 
catkin_make_isolated -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
``` 

* start yumi_descriotion 
``` 
roslaunch yumi_description display.launch model:='$(find yumi_description)/urdf/yumi.urdf'
``` 
* start tf broadcaster, for seting up transformation tree between the camera and world frame
``` 
rosrun robot_setup_tf tf_broadcaster
``` 
* for simulation ()
``` 
rosrun controller yumi_simulator.py
``` 
* for robot or robotstudio (warning: this activates egm and joint controllers and also closes egm and rapid when set_yumi_settings_and_start.py is closed)
```
roslaunch abb_robot_bringup_examples ex3_rws_and_egm_yumi_robot.launch robot_ip:=<robot controller's IP address>
rosrun robot_setup_tf set_yumi_settings_and_start.py
```

* start kdl_jacobian
``` 
rosrun controller kdl_jacobian
``` 

* Start the controller 
``` 
rosrun controller controllerMaster.py
``` 
* test trajectories 
``` 
rosrun path_planner test_trajectories.py
``` 