This code is based on urdf_tutorial, see the tutorials over at http://wiki.ros.org/urdf_tutorial

The urdf file is from https://github.com/kth-ros-pkg/yumi/tree/egm_modifications/yumi_description. 

To run
```
roslaunch yumi_description display.launch model:='$(find yumi_description)/urdf/yumi.urdf'
```
vizulizes in rviz ans subcribes to joint poisiitons, 