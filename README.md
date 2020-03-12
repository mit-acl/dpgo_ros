# DPGO_ROS

ROS wrapper for the DPGO library

## Building the package

This package assumes that DPGO has been built and installed, and that a catkin workspace has been initialized.

Clone this repository:
```
cd ~/catkin_ws/src
git clone git@bitbucket.org:yuluntian/dpgo_ros.git
``` 

Build with catkin:
```
catkin build
```

To run a demo:
```
# source workspace
source ~/catkin_ws/devel/setup.bash

# run demo of asynchronous DPGO
roslaunch dpgo_ros DPGO.launch
```
