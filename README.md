# DPGO_ROS

## Introduction
This repository implements a ROS wrapper for the DPGO library

## Requirements
This package assumes that [DPGO](https://gitlab.com/mit-acl/dpgo/dpgo) has been built and installed, and that a catkin workspace has been initialized.

## Building the package

Clone this repository:
```
cd ~/catkin_ws/src
git clone git@gitlab.com:mit-acl/dpgo/dpgo_ros.git
``` 

Build with catkin:
```
catkin build
```

To run a demo:
```
# source workspace
source ~/catkin_ws/devel/setup.bash

# run demo on example g2o dataset
roslaunch dpgo_ros DPGO.launch
```