# DPGO_ROS

## Introduction
This repository implements a ROS wrapper for the DPGO library

## Dependencies
Clone both this repository and [dpgo](https://gitlab.com/mit-acl/dpgo/dpgo) in the same catkin workspace. In addition, also clone the the `feature/multirobot` branch of [pose_graph_tools](https://github.com/MIT-SPARK/pose_graph_tools/tree/feature/multirobot) in the same catkin workspace.

## Building the package

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
