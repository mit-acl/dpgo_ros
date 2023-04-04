# dpgo_ros

## Introduction
This repository implements a ROS wrapper for the [distributed pose graph optimization (DPGO) library](https://github.com/mit-acl/dpgo).

<img src="data/sphere_screenshot.png" width="700">

## Dependencies
Inside a catkin workspace, please clone the following repositories. Use the default branch unless mentioned otherwise below.
* [catkin_simple](https://github.com/catkin/catkin_simple)
* [pose_graph_tools](https://github.com/MIT-SPARK/pose_graph_tools/tree/feature/multirobot): some tools and ROS messages for working with pose graphs.
* [dpgo](https://github.com/mit-acl/dpgo): core C++ library for distributed PGO. 
* [dpgo_ros](https://github.com/mit-acl/dpgo_ros): this repository.

## Building the package

Build with catkin:
```
catkin build
```

## Examples

To run a standalone demo using the benchmark Sphere dataset:
```
# source workspace
source ~/catkin_ws/devel/setup.bash

# run demo on example g2o dataset
roslaunch dpgo_ros dpgo_demo.launch
```

## Citations

If you are using this library, please cite the following papers:
```
@ARTICLE{Tian2021Distributed,
  author={Tian, Yulun and Khosoussi, Kasra and Rosen, David M. and How, Jonathan P.},
  journal={IEEE Transactions on Robotics}, 
  title={Distributed Certifiably Correct Pose-Graph Optimization}, 
  year={2021},
  volume={37},
  number={6},
  pages={2137-2156},
  doi={10.1109/TRO.2021.3072346}}

@ARTICLE{Tian2020Asynchronous,
  author={Tian, Yulun and Koppel, Alec and Bedi, Amrit Singh and How, Jonathan P.},
  journal={IEEE Robotics and Automation Letters}, 
  title={Asynchronous and Parallel Distributed Pose Graph Optimization}, 
  year={2020},
  volume={5},
  number={4},
  pages={5819-5826},
  doi={10.1109/LRA.2020.3010216}}
```


