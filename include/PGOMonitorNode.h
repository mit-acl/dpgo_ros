#ifndef PGOMONITORNODE_H
#define PGOMONITORNODE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <dpgo_ros/LiftedPose.h>
#include <dpgo_ros/LiftedPoseStamped.h>
#include <dpgo_ros/LiftedPoseArray.h>
#include "QuadraticProblem.h"

using namespace std;
using namespace DPGO;

namespace DPGO_ROS{

class PGOMonitorNode{
public:
	
	PGOMonitorNode(ros::NodeHandle nh_);

	~PGOMonitorNode();

	void YSubscribeCallback(const dpgo_ros::LiftedPoseArrayConstPtr& msg);


private:

	// ROS node handle
	ros::NodeHandle nh;

	// ROS subscriber
	ros::Subscriber YSubscriber;

	// Underlying Riemannian optimization problem
	QuadraticProblem* problem = nullptr;

	// Dictionary that maps local pose (robot_id, pose_id) to a global index 
	map<PoseID, unsigned> PoseMap;

	// Current solution
	Matrix Y;

	// Optimal solution (loaded from MATLAB)
	Matrix Yopt;

};

	
































}















#endif