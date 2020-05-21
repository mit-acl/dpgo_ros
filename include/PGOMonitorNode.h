/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

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

	void shutdown();

private:

	// ROS node handle
	ros::NodeHandle nh;

	// ROS subscriber
	ros::Subscriber YSubscriber;

	// Shutdown timer
	ros::Timer timer;

	// Underlying Riemannian optimization problem
	QuadraticProblem* problem = nullptr;

	// Dictionary that maps local pose (robot_id, pose_id) to a global index 
	map<PoseID, unsigned> PoseMap;

	// Current solution
	Matrix Y;

	// Optimal solution (loaded from MATLAB)
	Matrix Yopt;

	// Store history of optimality gap
	vector<double> optimalityGap;

	// Store history of gradient norm
	vector<double> gradnorm;

	// Store corresponding elapsed time in seconds
	vector<double> elapsedTime;

	// Start time
	ros::Time startTime;

	// Whether all robots are initialized
	vector<bool> initialized;
	bool time_initialized = false;


};

	
































}















#endif