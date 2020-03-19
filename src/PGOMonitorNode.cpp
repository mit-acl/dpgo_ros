/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */


#include "PGOMonitorNode.h"
#include <map>
#include <iostream>
#include <fstream>
#include "DPGO_utils.h"
#include "DPGO_types.h"
#include "RelativeSEMeasurement.h"
#include "dataUtils.h"


using namespace std;
using namespace DPGO;
using dpgo_ros::LiftedPose;
using dpgo_ros::LiftedPoseArray;

namespace DPGO_ROS{

	PGOMonitorNode::PGOMonitorNode(ros::NodeHandle nh_):nh(nh_){

		int num_robots = 0;
		nh.getParam("/num_robots", num_robots);
		ROS_WARN_STREAM("Number of robots: " << num_robots << ".");
		for(int i = 0; i < num_robots; ++i) initialized.push_back(false);

		string filename;
		nh.getParam("/dataset", filename);
		size_t N = 0; 
	    vector<RelativeSEMeasurement> dataset = DPGO::read_g2o_file(filename, N);
	    ROS_WARN_STREAM("Loaded dataset: " << filename << ".");


	    unsigned int num_poses_per_robot = N / num_robots;
	   	ROS_WARN_STREAM("Creating mapping from local to global pose... ");
	    for(unsigned robot = 0; robot < (unsigned) num_robots; ++robot){
	        unsigned startIdx = robot * num_poses_per_robot;
	        unsigned endIdx = (robot+1) * num_poses_per_robot; // non-inclusive
	        if (robot == (unsigned) num_robots - 1) endIdx = N;
	        for(unsigned idx = startIdx; idx < endIdx; ++idx){
	            unsigned localIdx = idx - startIdx; // this is the local ID of this pose
	            PoseID pose = make_pair(robot, localIdx);
	            PoseMap[pose] = idx;
	        }
	    }


	    int d = -1;
	    int r = -1;
		nh.getParam("/dimension", d);
		nh.getParam("/relaxation_rank", r);
		if (d <= 0){
			ROS_ERROR_STREAM("Dimendion cannot be negative!");
			ros::shutdown();
		}
		if (r < d){
			ROS_ERROR_STREAM("Relaxation rank cannot be smaller than dimension!");
			ros::shutdown();
		}


		Y = Matrix::Zero(r, (d+1) * N);
		SparseMatrix Q = constructConnectionLaplacianSE(dataset);
		SparseMatrix G(r, (d+1) * N);
		G.setZero();
		problem = new QuadraticProblem(N, d, r, Q, G);


		nh.getParam("/Yopt", filename);
		Yopt = read_matrix_from_file(filename);


		string solution_topic;
		nh.getParam("/solution_topic", solution_topic);
		YSubscriber = nh.subscribe(solution_topic, 1, &PGOMonitorNode::YSubscribeCallback, this);

		timer = nh.createTimer(ros::Duration(60), &PGOMonitorNode::shutdownCallback, this);
	}


	PGOMonitorNode::~PGOMonitorNode(){ delete problem; }


	void PGOMonitorNode::YSubscribeCallback(const dpgo_ros::LiftedPoseArrayConstPtr& msg){
		unsigned r = problem->relaxation_rank();
		unsigned d = problem->dimension();
		unsigned robot = msg->poses[0].robot_id.data;
		initialized[robot] = true;

		// Update the solution 
		for(size_t i = 0; i < msg->poses.size(); ++i){
			LiftedPose poseMsg = msg->poses[i];

			PoseID localID = make_pair(poseMsg.robot_id.data, poseMsg.pose_id.data);
			unsigned index = PoseMap[localID];

			Y.block(0,index*(d+1),r,d+1) = deserializeMatrix(r,d+1,poseMsg.pose);
		}


		// Do not record result if not all robots have been initialized
		for(size_t i = 0; i < initialized.size(); ++i){
			if (!initialized[i]) return;
		}

		if(!time_initialized){
			startTime = ros::Time::now();
			time_initialized = true;
		}

		double t = ros::Time::now().toSec() - startTime.toSec();
		optimalityGap.push_back(problem->f(Y) - problem->f(Yopt));
		gradnorm.push_back(problem->gradNorm(Y));
		elapsedTime.push_back(t);

		auto minOptGapIt = std::min_element(optimalityGap.begin(), optimalityGap.end());
		auto minGradNormIt = std::min_element(gradnorm.begin(), gradnorm.end());
		ROS_WARN_STREAM("Min optimality gap = " << *minOptGapIt << "; min gradnorm = " << *minGradNormIt);

	}

	void PGOMonitorNode::shutdownCallback(const ros::TimerEvent&){

		// save results to file
		std::string filename = "/home/yulun/catkin_ws/src/dpgo_ros/results/result.csv";
		std::ofstream file(filename.c_str());
		if (file.is_open()){
			ROS_INFO_STREAM("Writing results to file...");
			for(size_t i = 0; i < optimalityGap.size(); ++i){
				std::cout << ".";
				file << optimalityGap[i] << ","
				     << gradnorm[i] << ","
				     << elapsedTime[i] << std::endl;
			}
		    file.close();
		    std::cout << endl;
		}

		filename = "/home/yulun/catkin_ws/src/dpgo_ros/results/Y.csv";
		std::ofstream file2(filename.c_str());
		if (file2.is_open()){
			file2 << Y;
		    file2.close();
		}

		ros::shutdown();
	}
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "coordinator_node");
	
	ros::NodeHandle nh;

	DPGO_ROS::PGOMonitorNode node(nh);

	ros::spin();

	return 0;

}