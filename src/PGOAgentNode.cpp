/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */


#include "PGOAgentNode.h"
#include <map>
#include "dataUtils.h"
#include "DPGO_utils.h"
#include "DPGO_types.h"
#include "RelativeSEMeasurement.h"
#include <tf/tf.h>


using namespace std;
using namespace DPGO;
using dpgo_ros::LiftedPose;
using dpgo_ros::LiftedPoseStamped;
using dpgo_ros::LiftedPoseArray;

namespace DPGO_ROS{

PGOAgentNode::PGOAgentNode(ros::NodeHandle nh_, unsigned ID, const PGOAgentParameters& params):nh(nh_){
	agent = new PGOAgent(ID, params);
}

PGOAgentNode::~PGOAgentNode()
{	
	delete agent; // this also ensures that optimization thread is not running
}


void PGOAgentNode::trajectoryPublishCallback(const ros::TimerEvent&){
	unsigned n = agent->num_poses();
	unsigned d = agent->dimension();
	Matrix T = agent->getTrajectoryInGlobalFrame();

	// initialize ROS message
	nav_msgs::Path trajectory;
	trajectory.header.frame_id = "map";

	// populate path message
	for (unsigned i = 0; i < n; ++i){
		Matrix R = T.block(0,i*(d+1),d,d);
		Matrix t = T.block(0,i*(d+1)+d,d,1);

		tf::Point tTF;
		tf::Matrix3x3 RTF;

		if(d == 3){
			tTF.setValue(t(0), t(1), t(2));
			
			RTF.setValue(R(0,0),R(0,1),R(0,2),
						 R(1,0),R(1,1),R(1,2),
						 R(2,0),R(2,1),R(2,2));
		}
		else{
			assert(d == 2);
			tTF.setValue(t(0), t(1), 0);

			RTF.setValue(R(0,0),R(0,1),0,
						 R(1,0),R(1,1),0,
						 0,     0,     0);
		}

		tf::Quaternion quatTF;
		RTF.getRotation(quatTF);

		tf::Pose poseTF(quatTF, tTF);
		geometry_msgs::Pose poseMsg;
		tf::poseTFToMsg(poseTF, poseMsg);

		// create stamped message
		geometry_msgs::PoseStamped poseMsgStamped;
		poseMsgStamped.pose = poseMsg;
		poseMsgStamped.header.stamp = ros::Time::now();
		poseMsgStamped.header.frame_id = "map";

		trajectory.poses.push_back(poseMsgStamped);
	}

	trajectoryPublisher.publish(trajectory);
}


void PGOAgentNode::sharedPosePublishCallback(const ros::TimerEvent&){
	ros::Time timestamp = ros::Time::now();
	PoseDict sharedPoses = agent->getSharedPoses();

	unsigned r = agent->relaxation_rank();
	unsigned d = agent->dimension();
	unsigned cluster = agent->getCluster();

	LiftedPoseArray arrayMsg; 
	arrayMsg.header.stamp = timestamp;

	for(auto it = sharedPoses.begin(); it != sharedPoses.end(); ++it){
		PoseID nID = it->first;
		Matrix Y = it->second;

		LiftedPose poseMsg;
		poseMsg.dimension.data = d;
		poseMsg.relaxation_rank.data = r;
		poseMsg.cluster_id.data = cluster;
		poseMsg.robot_id.data = get<0>(nID);
		poseMsg.pose_id.data =  get<1>(nID);

		// Copy pose data from Eigen Matrix to pose message (row-major)
		poseMsg.pose = serializeMatrix(r,d+1,Y);

		arrayMsg.poses.push_back(poseMsg);
	}

	sharedPosePublisher.publish(arrayMsg);

}


void PGOAgentNode::sharedPoseSubscribeCallback(const dpgo_ros::LiftedPoseArrayConstPtr& msg){

	unsigned r = agent->relaxation_rank();
	unsigned d = agent->dimension();
	unsigned mID = agent->getID();

	for(size_t i = 0; i < msg->poses.size(); ++i){

		LiftedPose poseMsg = msg->poses[i];
		unsigned clusterID = poseMsg.cluster_id.data;
		unsigned neighborID = poseMsg.robot_id.data;
		unsigned neighborPoseID = poseMsg.pose_id.data;

		if(neighborID == mID) continue;

		// Copy pose data from pose message to Eigen Matrix (row-major)
		Matrix Y = deserializeMatrix(r,d+1,poseMsg.pose);

		agent->updateNeighborPose(clusterID, neighborID, neighborPoseID, Y);
	}
}


void PGOAgentNode::clusterAnchorPublishCallback(const ros::TimerEvent&){
	// TODO: general setting with initially disconnected global pose graph

	unsigned mID = agent->getID();
	unsigned d = agent->dimension();
	unsigned r = agent->relaxation_rank();
	
	
	if(mID == 0){
		LiftedPoseStamped msg;
		msg.header.stamp = ros::Time::now();

		msg.pose.dimension.data = d;
		msg.pose.relaxation_rank.data = r;
		msg.pose.cluster_id.data = agent->getCluster();
		msg.pose.robot_id.data = agent->getID();
		msg.pose.pose_id.data = 0; // always use the first pose as anchor

		// Copy pose data from Eigen Matrix to pose message (row-major)
		Matrix Y = agent->getYComponent(0);
		msg.pose.pose = serializeMatrix(r,d+1,Y);

		clusterAnchorPublisher.publish(msg);
	}
}


void PGOAgentNode::clusterAnchorSubscribeCallback(const dpgo_ros::LiftedPoseStampedConstPtr& msg){
	// TODO: general setting with initially disconnected global pose graph
	
	unsigned d = agent->dimension();
	unsigned r = agent->relaxation_rank();


	LiftedPose poseMsg = msg->pose;

	// Copy pose data from pose message to Eigen Matrix (row-major)
	Matrix Y = deserializeMatrix(r,d+1,poseMsg.pose);

	agent->setGlobalAnchor(Y);
}


void PGOAgentNode::YPublishCallback(const ros::TimerEvent&){

	ros::Time timestamp = ros::Time::now();
	unsigned n = agent->num_poses();
	unsigned r = agent->relaxation_rank();
	unsigned d = agent->dimension();
	unsigned cluster = agent->getCluster();
	unsigned mID = agent->getID();
	Matrix Y = agent->getY();

	LiftedPoseArray arrayMsg; 
	arrayMsg.header.stamp = timestamp;

	for(unsigned i = 0; i < n; ++i){
		Matrix Yi = Y.block(0,i*(d+1),r,d+1);

		LiftedPose poseMsg;
		poseMsg.dimension.data = d;
		poseMsg.relaxation_rank.data = r;
		poseMsg.cluster_id.data = cluster;
		poseMsg.robot_id.data = mID;
		poseMsg.pose_id.data =  i;

		// Copy pose data from Eigen Matrix to pose message (row-major)
		poseMsg.pose = serializeMatrix(r,d+1,Yi);

		arrayMsg.poses.push_back(poseMsg);
	}

	YPublisher.publish(arrayMsg);

}

void PGOAgentNode::poseInsertionCallback(const ros::TimerEvent&){

	if (OdometryQueue.empty()) return;

	// add the next odometry measurement
	addOdometry(OdometryQueue[0]);
	OdometryQueue.erase(OdometryQueue.begin());

	// add all loop closures attached to the new pose
	size_t index = agent->num_poses() - 1;
	for(size_t i = 0; i < PrivateLoopClosureQueue[index].size(); ++i){
		addPrivateLoopClosure(PrivateLoopClosureQueue[index][i]);
	}
	for(size_t i = 0; i < SharedLoopClosureQueue[index].size(); ++i){
		addSharedLoopClosure(SharedLoopClosureQueue[index][i]);
	}

}

}
