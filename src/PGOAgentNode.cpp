#include "PGOAgentNode.h"
#include <map>
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

	trajectoryPublisher = nh.advertise<nav_msgs::Path>("trajectory", 1);
	trajectoryPublishTimer = nh.createTimer(ros::Duration(0.5), &PGOAgentNode::trajectoryPublishCallback, this);

	string pose_update_topic;
	nh.getParam("/pose_update_topic", pose_update_topic);
	sharedPosePublisher = nh.advertise<LiftedPoseArray>(pose_update_topic, 1);
	sharedPosePublishTimer = nh.createTimer(ros::Duration(0.5), &PGOAgentNode::sharedPosePublishCallback, this);
	sharedPoseSubscriber = nh.subscribe(pose_update_topic, 1, &PGOAgentNode::sharedPoseSubscribeCallback, this);


	string cluster_anchor_topic;
	nh.getParam("/cluster_anchor_topic", cluster_anchor_topic);
	clusterAnchorPublisher = nh.advertise<LiftedPoseStamped>(cluster_anchor_topic, 1);
	clusterAnchorPublishTimer = nh.createTimer(ros::Duration(0.5), &PGOAgentNode::clusterAnchorPublishCallback, this);
	clusterAnchorSubscriber = nh.subscribe(cluster_anchor_topic, 1, &PGOAgentNode::clusterAnchorSubscribeCallback, this);

	string Y_topic;
	nh.getParam("/Y_topic", Y_topic);
	YPublisher = nh.advertise<LiftedPoseArray>(Y_topic, 1);
	YPublishTimer = nh.createTimer(ros::Duration(0.5), &PGOAgentNode::YPublishCallback, this);

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

		tf::Point tTF(t(0), t(1), t(2));

		// convert rotation matrix to quaternion
		tf::Matrix3x3 RTF(R(0,0),R(0,1),R(0,2),
						  R(1,0),R(1,1),R(1,2),
						  R(2,0),R(2,1),R(2,2));

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
		std_msgs::Float64 scalar;
		for(unsigned row = 0; row < r; ++row){
			for(unsigned col = 0; col < (d+1); ++col){
				scalar.data = Y(row,col);
				poseMsg.pose.push_back(scalar);
			}
		}


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
		unsigned neighborID = poseMsg.robot_id.data;
		unsigned neighborPoseID = poseMsg.pose_id.data;

		if(neighborID == mID) continue;

		// Copy pose data from pose message to Eigen Matrix (row-major)
		Matrix Y = Matrix::Zero(r,d+1);
		for(unsigned row = 0; row < r; ++row){
			for(unsigned col = 0; col < (d+1); ++col){
				unsigned index = row * (d+1) + col;
				std_msgs::Float64 scalar = poseMsg.pose[index];
				Y(row,col) = scalar.data;
			}
		}

		agent->updateNeighborPose(0, neighborID, neighborPoseID, Y);
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
		std_msgs::Float64 scalar;
		for(unsigned row = 0; row < r; ++row){
			for(unsigned col = 0; col < (d+1); ++col){
				scalar.data = Y(row,col);
				msg.pose.pose.push_back(scalar);
			}
		}

		clusterAnchorPublisher.publish(msg);
	}
}


void PGOAgentNode::clusterAnchorSubscribeCallback(const dpgo_ros::LiftedPoseStampedConstPtr& msg){
	// TODO: general setting with initially disconnected global pose graph
	
	unsigned d = agent->dimension();
	unsigned r = agent->relaxation_rank();


	LiftedPose poseMsg = msg->pose;

	// Copy pose data from pose message to Eigen Matrix (row-major)
	Matrix Y = Matrix::Zero(r,d+1);
	for(unsigned row = 0; row < r; ++row){
		for(unsigned col = 0; col < (d+1); ++col){
			unsigned index = row * (d+1) + col;
			std_msgs::Float64 scalar = poseMsg.pose[index];
			Y(row,col) = scalar.data;
		}
	}

	agent->setGlobalAnchor(Y);
}


void PGOAgentNode::YPublishCallback(const ros::TimerEvent&){

}

}
