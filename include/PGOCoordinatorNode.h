#ifndef PGOCOORDINATORNODE_H
#define PGOCOORDINATORNODE_H

#include <ros/ros.h>
#include <ros/console.h>
#include <dpgo_ros/LiftedPose.h>
#include <dpgo_ros/LiftedPoseStamped.h>
#include <dpgo_ros/LiftedPoseArray.h>

using namespace std;

namespace DPGO_ROS{

class PGOCoordinatorNode{
public:
	
	PGOCoordinatorNode(ros::NodeHandle nh_);

	~PGOCoordinatorNode();

	void YSubscribeCallback(const dpgo_ros::LiftedPoseArrayConstPtr& msg);


private:

	// ROS node handle
	ros::NodeHandle nh;


	// ROS subscriber
	ros::Subscriber YSubscriber;
	


};

	
































}















#endif