#include "PGOCoordinatorNode.h"

using namespace std;

namespace DPGO_ROS{

	PGOCoordinatorNode::PGOCoordinatorNode(ros::NodeHandle nh_):nh(nh_){

		string Y_topic;
		nh.getParam("/Y_topic", Y_topic);
		YSubscriber = nh.subscribe(Y_topic, 1, &PGOCoordinatorNode::YSubscribeCallback, this);

	}

	PGOCoordinatorNode::~PGOCoordinatorNode(){}



	void PGOCoordinatorNode::YSubscribeCallback(const dpgo_ros::LiftedPoseArrayConstPtr& msg){

	}

}



int main(int argc, char **argv) {
	ros::init(argc, argv, "coordinator_node");
	ros::NodeHandle nh;

	DPGO_ROS::PGOCoordinatorNode node(nh);

}