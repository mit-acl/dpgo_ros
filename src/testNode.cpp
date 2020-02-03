#include "distributed/PGOAgent.h"
#include <ros/ros.h>

using namespace DPGO;

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_node");
	ros::NodeHandle nh;

	PGOAgentParameters options(3,5,ROPTALG::RTR,true);
	PGOAgent agent(0, options);

	ros::spin();

	return 0;
}