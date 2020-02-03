#ifndef PGOAGENTNODE_H
#define PGOAGENTNODE_H

#include "distributed/PGOAgent.h"

#include <ros/ros.h>
#include <ros/console.h>

using namespace std;
using namespace DPGO;

namespace DPGO_ROS{

class PGOAgentNode{
public:
	PGOAgentNode(ros::NodeHandle nh_);

	~PGOAgentNode();

	/**
	Initialize the PGOAgent object
	*/
	void initialize(unsigned ID, const PGOAgentParameters& params);


	void startOptimizationLoop(double freq);


	void endOptimizationLoop();


private:

	// Underlying PGOAgent object that stores and optimizes local pose graph
	PGOAgent* agent = nullptr;

	// ROS node handle
	ros::NodeHandle nh;


};

}

#endif