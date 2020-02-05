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
	PGOAgentNode(ros::NodeHandle nh_, unsigned ID, const PGOAgentParameters& params);

	~PGOAgentNode();

	/**
    Add an odometric measurement of this robot.
    This function automatically initialize the new pose, by propagating odometry
    */
    void addOdometry(const RelativeSEMeasurement& factor){
    	agent->addOdometry(factor);
    }

    /**
    Add a private loop closure of this robot
    */
    void addPrivateLoopClosure(const RelativeSEMeasurement& factor){
    	agent->addPrivateLoopClosure(factor);
    }


    /**
    Add a shared loop closure between this robot and another
    */
    void addSharedLoopClosure(const RelativeSEMeasurement& factor){
    	agent->addSharedLoopClosure(factor);
    }


    /**
	Start the optimization loop in a separate thread
    */
	void startOptimizationLoop(double freq){
		agent->startOptimizationLoop(freq);
	}


	/**
	End the optimization loop, if running
	Execution is blocked until the optimization thread stops.
	*/
	void endOptimizationLoop(){
		agent->endOptimizationLoop();
	}


private:

	// Underlying PGOAgent object that stores and optimizes local pose graph
	PGOAgent* agent = nullptr;

	// ROS node handle
	ros::NodeHandle nh;


};

}

#endif