/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef PGOAGENTNODE_H
#define PGOAGENTNODE_H

#include "distributed/PGOAgent.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <dpgo_ros/LiftedPose.h>
#include <dpgo_ros/LiftedPoseStamped.h>
#include <dpgo_ros/LiftedPoseArray.h>

using namespace std;
using namespace DPGO;
using dpgo_ros::LiftedPose;
using dpgo_ros::LiftedPoseStamped;
using dpgo_ros::LiftedPoseArray;

namespace DPGO_ROS{

class PGOAgentNode{
public:

	PGOAgentNode(ros::NodeHandle nh_, unsigned ID, const PGOAgentParameters& params);


	~PGOAgentNode();


	/** Helper function to reset the internal solution
        In deployment, probably should not use this, except optionally at initialization
     */
	void setY(const Matrix& Yin){
		agent->setY(Yin);
	}


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
    Set maximum stepsize during Riemannian optimization (only used by RGD)
    */
    void setStepsize(double s){
        agent->setStepsize(s);
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


    /**
    Publish the currently estimated trajectory in the local frame
    */
    void trajectoryPublishCallback(const ros::TimerEvent&);


    /**
    Publish the current solution before rounding
    */
    void YPublishCallback(const ros::TimerEvent&);


    /**
	Publish the current values of all shared poses
    */
    void sharedPosePublishCallback(const ros::TimerEvent&);


    /**
    Participate in the bidding of cluster anchor
    */
    void clusterAnchorPublishCallback(const ros::TimerEvent&);


    /**
    Subscribe to shared poses from other robots
    */
    void sharedPoseSubscribeCallback(const dpgo_ros::LiftedPoseArrayConstPtr& msg);


    /**
    Subscribe to cluster anchor topic
    */
    void clusterAnchorSubscribeCallback(const dpgo_ros::LiftedPoseStampedConstPtr& msg);


    /**
	Set up trajectory publisher
    */
    void registerTrajectoryCallback(){
    	trajectoryPublisher = nh.advertise<nav_msgs::Path>("trajectory", 1);
		trajectoryPublishTimer = nh.createTimer(ros::Duration(0.5), &PGOAgentNode::trajectoryPublishCallback, this);
    }


    /**
	Set up pose update publisher and subscriber
    */
    void registerPoseUpdateCallback(const string& topic, const double rate){
    	sharedPosePublisher = nh.advertise<LiftedPoseArray>(topic, 1);
		sharedPosePublishTimer = nh.createTimer(ros::Duration(1/rate), &PGOAgentNode::sharedPosePublishCallback, this);
		sharedPoseSubscriber = nh.subscribe(topic, 1, &PGOAgentNode::sharedPoseSubscribeCallback, this);
    }


    /**
	Set up anchor publisher and subscriber
    */
    void registerAnchorCallback(const string& topic){
    	clusterAnchorPublisher = nh.advertise<LiftedPoseStamped>(topic, 1);
		clusterAnchorPublishTimer = nh.createTimer(ros::Duration(0.5), &PGOAgentNode::clusterAnchorPublishCallback, this);
		clusterAnchorSubscriber = nh.subscribe(topic, 1, &PGOAgentNode::clusterAnchorSubscribeCallback, this);
    }


    /**
	Set up solution publisher (for debugging)
    */
    void registerSolutionCallback(const string& topic){
    	YPublisher = nh.advertise<LiftedPoseArray>(topic, 1);
		YPublishTimer = nh.createTimer(ros::Duration(1), &PGOAgentNode::YPublishCallback, this);
    }



private:

	// Underlying PGOAgent object that stores and optimizes local pose graph
	PGOAgent* agent = nullptr;

	// ROS node handle
	ros::NodeHandle nh;

    // ROS timers
    ros::Timer trajectoryPublishTimer;
    ros::Timer sharedPosePublishTimer;
    ros::Timer clusterAnchorPublishTimer;
    ros::Timer YPublishTimer;
    ros::Timer poseInsertionTimer;

    // ROS publisher
    ros::Publisher trajectoryPublisher;
    ros::Publisher YPublisher;
    ros::Publisher sharedPosePublisher;
    ros::Publisher clusterAnchorPublisher;

    // ROS subscriber
    ros::Subscriber sharedPoseSubscriber;
    ros::Subscriber clusterAnchorSubscriber;


};

}

#endif