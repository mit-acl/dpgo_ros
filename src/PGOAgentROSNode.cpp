/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <dpgo_ros/PGOAgentROS.h>

#include <cassert>
#include <map>

using namespace DPGO;

/**
This script implements the entry point for running a single PGO Agent in ROS
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "agent_node");
  ros::NodeHandle nh;

  /**
  ###########################################
  Read unique ID of this agent
  ###########################################
  */
  int ID = -1;
  ros::param::get("~agent_id", ID);
  if (ID < 0) {
    ROS_ERROR_STREAM("Negative agent id! ");
    return -1;
  }

  /**
  ###########################################
  Load required options
  ###########################################
  */
  int d = -1;
  int r = -1;
  int num_robots = 0;
  if (!ros::param::get("~num_robots", num_robots)) {
    ROS_ERROR("Failed to get number of robots!");
    return -1;
  }
  if (num_robots <= 0) {
    ROS_ERROR_STREAM("Number of robots must be positive!");
    return -1;
  }
  if (ID >= num_robots) {
    ROS_ERROR_STREAM("ID greater than number of robots!");
    return -1;
  }
  if (!ros::param::get("~dimension", d)) {
    ROS_ERROR("Failed to get dimension!");
    return -1;
  }
  if (!ros::param::get("~relaxation_rank", r)) {
    ROS_ERROR("Failed to get relaxation rank!");
    return -1;
  }
  if (d != 3) {
    ROS_ERROR_STREAM("Dimension must be 3!");
    return -1;
  }
  if (r < d) {
    ROS_ERROR_STREAM("Relaxation rank cannot be smaller than dimension!");
    return -1;
  }

  dpgo_ros::PGOAgentROSParameters params(d, r, num_robots);

  /**
  ###########################################
  Load optional options
  ###########################################
  */
  // Run in asynchronous mode
  ros::param::get("~asynchronous", params.asynchronous);

  // Frequency of optimization loop in asynchronous mode
  ros::param::get("~asynchronous_rate", params.asynchronousOptimizationRate);

  // Cross-robot initialization
  ros::param::get("~multirobot_initialization", params.multirobotInitialization);
  if (!params.multirobotInitialization) {
    ROS_WARN("DPGO cross-robot initialization is OFF.");
  }

  // Nesterov acceleration parameters
  ros::param::get("~acceleration", params.acceleration);
  int restart_interval_int;
  if (ros::param::get("~restart_interval", restart_interval_int)) {
    params.restartInterval = (unsigned) restart_interval_int;
  }

  // Maximum number of iterations
  int max_iters_int;
  if (ros::param::get("~max_iteration_number", max_iters_int))
    params.maxNumIters = (unsigned) max_iters_int;

  // Maximum delayed iterations
  ros::param::get("~max_delayed_iterations", params.maxDelayedIterations);

  // Inter update sleep time
  ros::param::get("~inter_update_sleep_time", params.interUpdateSleepTime);
  
  // Stopping condition in terms of relative change
  ros::param::get("~relative_change_tolerance", params.relChangeTol);

  // Verbose flag
  ros::param::get("~verbose", params.verbose);

  // Publish iterate during optimization
  ros::param::get("~publish_iterate", params.publishIterate);

  // Logging
  params.logData = ros::param::get("~log_output_path", params.logDirectory);
  if (params.logDirectory.empty()) {
    params.logData = false;
  }

  // Robust cost function
  std::string costName;
  if (ros::param::get("~robust_cost_type", costName)) {
    if (costName == "L2") {
      params.robustCostParams.costType = RobustCostParameters::Type::L2;
    }
    else if (costName == "L1") {
      params.robustCostParams.costType = RobustCostParameters::Type::L1;
    }
    else if (costName == "Huber") {
      params.robustCostParams.costType = RobustCostParameters::Type::Huber;
    }
    else if(costName == "TLS") {
      params.robustCostParams.costType = RobustCostParameters::Type::TLS;
    }
    else if (costName == "GM") {
      params.robustCostParams.costType = RobustCostParameters::Type::GM;
    }
    else if (costName == "GNC_TLS") {
      params.robustCostParams.costType = RobustCostParameters::Type::GNC_TLS;
    }
    else {
      ROS_ERROR_STREAM("Unknown robust cost type: " << costName);
      ros::shutdown();
    }
  }

  // GNC parameters
  bool gnc_use_quantile = false;
  ros::param::get("~GNC_use_probability", gnc_use_quantile);
  if (gnc_use_quantile) {
    double gnc_quantile = 0.9;
    ros::param::get("~GNC_quantile", gnc_quantile);
    double gnc_barc = RobustCost::computeErrorThresholdAtQuantile(gnc_quantile, 3);
    params.robustCostParams.GNCBarc = gnc_barc;
    ROS_INFO("PGOAgentROS: set GNC confidence quantile at %f (barc %f).", gnc_quantile, gnc_barc);
  } else {
    double gnc_barc = 5.0;
    ros::param::get("~GNC_barc", gnc_barc);
    params.robustCostParams.GNCBarc = gnc_barc;
    ROS_INFO("PGOAgentROS: set GNC barc at %f.", gnc_barc);
  }
  ros::param::get("~GNC_mu_step", params.robustCostParams.GNCMuStep);
  ros::param::get("~GNC_init_mu", params.robustCostParams.GNCInitMu);
  ros::param::get("~robust_opt_warm_start", params.robustOptWarmStart);
  ros::param::get("~robust_opt_min_convergence_ratio", params.robustOptMinConvergenceRatio);
  int robust_opt_inner_iters;
  if (ros::param::get("~robust_opt_inner_iters", robust_opt_inner_iters)) {
    params.robustOptInnerIters = (unsigned) robust_opt_inner_iters;
  }
  int robust_init_min_inliers;
  if (ros::param::get("~robust_init_min_inliers", robust_init_min_inliers)) {
    params.robustInitMinInliers = (unsigned) robust_init_min_inliers;
  }

  // Print params
  ROS_INFO_STREAM("Initializing PGOAgent " << ID << " with params: \n" << params);

  /**
  ###########################################
  Initialize PGO agent
  ###########################################
  */
  dpgo_ros::PGOAgentROS agent(nh, ID, params);
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    agent.runOnce();
    rate.sleep();
  }
  return 0;
}