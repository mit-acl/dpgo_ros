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

  // Local Riemannian optimization options
  ros::param::get("~RTR_iterations", params.localOptimizationParams.RTR_iterations);
  ros::param::get("~RTR_tCG_iterations", params.localOptimizationParams.RTR_tCG_iterations);
  ros::param::get("~RTR_gradnorm_tol", params.localOptimizationParams.gradnorm_tol);

  // Local initialization
  std::string initMethodName;
  if (ros::param::get("~local_initialization_method", initMethodName)) {
    if (initMethodName == "Odometry") {
      params.localInitializationMethod = InitializationMethod::Odometry;
    }
    else if (initMethodName == "Chordal") {
      params.localInitializationMethod = InitializationMethod::Chordal;
    }
    else if (initMethodName == "GNC_TLS") {
      params.localInitializationMethod = InitializationMethod::GNC_TLS;
    }
    else {
      ROS_ERROR_STREAM("Invalid local initialization method: " << initMethodName);
    }
  }

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

  // Maximum delayed iterations
  ros::param::get("~max_delayed_iterations", params.maxDelayedIterations);

  // Inter update sleep time
  ros::param::get("~inter_update_sleep_time", params.interUpdateSleepTime);

  // Threshold for determining measurement weight convergence
  ros::param::get("~weight_convergence_threshold", params.weightConvergenceThreshold);

  // Timeout threshold for considering a robot disconnected
  ros::param::get("~timeout_threshold", params.timeoutThreshold);

  // Stopping condition in terms of relative change
  ros::param::get("~relative_change_tolerance", params.relChangeTol);

  // Verbose flag
  ros::param::get("~verbose", params.verbose);

  // Publish iterate during optimization
  ros::param::get("~publish_iterate", params.publishIterate);

  // Publish loop closures as ROS markers for visualization
  ros::param::get("~visualize_loop_closures", params.visualizeLoopClosures);

  // Completely reset dpgo after each distributed optimization round
  ros::param::get("~complete_reset", params.completeReset);

  // Try to recover and resume optimization after disconnection
  ros::param::get("~enable_recovery", params.enableRecovery);

  // Synchronize shared measurements between robots before each optimization round
  ros::param::get("~synchronize_measurements", params.synchronizeMeasurements);

  // Maximum multi-robot initialization attempts 
  ros::param::get("~max_distributed_init_steps", params.maxDistributedInitSteps);

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
    } else if (costName == "L1") {
      params.robustCostParams.costType = RobustCostParameters::Type::L1;
    } else if (costName == "Huber") {
      params.robustCostParams.costType = RobustCostParameters::Type::Huber;
    } else if (costName == "TLS") {
      params.robustCostParams.costType = RobustCostParameters::Type::TLS;
    } else if (costName == "GM") {
      params.robustCostParams.costType = RobustCostParameters::Type::GM;
    } else if (costName == "GNC_TLS") {
      params.robustCostParams.costType = RobustCostParameters::Type::GNC_TLS;
    } else {
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
  ros::param::get("~robust_opt_num_weight_updates", params.robustOptNumWeightUpdates);
  ros::param::get("~robust_opt_num_resets", params.robustOptNumResets);
  ros::param::get("~robust_opt_min_convergence_ratio", params.robustOptMinConvergenceRatio);
  int robust_opt_inner_iters_per_robot = 10;
  ros::param::get("~robust_opt_inner_iters_per_robot", robust_opt_inner_iters_per_robot);
  params.robustOptInnerIters = num_robots * robust_opt_inner_iters_per_robot;
  int robust_init_min_inliers;
  if (ros::param::get("~robust_init_min_inliers", robust_init_min_inliers)) {
    params.robustInitMinInliers = (unsigned) robust_init_min_inliers;
  }

  // Maximum number of iterations
  int max_iters_int;
  if (ros::param::get("~max_iteration_number", max_iters_int))
    params.maxNumIters = (unsigned) max_iters_int;
  // For robust optimization, we set the number of iterations based on the number of GNC iterations
  if (costName != "L2") {
    max_iters_int = (params.robustOptNumWeightUpdates + 1) * params.robustOptInnerIters - 2;
    max_iters_int = std::max(max_iters_int, 0);
    params.maxNumIters = (unsigned) max_iters_int;
  }

  // Update rule
  std::string update_rule_str;
  if (ros::param::get("~update_rule", update_rule_str)) {
    if (update_rule_str == "Uniform") {
      params.updateRule = dpgo_ros::PGOAgentROSParameters::UpdateRule::Uniform;
    } else if (update_rule_str == "RoundRobin") {
      params.updateRule = dpgo_ros::PGOAgentROSParameters::UpdateRule::RoundRobin;
    } else {
      ROS_ERROR_STREAM("Unknown update rule: " << update_rule_str);
      ros::shutdown();
    }
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