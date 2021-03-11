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

  PGOAgentParameters params(d, r, num_robots);

  /**
  ###########################################
  Load optional options
  ###########################################
  */
  // Nesterov acceleration parameters
  ros::param::get("~acceleration", params.acceleration);
  int restart_interval_int;
  if (ros::param::get("~restart_interval", restart_interval_int)) {
    params.restartInterval = (unsigned) restart_interval_int;
  }

  // Robust cost function
  std::string costName;
  if (ros::param::get("~robust_cost_type", costName)) {
    if (costName == "L2") {
      params.robustCostType = RobustCostType::L2;
    }
    else if (costName == "L1") {
      params.robustCostType = RobustCostType::L1;
    }
    else if (costName == "Huber") {
      params.robustCostType = RobustCostType::Huber;
    }
    else if(costName == "TLS") {
      params.robustCostType = RobustCostType::TLS;
    }
    else if (costName == "GM") {
      params.robustCostType = RobustCostType::GM;
    }
    else if (costName == "GNC_TLS") {
      params.robustCostType = RobustCostType::GNC_TLS;
    }
    else {
      ROS_ERROR_STREAM("Unknown robust cost type: " << costName);
      ros::shutdown();
    }
  }
  //ros::param::get("~GNC_barc", params.robustCostParams.GNCBarc);
  double gnc_quantile;
  if (ros::param::get("~GNC_quantile", gnc_quantile)) {
    double barc = RobustCost::computeErrorThresholdAtQuantile(gnc_quantile, 3);
    params.robustCostParams.GNCBarc = barc;
    ROS_INFO("PGOAgentRos: set GNC confidence at %f, barcsq: %f\n", gnc_quantile, barc * barc);
  }
  ros::param::get("~GNC_mu_step", params.robustCostParams.GNCMuStep);
  ros::param::get("~GNC_init_mu", params.robustCostParams.GNCInitMu);
  ros::param::get("~min_converged_loop_closure_ratio", params.minConvergedLoopClosureRatio);
  int weight_update_int;
  if (ros::param::get("~weight_update_interval", weight_update_int)) {
    params.weightUpdateInterval = (unsigned) weight_update_int;
  }

  int max_iters_int;
  if (ros::param::get("~max_iteration_number", max_iters_int))
    params.maxNumIters = (unsigned) max_iters_int;
  ros::param::get("~relative_change_tolerance", params.relChangeTol);
  ros::param::get("~verbose", params.verbose);
  params.logData = ros::param::get("~log_output_path", params.logDirectory);

  /**
  ###########################################
  Initialize PGO agent
  ###########################################
  */
  dpgo_ros::PGOAgentROS agent(nh, ID, params);
  ROS_INFO_STREAM("Initialized PGO Agent " << ID << ".");
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    agent.runOnce();
    rate.sleep();
  }
  return 0;
}