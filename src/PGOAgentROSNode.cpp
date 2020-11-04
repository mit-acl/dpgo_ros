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

  int ID = -1;
  ros::param::get("~agent_id", ID);
  if (ID < 0) {
    ROS_ERROR_STREAM("Negative agent id! ");
    return -1;
  }

  int num_robots = 0;
  if (!nh.getParam("/num_robots", num_robots)) {
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

  int d = -1;
  int r = -1;
  ROPTALG algorithm = ROPTALG::RTR;
  bool verbose = true;

  if (!nh.getParam("/dimension", d)) {
    ROS_ERROR("Failed to get dimension!");
    return -1;
  }
  if (!nh.getParam("/relaxation_rank", r)) {
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
  ROS_INFO_STREAM("Creating PGO Agent " << ID << " (d = " << d << ", "
                                            << " r = " << r << ")");

  std::string logDir;
  bool logOutput = ros::param::get("~log_output_path", logDir);

  PGOAgentParameters options(d, r, algorithm, verbose, logOutput, logDir);

  dpgo_ros::PGOAgentROS agent(nh, ID, options);

  ros::spin();

  return 0;
}