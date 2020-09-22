/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <dpgo_ros/PGOAgentROS.h>
#include <dpgo_ros/utils.h>
#include <tf/tf.h>

#include <map>

using namespace DPGO;

namespace dpgo_ros {

PGOAgentROS::PGOAgentROS(ros::NodeHandle nh_, unsigned ID,
                         const PGOAgentParameters& params)
    : PGOAgent(ID, params), nh(nh_) {
  queryLiftingMatrixServer = nh.advertiseService(
      "query_lifting_matrix", &PGOAgentROS::queryLiftingMatrixCallback, this);

  // Query robot 0 for lifting matrix
  if (getID() != 0) {
    std::string service_name = "/dpgo_agent_0/query_lifting_matrix";
    QueryLiftingMatrix query;
    query.request.robot_id = 0;
    bool query_success = ros::service::call(service_name, query);
    if (!query_success) {
      ROS_ERROR_STREAM("Failed to query lifting matrix!");
    }
    Matrix YLift = MatrixFromMsg(query.response.matrix);
    setLiftingMatrix(YLift);
  }
}

PGOAgentROS::~PGOAgentROS() {}

bool PGOAgentROS::queryLiftingMatrixCallback(
    dpgo_ros::QueryLiftingMatrixRequest& request,
    dpgo_ros::QueryLiftingMatrixResponse& response) {
  if (getID() != 0) {
    ROS_ERROR_STREAM("Agent "
                     << getID()
                     << " should not receive request for lifting matrix!");
    return false;
  }
  if (request.robot_id != 0) {
    ROS_ERROR_STREAM("Requested robot ID is not zero! ");
    return false;
  }
  Matrix YLift = getLiftingMatrix();
  response.matrix = MatrixToMsg(YLift);
  return true;
}

}  // namespace dpgo_ros
