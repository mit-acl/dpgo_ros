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
  // ROS subscriber
  commandSubscriber =
      nh.subscribe("/dpgo_command", 100, &PGOAgentROS::commandCallback, this);

  poseGraphSubscriber =
      nh.subscribe("pose_graph", 10, &PGOAgentROS::poseGraphCallback, this);

  // ROS service
  queryLiftingMatrixServer = nh.advertiseService(
      "query_lifting_matrix", &PGOAgentROS::queryLiftingMatrixCallback, this);

  // ROS publisher
  commandPublisher = nh.advertise<Command>("/dpgo_command", 100);

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

void PGOAgentROS::update() {
  ROS_INFO_STREAM("Agent " << getID() << " udpates!");

  
  // Randomly select a neighbor to update next
  ros::Duration(0.01).sleep(); 
  unsigned neighborID;
  if (!getRandomNeighbor(neighborID)){
    ROS_ERROR("This agent has no neighbor!");
  }
  Command msg;
  msg.command = Command::UPDATE;
  msg.executing_robot = neighborID;
  commandPublisher.publish(msg);
}

void PGOAgentROS::commandCallback(const CommandConstPtr& msg) {
  switch (msg->command) {
    case Command::INITIALIZE:
      ROS_ERROR("INITIALIZE not implemented!");
      break;

    case Command::TERMINATE:
      ROS_ERROR("TERMINATE not implemented!");
      break;

    case Command::UPDATE:
      if (msg->executing_robot == getID()) {
        // My turn to update!
        update();
      }
      break;

    default:
      ROS_ERROR("Invalid command!");
  }
}

void PGOAgentROS::poseGraphCallback(
    const pose_graph_tools::PoseGraphConstPtr& msg) {
  ROS_INFO_STREAM("Agent " << getID() << " receives " << msg->edges.size()
                           << " edges!");
  vector<RelativeSEMeasurement> odometry;
  vector<RelativeSEMeasurement> privateLoopClosures;
  vector<RelativeSEMeasurement> sharedLoopClosures;
  for (size_t i = 0; i < msg->edges.size(); ++i) {
    pose_graph_tools::PoseGraphEdge edge = msg->edges[i];
    RelativeSEMeasurement m = RelativeMeasurementFromMsg(edge);
    if (m.r1 != getID() && m.r2 != getID()) {
      ROS_ERROR_STREAM("Agent " << getID()
                                << " receives irrelevant measurements!");
    }
    if (m.r1 == m.r2) {
      if (m.p1 + 1 == m.p2) {
        odometry.push_back(m);
      } else {
        privateLoopClosures.push_back(m);
      }
    } else {
      sharedLoopClosures.push_back(m);
    }
  }
  setPoseGraph(odometry, privateLoopClosures, sharedLoopClosures);

  ROS_INFO_STREAM("Agent " << getID() << " created local pose graph with "
                           << num_poses() << " poses.");

  // First robot initiates update sequence
  if (getID() == 0){
    update();
  }
}

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
