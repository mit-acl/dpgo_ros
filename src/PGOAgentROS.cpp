/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <dpgo_ros/PGOAgentROS.h>
#include <dpgo_ros/utils.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <pose_graph_tools/PoseGraphQuery.h>
#include <tf/tf.h>

#include <map>

using namespace DPGO;

namespace dpgo_ros {

PGOAgentROS::PGOAgentROS(ros::NodeHandle nh_, unsigned ID,
                         const PGOAgentParameters& params)
    : PGOAgent(ID, params), nh(nh_), instance_number(0), iteration_number(0) {
  if (!nh.getParam("/relative_change_tolerance", RelativeChangeTolerance)) {
    ROS_ERROR("Failed to get relative change tolerance!");
    ros::shutdown();
  }

  int num_robots;
  if (!nh.getParam("/num_robots", num_robots))
    ROS_ERROR("Failed to query number of robots");
  relativeChanges.resize(num_robots, 1e3);

  // ROS subscriber
  statusSubscriber =
      nh.subscribe("/dpgo_status", 100, &PGOAgentROS::statusCallback, this);

  commandSubscriber =
      nh.subscribe("/dpgo_command", 100, &PGOAgentROS::commandCallback, this);

  // ROS service
  queryLiftingMatrixServer = nh.advertiseService(
      "query_lifting_matrix", &PGOAgentROS::queryLiftingMatrixCallback, this);

  queryPoseServer = nh.advertiseService("query_poses",
                                        &PGOAgentROS::queryPosesCallback, this);

  // ROS publisher
  statusPublisher = nh.advertise<Status>("/dpgo_status", 100);
  commandPublisher = nh.advertise<Command>("/dpgo_command", 100);
  poseArrayPublisher = nh.advertise<geometry_msgs::PoseArray>("trajectory", 1);
  pathPublisher = nh.advertise<nav_msgs::Path>("path", 1);

  // Query robot 0 for lifting matrix
  if (getID() != 0) {
    std::string service_name = "/dpgo_agent_0/query_lifting_matrix";
    QueryLiftingMatrix query;
    query.request.robot_id = 0;
    if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
      ROS_ERROR("Service to query lifting matrix does not exist!");
      ros::shutdown();
    }
    if (!ros::service::call(service_name, query)) {
      ROS_ERROR("Failed to query lifting matrix!");
      ros::shutdown();
    }
    Matrix YLift = MatrixFromMsg(query.response.matrix);
    setLiftingMatrix(YLift);
  }

  // First agent sends out the initialization signal
  if (getID() == 0) {
    ros::Duration(3.0).sleep();
    Command msg;
    msg.command = Command::INITIALIZE;
    commandPublisher.publish(msg);
  }
}

PGOAgentROS::~PGOAgentROS() {}

void PGOAgentROS::reset() {
  PGOAgent::reset();
  instance_number++;
  iteration_number = 0;
  for (size_t i = 0; i < relativeChanges.size(); ++i) {
    relativeChanges[i] = 1e3;
  }
}

void PGOAgentROS::update() {
  ROS_INFO_STREAM("Agent " << getID() << " udpating...");

  // Query neighbors for their public poses
  std::set<unsigned> neighborAgents = getNeighbors();
  for (unsigned neighborID : neighborAgents) {
    if (!requestPublicPosesFromAgent(neighborID)) {
      ROS_WARN_STREAM("Public poses from neighbor " << neighborID
                                                    << " are not available.");
    }
  }

  // Optimize!
  OptResult = optimize();
  if (!OptResult.success) {
    ROS_WARN("Skipped optimization!");
  } else {
    relativeChanges[getID()] = OptResult.relativeChange;
  }

  // Publish trajectory
  if (!publishTrajectory()) {
    ROS_ERROR("Failed to publish trajectory in global frame!");
  }

  // Publish status
  publishStatus();

  // Publish next command
  ros::Duration(0.01).sleep();
  publishCommand();
}

bool PGOAgentROS::requestPoseGraph() {
  // Query local pose graph
  pose_graph_tools::PoseGraphQuery query;
  query.request.robot_id = getID();
  std::string service_name =
      "/dpgo_agent_" + std::to_string(getID()) + "/query_pose_graph";
  if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
    ROS_ERROR_STREAM("ROS service " << service_name << " does not exist!");
    return false;
  }
  if (!ros::service::call(service_name, query)) {
    ROS_ERROR_STREAM("Failed to call ROS service " << service_name);
    return false;
  }

  // Set pose graph
  pose_graph_tools::PoseGraph pose_graph = query.response.pose_graph;
  vector<RelativeSEMeasurement> odometry;
  vector<RelativeSEMeasurement> privateLoopClosures;
  vector<RelativeSEMeasurement> sharedLoopClosures;
  for (size_t i = 0; i < pose_graph.edges.size(); ++i) {
    pose_graph_tools::PoseGraphEdge edge = pose_graph.edges[i];
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

  ROS_INFO_STREAM("Agent " << getID() << " receives local pose graph with "
                           << pose_graph.edges.size() << " edges"
                           << " and " << num_poses() << " poses.");

  return true;
}

bool PGOAgentROS::requestPublicPosesFromAgent(const unsigned& neighborID) {
  std::vector<unsigned> poseIndices = getNeighborPublicPoses(neighborID);

  // Call ROS service
  QueryPoses srv;
  srv.request.robot_id = neighborID;
  for (size_t i = 0; i < poseIndices.size(); ++i) {
    srv.request.pose_ids.push_back(poseIndices[i]);
  }
  std::string service_name =
      "/dpgo_agent_" + std::to_string(neighborID) + "/query_poses";

  if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
    ROS_ERROR_STREAM("ROS service " << service_name << " does not exist!");
    return false;
  }
  if (!ros::service::call(service_name, srv)) {
    ROS_ERROR_STREAM("Failed to call ROS service " << service_name);
    return false;
  }
  if (srv.response.poses.size() != srv.request.pose_ids.size()) {
    ROS_ERROR(
        "Number of replied poses does not match number of requested pose!");
    return false;
  }

  if (srv.response.poses[0].cluster_id != 0) {
    ROS_WARN("Received poses are not merged in active cluster yet.");
    return false;
  }

  for (size_t i = 0; i < srv.response.poses.size(); ++i) {
    LiftedPose poseNbr = srv.response.poses[i];
    Matrix Xnbr = MatrixFromMsg(poseNbr.pose);
    updateNeighborPose(poseNbr.cluster_id, poseNbr.robot_id, poseNbr.pose_id,
                       Xnbr);
  }

  return true;
}

void PGOAgentROS::publishCommand() {
  // Check termination condition
  bool should_terminate = true;
  for (size_t i = 0; i < relativeChanges.size(); ++i) {
    if (relativeChanges[i] > RelativeChangeTolerance) {
      should_terminate = false;
      break;
    }
  }
  // Publish!
  Command msg;
  if (should_terminate) {
    msg.command = Command::TERMINATE;
  } else {
    // Randomly select a neighbor to update next
    unsigned neighborID;
    if (!getRandomNeighbor(neighborID)) {
      ROS_ERROR("Failed to get random neighbor!");
    }
    msg.command = Command::UPDATE;
    msg.executing_robot = neighborID;
  }
  commandPublisher.publish(msg);
}

void PGOAgentROS::publishStatus() {
  Status msg;
  msg.robot_id = getID();
  msg.instance_number = instance_number;
  msg.iteration_number = iteration_number;
  msg.optimization_success = OptResult.success;
  msg.relative_change = OptResult.relativeChange;
  msg.objective_decrease = OptResult.fInit - OptResult.fOpt;
  statusPublisher.publish(msg);
}

bool PGOAgentROS::publishTrajectory() {
  Matrix globalAnchor;
  if (getID() == 0) {
    getXComponent(0, globalAnchor);
  } else {
    // Request global anchor from robot 0
    QueryPoses srv;
    srv.request.robot_id = 0;
    srv.request.pose_ids.push_back(0);
    std::string service_name = "/dpgo_agent_0/query_poses";
    if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
      ROS_ERROR_STREAM("ROS service " << service_name << " does not exist!");
      return false;
    }
    if (!ros::service::call(service_name, srv)) {
      ROS_ERROR_STREAM("Failed to call ROS service " << service_name);
      return false;
    }

    globalAnchor = MatrixFromMsg(srv.response.poses[0].pose);
  }

  Matrix T = getTrajectoryInGlobalFrame(globalAnchor);

  // Publish as pose array
  geometry_msgs::PoseArray pose_array =
      TrajectoryToPoseArray(dimension(), num_poses(), T);
  poseArrayPublisher.publish(pose_array);

  // Publish as path
  nav_msgs::Path path = TrajectoryToPath(dimension(), num_poses(), T);
  pathPublisher.publish(path);

  return true;
}

void PGOAgentROS::statusCallback(const StatusConstPtr& msg) {
  // Check that robots are in agreement of current iteration number
  if (msg->instance_number != instance_number) {
    ROS_ERROR("Instance number does not match!");
  }
  if (msg->iteration_number != iteration_number) {
    ROS_ERROR("Iteration number does not match!");
  }
  if (msg->optimization_success) {
    relativeChanges[msg->robot_id] = msg->relative_change;
  }
}

void PGOAgentROS::commandCallback(const CommandConstPtr& msg) {
  switch (msg->command) {
    case Command::INITIALIZE:
      ROS_INFO_STREAM("Agent " << getID() << " initiating...");
      requestPoseGraph();
      // First robot initiates update sequence
      if (getID() == 0) {
        ros::Duration(3).sleep();
        Command msg;
        msg.command = Command::UPDATE;
        msg.executing_robot = 0;
        commandPublisher.publish(msg);
      }
      break;

    case Command::TERMINATE:
      ROS_INFO_STREAM("Agent " << getID() << " reset.");
      reset();
      // First robot initiates next optimization round
      if (getID() == 0) {
        ros::Duration(3).sleep();
        Command msg;
        msg.command = Command::INITIALIZE;
        commandPublisher.publish(msg);
      }
      break;

    case Command::UPDATE:
      iteration_number++;  // increment iteration counter
      if (msg->executing_robot == getID()) {
        // My turn to update!
        update();
      }
      break;

    default:
      ROS_ERROR("Invalid command!");
  }
}

bool PGOAgentROS::queryLiftingMatrixCallback(
    QueryLiftingMatrixRequest& request, QueryLiftingMatrixResponse& response) {
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

bool PGOAgentROS::queryPosesCallback(QueryPosesRequest& request,
                                     QueryPosesResponse& response) {
  if (request.robot_id != getID()) {
    ROS_ERROR("Pose query addressed to wrong agent!");
    return false;
  }

  for (size_t i = 0; i < request.pose_ids.size(); ++i) {
    unsigned poseIndex = request.pose_ids[i];
    Matrix Xi;
    if (!getXComponent(poseIndex, Xi)) {
      ROS_ERROR("Requested pose index does not exist!");
      return false;
    }
    LiftedPose pose = constructLiftedPoseMsg(
        dimension(), relaxation_rank(), getCluster(), getID(), poseIndex, Xi);
    response.poses.push_back(pose);
  }

  return true;
}

}  // namespace dpgo_ros
