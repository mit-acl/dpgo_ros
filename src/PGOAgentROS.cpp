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
#include <pose_graph_tools/utils.h>
#include <tf/tf.h>

#include <map>
#include <random>

using namespace DPGO;

namespace dpgo_ros {

PGOAgentROS::PGOAgentROS(const ros::NodeHandle &nh_, unsigned ID,
                         const PGOAgentParameters &params)
    : PGOAgent(ID, params),
      nh(nh_),
      mOptimizationRequested(false),
      totalBytesReceived(0), iterationElapsedMs(0) {
  mTeamIterRequired.assign(mParams.numRobots, 0);
  mTeamIterReceived.assign(mParams.numRobots, 0);

  // ROS subscriber
  statusSubscriber =
      nh.subscribe("/dpgo_status", 100, &PGOAgentROS::statusCallback, this);

  commandSubscriber =
      nh.subscribe("/dpgo_command", 100, &PGOAgentROS::commandCallback, this);

  anchorSubscriber =
      nh.subscribe("/dpgo_anchor", 100, &PGOAgentROS::anchorCallback, this);

  publicPosesSubscriber =
      nh.subscribe("/dpgo_public_poses", 100, &PGOAgentROS::publicPosesCallback, this);

  // ROS service
  queryLiftingMatrixServer =
      nh.advertiseService("query_lifting_matrix", &PGOAgentROS::queryLiftingMatrixCallback, this);

  // ROS publisher
  anchorPublisher = nh.advertise<LiftedPose>("/dpgo_anchor", 100);
  statusPublisher = nh.advertise<Status>("/dpgo_status", 100);
  commandPublisher = nh.advertise<Command>("/dpgo_command", 100);
  publicPosesPublisher = nh.advertise<PublicPoses>("/dpgo_public_poses", 100);
  poseArrayPublisher = nh.advertise<geometry_msgs::PoseArray>("trajectory", 1);
  pathPublisher = nh.advertise<nav_msgs::Path>("path", 1);

  // Query robot 0 for lifting matrix
  if (getID() != 0) {
    std::string service_name = "/kimera0/dpgo_ros_node/query_lifting_matrix";
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
    ros::Duration(5).sleep();
    publishRequestPoseGraphCommand();
  }
}

void PGOAgentROS::runOnce() {
  if (mOptimizationRequested) {
    // Terminate if this agent does not have pose graph
    if (getState() == PGOAgentState::WAIT_FOR_DATA) {
      publishTerminateCommand();
    }

    // Check if this agent has received latest public poses from its neighbors
    bool ready = true;
    for (unsigned neighbor : getNeighbors()) {
      unsigned requiredIter = mTeamIterRequired[neighbor];
      if (mParams.acceleration) requiredIter = iteration_number() + 1;
      if (mTeamIterReceived[neighbor] < requiredIter) {
        ready = false;
        if (mParams.verbose) {
          ROS_WARN_STREAM(
              "Agent " << getID() << " waiting for neighbor " << neighbor << " to finish iteration " << requiredIter);
        }
      }
    }

    // If ready, perform optimization
    if (ready) {

      // Iterate
      auto startTime = std::chrono::high_resolution_clock::now();
      iterate(true);
      auto counter = std::chrono::high_resolution_clock::now() - startTime;
      iterationElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(counter).count();

      mOptimizationRequested = false;

      // First robot publish anchor
      if (getID() == 0) publishAnchor();

      // Publish status
      publishStatus();

      // Publish trajectory
      publishTrajectory();

      // Publish public poses
      publishPublicPoses(mParams.acceleration);

      // Log local iteration
      if (mParams.logData) {
        logIteration(mParams.logDirectory + "dpgo_log.csv");
      }

      // Check termination condition OR notify next robot to update
      if (shouldTerminate()) {
        publishTerminateCommand();
      } else {
        // Notify next robot to update
        publishUpdateCommand();
      }
    }
  }
}

void PGOAgentROS::reset() {
  PGOAgent::reset();
  mOptimizationRequested = false;
  mTeamIterRequired.assign(mParams.numRobots, 0);
  mTeamIterReceived.assign(mParams.numRobots, 0);
  totalBytesReceived = 0;
}

bool PGOAgentROS::requestPoseGraph() {
  // Query local pose graph
  pose_graph_tools::PoseGraphQuery query;
  query.request.robot_id = getID();
  std::string service_name = "/kimera" + std::to_string(getID()) +
      "/distributed_pcm/request_pose_graph";
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
  if (pose_graph.edges.empty()) {
    ROS_WARN("Received empty pose graph.");
    return false;
  }
  vector<RelativeSEMeasurement> odometry;
  vector<RelativeSEMeasurement> privateLoopClosures;
  vector<RelativeSEMeasurement> sharedLoopClosures;
  for (const auto &edge : pose_graph.edges) {
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

  ROS_INFO_STREAM(
      "Agent " << getID() << " receives local pose graph with "
               << odometry.size() << " odometry edges and "
               << privateLoopClosures.size() << " private loop closures and "
               << sharedLoopClosures.size() << " shared loop closures.");

  return true;
}

void PGOAgentROS::publishAnchor() {
  Matrix T0;
  getSharedPose(0, T0);
  LiftedPose msg = constructLiftedPoseMsg(dimension(), relaxation_rank(),
                                          getCluster(), getID(), 0, T0);
  anchorPublisher.publish(msg);
}

void PGOAgentROS::publishUpdateCommand() {
  Command msg;

  std::vector<unsigned> neighbors = getNeighbors();

  if (neighbors.empty()) {
    ROS_WARN_STREAM("Agent " << getID() << " does not have any neighbor.");
    msg.executing_robot = getID();
  } else {
    std::vector<double> neighborWeights(neighbors.size());
    for (size_t j = 0; j < neighbors.size(); ++j) {
      // TODO: improve over uniform sampling
      neighborWeights[j] = 1;
    }
    std::discrete_distribution<int> distribution(neighborWeights.begin(),
                                                 neighborWeights.end());
    std::random_device rd;
    std::mt19937 gen(rd());
    msg.executing_robot = neighbors[distribution(gen)];
  }

  msg.command = Command::UPDATE;
  msg.executing_iteration = iteration_number() + 1;
  commandPublisher.publish(msg);
}

void PGOAgentROS::publishTerminateCommand() {
  Command msg;
  msg.command = Command::TERMINATE;
  commandPublisher.publish(msg);
}

void PGOAgentROS::publishRequestPoseGraphCommand() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should send request pose graph command! ");
  }
  Command msg;
  msg.command = Command::REQUESTPOSEGRAPH;
  commandPublisher.publish(msg);
}

void PGOAgentROS::publishInitializeCommand() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should send INITIALIZE command!");
  }
  Command msg;
  msg.command = Command::INITIALIZE;
  commandPublisher.publish(msg);
}

void PGOAgentROS::publishStatus() {
  Status msg = statusToMsg(getStatus());
  statusPublisher.publish(msg);
}

bool PGOAgentROS::publishTrajectory() {
  Matrix T;
  if (!getTrajectoryInGlobalFrame(T)) {
    return false;
  }

  // Publish as pose array
  geometry_msgs::PoseArray pose_array =
      TrajectoryToPoseArray(dimension(), num_poses(), T);
  poseArrayPublisher.publish(pose_array);

  // Publish as path
  nav_msgs::Path path = TrajectoryToPath(dimension(), num_poses(), T);
  pathPublisher.publish(path);

  return true;
}

void PGOAgentROS::publishPublicPoses(bool aux) {
  PoseDict map;
  if (aux) {
    if (!getAuxSharedPoseDict(map)) {
      if (mParams.verbose) ROS_WARN_STREAM("Agent " << getID() << " cannot publish public poses! ");
      return;
    }
  } else {
    if (!getSharedPoseDict(map)) {
      if (mParams.verbose) ROS_WARN_STREAM("Agent " << getID() << " cannot publish public poses! ");
      return;
    }
  }

  PublicPoses msg;
  msg.robot_id = getID();
  msg.cluster_id = getCluster();
  msg.instance_number = instance_number();
  msg.iteration_number = iteration_number();
  msg.is_auxiliary = aux;

  for (auto &sharedPose : map) {
    PoseID nID = sharedPose.first;
    Matrix var = sharedPose.second;
    assert(std::get<0>(nID) == getID());
    msg.pose_ids.push_back(std::get<1>(nID));
    msg.poses.push_back(MatrixToMsg(var));
  }
  publicPosesPublisher.publish(msg);
}

bool PGOAgentROS::createLogFile(const std::string &filename) {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }
  // Instance number, global iteration number, Number of poses, total bytes
  // received, iteration time (sec), relative change
  file << "instance, iteration, num_poses, total_bytes_received, "
          "iteration_time_sec, relative_change \n";
  file.close();
  return true;
}

bool PGOAgentROS::logIteration(const std::string &filename) const {
  std::ofstream file;
  file.open(filename, std::ios_base::app);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }

  // Instance number, global iteration number, Number of poses, total bytes
  // received, iteration time (sec), relative change
  file << instance_number() << ",";
  file << iteration_number() << ",";
  file << num_poses() << ",";
  file << totalBytesReceived << ",";
  file << iterationElapsedMs / 1e3 << ",";
  file << mStatus.relativeChange << "\n";
  file.close();
  return true;
}

void PGOAgentROS::anchorCallback(const LiftedPoseConstPtr &msg) {
  if (msg->robot_id != 0 || msg->pose_id != 0) {
    ROS_ERROR("Received wrong pose as anchor!");
    ros::shutdown();
  }
  setGlobalAnchor(MatrixFromMsg(msg->pose));
}

void PGOAgentROS::statusCallback(const StatusConstPtr &msg) {
  // Check that robots are in agreement of current iteration number
  if (msg->instance_number != instance_number()) {
    ROS_ERROR("Instance number does not match!");
  }
  if (msg->iteration_number != iteration_number()) {
    ROS_ERROR("Iteration number does not match!");
  }

  mTeamStatus[msg->robot_id] = statusFromMsg(*msg);
}

void PGOAgentROS::commandCallback(const CommandConstPtr &msg) {
  switch (msg->command) {
    case Command::REQUESTPOSEGRAPH: {
      ROS_INFO_STREAM("Agent " << getID() << " initiates round "
                               << instance_number() << "...");
      // Request latest pose graph
      requestPoseGraph();
      // Create log file for new round
      if (mParams.logData) {
        createLogFile(mParams.logDirectory + "dpgo_log.csv");
      }
      publishStatus();
      // Enter initialization round
      if (getID() == 0) {
        if (getState() == PGOAgentState::INITIALIZED) publishAnchor();
        ros::Duration(1).sleep();
        publishInitializeCommand();
      }
      break;
    }

    case Command::TERMINATE: {
      ROS_INFO_STREAM("Agent " << getID() << " received terminate command. ");
      // Publish optimized trajectory
      publishTrajectory();
      // Reset!
      reset();
      // First robot initiates next optimization round
      if (getID() == 0) {
        ros::Duration(30).sleep();
        publishRequestPoseGraphCommand();
      }
      break;
    }

    case Command::INITIALIZE: {
      publishPublicPoses(false);
      publishTrajectory();
      publishStatus();
      if (getID() == 0) {
        ros::Duration(0.1).sleep();
        for (auto status : mTeamStatus) {
          if (status.state == PGOAgentState::WAIT_FOR_DATA) {
            ROS_WARN_STREAM("Agent " << status.agentID << " has not received data. Terminate this round...");
            publishTerminateCommand();
            return;
          }
          if (status.state != PGOAgentState::INITIALIZED) {
            ROS_WARN_STREAM("Agent " << status.agentID << " not yet initialized.");
            publishInitializeCommand();
            return;
          }
        }
        Command cmdMsg;
        cmdMsg.command = Command::UPDATE;
        cmdMsg.executing_robot = 0;
        cmdMsg.executing_iteration = iteration_number() + 1;
        commandPublisher.publish(cmdMsg);
      }
      break;
    }

    case Command::UPDATE: {
      // Update local record
      mTeamIterRequired[msg->executing_robot] = msg->executing_iteration;
      if (msg->executing_iteration != iteration_number() + 1) {
        ROS_ERROR("Update iteration does not match local iteration!");
      }

      if (msg->executing_robot == getID()) {
        mOptimizationRequested = true;
      } else {
        // Agents that are not selected for optimization can iterate immediately
        iterate(false);
        // When acceleration is used, agents' estimates change at every iteration
        // even if the agent has not been selected to perform optimization
        if (mParams.acceleration) {
          publishPublicPoses(true);
        }
      }
      break;
    }

    default:ROS_ERROR("Invalid command!");
  }
}

void PGOAgentROS::publicPosesCallback(const PublicPosesConstPtr &msg) {

  std::vector<unsigned> neighbors = getNeighbors();
  if (std::find(neighbors.begin(), neighbors.end(), msg->robot_id) == neighbors.end()) {
    // Discard messages send by non-neighbors
    return;
  }

  if (msg->cluster_id != 0) {
    if (mParams.verbose) {
      ROS_WARN("Received poses are not merged in active cluster yet.");
    }
    return;
  }

  for (size_t j = 0; j < msg->pose_ids.size(); ++j) {
    size_t poseID = msg->pose_ids[j];
    Matrix poseVal = MatrixFromMsg(msg->poses[j]);
    if (msg->is_auxiliary) {
      updateAuxNeighborPose(msg->cluster_id, msg->robot_id, poseID, poseVal);
    } else {
      updateNeighborPose(msg->cluster_id, msg->robot_id, poseID, poseVal);
    }
  }

  // Update local bookkeeping
  mTeamIterReceived[msg->robot_id] = msg->iteration_number;
  totalBytesReceived += computePublicPosesMsgSize(*msg);
}

bool PGOAgentROS::queryLiftingMatrixCallback(
    QueryLiftingMatrixRequest &request, QueryLiftingMatrixResponse &response) {
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
  Matrix YLift;
  if (!getLiftingMatrix(YLift)) {
    ROS_ERROR("Queried lifting matrix is not available! ");
    return false;
  }
  response.matrix = MatrixToMsg(YLift);
  return true;
}

}  // namespace dpgo_ros
