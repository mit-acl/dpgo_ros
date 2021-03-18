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
      mInitStepsDone(0),
      mTotalBytesReceived(0),
      mIterationElapsedMs(0) {
  mTeamIterRequired.assign(mParams.numRobots, 0);
  mTeamIterReceived.assign(mParams.numRobots, 0);

  // ROS subscriber
  mStatusSubscriber =
      nh.subscribe("/dpgo_status", 100, &PGOAgentROS::statusCallback, this);

  mCommandSubscriber =
      nh.subscribe("/dpgo_command", 100, &PGOAgentROS::commandCallback, this);

  mAnchorSubscriber =
      nh.subscribe("/dpgo_anchor", 100, &PGOAgentROS::anchorCallback, this);

  mPublicPosesSubscriber =
      nh.subscribe("/dpgo_public_poses", 100, &PGOAgentROS::publicPosesCallback, this);

  mMeasurementWeightsSubscriber =
      nh.subscribe("/dpgo_measurement_weights", 100, &PGOAgentROS::measurementWeightsCallback, this);

  // ROS service
  mQueryLiftingMatrixServer =
      nh.advertiseService("query_lifting_matrix", &PGOAgentROS::queryLiftingMatrixCallback, this);

  // ROS publisher
  mAnchorPublisher = nh.advertise<PublicPoses>("/dpgo_anchor", 100);
  mStatusPublisher = nh.advertise<Status>("/dpgo_status", 100);
  mCommandPublisher = nh.advertise<Command>("/dpgo_command", 100);
  mMeasurementWeightsPublisher = nh.advertise<RelativeMeasurementWeights>("/dpgo_measurement_weights", 100);
  mPublicPosesPublisher = nh.advertise<PublicPoses>("/dpgo_public_poses", 100);
  mPoseArrayPublisher = nh.advertise<geometry_msgs::PoseArray>("trajectory", 5);
  mPathPublisher = nh.advertise<nav_msgs::Path>("path", 5);
  mPoseGraphPublisher = nh.advertise<pose_graph_tools::PoseGraph>("optimized_pose_graph", 5);

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
      int requiredIter = mTeamIterRequired[neighbor];
      if (mParams.acceleration) requiredIter = (int) iteration_number() + 1;
      // TODO: allow delays to speed up executions
      requiredIter = requiredIter - 3;
      if ((int) mTeamIterReceived[neighbor] < requiredIter) {
        ready = false;
        if (mParams.verbose) {
          ROS_WARN("Robot %u iteration %u waits for neighbor %u iteration %u (last received iteration %u).",
                   getID(), iteration_number() + 1, neighbor, requiredIter, mTeamIterReceived[neighbor]);
        }
      }
    }

    // If ready, perform optimization
    if (ready) {

      // Iterate
      auto startTime = std::chrono::high_resolution_clock::now();
      iterate(true);
      auto counter = std::chrono::high_resolution_clock::now() - startTime;
      mIterationElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(counter).count();
      mOptimizationRequested = false;

      // First robot publish anchor
      if (getID() == 0) publishAnchor();

      // Publish status
      publishStatus();

      // Publish trajectory
      // publishTrajectory();

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

  if (mPublishPublicPosesRequested) {
    publishPublicPoses(false);
    if (mParams.acceleration) publishPublicPoses(true);
    mPublishPublicPosesRequested = false;
  }

  if (mPublishWeightsRequested) {
    publishMeasurementWeights();
    mPublishWeightsRequested = false;
  }

}

void PGOAgentROS::reset() {
  PGOAgent::reset();
  mInitStepsDone = 0;
  mTeamIterRequired.assign(mParams.numRobots, 0);
  mTeamIterReceived.assign(mParams.numRobots, 0);
  mTotalBytesReceived = 0;
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

  pose_graph_tools::PoseGraph pose_graph = query.response.pose_graph;
  if (pose_graph.edges.empty()) {
    ROS_WARN("Received empty pose graph.");
    return false;
  }

  // Process edges
  vector<RelativeSEMeasurement> odometry;
  vector<RelativeSEMeasurement> privateLoopClosures;
  vector<RelativeSEMeasurement> sharedLoopClosures;
  for (const auto &edge : pose_graph.edges) {
    RelativeSEMeasurement m = RelativeMeasurementFromMsg(edge);
    if (m.r1 != getID() && m.r2 != getID()) {
      ROS_ERROR("Robot %u received irrelevant measurement! ", getID());
    }
    if (m.r1 == m.r2) {
      if (m.p1 + 1 == m.p2) {
        m.isKnownInlier = true;
        odometry.push_back(m);
      } else {
        m.isKnownInlier = false;
        privateLoopClosures.push_back(m);
      }
    } else {
      m.isKnownInlier = false;
      sharedLoopClosures.push_back(m);
    }
  }

  // Process nodes
  Matrix TInit;
  if (!pose_graph.nodes.empty()) {
    // Only look at nodes that belong to this robot
    vector<pose_graph_tools::PoseGraphNode> nodes_filtered;
    for (const auto &node : pose_graph.nodes) {
      if ((unsigned) node.robot_id == getID()) nodes_filtered.push_back(node);
    }
    // If pose graph contains initial guess for the poses, we will use them
    // Otherwise, TInit can be left as empty and DPGO will perform automatic initialization
    size_t num_poses = nodes_filtered.size();
    TInit = Matrix::Zero(dimension(), (dimension() + 1) * num_poses);
    for (const auto &node : nodes_filtered) {
      assert((unsigned) node.robot_id == getID());
      size_t index = node.key;
      assert(index >= 0 && index < num_poses);
      Matrix R = RotationFromPoseMsg(node.pose);
      Matrix t = TranslationFromPoseMsg(node.pose);
      TInit.block(0, index * (d + 1), d, d) = R;
      TInit.block(0, index * (d + 1) + d, d, 1) = t;
    }
    ROS_WARN("Using provided initial trajectory with %zu poses", num_poses);
  }

  setPoseGraph(odometry, privateLoopClosures, sharedLoopClosures, TInit);
  ROS_INFO("Robot %u receives updated pose graph. "
           "Number of odometry edges = %zu, "
           "number of private loop closures = %zu, "
           "number of shared loop closures = %zu. ",
           getID(), odometry.size(), privateLoopClosures.size(), sharedLoopClosures.size());

  return true;
}

void PGOAgentROS::publishAnchor() {
  Matrix T0;
  getSharedPose(0, T0);

  PublicPoses msg;
  msg.robot_id = getID();
  msg.cluster_id = getCluster();
  msg.instance_number = instance_number();
  msg.iteration_number = iteration_number();
  msg.is_auxiliary = false;
  msg.pose_ids.push_back(0);
  msg.poses.push_back(MatrixToMsg(T0));

  mAnchorPublisher.publish(msg);
}

void PGOAgentROS::publishUpdateCommand() {
  Command msg;

  std::vector<unsigned> neighbors = getNeighbors();

  if (neighbors.empty()) {
    ROS_WARN("Robot %u does not have any neighbor!", getID());
    msg.executing_robot = getID();
  } else {
    // Uniform sampling of neighbors
    std::vector<double> neighborWeights(neighbors.size());
    for (size_t j = 0; j < neighbors.size(); ++j) {
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
  mCommandPublisher.publish(msg);
}

void PGOAgentROS::publishTerminateCommand() {
  Command msg;
  msg.command = Command::TERMINATE;
  mCommandPublisher.publish(msg);
  ROS_INFO("Published TERMINATE command.");
}

void PGOAgentROS::publishRequestPoseGraphCommand() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should send request pose graph command! ");
  }
  Command msg;
  msg.command = Command::REQUESTPOSEGRAPH;
  mCommandPublisher.publish(msg);
  ROS_INFO("Published REQUESTPOSEGRAPH command.");
}

void PGOAgentROS::publishInitializeCommand() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should send INITIALIZE command!");
  }
  Command msg;
  msg.command = Command::INITIALIZE;
  mCommandPublisher.publish(msg);
  mInitStepsDone++;
  ROS_INFO("Published INITIALIZE command.");
}

void PGOAgentROS::publishStatus() {
  Status msg = statusToMsg(getStatus());
  mStatusPublisher.publish(msg);
}

bool PGOAgentROS::publishTrajectory() {
  Matrix T;
  if (!getTrajectoryInGlobalFrame(T)) {
    return false;
  }

  // Publish as pose array
  geometry_msgs::PoseArray pose_array =
      TrajectoryToPoseArray(dimension(), num_poses(), T);
  mPoseArrayPublisher.publish(pose_array);

  // Publish as path
  nav_msgs::Path path = TrajectoryToPath(dimension(), num_poses(), T);
  mPathPublisher.publish(path);

  // Publish as optimized pose graph
  pose_graph_tools::PoseGraph pose_graph = TrajectoryToPoseGraphMsg(getID(), dimension(), num_poses(), T);
  mPoseGraphPublisher.publish(pose_graph);

  return true;
}

void PGOAgentROS::publishPublicPoses(bool aux) {
  PoseDict map;
  if (aux) {
    if (!getAuxSharedPoseDict(map)) {
      if (mParams.verbose)
        ROS_WARN("Robot %u is not initialized to publish public poses! ", getID());
      return;
    }
  } else {
    if (!getSharedPoseDict(map)) {
      if (mParams.verbose)
        ROS_WARN("Robot %u is not initialized to publish public poses! ", getID());
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
  mPublicPosesPublisher.publish(msg);
}

void PGOAgentROS::publishMeasurementWeights() {
  RelativeMeasurementWeights msg;
  for (const auto &m: sharedLoopClosures) {
    unsigned otherID = 0;
    if (m.r1 == getID()) {
      otherID = m.r2;
    } else {
      otherID = m.r1;
    }
    if (otherID > getID()) {
      msg.src_robot_ids.push_back(m.r1);
      msg.dst_robot_ids.push_back(m.r2);
      msg.src_pose_ids.push_back(m.p1);
      msg.dst_pose_ids.push_back(m.p2);
      msg.weights.push_back(m.weight);
    }
  }
  if (!msg.weights.empty()) {
    mMeasurementWeightsPublisher.publish(msg);
  }
}

bool PGOAgentROS::createLogFile(const std::string &filename) {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }
  // Instance number, global iteration number, Number of poses, total bytes
  // received, iteration time (sec), total elapsed time (sec), relative change
  file << "instance, iteration, num_poses, total_bytes_received, "
          "iteration_time_sec, total_time_sec, relative_change \n";
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

  // Compute total elapsed time since beginning of optimization
  auto counter = std::chrono::high_resolution_clock::now() - mGlobalStartTime;
  double globalElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(counter).count();

  // Instance number, global iteration number, Number of poses, total bytes
  // received, iteration time (sec), total elapsed time (sec), relative change
  file << instance_number() << ",";
  file << iteration_number() << ",";
  file << num_poses() << ",";
  file << mTotalBytesReceived << ",";
  file << mIterationElapsedMs / 1e3 << ",";
  file << globalElapsedMs / 1e3 << ",";
  file << mStatus.relativeChange << "\n";
  file.close();
  return true;
}

void PGOAgentROS::anchorCallback(const PublicPosesConstPtr &msg) {
  if (msg->robot_id != 0 || msg->pose_ids[0] != 0) {
    ROS_ERROR("Received wrong pose as anchor!");
    ros::shutdown();
  }
  setGlobalAnchor(MatrixFromMsg(msg->poses[0]));
}

void PGOAgentROS::statusCallback(const StatusConstPtr &msg) {
  // Check that robots are in agreement of current iteration number
  if (msg->instance_number != instance_number()) {
    ROS_ERROR("Local instance number at robot %u does not match with neighbor %u!"
              "Local instance number = %u, received instance number = %u.",
              getID(), msg->robot_id, instance_number(), msg->instance_number);
  }
  if (msg->iteration_number != iteration_number()) {
    ROS_ERROR("Local iteration number at robot %u does not match with neighbor %u!"
              "Local iteration number = %u, received instance number = %u.",
              getID(), msg->robot_id, iteration_number(), msg->iteration_number);
  }

  mTeamStatus[msg->robot_id] = statusFromMsg(*msg);
}

void PGOAgentROS::commandCallback(const CommandConstPtr &msg) {
  switch (msg->command) {
    case Command::REQUESTPOSEGRAPH: {
      ROS_INFO("Robot %u requesting pose graph for round %u", getID(), instance_number());
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
      ROS_INFO("Robot %u received TERMINATE command. ", getID());
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
      mGlobalStartTime = std::chrono::high_resolution_clock::now();
      publishPublicPoses(false);
      // publishTrajectory();
      publishStatus();
      if (getID() == 0) {
        ros::Duration(0.1).sleep();
        if (mInitStepsDone > 20) {
          ROS_WARN("Exceeded maximum number of initialization steps. ");
          publishTerminateCommand();
          return;
        }
        for (auto status : mTeamStatus) {
          if (status.state == PGOAgentState::WAIT_FOR_DATA) {
            ROS_WARN("Robot %u has not received data. Send INITIALIZE command again.", status.agentID);
            publishTerminateCommand();
            return;
          }
          if (status.state != PGOAgentState::INITIALIZED) {
            ROS_WARN("Robot %u has not initialized in global frame. Send INITIALIZE command again.", status.agentID);
            publishInitializeCommand();
            return;
          }
        }
        Command cmdMsg;
        cmdMsg.command = Command::UPDATE;
        cmdMsg.executing_robot = 0;
        cmdMsg.executing_iteration = iteration_number() + 1;
        mCommandPublisher.publish(cmdMsg);
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
  mTotalBytesReceived += computePublicPosesMsgSize(*msg);
}

void PGOAgentROS::measurementWeightsCallback(const RelativeMeasurementWeightsConstPtr &msg) {
  if (getState() != PGOAgentState::INITIALIZED) return;

  for (size_t k = 0; k < msg->weights.size(); ++k) {
    unsigned robotSrc = msg->src_robot_ids[k];
    unsigned robotDst = msg->dst_robot_ids[k];
    unsigned poseSrc = msg->src_pose_ids[k];
    unsigned poseDst = msg->dst_pose_ids[k];
    double w = msg->weights[k];

    unsigned otherID;
    if (robotSrc == getID()) {
      otherID = robotDst;
    } else if (robotDst == getID()) {
      otherID = robotSrc;
    } else {
      continue;
    }
    if (otherID < getID()) {
      try {
        auto &mMeasurement = findSharedLoopClosure(robotSrc, poseSrc, robotDst, poseDst);
        mMeasurement.weight = w;
//        if (mParams.verbose) {
//          ROS_INFO("Robot %u receives updated weights: (%u, %u) -> (%u, %u), new weight = %f",
//                   getID(), robotSrc, poseSrc, robotDst, poseDst, w);
//        }
      }
      catch (const std::runtime_error &e) {
        ROS_ERROR("Cannot find specified shared loop closure (%u, %u) -> (%u, %u)",
                  robotSrc, poseSrc, robotDst, poseDst);
      }

    }
  }
}

bool PGOAgentROS::queryLiftingMatrixCallback(
    QueryLiftingMatrixRequest &request, QueryLiftingMatrixResponse &response) {
  if (getID() != 0) {
    ROS_ERROR("Agent %u should not receive request for lifting matrix!", getID());
    return false;
  }
  if (request.robot_id != 0) {
    ROS_ERROR("Requested robot ID is not zero! ");
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
