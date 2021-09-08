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
#include <visualization_msgs/Marker.h>
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
  for (size_t robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    std::string topic_prefix = "/kimera" + std::to_string(robot_id) + "/dpgo_ros_node/";
    mLiftingMatrixSubscriber.push_back(
        nh.subscribe(topic_prefix + "lifting_matrix", 1000, &PGOAgentROS::liftingMatrixCallback, this));
    mStatusSubscriber.push_back(
        nh.subscribe(topic_prefix + "status", 1000, &PGOAgentROS::statusCallback, this));
    mCommandSubscriber.push_back(
        nh.subscribe(topic_prefix + "command", 1000, &PGOAgentROS::commandCallback, this));
    mAnchorSubscriber.push_back(
        nh.subscribe(topic_prefix + "anchor", 1000, &PGOAgentROS::anchorCallback, this));
    mPublicPosesSubscriber.push_back(
        nh.subscribe(topic_prefix + "public_poses", 1000, &PGOAgentROS::publicPosesCallback, this));
    mMeasurementWeightsSubscriber.push_back(
        nh.subscribe(topic_prefix + "measurement_weights", 1000, &PGOAgentROS::measurementWeightsCallback, this));
  }

  // ROS publisher
  mLiftingMatrixPublisher = nh.advertise<MatrixMsg>("lifting_matrix", 1000);
  mAnchorPublisher = nh.advertise<PublicPoses>("anchor", 1000);
  mStatusPublisher = nh.advertise<Status>("status", 1000);
  mCommandPublisher = nh.advertise<Command>("command", 1000);
  mMeasurementWeightsPublisher = nh.advertise<RelativeMeasurementWeights>("measurement_weights", 1000);
  mPublicPosesPublisher = nh.advertise<PublicPoses>("public_poses", 1000);
  mPoseArrayPublisher = nh.advertise<geometry_msgs::PoseArray>("trajectory", 5);
  mPathPublisher = nh.advertise<nav_msgs::Path>("path", 5);
  mPoseGraphPublisher = nh.advertise<pose_graph_tools::PoseGraph>("optimized_pose_graph", 5);
  mLoopClosurePublisher = nh.advertise<visualization_msgs::Marker>("loop_closures", 5);

  // ROS timer
  timer = nh.createTimer(ros::Duration(1), &PGOAgentROS::timerCallback, this);

  // First robot publishes lifting matrix
  if (getID() == 0) {
    for (size_t iter_ = 0; iter_ < 50; ++iter_) {
      publishLiftingMatrix();
      ros::Duration(0.1).sleep();
    }
    publishRequestPoseGraphCommand();
  }
}

void PGOAgentROS::runOnce() {
  if (mOptimizationRequested) {
    // Terminate if this agent does not have pose graph
    if (mState == PGOAgentState::WAIT_FOR_DATA) {
      publishTerminateCommand();
    }

    // Check if this agent has received latest public poses from its neighbors
    bool ready = true;
    for (unsigned neighbor : getNeighbors()) {
      int requiredIter = (int) mTeamIterRequired[neighbor];
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

      // Publish loop closures
      publishLoopClosures();

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

  if (mID == 0 && mState == PGOAgentState::INITIALIZED) {
    // Terminate if quiet for long time (possible message drop)
    auto counter = std::chrono::high_resolution_clock::now() - mLastCommandTime;
    double elapsedSecond = std::chrono::duration_cast<std::chrono::milliseconds>(counter).count() / 1e3;
    if (elapsedSecond > 120) {
      ROS_WARN("Last command is 120 sec ago. Send Terminate command.");
      publishTerminateCommand();
    }
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
    ROS_WARN("Using provided initial trajectory with %zu poses.", num_poses);
  }

  setPoseGraph(odometry, privateLoopClosures, sharedLoopClosures, TInit);
  ROS_INFO("Robot %u receives updated pose graph. "
           "Number of odometry edges = %zu, "
           "number of private loop closures = %zu, "
           "number of shared loop closures = %zu. ",
           getID(), odometry.size(), privateLoopClosures.size(), sharedLoopClosures.size());

  return true;
}

void PGOAgentROS::publishLiftingMatrix() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should publish lifting matrix!");
  }
  Matrix YLift;
  if (!getLiftingMatrix(YLift)) {
    ROS_ERROR("Queried lifting matrix is not available! ");
  }
  MatrixMsg msg = MatrixToMsg(YLift);
  mLiftingMatrixPublisher.publish(msg);
}

void PGOAgentROS::publishAnchor() {
  Matrix T0;
  getSharedPose(0, T0);

  PublicPoses msg;
  msg.robot_id = getID();
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
  if (mParams.verbose)
    ROS_INFO("Robot %u informs %u to update at iteration %u.", getID(), msg.executing_robot, msg.executing_iteration);
}

void PGOAgentROS::publishTerminateCommand() {
  Command msg;
  msg.command = Command::TERMINATE;
  mCommandPublisher.publish(msg);
  ROS_INFO("Robot %u published TERMINATE command.", getID());
}

void PGOAgentROS::publishRequestPoseGraphCommand() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should send request pose graph command! ");
  }
  Command msg;
  msg.command = Command::REQUESTPOSEGRAPH;
  mCommandPublisher.publish(msg);
  ROS_INFO("Robot %u published REQUESTPOSEGRAPH command.", getID());
}

void PGOAgentROS::publishInitializeCommand() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should send INITIALIZE command!");
  }
  Command msg;
  msg.command = Command::INITIALIZE;
  mCommandPublisher.publish(msg);
  mInitStepsDone++;
  ROS_INFO("Robot %u published INITIALIZE command.", getID());
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
    if (!getAuxSharedPoseDict(map)) return;
  } else {
    if (!getSharedPoseDict(map)) return;
  }

  PublicPoses msg;
  msg.robot_id = getID();
  msg.instance_number = instance_number();
  msg.iteration_number = iteration_number();
  msg.is_auxiliary = aux;

  for (const auto &sharedPose : map) {
    const PoseID nID = sharedPose.first;
    const auto &matrix = sharedPose.second;
    assert(std::get<0>(nID) == getID());
    msg.pose_ids.push_back(std::get<1>(nID));
    msg.poses.push_back(MatrixToMsg(matrix));
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

void PGOAgentROS::publishLoopClosures() {
  if (mState != PGOAgentState::INITIALIZED) return;
  visualization_msgs::Marker line_list;
  line_list.id = (int) getID();
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.3;
  line_list.header.frame_id = "/world";
  line_list.color.b = 1.0;
  line_list.color.a = 1.0;
  line_list.pose.orientation.x = 0.0;
  line_list.pose.orientation.y = 0.0;
  line_list.pose.orientation.z = 0.0;
  line_list.pose.orientation.w = 1.0;
  line_list.action = visualization_msgs::Marker::ADD;
  for (const auto &measurement : privateLoopClosures) {
    Matrix T1, T2, t1, t2;
    bool b1, b2;
    geometry_msgs::Point p1, p2;
    b1 = getPoseInGlobalFrame(measurement.p1, T1);
    b2 = getPoseInGlobalFrame(measurement.p2, T2);
    if (b1 && b2) {
      t1 = T1.block(0, d, d, 1);
      t2 = T2.block(0, d, d, 1);
      p1.x = t1(0);
      p1.y = t1(1);
      p1.z = t1(2);
      p2.x = t2(0);
      p2.y = t2(1);
      p2.z = t2(2);
      line_list.points.push_back(p1);
      line_list.points.push_back(p2);
    }
  }
  for (const auto &measurement : sharedLoopClosures) {
    if (measurement.r1 == getID() && measurement.r2 < getID()) continue;
    if (measurement.r2 == getID() && measurement.r1 < getID()) continue;
    Matrix mT, nT;
    Matrix mt, nt;
    bool mb, nb;
    if (measurement.r1 == getID()) {
      mb = getPoseInGlobalFrame(measurement.p1, mT);
      nb = getNeighborPoseInGlobalFrame(measurement.r2, measurement.p2, nT);
    } else {
      mb = getPoseInGlobalFrame(measurement.p2, mT);
      nb = getNeighborPoseInGlobalFrame(measurement.r1, measurement.p1, nT);
    }
    if (mb && nb) {
      mt = mT.block(0, d, d, 1);
      nt = nT.block(0, d, d, 1);
      geometry_msgs::Point mp, np;
      mp.x = mt(0);
      mp.y = mt(1);
      mp.z = mt(2);
      np.x = nt(0);
      np.y = nt(1);
      np.z = nt(2);
      line_list.points.push_back(mp);
      line_list.points.push_back(np);
    } else {
      // ROS_WARN("Robot %u cannot publish loop closure: (%zu,%zu) -> (%zu,%zu)",
      //         getID(), measurement.r1, measurement.p1, measurement.r2, measurement.p2);
    }
  }
  if (!line_list.points.empty()) mLoopClosurePublisher.publish(line_list);
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
  auto globalElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(counter).count();

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

void PGOAgentROS::liftingMatrixCallback(const MatrixMsgConstPtr &msg) {
  // if (mParams.verbose) {
  //   ROS_INFO("Robot %u receives lifting matrix.", getID());
  // }
  setLiftingMatrix(MatrixFromMsg(*msg));
}

void PGOAgentROS::anchorCallback(const PublicPosesConstPtr &msg) {
  if (msg->robot_id != 0 || msg->pose_ids[0] != 0) {
    ROS_ERROR("Received wrong pose as anchor!");
    ros::shutdown();
  }
  setGlobalAnchor(MatrixFromMsg(msg->poses[0]));
}

void PGOAgentROS::statusCallback(const StatusConstPtr &msg) {
  setNeighborStatus(statusFromMsg(*msg));
}

void PGOAgentROS::commandCallback(const CommandConstPtr &msg) {
  mLastCommandTime = std::chrono::high_resolution_clock::now();

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
        if (mState == PGOAgentState::INITIALIZED) publishAnchor();
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
      if (getID() == 0)
        publishLiftingMatrix();
      // publishTrajectory();
      publishStatus();
      if (getID() == 0) {
        ros::Duration(0.1).sleep();
        if (mInitStepsDone > 1000) {
          ROS_WARN("Exceeded maximum number of initialization steps. Send TERMINATE command.");
          publishTerminateCommand();
          return;
        }
        for (auto status : mTeamStatus) {
          if (status.state == PGOAgentState::WAIT_FOR_DATA) {
            ROS_WARN("Robot %u has not received data.", status.agentID);
            publishInitializeCommand();
            return;
          }
          if (status.state != PGOAgentState::INITIALIZED) {
            ROS_WARN("Robot %u has not initialized in global frame.", status.agentID);
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
        ROS_WARN("Update iteration does not match local iteration. (received: %u, local: %u)",
                 msg->executing_iteration,
                 iteration_number() + 1);
      }

      if (msg->executing_robot == getID()) {
        mOptimizationRequested = true;
        if (mParams.verbose) ROS_INFO("Robot %u receives command to update.", getID());
      } else {
        // Agents that are not selected for optimization can iterate immediately
        iterate(false);
        publishStatus();
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

  // Generate a random permutation of indices
  PoseDict poseDict;
  for (size_t index = 0; index < msg->pose_ids.size(); ++index) {
    const PoseID nID = std::make_pair(msg->robot_id, msg->pose_ids.at(index));
    const auto matrix = MatrixFromMsg(msg->poses.at(index));
    poseDict.emplace(nID, matrix);
  }
  if (!msg->is_auxiliary) {
    updateNeighborPoses(msg->robot_id, poseDict);
  } else {
    updateAuxNeighborPoses(msg->robot_id, poseDict);
  }

  // Update local bookkeeping
  mTeamIterReceived[msg->robot_id] = msg->iteration_number;
  mTotalBytesReceived += computePublicPosesMsgSize(*msg);
}

void PGOAgentROS::measurementWeightsCallback(const RelativeMeasurementWeightsConstPtr &msg) {
  if (mState != PGOAgentState::INITIALIZED) return;

  for (size_t k = 0; k < msg->weights.size(); ++k) {
    const unsigned robotSrc = msg->src_robot_ids[k];
    const unsigned robotDst = msg->dst_robot_ids[k];
    const unsigned poseSrc = msg->src_pose_ids[k];
    const unsigned poseDst = msg->dst_pose_ids[k];
    const PoseID srcID = std::make_pair(robotSrc, poseSrc);
    const PoseID dstID = std::make_pair(robotDst, poseDst);
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
        auto &mMeasurement = findSharedLoopClosure(srcID, dstID);
        mMeasurement.weight = w;
      }
      catch (const std::runtime_error &e) {
        ROS_ERROR("Cannot find specified shared loop closure (%u, %u) -> (%u, %u)",
                  robotSrc, poseSrc, robotDst, poseDst);
      }

    }
  }
}

void PGOAgentROS::timerCallback(const ros::TimerEvent &event) {
  publishStatus();
  publishLoopClosures();
  if (mState == PGOAgentState::INITIALIZED) {
    publishPublicPoses(false);
  }
}

}  // namespace dpgo_ros
