/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <dpgo_ros/PGOAgentROS.h>
#include <dpgo_ros/utils.h>
#include <DPGO/DPGO_solver.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <pose_graph_tools/PoseGraphQuery.h>
#include <pose_graph_tools/utils.h>
#include <glog/logging.h>
#include <map>
#include <random>

using namespace DPGO;

namespace dpgo_ros {

PGOAgentROS::PGOAgentROS(const ros::NodeHandle &nh_, unsigned ID,
                         const PGOAgentROSParameters &params)
    : PGOAgent(ID, params),
      nh(nh_),
      mParamsROS(params),
      mClusterID(ID),
      mInitStepsDone(0),
      mTotalBytesReceived(0),
      mIterationElapsedMs(0) {
  mTeamIterRequired.assign(mParams.numRobots, 0);
  mTeamIterReceived.assign(mParams.numRobots, 0);
  mTeamReceivedSharedLoopClosures.assign(mParams.numRobots, false);
  mTeamConnected.assign(mParams.numRobots, true);

  // Load robot names
  for (size_t id = 0; id < mParams.numRobots; id++) {
    std::string robot_name = "kimera" + std::to_string(id);
    ros::param::get("~robot" + std::to_string(id) + "_name", robot_name);
    mRobotNames[id] = robot_name;
  }

  // ROS subscriber
  for (size_t robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    std::string topic_prefix = "/" + mRobotNames.at(robot_id) + "/dpgo_ros_node/";
    mLiftingMatrixSubscriber.push_back(
        nh.subscribe(topic_prefix + "lifting_matrix", 100, &PGOAgentROS::liftingMatrixCallback, this));
    mStatusSubscriber.push_back(
        nh.subscribe(topic_prefix + "status", 100, &PGOAgentROS::statusCallback, this));
    mCommandSubscriber.push_back(
        nh.subscribe(topic_prefix + "command", 100, &PGOAgentROS::commandCallback, this));
    mAnchorSubscriber.push_back(
        nh.subscribe(topic_prefix + "anchor", 100, &PGOAgentROS::anchorCallback, this));
    mPublicPosesSubscriber.push_back(
        nh.subscribe(topic_prefix + "public_poses", 100, &PGOAgentROS::publicPosesCallback, this));
    mSharedLoopClosureSubscriber.push_back(
        nh.subscribe(topic_prefix + "public_measurements", 100, &PGOAgentROS::publicMeasurementsCallback, this));
  }
  mConnectivitySubscriber =
      nh.subscribe("/" + mRobotNames.at(mID) + "/connected_peer_ids", 5,
                   &PGOAgentROS::connectivityCallback, this);

  for (size_t robot_id = 0; robot_id < getID(); ++robot_id) {
    std::string topic_prefix = "/" + mRobotNames.at(robot_id) + "/dpgo_ros_node/";
    mMeasurementWeightsSubscriber.push_back(
        nh.subscribe(topic_prefix + "measurement_weights", 100, &PGOAgentROS::measurementWeightsCallback, this));
  }

  // ROS publisher
  mLiftingMatrixPublisher = nh.advertise<MatrixMsg>("lifting_matrix", 1);
  mAnchorPublisher = nh.advertise<PublicPoses>("anchor", 1);
  mStatusPublisher = nh.advertise<Status>("status", 1);
  mCommandPublisher = nh.advertise<Command>("command", 20);
  mPublicPosesPublisher = nh.advertise<PublicPoses>("public_poses", 20);
  mPublicMeasurementsPublisher = nh.advertise<RelativeMeasurementList>("public_measurements", 20);
  mMeasurementWeightsPublisher = nh.advertise<RelativeMeasurementWeights>("measurement_weights", 20);
  mPoseArrayPublisher = nh.advertise<geometry_msgs::PoseArray>("trajectory", 1);
  mPathPublisher = nh.advertise<nav_msgs::Path>("path", 1);
  mPoseGraphPublisher = nh.advertise<pose_graph_tools::PoseGraph>("optimized_pose_graph", 1);
  mLoopClosureMarkerPublisher = nh.advertise<visualization_msgs::Marker>("loop_closures", 1);

  // ROS timer
  timer = nh.createTimer(ros::Duration(3.0), &PGOAgentROS::timerCallback, this);
  mVisualizationTimer = nh.createTimer(ros::Duration(30.0), &PGOAgentROS::visualizationTimerCallback, this);

  // Initially, assume each robot is in a separate cluster
  resetRobotClusterIDs();

  // Publish lifting matrix
  for (size_t iter_ = 0; iter_ < 10; ++iter_) {
    publishNoopCommand();
    ros::Duration(0.5).sleep();
  }
  mLastResetTime = ros::Time::now();
  mLaunchTime = ros::Time::now();
  mLastCommandTime = ros::Time::now();
}

void PGOAgentROS::runOnce() {
  if (mParams.asynchronous) {
    runOnceAsynchronous();
  } else {
    runOnceSynchronous();
  }

  if (mPublishPublicPosesRequested) {
    publishPublicPoses(false);
    if (mParams.acceleration) publishPublicPoses(true);
    mPublishPublicPosesRequested = false;
  }

  checkCommandTimeout();
  if (isLeader() && mState == PGOAgentState::INITIALIZED) {
    checkDisconnectedRobot();
  }
}

void PGOAgentROS::runOnceAsynchronous() {
  if (mPublishAsynchronousRequested) {
    if (isLeader()) publishAnchor();
    publishStatus();
    if (mParamsROS.publishIterate) {
      storeOptimizedTrajectory();
      storeLoopClosureMarkers();
      publishOptimizedTrajectory();
      publishLoopClosureMarkers();
    }
    logIteration();
    mPublishAsynchronousRequested = false;
  }

  // Check for termination condition
  if (isLeader() && shouldTerminate()) {
    publishTerminateCommand();
  }
}

void PGOAgentROS::runOnceSynchronous() {
  CHECK(!mParams.asynchronous);

  // Perform an optimization step
  if (mSynchronousOptimizationRequested) {

    // Check if ready to perform iterate
    bool ready = true;
    for (unsigned neighbor : mPoseGraph->activeNeighborIDs()) {
      int requiredIter = (int) mTeamIterRequired[neighbor];
      if (mParams.acceleration) requiredIter = (int) iteration_number() + 1;
      requiredIter = requiredIter - mParamsROS.maxDelayedIterations;
      if ((int) mTeamIterReceived[neighbor] < requiredIter) {
        if (mParams.verbose) {
          ROS_WARN("Robot %u iteration %u waits for neighbor %u iteration %u (last received iteration %u).",
                   getID(), iteration_number() + 1, neighbor, requiredIter, mTeamIterReceived[neighbor]);
        }
        ready = false;
      }
    }

    // Perform iterate with optimization if ready
    if (ready) {
      // Beta feature: Apply stored neighbor poses and edge weights for inactive robots
      // setInactiveNeighborPoses();
      // setInactiveEdgeWeights();
      // mPoseGraph->useInactiveNeighbors(true);
      
      // Iterate
      auto startTime = std::chrono::high_resolution_clock::now();
      iterate(true);
      auto counter = std::chrono::high_resolution_clock::now() - startTime;
      mIterationElapsedMs = (double) std::chrono::duration_cast<std::chrono::milliseconds>(counter).count();
      mSynchronousOptimizationRequested = false;
      ROS_INFO("Robot %u iteration %u: func_decr=%.1e, grad_init=%.1e, grad_opt=%.1e.", 
               getID(), 
               iteration_number(),
               mLocalOptResult.fInit - mLocalOptResult.fOpt,
               mLocalOptResult.gradNormInit, 
               mLocalOptResult.gradNormOpt);

      // First robot publish anchor
      if (isLeader()) {
        publishAnchor();
      }

      // Publish status
      publishStatus();

      // Publish trajectory
      if (mParamsROS.publishIterate) {
        storeOptimizedTrajectory();
        storeLoopClosureMarkers();
        publishOptimizedTrajectory();
        publishLoopClosureMarkers();
      }

      // Log local iteration
      logIteration();

      // Print information
      if (isLeader() && mParams.verbose) {
        ROS_INFO("Num weight updates done: %i, num inner iters: %i.", mWeightUpdateCount, mRobustOptInnerIter);
        for (size_t robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
          if (!isRobotActive(robot_id)) continue;
          const auto &it = mTeamStatus.find(robot_id);
          if (it != mTeamStatus.end()) {
            const auto &robot_status = it->second;
            ROS_INFO("Robot %zu relative change %f.", robot_id, robot_status.relativeChange);
          } else {
            ROS_INFO("Robot %zu status unavailable.", robot_id);
          }
        }
      }

      // Check termination condition OR notify next robot to update
      if (isLeader()) {
        if (shouldTerminate()) {
          publishTerminateCommand();
        } else if (shouldUpdateMeasurementWeights()) {
          publishUpdateWeightCommand();
        } else {
          publishUpdateCommand();
        }
      } else {
        publishUpdateCommand();
      }
    }
  }
}

void PGOAgentROS::reset() {
  PGOAgent::reset();
  mSynchronousOptimizationRequested = false;
  mTryInitializeRequested = false;
  mInitStepsDone = 0;
  mTeamIterRequired.assign(mParams.numRobots, 0);
  mTeamIterReceived.assign(mParams.numRobots, 0);
  mTeamReceivedSharedLoopClosures.assign(mParams.numRobots, false);
  mTotalBytesReceived = 0;
  mTeamStatusMsg.clear();
  if (mIterationLog.is_open()) {
    mIterationLog.close();
  }
  if (mParamsROS.completeReset) {
    ROS_WARN("Reset DPGO completely.");
    mPoseGraph = std::make_shared<PoseGraph>(mID, r, d);  // Reset pose graph
    mCachedPoses.reset();  // Reset stored trajectory estimate
    mCachedLoopClosureMarkers.reset();
  }
  resetRobotClusterIDs();
  mLastResetTime = ros::Time::now();
}

bool PGOAgentROS::requestPoseGraph() {
  // Query local pose graph
  pose_graph_tools::PoseGraphQuery query;
  query.request.robot_id = getID();
  std::string service_name = "/" + mRobotNames.at(getID()) +
      "/distributed_loop_closure/request_pose_graph";
  if (!ros::service::waitForService(service_name, ros::Duration(5.0))) {
    ROS_ERROR_STREAM("ROS service " << service_name << " does not exist!");
    return false;
  }
  if (!ros::service::call(service_name, query)) {
    ROS_ERROR_STREAM("Failed to call ROS service " << service_name);
    return false;
  }

  pose_graph_tools::PoseGraph pose_graph = query.response.pose_graph;
  if (pose_graph.edges.size() <= 1) {
    ROS_WARN("Received empty pose graph.");
    return false;
  }

  // Process edges
  unsigned int num_measurements_before = mPoseGraph->numMeasurements();
  for (const auto &edge : pose_graph.edges) {
    RelativeSEMeasurement m = RelativeMeasurementFromMsg(edge);
    if (m.r1 != getID() && m.r2 != getID()) {
      ROS_ERROR("Robot %u received irrelevant measurement! ", getID());
    }
    addMeasurement(m);
  }
  unsigned int num_measurements_after = mPoseGraph->numMeasurements();
  ROS_INFO("Received pose graph from ROS service (%u new measurements).",
           num_measurements_after - num_measurements_before);

  // Process nodes
  PoseArray initial_poses(dimension(), num_poses());
  if (!pose_graph.nodes.empty()) {
    // Filter nodes that do not belong to this robot
    vector<pose_graph_tools::PoseGraphNode> nodes_filtered;
    for (const auto &node : pose_graph.nodes) {
      if ((unsigned) node.robot_id == getID()) nodes_filtered.push_back(node);
    }
    // If pose graph contains initial guess for the poses, we will use them
    size_t num_nodes = nodes_filtered.size();
    if (num_nodes == num_poses()) {
      for (const auto &node : nodes_filtered) {
        assert((unsigned) node.robot_id == getID());
        size_t index = node.key;
        assert(index >= 0 && index < num_poses());
        initial_poses.rotation(index) = RotationFromPoseMsg(node.pose);
        initial_poses.translation(index) = TranslationFromPoseMsg(node.pose);
      }
    }
  }

  mTeamReceivedSharedLoopClosures.assign(mParams.numRobots, false);
  for (size_t robot_id = getID(); robot_id < mParams.numRobots; ++robot_id)
    mTeamReceivedSharedLoopClosures[robot_id] = true;
  mTryInitializeRequested = true;
  return true;
}

bool PGOAgentROS::tryInitialize() {
  // Before initialization, we need to received inter-robot loop closures from
  // all preceeding robots.
  bool ready = true;
  for (unsigned robot_id = 0; robot_id < getID(); ++robot_id) {
    // Skip if this preceeding robot is excluded from optimization
    if (!isRobotActive(robot_id)) {
      continue;
    }
    if (!mTeamReceivedSharedLoopClosures[robot_id]) {
      ROS_INFO("Robot %u waiting for shared loop closures from robot %u.",
               getID(), robot_id);
      ready = false;
      break;
    }
  }
  if (ready) {
    ROS_INFO("Robot %u sets pose graph. "
             "num_poses:%u, odom:%u, local_lc:%u, shared_lc:%u.",
             getID(),
             num_poses(),
             mPoseGraph->numOdometry(),
             mPoseGraph->numPrivateLoopClosures(),
             mPoseGraph->numSharedLoopClosures());
    if (mCachedPoses.has_value()) {
      // Result from previous round is available, and we use it to directly
      // initialize in global frame
      ROS_INFO("Initialize in global frame using result from previous round.");
      const auto TPrev = mCachedPoses.value();
      const auto TInit = odometryInitialization(mPoseGraph->odometry(), &TPrev);
      initialize(&TInit);
      initializeInGlobalFrame(Pose(d));
      initializeGlobalAnchor();
      // Leader adds a prior to prevent drifting in the global frame
      if (getClusterID() != 0 && isLeader()) {
        Matrix M;
        if (getSharedPose(0, M)) {
          LiftedPose prior(relaxation_rank(), dimension());
          prior.setData(M);
          mPoseGraph->setPrior(0, prior);
          ROS_INFO("Robot %u sets prior.", getID());
        } else {
          ROS_ERROR("Robot %u fails to set prior!", getID());
        }
      }
    } else {
      // No result is available so far, we will attempt to initialize with
      // robust initialization
      initialize();
    }
    mTryInitializeRequested = false;
  }
  return ready;
}

bool PGOAgentROS::isRobotConnected(unsigned robot_id) const {
  if (robot_id >= mParams.numRobots) {
    return false;
  }
  if (robot_id == getID()) {
    return true;
  }
  return mTeamConnected[robot_id];
}

void PGOAgentROS::setActiveRobots() {
  for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    if (isRobotConnected(robot_id) && getRobotClusterID(robot_id) == getID()) {
      ROS_INFO("Set robot %u to active.", robot_id);
      setRobotActive(robot_id, true);
    } else {
      ROS_WARN("Set robot %u to inactive.", robot_id);
      setRobotActive(robot_id, false);
    }
  }
}

void PGOAgentROS::updateActiveRobots(const CommandConstPtr &msg) {
  std::set<unsigned> active_robots_set(msg->active_robots.begin(),
                                       msg->active_robots.end());
  for (unsigned int robot_id = 0; robot_id < mParams.numRobots; robot_id++) {
    if (active_robots_set.find(robot_id) == active_robots_set.end()) {
      setRobotActive(robot_id, false);
    } else {
      setRobotActive(robot_id, true);
    }
  }
}

void PGOAgentROS::publishLiftingMatrix() {
  Matrix YLift;
  if (!getLiftingMatrix(YLift)) {
    ROS_WARN("Lifting matrix does not exist! ");
    return;
  }
  MatrixMsg msg = MatrixToMsg(YLift);
  mLiftingMatrixPublisher.publish(msg);
}

void PGOAgentROS::publishAnchor() {
  // We assume the anchor is always the first pose of the first robot
  if (!isLeader()) {
    ROS_ERROR("Only leader robot should publish anchor!");
    return;
  }
  if (mState != PGOAgentState::INITIALIZED) {
    ROS_WARN("Cannot publish anchor: not initialized.");
    return;
  }
  Matrix T0;
  if (getID() == 0) {
    getSharedPose(0, T0);
  } else {
    if (!globalAnchor.has_value()) {
      return;
    }
    T0 = globalAnchor.value().getData();
  }
  PublicPoses msg;
  msg.robot_id = 0;
  msg.instance_number = instance_number();
  msg.iteration_number = iteration_number();
  msg.is_auxiliary = false;
  msg.pose_ids.push_back(0);
  msg.poses.push_back(MatrixToMsg(T0));

  mAnchorPublisher.publish(msg);
}

void PGOAgentROS::publishUpdateCommand() {
  unsigned selected_robot = 0;
  switch (mParamsROS.updateRule) {
    case PGOAgentROSParameters::UpdateRule::Uniform: {
      // Uniform sampling of all active robots
      std::vector<unsigned> active_robots;
      for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
        if (isRobotActive(robot_id) && isRobotInitialized(robot_id)) {
          active_robots.push_back(robot_id);
        }
      }
      size_t num_active_robots = active_robots.size();
      std::vector<double> weights(num_active_robots, 1.0);
      std::discrete_distribution<int> distribution(weights.begin(),
                                                   weights.end());
      std::random_device rd;
      std::mt19937 gen(rd());
      selected_robot = active_robots[distribution(gen)];
      break;
    }
    case PGOAgentROSParameters::UpdateRule::RoundRobin: {
      // Round robin updates
      unsigned next_robot_id = (getID() + 1) % mParams.numRobots;
      while (!isRobotActive(next_robot_id) ||
          !isRobotInitialized(next_robot_id)) {
        next_robot_id = (next_robot_id + 1) % mParams.numRobots;
      }
      selected_robot = next_robot_id;
      break;
    }
  }
  publishUpdateCommand(selected_robot);
}

void PGOAgentROS::publishUpdateCommand(unsigned robot_id) {
  if (!isRobotActive(robot_id)) {
    ROS_ERROR("Next robot to update %u is not active!", robot_id);
    return;
  }
  CHECK(!mParams.asynchronous);
  if (mParamsROS.interUpdateSleepTime > 1e-3)
    ros::Duration(mParamsROS.interUpdateSleepTime).sleep();
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.command = Command::UPDATE;
  msg.cluster_id = getClusterID();
  msg.publishing_robot = getID();
  msg.executing_robot = robot_id;
  msg.executing_iteration = iteration_number() + 1;
  ROS_INFO_STREAM("Send UPDATE to robot " << msg.executing_robot
                                          << " to perform iteration "
                                          << msg.executing_iteration << ".");
  mCommandPublisher.publish(msg);
}

void PGOAgentROS::publishTerminateCommand() {
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.cluster_id = getClusterID();
  msg.command = Command::TERMINATE;
  mCommandPublisher.publish(msg);
  ROS_INFO("Robot %u published TERMINATE command.", getID());
}

void PGOAgentROS::publishHardTerminateCommand() {
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.cluster_id = getClusterID();
  msg.command = Command::HARD_TERMINATE;
  mCommandPublisher.publish(msg);
  ROS_INFO("Robot %u published HARD TERMINATE command.", getID());
}

void PGOAgentROS::publishUpdateWeightCommand() {
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.cluster_id = getClusterID();
  msg.command = Command::UPDATE_WEIGHT;
  mCommandPublisher.publish(msg);
  ROS_INFO("Robot %u published UPDATE_WEIGHT command (num inner iters %i).", 
           getID(), mRobustOptInnerIter);
}

void PGOAgentROS::publishRequestPoseGraphCommand() {
  if (!isLeader()) {
    ROS_ERROR("Only leader should send request pose graph command! ");
    return;
  }
  setActiveRobots();
  if (numActiveRobots() == 1) {
    ROS_WARN("Not enough active robots. Do not publish request pose graph command.");
    return;
  }
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.cluster_id = getClusterID();
  msg.command = Command::REQUEST_POSE_GRAPH;
  for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    if (isRobotActive(robot_id)) {
      msg.active_robots.push_back(robot_id);
    }
  }
  mCommandPublisher.publish(msg);
  ROS_INFO("Robot %u published REQUEST_POSE_GRAPH command.", getID());
}

void PGOAgentROS::publishInitializeCommand() {
  if (!isLeader()) {
    ROS_ERROR("Only leader should send INITIALIZE command!");
  }
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.cluster_id = getClusterID();
  msg.command = Command::INITIALIZE;
  mCommandPublisher.publish(msg);
  mInitStepsDone++;
  mPublishInitializeCommandRequested = false;
  ROS_INFO("Robot %u published INITIALIZE command.", getID());
}

void PGOAgentROS::publishActiveRobotsCommand() {
  if (!isLeader()) {
    ROS_ERROR("Only leader should publish active robots!");
    return;
  }
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.cluster_id = getClusterID();
  msg.command = Command::SET_ACTIVE_ROBOTS;
  for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    if (isRobotActive(robot_id)) {
      msg.active_robots.push_back(robot_id);
    }
  }

  mCommandPublisher.publish(msg);
}

void PGOAgentROS::publishNoopCommand() {
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.cluster_id = getClusterID();
  msg.command = Command::NOOP;
  mCommandPublisher.publish(msg);
}

void PGOAgentROS::publishStatus() {
  Status msg = statusToMsg(getStatus());
  msg.cluster_id = getClusterID();
  msg.header.stamp = ros::Time::now();
  mStatusPublisher.publish(msg);
}

void PGOAgentROS::storeOptimizedTrajectory() {
  PoseArray T(dimension(), num_poses());
  if (getTrajectoryInGlobalFrame(T)) {
    mCachedPoses.emplace(T);
  }
}

void PGOAgentROS::publishTrajectory(const PoseArray &T) {
  // Publish as pose array
  geometry_msgs::PoseArray pose_array =
      TrajectoryToPoseArray(T.d(), T.n(), T.getData());
  mPoseArrayPublisher.publish(pose_array);

  // Publish as path
  nav_msgs::Path path = TrajectoryToPath(T.d(), T.n(), T.getData());
  mPathPublisher.publish(path);

  // Publish as optimized pose graph
  pose_graph_tools::PoseGraph pose_graph = TrajectoryToPoseGraphMsg(getID(), T.d(), T.n(), T.getData());
  mPoseGraphPublisher.publish(pose_graph);
}

void PGOAgentROS::publishOptimizedTrajectory() {
  if (!isRobotActive(getID()))
    return;
  if (!mCachedPoses.has_value())
    return;
  publishTrajectory(mCachedPoses.value());
}

void PGOAgentROS::publishPublicPoses(bool aux) {
  for (unsigned neighbor : getNeighbors()) {
    PoseDict map;
    if (aux) {
      if (!getAuxSharedPoseDictWithNeighbor(map, neighbor)) return;
    } else {
      if (!getSharedPoseDictWithNeighbor(map, neighbor)) return;
    }
    if (map.empty())
      continue;

    PublicPoses msg;
    msg.robot_id = getID();
    msg.cluster_id = getClusterID();
    msg.destination_robot_id = neighbor;
    msg.instance_number = instance_number();
    msg.iteration_number = iteration_number();
    msg.is_auxiliary = aux;

    for (const auto &sharedPose : map) {
      const PoseID nID = sharedPose.first;
      const auto &matrix = sharedPose.second.getData();
      CHECK_EQ(nID.robot_id, getID());
      msg.pose_ids.push_back(nID.frame_id);
      msg.poses.push_back(MatrixToMsg(matrix));
    }
    mPublicPosesPublisher.publish(msg);
  }
}

void PGOAgentROS::publishPublicMeasurements() {
  std::map<unsigned, RelativeMeasurementList> msg_map;
  for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    RelativeMeasurementList msg;
    msg.from_robot = getID();
    msg.from_cluster = getClusterID();
    msg.to_robot = robot_id;
    msg_map[robot_id] = msg;
  }
  for (const auto &m : mPoseGraph->sharedLoopClosures()) {
    unsigned otherID = 0;
    if (m.r1 == getID()) {
      otherID = m.r2;
    } else {
      otherID = m.r1;
    }
    CHECK(msg_map.find(otherID) != msg_map.end());
    const auto edge = RelativeMeasurementToMsg(m);
    msg_map[otherID].edges.push_back(edge);
  }
  for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id)
    mPublicMeasurementsPublisher.publish(msg_map[robot_id]);
}

void PGOAgentROS::publishMeasurementWeights() {
  // if (mState != PGOAgentState::INITIALIZED) return;

  std::map<unsigned, RelativeMeasurementWeights> msg_map;
  for (const auto &m : mPoseGraph->sharedLoopClosures()) {
    unsigned otherID = 0;
    if (m.r1 == getID()) {
      otherID = m.r2;
    } else {
      otherID = m.r1;
    }
    if (otherID > getID()) {
      if (msg_map.find(otherID) == msg_map.end()) {
        RelativeMeasurementWeights msg;
        msg.robot_id = getID();
        msg.cluster_id = getClusterID();
        msg.destination_robot_id = otherID;
        msg_map[otherID] = msg;
      }
      msg_map[otherID].src_robot_ids.push_back(m.r1);
      msg_map[otherID].dst_robot_ids.push_back(m.r2);
      msg_map[otherID].src_pose_ids.push_back(m.p1);
      msg_map[otherID].dst_pose_ids.push_back(m.p2);
      msg_map[otherID].weights.push_back(m.weight);
      msg_map[otherID].fixed_weights.push_back(m.fixedWeight);
    }
  }
  for (const auto &it : msg_map) {
    const auto &msg = it.second;
    if (!msg.weights.empty()) {
      mMeasurementWeightsPublisher.publish(msg);
    }
  }
}

void PGOAgentROS::storeLoopClosureMarkers() {
  if (mState != PGOAgentState::INITIALIZED) return;
  double weight_tol = mParamsROS.weightConvergenceThreshold;
  visualization_msgs::Marker line_list;
  line_list.id = (int) getID();
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.1;
  line_list.header.frame_id = "/world";
  line_list.color.a = 1.0;
  line_list.pose.orientation.x = 0.0;
  line_list.pose.orientation.y = 0.0;
  line_list.pose.orientation.z = 0.0;
  line_list.pose.orientation.w = 1.0;
  line_list.action = visualization_msgs::Marker::ADD;
  for (const auto &measurement : mPoseGraph->privateLoopClosures()) {
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
      std_msgs::ColorRGBA line_color;
      line_color.a = 1;
      if (measurement.weight > 1 - weight_tol) {
        line_color.g = 1;
      } else if (measurement.weight < weight_tol) {
        line_color.r = 1;
      } else {
        line_color.b = 1;
      }
      line_list.colors.push_back(line_color);
      line_list.colors.push_back(line_color);
    }
  }
  for (const auto &measurement : mPoseGraph->sharedLoopClosures()) {
    Matrix mT, nT;
    Matrix mt, nt;
    bool mb, nb;
    unsigned neighbor_id;
    if (measurement.r1 == getID()) {
      neighbor_id = measurement.r2;
      mb = getPoseInGlobalFrame(measurement.p1, mT);
      nb = getNeighborPoseInGlobalFrame(measurement.r2, measurement.p2, nT);
    } else {
      neighbor_id = measurement.r1;
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
      std_msgs::ColorRGBA line_color;
      line_color.a = 1;
      if (!isRobotActive(neighbor_id)) {
        // Black
      } else if (measurement.weight > 1 - weight_tol) {
        line_color.g = 1;
      } else if (measurement.weight < weight_tol) {
        line_color.r = 1;
      } else {
        line_color.b = 1;
      }
      line_list.colors.push_back(line_color);
      line_list.colors.push_back(line_color);
    } 
  }
  if (!line_list.points.empty())
    mCachedLoopClosureMarkers.emplace(line_list);
}

void PGOAgentROS::publishLoopClosureMarkers() {
  if (mCachedLoopClosureMarkers.has_value())
    mLoopClosureMarkerPublisher.publish(mCachedLoopClosureMarkers.value());
}

bool PGOAgentROS::createIterationLog(const std::string &filename) {
  if (mIterationLog.is_open())
    mIterationLog.close();
  mIterationLog.open(filename);
  if (!mIterationLog.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }
  // Robot ID, Cluster ID, global iteration number, Number of poses, total bytes
  // received, iteration time (sec), total elapsed time (sec), relative change
  mIterationLog << "robot_id, cluster_id, iteration, num_poses, total_bytes_received, "
                   "iteration_time_sec, total_time_sec, relative_change \n";
  mIterationLog.flush();
  return true;
}

bool PGOAgentROS::logIteration() {
  if (!mParams.logData) {
    return false;
  }
  if (!mIterationLog.is_open()) {
    ROS_ERROR_STREAM("No iteration log file!");
    return false;
  }

  // Compute total elapsed time since beginning of optimization
  double globalElapsedSec = (ros::Time::now() - mGlobalStartTime).toSec();

  // Robot ID, Cluster ID, global iteration number, Number of poses, total bytes
  // received, iteration time (sec), total elapsed time (sec), relative change
  mIterationLog << getID() << ",";
  mIterationLog << getClusterID() << ",";
  mIterationLog << iteration_number() << ",";
  mIterationLog << num_poses() << ",";
  mIterationLog << mTotalBytesReceived << ",";
  mIterationLog << mIterationElapsedMs / 1e3 << ",";
  mIterationLog << globalElapsedSec << ",";
  mIterationLog << mStatus.relativeChange << "\n";
  mIterationLog.flush();
  return true;
}

bool PGOAgentROS::logString(const std::string &str) {
  if (!mParams.logData) {
    return false;
  }
  if (!mIterationLog.is_open()) {
    ROS_WARN_STREAM("No iteration log file!");
    return false;
  }
  mIterationLog << str << "\n";
  mIterationLog.flush();
  return true;
}

void PGOAgentROS::connectivityCallback(
    const std_msgs::UInt16MultiArrayConstPtr &msg) {
  std::set<unsigned> connected_ids(msg->data.begin(), msg->data.end());
  for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    if (robot_id == getID()) {
      mTeamConnected[robot_id] = true;
    } else if (connected_ids.find(robot_id) != connected_ids.end()) {
      mTeamConnected[robot_id] = true;
    } else {
      // ROS_WARN("Robot %u is disconnected.", robot_id);
      mTeamConnected[robot_id] = false;
    }
  }
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
  const auto &received_msg = *msg;
  const auto &it = mTeamStatusMsg.find(msg->robot_id);
  // Ignore message with outdated timestamp
  if (it != mTeamStatusMsg.end()) {
    const auto latest_msg = it->second;
    if (latest_msg.header.stamp > received_msg.header.stamp) {
      ROS_WARN("Received outdated status from robot %u.", msg->robot_id);
      return;
    }
  }
  mTeamStatusMsg[msg->robot_id] = received_msg;
  
  setRobotClusterID(msg->robot_id, msg->cluster_id);
  if (msg->cluster_id == getClusterID()) {
    setNeighborStatus(statusFromMsg(received_msg));;
  }
}

void PGOAgentROS::commandCallback(const CommandConstPtr &msg) {
  if (msg->cluster_id != getClusterID()) {
    ROS_WARN("Ignore command from wrong cluster (%u vs. %u).", msg->cluster_id,
             getClusterID());
    return;
  }
  // Update latest command time
  // Ignore commands that are periodically published
  if (msg->command != Command::NOOP && msg->command != Command::SET_ACTIVE_ROBOTS) {
    mLastCommandTime = ros::Time::now();
  }

  switch (msg->command) {
    case Command::REQUEST_POSE_GRAPH: {
      if (msg->publishing_robot != getClusterID()) {
        ROS_WARN("Ignore REQUEST_POSE_GRAPH command from non-leader %u.",
                 msg->publishing_robot);
        return;
      }
      ROS_INFO("Robot %u received REQUEST_POSE_GRAPH command.", getID());
      if (mState != PGOAgentState::WAIT_FOR_DATA) {
        ROS_WARN_STREAM("Robot " << getID() << " status is not WAIT_FOR_DATA. Reset...");
        reset();
      }
      // Update local record of currently active robots
      updateActiveRobots(msg);
      // Request latest pose graph
      bool received_pose_graph = requestPoseGraph();
      // Create log file for new round
      if (mParams.logData && received_pose_graph) {
        auto time_since_launch = ros::Time::now() - mLaunchTime;
        int sec_since_launch = int(time_since_launch.toSec());
        std::string log_path = mParams.logDirectory + "dpgo_log_" + std::to_string(sec_since_launch) + ".csv";
        createIterationLog(log_path);
      }
      publishStatus();
      // Enter initialization round
      if (isLeader()) {
        if (!received_pose_graph) {
          publishHardTerminateCommand();
        } else {
          publishAnchor();
          publishInitializeCommand();
        }
      }
      break;
    }

    case Command::TERMINATE: {
      ROS_INFO("Robot %u received TERMINATE command. ", getID());
      if (!isRobotActive(getID())) {
        reset();
        break;
      }
      logString("TERMINATE");
      // When running distributed GNC, fix loop closures that have converged
      if (mParams.robustCostParams.costType ==
          RobustCostParameters::Type::GNC_TLS) {
        double residual = 0;
        double weight = 0;
        for (auto &m : mPoseGraph->activeLoopClosures()) {
          if (!m->fixedWeight && computeMeasurementResidual(*m, &residual)) {
            weight = mRobustCost.weight(residual);
            if (residual < mParams.robustCostParams.GNCBarc) {
              m->weight = 1;
              m->fixedWeight = true;
            } else if (weight < mParamsROS.weightConvergenceThreshold) {
              ROS_INFO("Reject measurement with residual %f and weight %f.", residual, weight);
              m->weight = 0;
              m->fixedWeight = true;
            } 
          }
        }
        const auto stat = mPoseGraph->statistics();
        ROS_INFO(
            "Robot %u loop closure statistics:\n "
            "accepted: %f\n "
            "rejected: %f\n "
            "undecided: %f\n",
            mID, 
            stat.accept_loop_closures, 
            stat.reject_loop_closures, 
            stat.undecided_loop_closures);
        publishMeasurementWeights();
      }

      // Store and publish optimized trajectory in global frame
      storeOptimizedTrajectory();
      storeLoopClosureMarkers();
      storeActiveNeighborPoses();
      storeActiveEdgeWeights();
      publishOptimizedTrajectory();
      publishLoopClosureMarkers();
      reset();
      break;
    }

    case Command::HARD_TERMINATE: {
      ROS_INFO("Robot %u received HARD TERMINATE command. ", getID());
      logString("HARD_TERMINATE");
      reset();
      break;
    }

    case Command::INITIALIZE: {
      // TODO: ignore if status == WAIT_FOR_DATA
      if (msg->publishing_robot != getClusterID()) {
        ROS_WARN("Ignore INITIALIZE command from non-leader %u.",
                 msg->publishing_robot);
        return;
      }
      mGlobalStartTime = ros::Time::now();
      publishPublicMeasurements();
      publishPublicPoses(false);
      publishStatus();
      if (isLeader()) {
        publishLiftingMatrix();
        // updateActiveRobots();
        publishActiveRobotsCommand();
        ros::Duration(0.1).sleep();

        // Check the status of all robots
        bool all_initialized = true;
        int num_initialized_robots = 0;
        for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
          if (!isRobotActive(robot_id)) {
            // Ignore inactive robots
            continue;
          }
          if (!hasNeighborStatus(robot_id)) {
            ROS_WARN("Robot %u status not available.", robot_id);
            all_initialized = false;
            continue;
          }
          const auto status = getNeighborStatus(robot_id);
          if (status.state == PGOAgentState::WAIT_FOR_DATA) {
            ROS_WARN("Robot %u has not received pose graph.", status.agentID);
            all_initialized = false;
          } else if (status.state == PGOAgentState::WAIT_FOR_INITIALIZATION) {
            ROS_WARN("Robot %u has not initialized in global frame.", status.agentID);
            all_initialized = false;
          } else if (status.state == PGOAgentState::INITIALIZED) {
            num_initialized_robots++;
          }
        }

        if (!all_initialized && mInitStepsDone <= mParamsROS.maxDistributedInitSteps) {
          // Keep waiting for more robots to initialize
          mPublishInitializeCommandRequested = true;
          return;
        } else {
          // Start distributed optimization if more than 1 robot is initialized
          if (num_initialized_robots > 1) {
            ROS_INFO("Start distributed optimization with %i/%zu active robots.",
                     num_initialized_robots, numActiveRobots());
            // Set robots that are not initialized to inactive
            for (unsigned int robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
              if (isRobotActive(robot_id) && isRobotInitialized(robot_id) && isRobotConnected(robot_id)) {
                setRobotActive(robot_id, true);
              } else {
                setRobotActive(robot_id, false);
              }
            }
            publishActiveRobotsCommand();
            publishUpdateCommand(getID());  // Kick off optimization
          } else {
            ROS_WARN("Not enough robots initialized.");
            publishHardTerminateCommand();
          }
        }
      }
      break;
    }

    case Command::UPDATE: {
      CHECK(!mParams.asynchronous);
      // Handle case when this robot is not active
      if (!isRobotActive(getID())) {
        ROS_WARN_STREAM("Robot " << getID() << " is deactivated. Ignore update command... ");
        return;
      }
      // Handle edge case when robots are out of sync
      if (mState != PGOAgentState::INITIALIZED) {
        ROS_WARN_STREAM("Robot " << getID() << " is not initialized. Ignore update command...");
        return;
      }
      // Update local record
      mTeamIterRequired[msg->executing_robot] = msg->executing_iteration;
      if (msg->executing_iteration != iteration_number() + 1) {
        ROS_WARN("Update iteration does not match local iteration. (received: %u, local: %u)",
                 msg->executing_iteration,
                 iteration_number() + 1);
      }
      if (msg->executing_robot == getID()) {
        mSynchronousOptimizationRequested = true;
        if (mParams.verbose) ROS_INFO("Robot %u to update at iteration %u.", getID(), msg->executing_iteration);
      } else {
        // Agents that are not selected for optimization can iterate immediately
        iterate(false);
        publishStatus();
      }
      break;
    }

    case Command::UPDATE_WEIGHT: {
      CHECK(!mParams.asynchronous);
      if (!isRobotActive(getID())) {
        ROS_WARN_STREAM("Robot " << getID() << " is deactivated. Ignore UPDATE_WEIGHT command... ");
        return;
      }
      logString("UPDATE_WEIGHT");
      updateMeasurementWeights();
      // Require latest iterations from all neighbor robots
      ROS_WARN("Require latest iteration %d from all neighbors.", iteration_number());
      for (const auto &neighbor : getNeighbors()) {
        mTeamIterRequired[neighbor] = iteration_number();
      }
      publishMeasurementWeights();
      publishPublicPoses(false);
      if (mParams.acceleration) publishPublicPoses(true);
      publishStatus();
      // The first resumes optimization by sending UPDATE command
      if (isLeader()) {
        publishUpdateCommand();
      }
      break;
    }

    case Command::SET_ACTIVE_ROBOTS: {
      if (msg->publishing_robot != getClusterID()) {
        ROS_WARN("Ignore SET_ACTIVE_ROBOTS command from non-leader %u.",
                 msg->publishing_robot);
        return;
      }
      // Update local record of currently active robots
      updateActiveRobots(msg);
      break;
    }

    case Command::NOOP: {
      // Do nothing
      break;
    }

    default:ROS_ERROR("Invalid command!");
  }
}

void PGOAgentROS::publicPosesCallback(const PublicPosesConstPtr &msg) {

  // Discard message sent by robots in other clusters
  if (msg->cluster_id != getClusterID()) {
    return;
  }
  
  std::vector<unsigned> neighbors = getNeighbors();
  if (std::find(neighbors.begin(), neighbors.end(), msg->robot_id) == neighbors.end()) {
    // Discard messages send by non-neighbors
    return;
  }

  // Generate a random permutation of indices
  PoseDict poseDict;
  for (size_t index = 0; index < msg->pose_ids.size(); ++index) {
    const PoseID nID(msg->robot_id, msg->pose_ids.at(index));
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

void PGOAgentROS::publicMeasurementsCallback(const RelativeMeasurementListConstPtr &msg) {
  // Ignore if does not have local odometry
  if (mPoseGraph->numOdometry() == 0)
    return;
  // Ignore if already received inter-robot loop closures from this robot
  if (mTeamReceivedSharedLoopClosures[msg->from_robot])
    return;
  // Ignore if from another cluster
  if (msg->from_cluster != getClusterID()) 
    return;
  mTeamReceivedSharedLoopClosures[msg->from_robot] = true;

  // Add inter-robot loop closures that involve this robot
  const auto num_before = mPoseGraph->numSharedLoopClosures();
  for (const auto &e : msg->edges) {
    if (e.robot_from == (int) getID() || e.robot_to == (int) getID()) {
      const auto measurement = RelativeMeasurementFromMsg(e);
      addMeasurement(measurement);
    }
  }
  const auto num_after = mPoseGraph->numSharedLoopClosures();
  ROS_INFO("Robot %u received measurements from %u: "
           "added %u missing measurements.", getID(), msg->from_robot, num_after - num_before);
}

void PGOAgentROS::measurementWeightsCallback(const RelativeMeasurementWeightsConstPtr &msg) {
  // if (mState != PGOAgentState::INITIALIZED) return;
  if (msg->destination_robot_id != getID()) return;
  if (msg->cluster_id != getClusterID()) return;
  bool weights_updated = false;
  for (size_t k = 0; k < msg->weights.size(); ++k) {
    const unsigned robotSrc = msg->src_robot_ids[k];
    const unsigned robotDst = msg->dst_robot_ids[k];
    const unsigned poseSrc = msg->src_pose_ids[k];
    const unsigned poseDst = msg->dst_pose_ids[k];
    const PoseID srcID(robotSrc, poseSrc);
    const PoseID dstID(robotDst, poseDst);
    double w = msg->weights[k];
    bool fixed = msg->fixed_weights[k];

    unsigned otherID;
    if (robotSrc == getID() && robotDst != getID()) {
      otherID = robotDst;
    } else if (robotDst == getID() && robotSrc != getID()) {
      otherID = robotSrc;
    } else {
      ROS_ERROR("Received weight for irrelevant measurement!");
      continue;
    }
    if (!isRobotActive(otherID)) continue;
    if (otherID < getID()) {
      if (setMeasurementWeight(srcID, dstID, w, fixed))
        weights_updated = true;
      else {
        ROS_WARN("Cannot find specified shared loop closure (%u, %u) -> (%u, %u)",
                  robotSrc, poseSrc, robotDst, poseDst);
      }
    }
  }
  if (weights_updated) {
    // Need to recompute data matrices in the pose graph
    mPoseGraph->clearDataMatrices();
  }
}

void PGOAgentROS::timerCallback(const ros::TimerEvent &event) {
  publishNoopCommand();
  publishLiftingMatrix();
  if (mPublishInitializeCommandRequested) {
    publishInitializeCommand();
  }
  if (mTryInitializeRequested) {
    tryInitialize();
  }
  if (mState == PGOAgentState::WAIT_FOR_DATA) {
    // Update leader robot when idle
    updateCluster();
    // Initialize a new round of dpgo
    int elapsed_sec = (ros::Time::now() - mLastResetTime).toSec();
    if (isLeader() && elapsed_sec > 10) {
      publishRequestPoseGraphCommand();
    }
  }
  if (mState == PGOAgentState::INITIALIZED) {
    publishPublicPoses(false);
    if (mParamsROS.acceleration)
      publishPublicPoses(true);
    publishMeasurementWeights();
    if (isLeader()) {
      publishAnchor();
      publishActiveRobotsCommand();
    }
  }
  publishStatus();
}

void PGOAgentROS::visualizationTimerCallback(const ros::TimerEvent &event) {
  publishOptimizedTrajectory();
  publishLoopClosureMarkers();
}

void PGOAgentROS::storeActiveNeighborPoses() {
  Matrix matrix;
  int num_poses_stored = 0;
  for (const auto &nbr_pose_id : mPoseGraph->activeNeighborPublicPoseIDs()) {
    if (getNeighborPoseInGlobalFrame(nbr_pose_id.robot_id, 
                                     nbr_pose_id.frame_id,
                                     matrix)) {
      Pose T(dimension());
      T.setData(matrix);
      mCachedNeighborPoses[nbr_pose_id] = T;
      num_poses_stored++;
    }
  }
  ROS_INFO("Stored %i neighbor poses in world frame.", num_poses_stored);
}

void PGOAgentROS::setInactiveNeighborPoses() {
  if (!YLift) {
    ROS_WARN("Missing lifting matrix! Cannot apply neighbor poses.");
    return;
  }
  int num_poses_initialized = 0;
  for (const auto &it : mCachedNeighborPoses) {
    const auto &pose_id = it.first;
    // Active neighbors will transmit their poses
    // Therefore we only use stored poses for inactive neighbors
    if (!isRobotActive(pose_id.robot_id)) {
      const auto &Ti = it.second;
      Matrix Xi_mat = YLift.value() * Ti.getData();
      LiftedPose Xi(r, d);
      Xi.setData(Xi_mat);
      neighborPoseDict[pose_id] = Xi;
      num_poses_initialized++;
    }
  }
  ROS_INFO("Set %i inactive neighbor poses.", num_poses_initialized);
}

void PGOAgentROS::storeActiveEdgeWeights() {
  int num_edges_stored = 0;
  for (const RelativeSEMeasurement *m: mPoseGraph->activeLoopClosures()) {
    const PoseID src_id(m->r1, m->p1);
    const PoseID dst_id(m->r2, m->p2);
    const EdgeID edge_id(src_id, dst_id);
    if (edge_id.isSharedLoopClosure()) {
      mCachedEdgeWeights[edge_id] = m->weight;
      num_edges_stored++;
    }
  }
  ROS_INFO("Stored %i active edge weights.", num_edges_stored);
}

void PGOAgentROS::setInactiveEdgeWeights() {
  int num_edges_set = 0;
  for (RelativeSEMeasurement *m: mPoseGraph->inactiveLoopClosures()) {
    const PoseID src_id(m->r1, m->p1);
    const PoseID dst_id(m->r2, m->p2);
    const EdgeID edge_id(src_id, dst_id);
    const auto &it = mCachedEdgeWeights.find(edge_id);
    if (it != mCachedEdgeWeights.end()) {
      m->weight = it->second;
      num_edges_set++;
    } 
  }
  ROS_INFO("Set %i inactive edge weights.", num_edges_set);
}

void PGOAgentROS::initializeGlobalAnchor() {
  if (!YLift) {
    ROS_WARN("Missing lifting matrix! Cannot initialize global anchor.");
    return;
  }
  LiftedPose X(r, d);
  X.rotation() = YLift.value();
  X.translation() = Vector::Zero(r);
  setGlobalAnchor(X.getData());
  ROS_INFO("Initialized global anchor.");
}

unsigned PGOAgentROS::getClusterID() const {
  return mClusterID;
}

bool PGOAgentROS::isLeader() const {
  return getID() == getClusterID();
}

void PGOAgentROS::updateCluster() {
  for (unsigned int robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    if (isRobotConnected(robot_id)) {
      mClusterID = robot_id;
      break;
    }
  }
  ROS_INFO("Robot %u joins cluster %u.", getID(), mClusterID);
}

unsigned PGOAgentROS::getRobotClusterID(unsigned robot_id) const {
  if (robot_id > mParams.numRobots) {
    ROS_ERROR("Robot ID %u larger than number of robots.", robot_id);
    return robot_id;
  }
  return mTeamClusterID[robot_id];
}

void PGOAgentROS::setRobotClusterID(unsigned robot_id, unsigned cluster_id) {
  if (robot_id > mParams.numRobots) {
    ROS_ERROR("Robot ID %u larger than number of robots.", robot_id);
    return;
  }
  if (cluster_id > mParams.numRobots) {
    ROS_ERROR("Cluster ID %u larger than number of robots.", cluster_id);
    return;
  }
  mTeamClusterID[robot_id] = cluster_id;
}

void PGOAgentROS::resetRobotClusterIDs() {
  mTeamClusterID.assign(mParams.numRobots, 0);
  for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    mTeamClusterID[robot_id] = robot_id;
  }
}

void PGOAgentROS::checkCommandTimeout() {
  // Timeout if command channel quiet for long time 
  // This usually happen when robots get disconnected 
  double elapsedSecond = (ros::Time::now() - mLastCommandTime).toSec();
  if (elapsedSecond > mParamsROS.timeoutThreshold) {
    ROS_WARN("Robot %u timeout: last command was %f sec ago.",
             getID(), elapsedSecond);
    reset();
    if (isLeader()) {
      publishHardTerminateCommand();
    }
    mLastCommandTime = ros::Time::now();
  }
}

void PGOAgentROS::checkDisconnectedRobot() {
  for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
    if (isRobotActive(robot_id) && !isRobotConnected(robot_id)) {
      ROS_WARN("Active robot %u is disconnected.", robot_id);
      publishHardTerminateCommand();
      break;
    }
  }
}

}  // namespace dpgo_ros
