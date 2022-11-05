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
      mInitStepsDone(0),
      mTotalBytesReceived(0),
      mIterationElapsedMs(0) {
  mTeamIterRequired.assign(mParams.numRobots, 0);
  mTeamIterReceived.assign(mParams.numRobots, 0);
  mTeamReceivedSharedLoopClosures.assign(mParams.numRobots, false);

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

  if (mPublishWeightsRequested) {
    publishMeasurementWeights();
    mPublishWeightsRequested = false;
  }

  if (mState == PGOAgentState::INITIALIZED) {
    // Terminate if quiet for long time (possible message drop)
    auto counter = std::chrono::high_resolution_clock::now() - mLastCommandTime;
    double elapsedSecond = (double) std::chrono::duration_cast<std::chrono::milliseconds>(counter).count() / 1e3;
    if (elapsedSecond > 60) {
      ROS_WARN("Agent %u terminate: last command was 60 sec ago.", getID());
      reset();
      if (getID() == 0) publishHardTerminateCommand();
    }
  }

}

void PGOAgentROS::reset() {
  PGOAgent::reset();
  mSynchronousOptimizationRequested = false;
  mInitStepsDone = 0;
  mTeamIterRequired.assign(mParams.numRobots, 0);
  mTeamIterReceived.assign(mParams.numRobots, 0);
  mTeamReceivedSharedLoopClosures.assign(mParams.numRobots, false);
  mTotalBytesReceived = 0;
  mInitPoses.reset();
  mTeamStatusMsg.clear();
  if (mIterationLog.is_open())
    mIterationLog.close();
}

bool PGOAgentROS::initializePoseGraph() {
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

  ROS_INFO("Received pose graph from ROS service.");

  // Process edges
  for (const auto &edge : pose_graph.edges) {
    RelativeSEMeasurement m = RelativeMeasurementFromMsg(edge);
    if (m.r1 != getID() && m.r2 != getID()) {
      ROS_ERROR("Robot %u received irrelevant measurement! ", getID());
    }
    addMeasurement(m);
  }

  // Process nodes
  mInitPoses.reset();
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
      mInitPoses.emplace(initial_poses);
    }
  }

  mTeamReceivedSharedLoopClosures.assign(mParams.numRobots, false);
  for (size_t robot_id = getID(); robot_id < mParams.numRobots; ++robot_id)
    mTeamReceivedSharedLoopClosures[robot_id] = true;
  tryInitializeOptimization();
  return true;
}

bool PGOAgentROS::tryInitializeOptimization() {
  // If received shared loop closures from all robots, proceed to initialize optimization
  bool ready = true;
  for (const auto &b : mTeamReceivedSharedLoopClosures) {
    if (!b) ready = false;
  }
  if (ready) {
    initialize();
    ROS_INFO("Robot %u sets pose graph. "
             "num_poses:%u, odom:%u, local_lc:%u, shared_lc:%u, init_guess:%d",
             getID(),
             num_poses(),
             mPoseGraph->numOdometry(),
             mPoseGraph->numPrivateLoopClosures(),
             mPoseGraph->numSharedLoopClosures(),
             mInitPoses.has_value());
  }
  return ready;
}

void PGOAgentROS::runOnceAsynchronous() {
  if (mPublishAsynchronousRequested) {
    if (getID() == 0) publishAnchor();
    publishStatus();
    if (mParamsROS.publishIterate) {
      publishTrajectory();
      publishLoopClosureMarkers();
    }
    if (mParams.logData) {
      logIteration();
    }
    mPublishAsynchronousRequested = false;
  }

  // Check for termination condition
  if (getID() == 0 && shouldTerminate()) {
    publishTerminateCommand();
  }
}

void PGOAgentROS::runOnceSynchronous() {
  CHECK(!mParams.asynchronous);

  // Perform an optimization step
  if (mSynchronousOptimizationRequested) {

    // Check if ready to perform iterate
    bool ready = true;
    for (unsigned neighbor : getNeighbors()) {
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
      // Iterate
      auto startTime = std::chrono::high_resolution_clock::now();
      iterate(true);
      auto counter = std::chrono::high_resolution_clock::now() - startTime;
      mIterationElapsedMs = (double) std::chrono::duration_cast<std::chrono::milliseconds>(counter).count();
      mSynchronousOptimizationRequested = false;

      // First robot publish anchor
      if (getID() == 0) publishAnchor();

      // Publish status
      publishStatus();

      // Publish trajectory
      if (mParamsROS.publishIterate) {
        publishTrajectory();
        publishLoopClosureMarkers();
      }

      // Log local iteration
      if (mParams.logData) {
        logIteration();
      }

      // Check termination condition OR notify next robot to update
      if (shouldTerminate()) {
        publishTerminateCommand();
      } else {
        // Notify next robot to update
        if (mParamsROS.interUpdateSleepTime > 1e-3)
          ros::Duration(mParamsROS.interUpdateSleepTime).sleep();
        publishUpdateCommand();
      }
    }
  }
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
  CHECK(!mParams.asynchronous);
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();

  switch(mParamsROS.updateRule) {
    case PGOAgentROSParameters::UpdateRule::Uniform: {
      // Uniform sampling of all robots
      std::vector<double> neighborWeights(mParams.numRobots);
      for (size_t j = 0; j < mParams.numRobots; ++j) {
        neighborWeights[j] = 1;
      }
      std::discrete_distribution<int> distribution(neighborWeights.begin(),
                                                   neighborWeights.end());
      std::random_device rd;
      std::mt19937 gen(rd());
      msg.executing_robot = distribution(gen);
      break;
    }
    case PGOAgentROSParameters::UpdateRule::RoundRobin: {
      // Round robin updates
      msg.executing_robot = (getID() + 1) % mParams.numRobots;
      break;
    }
  }
  msg.command = Command::UPDATE;
  msg.executing_iteration = iteration_number() + 1;
  ROS_INFO_STREAM("Send UPDATE to robot " << msg.executing_robot << " to perform iteration " << msg.executing_iteration);
  mCommandPublisher.publish(msg);
}

void PGOAgentROS::publishTerminateCommand() {
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.command = Command::TERMINATE;
  mCommandPublisher.publish(msg);
  ROS_INFO("Robot %u published TERMINATE command.", getID());
}

void PGOAgentROS::publishHardTerminateCommand() {
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.command = Command::HARD_TERMINATE;
  mCommandPublisher.publish(msg);
  ROS_INFO("Robot %u published HARD TERMINATE command.", getID());
}

void PGOAgentROS::publishRequestPoseGraphCommand() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should send request pose graph command! ");
  }
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.command = Command::REQUESTPOSEGRAPH;
  mCommandPublisher.publish(msg);
  ROS_INFO("Robot %u published REQUESTPOSEGRAPH command.", getID());
}

void PGOAgentROS::publishInitializeCommand() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should send INITIALIZE command!");
  }
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.command = Command::INITIALIZE;
  mCommandPublisher.publish(msg);
  mInitStepsDone++;
  ROS_INFO("Robot %u published INITIALIZE command.", getID());
}

void PGOAgentROS::publishNoopCommand() {
  Command msg;
  msg.header.stamp = ros::Time::now();
  msg.publishing_robot = getID();
  msg.command = Command::NOOP;
  mCommandPublisher.publish(msg);
}

void PGOAgentROS::publishStatus() {
  Status msg = statusToMsg(getStatus());
  msg.header.stamp = ros::Time::now();
  mStatusPublisher.publish(msg);
}

bool PGOAgentROS::publishTrajectory() {
  Matrix T;
  if (!getTrajectoryInGlobalFrame(T)) {
    return false;
  }
  randomSleep(5.0);

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
  for (unsigned neighbor: getNeighbors()) {
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
        msg.destination_robot_id = otherID;
        msg_map[otherID] = msg;
      }
      msg_map[otherID].src_robot_ids.push_back(m.r1);
      msg_map[otherID].dst_robot_ids.push_back(m.r2);
      msg_map[otherID].src_pose_ids.push_back(m.p1);
      msg_map[otherID].dst_pose_ids.push_back(m.p2);
      msg_map[otherID].weights.push_back(m.weight);
    }
  }
  for (const auto& it: msg_map) {
    const auto &msg = it.second;
    if (!msg.weights.empty()) {
      if (msg.destination_robot_id <= getID()) {
        ROS_ERROR("Attempt to send measurement weights to robot %i.", 
                  msg.destination_robot_id);
      } else {
        mMeasurementWeightsPublisher.publish(msg);
      }
    }
  }
}

void PGOAgentROS::publishLoopClosureMarkers() {
  if (mState != PGOAgentState::INITIALIZED) return;

  randomSleep(5.0);

  visualization_msgs::Marker line_list;
  line_list.id = (int) getID();
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.1;
  line_list.header.frame_id = "/world";
  line_list.color.b = 1.0;
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
    }
  }
  for (const auto &measurement : mPoseGraph->sharedLoopClosures()) {
    Matrix mT, nT;
    Matrix mt, nt;
    bool mb, nb;
    size_t neighbor_id;
    if (measurement.r1 == getID()) {
      neighbor_id = measurement.r2;
      mb = getPoseInGlobalFrame(measurement.p1, mT);
      nb = getNeighborPoseInGlobalFrame(measurement.r2, measurement.p2, nT);
    } else {
      neighbor_id = measurement.r1;
      mb = getPoseInGlobalFrame(measurement.p2, mT);
      nb = getNeighborPoseInGlobalFrame(measurement.r1, measurement.p1, nT);
    }
    if (mb && nb && getID() < neighbor_id) {
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
  if (!line_list.points.empty()) mLoopClosureMarkerPublisher.publish(line_list);
}

bool PGOAgentROS::createIterationLog(const std::string &filename) {
  if (mIterationLog.is_open())
    mIterationLog.close();
  mIterationLog.open(filename);
  if (!mIterationLog.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }
  // Instance number, global iteration number, Number of poses, total bytes
  // received, iteration time (sec), total elapsed time (sec), relative change
  mIterationLog << "instance, iteration, num_poses, total_bytes_received, "
                   "iteration_time_sec, total_time_sec, relative_change \n";
  mIterationLog.flush();
  return true;
}

bool PGOAgentROS::logIteration() {
  if (!mIterationLog.is_open()) {
    ROS_ERROR_STREAM("No iteration log file!");
    return false;
  }

  // Compute total elapsed time since beginning of optimization
  auto counter = std::chrono::high_resolution_clock::now() - mGlobalStartTime;
  double globalElapsedMs = (double) std::chrono::duration_cast<std::chrono::milliseconds>(counter).count();

  // Instance number, global iteration number, Number of poses, total bytes
  // received, iteration time (sec), total elapsed time (sec), relative change
  mIterationLog << instance_number() << ",";
  mIterationLog << iteration_number() << ",";
  mIterationLog << num_poses() << ",";
  mIterationLog << mTotalBytesReceived << ",";
  mIterationLog << mIterationElapsedMs / 1e3 << ",";
  mIterationLog << globalElapsedMs / 1e3 << ",";
  mIterationLog << mStatus.relativeChange << "\n";
  mIterationLog.flush();
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
  setNeighborStatus(statusFromMsg(received_msg));
}

void PGOAgentROS::commandCallback(const CommandConstPtr &msg) {

  switch (msg->command) {
    case Command::REQUESTPOSEGRAPH: {
      mLastCommandTime = std::chrono::high_resolution_clock::now();
      ROS_INFO("Robot %u received REQUESTPOSEGRAPH command.", getID());
      if (mState != PGOAgentState::WAIT_FOR_DATA) {
        ROS_WARN_STREAM("Robot " << getID() << " receives REQUESTPOSEGRAPH command, but status is not WAIT_FOR_DATA. ");
        ROS_WARN_STREAM("Robot " << getID() << " reset... ");
        reset();
      }
      // Request latest pose graph
      initializePoseGraph();
      // Create log file for new round
      if (mParams.logData) {
        createIterationLog(mParams.logDirectory + "dpgo_log_" + std::to_string(instance_number()) + ".csv");
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
      mLastCommandTime = std::chrono::high_resolution_clock::now();
      ROS_INFO("Robot %u received TERMINATE command. ", getID());
      // Publish optimized trajectory
      publishTrajectory();
      publishLoopClosureMarkers();
      
      reset();
      // First robot initiates next optimization round
      if (getID() == 0) {
        ros::Duration(10).sleep();
        publishRequestPoseGraphCommand();
      }
      break;
    }

    case Command::HARD_TERMINATE: {
      mLastCommandTime = std::chrono::high_resolution_clock::now();
      ROS_INFO("Robot %u received HARD TERMINATE command. ", getID());
      reset();
      // First robot initiates next optimization round
      if (getID() == 0) {
        ros::Duration(10).sleep();
        publishRequestPoseGraphCommand();
      }
      break;
    }

    case Command::INITIALIZE: {
      mLastCommandTime = std::chrono::high_resolution_clock::now();
      mGlobalStartTime = std::chrono::high_resolution_clock::now();
      publishPublicMeasurements();
      publishPublicPoses(false);
      if (getID() == 0)
        publishLiftingMatrix();
      publishStatus();
      if (getID() == 0) {
        ros::Duration(0.1).sleep();
        if (mInitStepsDone > mParamsROS.maxDistributedInitSteps) {
          ROS_WARN("Exceeded maximum number of initialization steps. Send TERMINATE command.");
          publishHardTerminateCommand();
          return;
        }
        for (unsigned robot_id = 0; robot_id < mParams.numRobots; ++robot_id) {
          const auto &it = mTeamStatus.find(robot_id);
          if (it == mTeamStatus.end()) {
            ROS_WARN("Robot %u status not available.", robot_id);
            mPublishInitializeCommandRequested = true;
            return;
          }
          const auto &status = it->second;
          if (status.state == PGOAgentState::WAIT_FOR_DATA) {
            ROS_WARN("Robot %u has not received pose graph.", status.agentID);
            mPublishInitializeCommandRequested = true;
            return;
          }
          if (status.state == PGOAgentState::WAIT_FOR_INITIALIZATION) {
            ROS_WARN("Robot %u has not initialized in global frame.", status.agentID);
            mPublishInitializeCommandRequested = true;
            return;
          }
        }
        // In the synchronous mode, kick off optimization by sending UPDATE command
        if (!mParams.asynchronous) {
          Command cmdMsg;
          cmdMsg.command = Command::UPDATE;
          cmdMsg.executing_robot = 0;
          cmdMsg.executing_iteration = iteration_number() + 1;
          mCommandPublisher.publish(cmdMsg);
        }
      }
      break;
    }

    case Command::UPDATE: {
      mLastCommandTime = std::chrono::high_resolution_clock::now();
      CHECK(!mParams.asynchronous);
      // Handle edge case when robots are out of sync
      if (mState != PGOAgentState::INITIALIZED) {
        ROS_WARN_STREAM("Robot " << getID() << " receives UPDATE command, but status is not INITIALIZED. ");
        ROS_WARN_STREAM("Robot " << getID() << " reset... ");
        reset();
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

    case Command::NOOP: {
      // Do nothing
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

  // Proceed to initialize optimization
  tryInitializeOptimization();
}

void PGOAgentROS::measurementWeightsCallback(const RelativeMeasurementWeightsConstPtr &msg) {
  if (mState != PGOAgentState::INITIALIZED) return;
  bool weights_updated = false;
  for (size_t k = 0; k < msg->weights.size(); ++k) {
    const unsigned robotSrc = msg->src_robot_ids[k];
    const unsigned robotDst = msg->dst_robot_ids[k];
    const unsigned poseSrc = msg->src_pose_ids[k];
    const unsigned poseDst = msg->dst_pose_ids[k];
    const PoseID srcID(robotSrc, poseSrc);
    const PoseID dstID(robotDst, poseDst);
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
      if (setPublicMeasurementWeight(srcID, dstID, w))
        weights_updated = true;
      else {
        ROS_ERROR("Cannot find specified shared loop closure (%u, %u) -> (%u, %u)",
                  robotSrc, poseSrc, robotDst, poseDst);
        publishHardTerminateCommand();
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
  publishStatus();
  if (mPublishInitializeCommandRequested) {
    publishInitializeCommand();
    mPublishInitializeCommandRequested = false;
  }
  if (mState == PGOAgentState::INITIALIZED) {
    publishPublicPoses(false);
    if (mParamsROS.acceleration) publishPublicPoses(true);
  }
}

void PGOAgentROS::randomSleep(double sec) {
  if (sec < 1e-3)
    return;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> distribution(0.0, sec);
  double sleep_time = distribution(gen);
  ROS_INFO("Sleep %f sec...", sleep_time);
  ros::Duration(sleep_time).sleep();
}
}  // namespace dpgo_ros
