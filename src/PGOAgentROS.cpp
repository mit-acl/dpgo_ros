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

PGOAgentROS::PGOAgentROS(ros::NodeHandle nh_, unsigned ID,
                         const PGOAgentParameters& params)
    : PGOAgent(ID, params),
      nh(nh_),
      instance_number(0),
      iteration_number(0),
      hasPoseGraph(false),
      savedInitialization(false),
      totalBytesReceived(0) {
  logOutput = ros::param::get("~log_output_path", logOutputDirectory);

  if (!nh.getParam("/relative_change_tolerance", RelativeChangeTolerance)) {
    ROS_ERROR("Failed to get relative change tolerance!");
    ros::shutdown();
  }

  int MaxIterationNumberInt;
  if (!nh.getParam("/max_iteration_number", MaxIterationNumberInt)) {
    ROS_ERROR("Failed to get maximum iteration number!");
    ros::shutdown();
  }
  if (MaxIterationNumberInt <= 0) {
    ROS_ERROR("Maximum iteration number must be positive!");
    ros::shutdown();
  }
  MaxIterationNumber = (unsigned)MaxIterationNumberInt;

  int num_robots;
  if (!nh.getParam("/num_robots", num_robots))
    ROS_ERROR("Failed to query number of robots");
  relativeChanges.resize(num_robots, 1e3);

  // ROS subscriber
  statusSubscriber =
      nh.subscribe("/dpgo_status", 100, &PGOAgentROS::statusCallback, this);

  commandSubscriber =
      nh.subscribe("/dpgo_command", 100, &PGOAgentROS::commandCallback, this);

  anchorSubscriber =
      nh.subscribe("/dpgo_anchor", 100, &PGOAgentROS::anchorCallback, this);

  // ROS service
  queryLiftingMatrixServer = nh.advertiseService(
      "query_lifting_matrix", &PGOAgentROS::queryLiftingMatrixCallback, this);

  queryPoseServer = nh.advertiseService("query_poses",
                                        &PGOAgentROS::queryPosesCallback, this);

  // ROS publisher
  anchorPublisher = nh.advertise<LiftedPose>("/dpgo_anchor", 100);
  statusPublisher = nh.advertise<Status>("/dpgo_status", 100);
  commandPublisher = nh.advertise<Command>("/dpgo_command", 100);
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
    ros::Duration(10).sleep();
    publishInitializeCommand();
  }
}

PGOAgentROS::~PGOAgentROS() {}

void PGOAgentROS::reset() {
  PGOAgent::reset();
  instance_number++;
  iteration_number = 0;
  hasPoseGraph = false;
  savedInitialization = false;
  totalBytesReceived = 0;
  for (size_t i = 0; i < relativeChanges.size(); ++i) {
    relativeChanges[i] = 1e3;
  }
}

void PGOAgentROS::update() {
  ROS_INFO_STREAM("Agent " << getID() << " updating...");
  auto startTime = std::chrono::high_resolution_clock::now();

  // Query neighbors for their public poses
  std::vector<unsigned> neighborAgents = getNeighbors();
  for (unsigned neighborID : neighborAgents) {
    if (!requestPublicPosesFromAgent(neighborID)) {
      ROS_WARN_STREAM("Public poses from neighbor " << neighborID
                                                    << " are not available.");
    }
  }

  // Save initial trajectory
  if (!savedInitialization && getState() == PGOAgentState::INITIALIZED) {
    if (logOutput) {
      logTrajectory(logOutputDirectory + "dpgo_initial_" +
                    std::to_string(instance_number) + ".csv");
    }
    savedInitialization = true;
  }

  // Optimize!
  OptResult = optimize();
  if (!OptResult.success) {
    ROS_WARN("Skipped optimization!");
  } else {
    relativeChanges[getID()] = OptResult.relativeChange;
  }

  // Record overall elapsed time
  auto counter = std::chrono::high_resolution_clock::now() - startTime;
  iterationElapsedMs =
      std::chrono::duration_cast<std::chrono::milliseconds>(counter).count();

  // Log iteration information
  if (logOutput) {
    logIteration(logOutputDirectory + "dpgo_log_" +
                 std::to_string(instance_number) + ".csv");
  }
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
  // Save pose graph
  if (logOutput) {
    pose_graph_tools::savePoseGraphMsgToFile(
        pose_graph, logOutputDirectory + "dpgo_pose_graph_" +
                        std::to_string(instance_number) + ".csv");
  }
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

  ROS_INFO_STREAM(
      "Agent " << getID() << " receives local pose graph with "
               << odometry.size() << " odometry edges and "
               << privateLoopClosures.size() << " private loop closures and "
               << sharedLoopClosures.size() << " shared loop closures.");

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
      "/kimera" + std::to_string(neighborID) + "/dpgo_ros_node/query_poses";

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
    totalBytesReceived += computeLiftedPosePayloadBytes(poseNbr);
  }

  return true;
}

bool PGOAgentROS::shouldTerminate() {
  // terminate if the calling agent does not have pose graph
  if (!hasPoseGraph) {
    return true;
  }

  // terminate if reached maximum iterations
  if (iteration_number > MaxIterationNumber) {
    ROS_WARN("DPGO reached maximum iterations.");
    return true;
  }

  // terminate if all agents satisfy relative change condition
  for (size_t i = 0; i < relativeChanges.size(); ++i) {
    if (relativeChanges[i] > RelativeChangeTolerance) {
      return false;
    }
  }

  return true;
}

void PGOAgentROS::publishAnchor() {
  Matrix T0;
  getXComponent(0, T0);
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
      unsigned nID = neighbors[j];
      neighborWeights[j] = relativeChanges[nID];
    }
    std::discrete_distribution<int> distribution(neighborWeights.begin(),
                                                 neighborWeights.end());
    std::cout << std::endl;
    std::random_device rd;
    std::mt19937 gen(rd());
    msg.executing_robot = neighbors[distribution(gen)];
  }

  msg.command = Command::UPDATE;
  commandPublisher.publish(msg);
}

void PGOAgentROS::publishTerminateCommand() {
  Command msg;
  msg.command = Command::TERMINATE;
  commandPublisher.publish(msg);
}

void PGOAgentROS::publishInitializeCommand() {
  if (getID() != 0) {
    ROS_ERROR("Only robot 0 should send INITIALIZE command! ");
  }
  Command msg;
  msg.command = Command::INITIALIZE;
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
  if (globalAnchor.rows() != relaxation_rank() ||
      globalAnchor.cols() != dimension() + 1) {
    ROS_WARN("Did not receive anchor to compute trajectory in global frame.");
    return false;
  }

  Matrix T;
  if (!getTrajectoryInGlobalFrame(globalAnchor, T)) {
    ROS_WARN("Failed to compute trajectory in global frame!");
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

bool PGOAgentROS::logTrajectory(const std::string& filename) {
  if (globalAnchor.rows() != relaxation_rank() ||
      globalAnchor.cols() != dimension() + 1) {
    ROS_WARN("Did not receive anchor to compute trajectory in global frame.");
    return false;
  }

  Matrix T;
  if (!getTrajectoryInGlobalFrame(globalAnchor, T)) {
    ROS_WARN("Failed to compute trajectory in global frame!");
    return false;
  }

  // Convert trajectory to a pose array
  geometry_msgs::PoseArray pose_array =
      TrajectoryToPoseArray(dimension(), num_poses(), T);

  // Save to specified place
  savePoseArrayToFile(pose_array, filename);

  return true;
}

bool PGOAgentROS::createLogFile(const std::string& filename) {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }

  file << "instance, iteration, num_poses, total_bytes_received, "
          "optimization_time_sec, func_decrease, relative_change \n";
  file.close();
  return true;
}

bool PGOAgentROS::logIteration(const std::string& filename) {
  std::ofstream file;
  file.open(filename, std::ios_base::app);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }

  // Instance number, global iteration number, Number of poses, total bytes
  // received, overall iteration time (sec), optimization only time (sec),
  // function decrease, relative change
  file << instance_number << ",";
  file << iteration_number << ",";
  file << num_poses() << ",";
  file << totalBytesReceived << ",";
  file << iterationElapsedMs / 1e3 << ",";
  file << OptResult.elapsedMs / 1e3 << ",";
  file << OptResult.fOpt - OptResult.fInit << ",";
  file << OptResult.relativeChange << "\n";
  file.close();
  return true;
}

void PGOAgentROS::anchorCallback(const LiftedPoseConstPtr& msg) {
  if (msg->robot_id != 0 || msg->pose_id != 0) {
    ROS_ERROR("Received wrong pose as anchor!");
    ros::shutdown();
  }
  globalAnchor = MatrixFromMsg(msg->pose);
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
      ROS_INFO_STREAM("Agent " << getID() << " initiates round "
                               << instance_number << "...");
      // Request latest pose graph
      hasPoseGraph = requestPoseGraph();
      // Create log file for new round
      if (logOutput) {
        createLogFile(logOutputDirectory + "dpgo_log_" +
                      std::to_string(instance_number) + ".csv");
      }
      // First robot initiates update sequence
      if (getID() == 0) {
        if (hasPoseGraph) publishAnchor();
        ros::Duration(1).sleep();
        Command msg;
        msg.command = Command::UPDATE;
        msg.executing_robot = 0;
        commandPublisher.publish(msg);
      }
      break;

    case Command::TERMINATE: {
      ROS_INFO_STREAM("Agent " << getID() << " received terminate command. ");
      // Publish optimized trajectory
      publishTrajectory();
      // Log optimized trajectory
      if (logOutput) {
        logTrajectory(logOutputDirectory + "dpgo_optimized_" +
                      std::to_string(instance_number) + ".csv");
      }
      // Reset!
      reset();
      // First robot initiates next optimization round
      if (getID() == 0) {
        ros::Duration(30).sleep();
        publishInitializeCommand();
      }
      break;
    }

    case Command::UPDATE:
      iteration_number++;  // increment iteration counter
      if (msg->executing_robot == getID()) {
        // Check termination condition
        if (shouldTerminate()) {
          publishTerminateCommand();
          return;
        }

        // Update my estimate
        update();

        // The first robot should also publishes its anchor
        if (getID() == 0) publishAnchor();

        // Publish status
        publishStatus();

        // Notify next robot to update
        publishUpdateCommand();
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
