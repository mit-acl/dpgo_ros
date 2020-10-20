/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef PGOAGENTROS_H
#define PGOAGENTROS_H

#include <DPGO/PGOAgent.h>
#include <dpgo_ros/Command.h>
#include <dpgo_ros/QueryLiftingMatrix.h>
#include <dpgo_ros/QueryPoses.h>
#include <dpgo_ros/Status.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/console.h>
#include <ros/ros.h>

using namespace DPGO;

namespace dpgo_ros {

class PGOAgentROS : public PGOAgent {
 public:
  PGOAgentROS(ros::NodeHandle nh_, unsigned ID,
              const PGOAgentParameters& params);

  ~PGOAgentROS();

 private:
  // ROS node handle
  ros::NodeHandle nh;

  // Current instance number
  unsigned instance_number;

  // Current iteration number
  unsigned iteration_number;

  // Received pose graph
  bool has_pose_graph;

  // Saved initialization to file
  bool saved_initialization;

  // Total bytes of public poses received
  size_t bytes_received;

  // Flag to log data 
  bool logOutput;
  std::string logOutputDirectory;

  // Latest relative changes of all robots
  std::vector<double> relativeChanges;

  // Termination condition
  double RelativeChangeTolerance;

  // Maximum number of iterations
  unsigned MaxIterationNumber;

  // Latest optimization result
  ROPTResult OptResult;

  // Anchor needed for rounding
  Matrix globalAnchor;

  // Reset the pose graph. This function overrides the function from the base
  // class.
  void reset();

  // Apply local update
  void update();

  // Request latest local pose graph
  bool requestPoseGraph();

  // Request latest public poses from a neighboring agent
  bool requestPublicPosesFromAgent(const unsigned& neighborID);

  // Check DPGO termination conditions
  bool shouldTerminate();

  // Publish status
  void publishStatus();

  // Publish initialize command
  void publishInitializeCommand();

  // Publish update command
  void publishUpdateCommand();

  // Publish termination command
  void publishTerminateCommand();

  // Publish anchor
  void publishAnchor();

  // Publish trajectory
  bool publishTrajectory();

  // Log current trajectory to file
  bool logTrajectory(const std::string& filename);

  // Log statistics to file
  bool createLogFile(const std::string& filename);
  bool logIteration(const std::string& filename);

  // ROS callbacks
  void anchorCallback(const LiftedPoseConstPtr& msg);
  void statusCallback(const StatusConstPtr& msg);
  void commandCallback(const CommandConstPtr& msg);
  bool queryLiftingMatrixCallback(QueryLiftingMatrixRequest& request,
                                  QueryLiftingMatrixResponse& response);
  bool queryPosesCallback(QueryPosesRequest& request,
                          QueryPosesResponse& response);

  // ROS publisher
  ros::Publisher anchorPublisher;
  ros::Publisher statusPublisher;
  ros::Publisher commandPublisher;
  ros::Publisher poseArrayPublisher;
  ros::Publisher pathPublisher;

  // ROS subscriber
  ros::Subscriber statusSubscriber;
  ros::Subscriber commandSubscriber;
  ros::Subscriber anchorSubscriber;

  // ROS service server
  ros::ServiceServer queryLiftingMatrixServer;
  ros::ServiceServer queryPoseServer;
};

}  // namespace dpgo_ros

#endif