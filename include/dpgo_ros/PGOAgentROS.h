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
  PGOAgentROS(const ros::NodeHandle& nh_, unsigned ID,
              const PGOAgentParameters &params);

  ~PGOAgentROS();

 private:
  // ROS node handle
  ros::NodeHandle nh;

  // Total bytes of public poses received
  size_t totalBytesReceived;

  // Elapsed time for the latest update
  double iterationElapsedMs;

  // Latest optimization result
  ROPTResult OptResult;

  // Reset the pose graph. This function overrides the function from the base
  // class.
  void reset();

  // Apply local update
  void update();

  // Request latest local pose graph
  bool requestPoseGraph();

  // Request latest public poses from a neighboring agent
  bool requestPublicPosesFromAgent(const unsigned &neighborID);

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

  // Log iteration
  static bool createLogFile(const std::string &filename);

  bool logIteration(const std::string &filename) const;

  // ROS callbacks
  void anchorCallback(const LiftedPoseConstPtr &msg);
  void statusCallback(const StatusConstPtr &msg);
  void commandCallback(const CommandConstPtr &msg);
  bool queryLiftingMatrixCallback(QueryLiftingMatrixRequest &request,
                                  QueryLiftingMatrixResponse &response);
  bool queryPosesCallback(QueryPosesRequest &request,
                          QueryPosesResponse &response);

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