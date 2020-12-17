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
#include <dpgo_ros/PublicPoses.h>
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

  ~PGOAgentROS() = default;

  /**
   * @brief Function to be called at every ROS spin.
   */
  void runOnce();

 private:
  // ROS node handle
  ros::NodeHandle nh;

  // True if this agent is scheduled to perform optimization
  bool mOptimizationRequested;

  // Number of initialization steps performed
  size_t initCount;

  // Total bytes of public poses received
  size_t totalBytesReceived;

  // Elapsed time for the latest update
  double iterationElapsedMs;

  // Data structures to enforce synchronization during iterations
  std::vector<unsigned> mTeamIterReceived;
  std::vector<unsigned> mTeamIterRequired;

  // Reset the pose graph. This function overrides the function from the base
  // class.
  void reset();

  // Request latest local pose graph
  bool requestPoseGraph();

  // Publish status
  void publishStatus();

  // Publish command to request pose graph
  void publishRequestPoseGraphCommand();

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

  // Publish latest public poses
  void publishPublicPoses(bool aux);

  // Log iteration
  static bool createLogFile(const std::string &filename);

  bool logIteration(const std::string &filename) const;

  // ROS callbacks
  void anchorCallback(const LiftedPoseConstPtr &msg);
  void statusCallback(const StatusConstPtr &msg);
  void commandCallback(const CommandConstPtr &msg);
  void publicPosesCallback(const PublicPosesConstPtr &msg);
  bool queryLiftingMatrixCallback(QueryLiftingMatrixRequest &request, QueryLiftingMatrixResponse &response);

  // ROS publisher
  ros::Publisher anchorPublisher;
  ros::Publisher statusPublisher;
  ros::Publisher commandPublisher;
  ros::Publisher publicPosesPublisher;
  ros::Publisher poseArrayPublisher;
  ros::Publisher pathPublisher;

  // ROS subscriber
  ros::Subscriber statusSubscriber;
  ros::Subscriber commandSubscriber;
  ros::Subscriber anchorSubscriber;
  ros::Subscriber publicPosesSubscriber;

  // ROS service server
  ros::ServiceServer queryLiftingMatrixServer;
};

}  // namespace dpgo_ros

#endif