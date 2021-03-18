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
#include <dpgo_ros/RelativeMeasurementWeights.h>
#include <dpgo_ros/QueryLiftingMatrix.h>
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

  // Number of initialization steps performed
  size_t mInitStepsDone;

  // Total bytes of public poses received
  size_t mTotalBytesReceived;

  // Elapsed time for the latest update
  double mIterationElapsedMs;

  // Global optimization start time
  std::chrono::time_point<std::chrono::high_resolution_clock> mGlobalStartTime;

  // Data structures to enforce synchronization during iterations
  std::vector<unsigned> mTeamIterReceived;
  std::vector<unsigned> mTeamIterRequired;

  // Reset the pose graph. This function overrides the function from the base
  // class.
  void reset() override;

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

  // Publish latest weights for the responsible inter-robot loop closures
  void publishMeasurementWeights();

  // Log iteration
  static bool createLogFile(const std::string &filename);

  bool logIteration(const std::string &filename) const;

  // ROS callbacks
  void anchorCallback(const PublicPosesConstPtr &msg);
  void statusCallback(const StatusConstPtr &msg);
  void commandCallback(const CommandConstPtr &msg);
  void publicPosesCallback(const PublicPosesConstPtr &msg);
  void measurementWeightsCallback(const RelativeMeasurementWeightsConstPtr &msg);
  bool queryLiftingMatrixCallback(QueryLiftingMatrixRequest &request, QueryLiftingMatrixResponse &response);

  // ROS publisher
  ros::Publisher mAnchorPublisher;
  ros::Publisher mStatusPublisher;
  ros::Publisher mCommandPublisher;
  ros::Publisher mPublicPosesPublisher;
  ros::Publisher mMeasurementWeightsPublisher;
  ros::Publisher mPoseArrayPublisher;  // Publish optimized trajectory
  ros::Publisher mPathPublisher;       // Publish optimized trajectory
  ros::Publisher mPoseGraphPublisher;  // Publish optimized pose graph

  // ROS subscriber
  ros::Subscriber mStatusSubscriber;
  ros::Subscriber mCommandSubscriber;
  ros::Subscriber mAnchorSubscriber;
  ros::Subscriber mPublicPosesSubscriber;
  ros::Subscriber mMeasurementWeightsSubscriber;

  // ROS service server
  ros::ServiceServer mQueryLiftingMatrixServer;
};

}  // namespace dpgo_ros

#endif