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
#include <dpgo_ros/RelativeMeasurementList.h>
#include <dpgo_ros/RelativeMeasurementWeights.h>
#include <dpgo_ros/QueryLiftingMatrix.h>
#include <dpgo_ros/Status.h>
#include <pose_graph_tools/PoseGraph.h>
#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <ros/ros.h>

using namespace DPGO;

namespace dpgo_ros {

typedef std::vector<ros::Subscriber> SubscriberVector;

/**
 * @brief This class extends PGOAgentParameters with several ROS related settings
 */
class PGOAgentROSParameters : public PGOAgentParameters {
 public:
  enum class UpdateRule {
    Uniform, // Uniform sampling 
    RoundRobin  // Round robin
  };

  // Rule to select the next robot for update
  UpdateRule updateRule;

  // Publish intermediate iterates during optimization
  bool publishIterate;

  // Maximum attempts for multi-robot initialization
  int maxDistributedInitSteps;

  // Maximum allowed delay from other robots (specified as number of iterations)
  int maxDelayedIterations;

  // Sleep time before telling next robot to update during optimization
  double interUpdateSleepTime;

  // Default constructor
  PGOAgentROSParameters(unsigned dIn, unsigned rIn, unsigned numRobotsIn)
      : PGOAgentParameters(dIn, rIn, numRobotsIn),
        updateRule(UpdateRule::Uniform),
        publishIterate(false),
        maxDistributedInitSteps(30),
        maxDelayedIterations(3),
        interUpdateSleepTime(0) {}

  inline friend std::ostream &operator<<(
      std::ostream &os, const PGOAgentROSParameters &params) {
    // First print base class
    os << (const PGOAgentParameters &) params;
    // Then print additional options defined in the derived class
    os << "PGOAgentROS parameters: " << std::endl;
    os << "Update rule: " << updateRuleToString(params.updateRule) << std::endl; 
    os << "Publish iterate: " << params.publishIterate << std::endl;
    os << "Maximum distributed initialization attempts: " << params.maxDistributedInitSteps << std::endl;
    os << "Maximum delayed iterations: " << params.maxDelayedIterations << std::endl;
    os << "Inter update sleep time: " << params.interUpdateSleepTime << std::endl;
    return os;
  }

  inline static std::string updateRuleToString(UpdateRule rule) {
    switch (rule) {
      case UpdateRule::Uniform: {
        return "Uniform";
      }
      case UpdateRule::RoundRobin: {
        return "RoundRobin";
      }
    }
    return "";
  }
};

class PGOAgentROS : public PGOAgent {
 public:
  PGOAgentROS(const ros::NodeHandle &nh_, unsigned ID,
              const PGOAgentROSParameters &params);

  ~PGOAgentROS() = default;

  /**
   * @brief Function to be called at every ROS spin.
   */
  void runOnce();

 private:
  // ROS node handle
  ros::NodeHandle nh;

  // A copy of the parameter struct
  const PGOAgentROSParameters mParamsROS;

  // Received request to iterate with optimization in synchronous mode
  bool mSynchronousOptimizationRequested = false;

  // Flag to publish INITIALIZE command
  bool mPublishInitializeCommandRequested = false;

  // Handle to log file
  std::ofstream mIterationLog;

  // Number of initialization steps performed
  int mInitStepsDone;

  // Total bytes of public poses received
  size_t mTotalBytesReceived;

  // Elapsed time for the latest update
  double mIterationElapsedMs;

  // Global optimization start time
  std::chrono::time_point<std::chrono::high_resolution_clock> mGlobalStartTime, mLastCommandTime;

  // Map from robot ID to name
  std::map<unsigned, std::string> mRobotNames;

  // Store latest status message from other robots
  std::map<unsigned, dpgo_ros::Status> mTeamStatusMsg;

  // Data structures to enforce synchronization during iterations
  std::vector<unsigned> mTeamIterReceived;
  std::vector<unsigned> mTeamIterRequired;
  std::vector<bool> mTeamReceivedSharedLoopClosures;

  // Store the latest optimized trajectory and loop closures for visualization
  std::optional<PoseArray> mCachedPoses;
  std::optional<visualization_msgs::Marker> mCachedLoopClosureMarkers;

  // Reset the pose graph. This function overrides the function from the base class.
  void reset() override;

  // Tasks to run in synchronous mode at every ROS spin
  void runOnceSynchronous();

  // Tasks to run in asynchronous mode at every ROS spin
  void runOnceAsynchronous();

  // Request latest local pose graph
  bool initializePoseGraph();

  // Attempt to initialize optimization
  bool tryInitializeOptimization();

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

  // Publish hard termination command
  void publishHardTerminateCommand();

  // Publish No op command (for debugging)
  void publishNoopCommand();

  // Publish lifting matrix
  void publishLiftingMatrix();

  // Publish anchor
  void publishAnchor();

  // Publish trajectory
  void storeOptimizedTrajectory();
  void publishStoredTrajectory();
  void publishLatestTrajectory();

  // Publish latest public poses
  void publishPublicPoses(bool aux = false);

  // Publish shared loop closures between this robot and others
  void publishPublicMeasurements();

  // Publish weights for the responsible inter-robot loop closures
  void publishMeasurementWeights();

  // Publish loop closures for visualization
  void storeLoopClosureMarkers();
  void publishStoredLoopClosureMarkers();
  void publishLatestLoopClosureMarkers();

  // Log iteration
  bool createIterationLog(const std::string &filename);
  bool logIteration();

  // Sleep for a randomly generated between (0, sec)
  void randomSleep(double sec);

  // ROS callbacks
  void liftingMatrixCallback(const MatrixMsgConstPtr &msg);
  void anchorCallback(const PublicPosesConstPtr &msg);
  void statusCallback(const StatusConstPtr &msg);
  void commandCallback(const CommandConstPtr &msg);
  void publicPosesCallback(const PublicPosesConstPtr &msg);
  void publicMeasurementsCallback(const RelativeMeasurementListConstPtr &msg);
  void measurementWeightsCallback(const RelativeMeasurementWeightsConstPtr &msg);
  void timerCallback(const ros::TimerEvent &event);
  void visualizationTimerCallback(const ros::TimerEvent &event);

  // ROS publisher
  ros::Publisher mLiftingMatrixPublisher;
  ros::Publisher mAnchorPublisher;
  ros::Publisher mStatusPublisher;
  ros::Publisher mCommandPublisher;
  ros::Publisher mPublicPosesPublisher;
  ros::Publisher mPublicMeasurementsPublisher;
  ros::Publisher mMeasurementWeightsPublisher;
  ros::Publisher mPoseArrayPublisher;    // Publish optimized trajectory
  ros::Publisher mPathPublisher;         // Publish optimized trajectory
  ros::Publisher mPoseGraphPublisher;    // Publish optimized pose graph
  ros::Publisher mLoopClosureMarkerPublisher;  // Publish loop closures for visualization

  // ROS subscriber
  SubscriberVector mLiftingMatrixSubscriber;
  SubscriberVector mStatusSubscriber;
  SubscriberVector mCommandSubscriber;
  SubscriberVector mAnchorSubscriber;
  SubscriberVector mPublicPosesSubscriber;
  SubscriberVector mSharedLoopClosureSubscriber;
  SubscriberVector mMeasurementWeightsSubscriber;

  // ROS timer
  ros::Timer timer;
  ros::Timer mVisualizationTimer;
};

}  // namespace dpgo_ros

#endif