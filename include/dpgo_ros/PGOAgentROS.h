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
#include <pose_graph_tools/pose_graph.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt16MultiArray.h>
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

  // If true dpgo will publish loop closure as ROS markers
  bool visualizeLoopClosures;

  // Completely reset dpgo after each distributed optimization round
  bool completeReset;

  // Synchronize shared measurements between robots before each optimization round
  bool synchronizeMeasurements;

  // Let dpgo try to recover if some robots disconnect during distributed optimization
  bool enableRecovery;

  // Maximum attempts for multi-robot initialization
  int maxDistributedInitSteps;

  // Maximum allowed delay from other robots (specified as number of iterations)
  int maxDelayedIterations;

  // Threshold used when determining if loop closure weight converged
  double weightConvergenceThreshold;

  // Sleep time before telling next robot to update during optimization
  double interUpdateSleepTime;

  // Maximum time in seconds before considering a robot disconnected
  double timeoutThreshold;

  // Default constructor
  PGOAgentROSParameters(unsigned dIn, unsigned rIn, unsigned numRobotsIn)
      : PGOAgentParameters(dIn, rIn, numRobotsIn),
        updateRule(UpdateRule::Uniform),
        publishIterate(false),
        visualizeLoopClosures(false),
        completeReset(false),
        synchronizeMeasurements(true),
        enableRecovery(true),
        maxDistributedInitSteps(30),
        maxDelayedIterations(3),
        weightConvergenceThreshold(1e-6),
        interUpdateSleepTime(0),
        timeoutThreshold(15) {}

  inline friend std::ostream &operator<<(
      std::ostream &os, const PGOAgentROSParameters &params) {
    // First print base class
    os << (const PGOAgentParameters &) params;
    // Then print additional options defined in the derived class
    os << "PGOAgentROS parameters: " << std::endl;
    os << "Update rule: " << updateRuleToString(params.updateRule) << std::endl; 
    os << "Publish iterate: " << params.publishIterate << std::endl;
    os << "Visualize loop closures: " << params.visualizeLoopClosures << std::endl;
    os << "Complete reset: " << params.completeReset << std::endl;
    os << "Enable recovery: " << params.enableRecovery << std::endl;
    os << "Synchronize measurements: " << params.synchronizeMeasurements << std::endl;
    os << "Maximum distributed initialization attempts: " << params.maxDistributedInitSteps << std::endl;
    os << "Maximum delayed iterations: " << params.maxDelayedIterations << std::endl;
    os << "Measurement weight convergence threshold: " << params.weightConvergenceThreshold << std::endl;
    os << "Inter update sleep time: " << params.interUpdateSleepTime << std::endl;
    os << "Timeout threshold: " << params.timeoutThreshold << std::endl;
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

  // ID of the cluster that this robot belongs to
  unsigned mClusterID;

  // Received request to iterate with optimization in synchronous mode
  bool mSynchronousOptimizationRequested = false;

  // Flag to publish INITIALIZE command
  bool mPublishInitializeCommandRequested = false;

  // Flag to attempt initialization
  bool mTryInitializeRequested = false;

  // Handle to log file
  std::ofstream mIterationLog;

  // Number of initialization steps performed
  int mInitStepsDone;

  // Total bytes of public poses received
  size_t mTotalBytesReceived;

  // Elapsed time for the latest update
  double mIterationElapsedMs;

  // Global optimization start time
  ros::Time mGlobalStartTime, mLastCommandTime;

  // Map from robot ID to name
  std::map<unsigned, std::string> mRobotNames;

  // Store latest status message from other robots
  std::map<unsigned, dpgo_ros::Status> mTeamStatusMsg;

  // Data structures to enforce synchronization during iterations
  std::vector<unsigned> mTeamIterReceived;
  std::vector<unsigned> mTeamIterRequired;
  std::vector<bool> mTeamReceivedSharedLoopClosures;
  
  // Store if other robots are currently connected 
  std::vector<bool> mTeamConnected;  

  // Store the current cluster each robot belongs to
  std::vector<unsigned> mTeamClusterID;

  // Store the latest optimized trajectory and loop closures for visualization
  std::optional<PoseArray> mCachedPoses;
  std::optional<visualization_msgs::Marker> mCachedLoopClosureMarkers;

  // Store the latest SE(d) poses from neighbors in the global frame 
  std::map<PoseID, Pose, ComparePoseID> mCachedNeighborPoses; 

  // Store the latest measurement weights with neighbors
  std::unordered_map<EdgeID, double, HashEdgeID> mCachedEdgeWeights;

  // Last time reset is called
  ros::Time mLastResetTime;

  // Time this node is launched
  ros::Time mLaunchTime;

  // Time this node last performed an iteration
  std::optional<ros::Time> mLastUpdateTime;

  // Reset the pose graph. This function overrides the function from the base class.
  void reset() override;

  // Tasks to run in synchronous mode at every ROS spin
  void runOnceSynchronous();

  // Tasks to run in asynchronous mode at every ROS spin
  void runOnceAsynchronous();

  // Request latest local pose graph
  bool requestPoseGraph();

  // Attempt to initialize optimization
  bool tryInitialize();

  // Get the ID of the current cluster
  unsigned getClusterID() const;

  // Return true if this robot is currently serving as the leader of the cluster
  bool isLeader() const;

  // Update cluster for this robot
  void updateCluster();

  // Get the cluster a robot belongs to
  unsigned getRobotClusterID(unsigned robot_id) const;

  // Set the cluster a robot belongs to
  void setRobotClusterID(unsigned robot_id, unsigned cluster_id);
  void resetRobotClusterIDs();

  // Return true if the robot is connected
  bool isRobotConnected(unsigned robot_id) const;

  // Update the set of active robots based on connectivity
  void setActiveRobots();

  // Update the set of active robots based on input vector
  void updateActiveRobots(const CommandConstPtr &msg);

  // Publish status
  void publishStatus();

  // Publish command to request pose graph
  void publishRequestPoseGraphCommand();

  // Publish initialize command
  void publishInitializeCommand();

  // Publish update command
  void publishUpdateCommand();
  // Publish update command and specify next robot to update
  void publishUpdateCommand(unsigned robot_id);

  // Publish recover command
  void publishRecoverCommand();

  // Publish termination command
  void publishTerminateCommand();

  // Publish hard termination command
  void publishHardTerminateCommand();

  // Publish weight update command
  void publishUpdateWeightCommand();

  // Publish the list of active robots
  void publishActiveRobotsCommand();

  // Publish No op command (for debugging)
  void publishNoopCommand();

  // Publish lifting matrix
  void publishLiftingMatrix();

  // Publish anchor
  void publishAnchor();

  // Check timeout
  void checkTimeout();

  // Check disconnected robot
  bool checkDisconnectedRobot();

  // Publish trajectory
  void storeOptimizedTrajectory();
  void publishTrajectory(const PoseArray &T);
  void publishOptimizedTrajectory();

  // Publish trajectory estimates from the latest iteration in distributed optimization.
  // This function is mostly for visualization and debugging purpose.
  void publishIterate();

  // Publish latest public poses
  void publishPublicPoses(bool aux = false);

  // Publish shared loop closures between this robot and others
  void publishPublicMeasurements();

  // Publish weights for the responsible inter-robot loop closures
  void publishMeasurementWeights();

  // Publish loop closures for visualization
  void storeLoopClosureMarkers();
  void publishLoopClosureMarkers();

  // Store neighbor SE(d) poses in the global frame
  void storeActiveNeighborPoses();
  void setInactiveNeighborPoses();

  // Store edge weights
  void storeActiveEdgeWeights();
  void setInactiveEdgeWeights();

  // Initialize global anchor using stored information
  void initializeGlobalAnchor();

  // Log iteration
  bool createIterationLog(const std::string &filename);
  bool logIteration();
  bool logString(const std::string &str);

  // ROS callbacks
  void connectivityCallback(const std_msgs::UInt16MultiArrayConstPtr &msg);
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
  ros::Subscriber mConnectivitySubscriber;

  // ROS timer
  ros::Timer timer;
  ros::Timer mVisualizationTimer;
};

}  // namespace dpgo_ros

#endif
