/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <DPGO/DPGO_utils.h>
#include <dpgo_ros/utils.h>
#include <pose_graph_tools/PoseGraph.h>
#include <pose_graph_tools/PoseGraphQuery.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <map>
#include <vector>

using pose_graph_tools::PoseGraph;
using std::map;
using std::string;
using std::vector;
using namespace DPGO;

class DatasetPublisher {
 public:
  DatasetPublisher(ros::NodeHandle nh_) : nh(nh_) {
    int num_robots = 0;
    if (!nh.getParam("/num_robots", num_robots)) {
      ROS_ERROR_STREAM("Failed to get number of robots!");
    }

    string filename;
    if (!nh.getParam("/dataset", filename)) {
      ROS_ERROR_STREAM("Failed to get dataset!");
    }

    size_t num_poses;
    vector<RelativeSEMeasurement> dataset = read_g2o_file(filename, num_poses);
    ROS_INFO_STREAM("Loaded dataset" << filename << " with " << num_poses
                                     << " total poses.");
    unsigned int n = num_poses;
    unsigned int num_poses_per_robot = n / num_robots;
    if (num_poses_per_robot <= 0) {
      ROS_ERROR_STREAM(
          "Number of robots must be smaller than total number of poses!");
    }

    ROS_INFO_STREAM(
        "Creating mapping from global pose index to local pose index...");
    map<unsigned, PoseID> PoseMap;
    for (unsigned robot = 0; robot < (unsigned)num_robots; ++robot) {
      unsigned startIdx = robot * num_poses_per_robot;
      unsigned endIdx = (robot + 1) * num_poses_per_robot;  // non-inclusive
      if (robot == (unsigned)num_robots - 1) endIdx = n;
      for (unsigned idx = startIdx; idx < endIdx; ++idx) {
        unsigned localIdx =
            idx - startIdx;  // this is the local ID of this pose
        PoseID pose = std::make_pair(robot, localIdx);
        PoseMap[idx] = pose;
      }
    }

    vector<vector<RelativeSEMeasurement>> odometry(num_robots);
    vector<vector<RelativeSEMeasurement>> private_loop_closures(num_robots);
    vector<vector<RelativeSEMeasurement>> shared_loop_closure(num_robots);
    for (size_t k = 0; k < dataset.size(); ++k) {
      RelativeSEMeasurement mIn = dataset[k];
      PoseID src = PoseMap[mIn.p1];
      PoseID dst = PoseMap[mIn.p2];

      unsigned srcRobot = src.first;
      unsigned srcIdx = src.second;
      unsigned dstRobot = dst.first;
      unsigned dstIdx = dst.second;

      RelativeSEMeasurement m(srcRobot, dstRobot, srcIdx, dstIdx, mIn.R, mIn.t,
                              mIn.kappa, mIn.tau);

      if (srcRobot == dstRobot) {
        // private measurement
        if (srcIdx + 1 == dstIdx) {
          // Odometry
          odometry[srcRobot].push_back(m);
        } else {
          // private loop closure
          private_loop_closures[srcRobot].push_back(m);
        }
      } else {
        // shared measurement
        shared_loop_closure[srcRobot].push_back(m);
        shared_loop_closure[dstRobot].push_back(m);
      }
    }

    for (size_t robot = 0; robot < (unsigned)num_robots; ++robot) {
      pose_graph_tools::PoseGraph pose_graph;
      // Add odometry factors
      for (size_t k = 0; k < odometry[robot].size(); ++k) {
        pose_graph_tools::PoseGraphEdge edge =
            dpgo_ros::RelativeMeasurementToMsg(odometry[robot][k]);
        pose_graph.edges.push_back(edge);
      }
      // Add private loop closures
      for (size_t k = 0; k < private_loop_closures[robot].size(); ++k) {
        pose_graph_tools::PoseGraphEdge edge =
            dpgo_ros::RelativeMeasurementToMsg(private_loop_closures[robot][k]);
        pose_graph.edges.push_back(edge);
      }
      // Add shared loop closures
      for (size_t k = 0; k < shared_loop_closure[robot].size(); ++k) {
        pose_graph_tools::PoseGraphEdge edge =
            dpgo_ros::RelativeMeasurementToMsg(shared_loop_closure[robot][k]);
        pose_graph.edges.push_back(edge);
      }
      poseGraphs.push_back(pose_graph);
    }

    for (size_t id = 0; id < (unsigned)num_robots; ++id) {
      string service_name = "/kimera" + std::to_string(id) + "/distributed_pcm/request_pose_graph";
      ros::ServiceServer server = nh.advertiseService(
          service_name, &DatasetPublisher::queryPoseGraphCallback, this);
      poseGraphServers.push_back(server);
    }
  }

  ~DatasetPublisher() {}

 private:
  ros::NodeHandle nh;
  vector<PoseGraph> poseGraphs;
  vector<ros::ServiceServer> poseGraphServers;
  bool queryPoseGraphCallback(
      pose_graph_tools::PoseGraphQueryRequest& request,
      pose_graph_tools::PoseGraphQueryResponse& response) {
    if (request.robot_id >= poseGraphs.size()) {
      ROS_ERROR("DatasetPublisher: requested robot does not exist!");
      return false;
    }
    response.pose_graph = poseGraphs[request.robot_id];
    return true;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dataset_publisher_node");
  ros::NodeHandle nh;
  DatasetPublisher dataset_publisher(nh);
  ros::spin();

  return 0;
}