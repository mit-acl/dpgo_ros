/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <DPGO/DPGO_types.h>
#include <DPGO/DPGO_utils.h>
#include <dpgo_ros/utils.h>
#include <tf/tf.h>

#include <map>

using namespace DPGO;
using pose_graph_tools::PoseGraphEdge;

namespace dpgo_ros {

std::vector<double> serializeMatrix(const size_t rows, const size_t cols,
                                    const Matrix& Mat) {
  assert((size_t)Mat.rows() == rows);
  assert((size_t)Mat.cols() == cols);

  std::vector<double> v;

  for (size_t row = 0; row < rows; ++row) {
    for (size_t col = 0; col < cols; ++col) {
      double scalar = Mat(row, col);
      v.push_back(scalar);
    }
  }

  return v;
}

Matrix deserializeMatrix(const size_t rows, const size_t cols,
                         const std::vector<double>& v) {
  assert(v.size() == rows * cols);
  Matrix Mat = Matrix::Zero(rows, cols);
  for (size_t row = 0; row < rows; ++row) {
    for (size_t col = 0; col < cols; ++col) {
      size_t index = row * cols + col;
      Mat(row, col) = v[index];
    }
  }

  return Mat;
}

MatrixMsg MatrixToMsg(const Matrix& Mat) {
  MatrixMsg msg;
  msg.rows = Mat.rows();
  msg.cols = Mat.cols();
  msg.values = serializeMatrix(msg.rows, msg.cols, Mat);
  return msg;
}

Matrix MatrixFromMsg(const MatrixMsg& msg) {
  return deserializeMatrix(msg.rows, msg.cols, msg.values);
}

LiftedPose constructLiftedPoseMsg(const size_t dimension,
                                  const size_t relaxation_rank,
                                  const size_t cluster_id,
                                  const size_t robot_id, const size_t pose_id,
                                  const Matrix pose) {
  assert(pose.rows() == (int)relaxation_rank);
  assert(pose.cols() == (int)dimension + 1);
  LiftedPose msg;
  msg.cluster_id = cluster_id;
  msg.robot_id = robot_id;
  msg.pose_id = pose_id;
  msg.pose = MatrixToMsg(pose);
  return msg;
}

size_t computeLiftedPosePayloadBytes(const LiftedPose& msg) {
  size_t bytes = 0;
  bytes += sizeof(msg.cluster_id);
  bytes += sizeof(msg.robot_id);
  bytes += sizeof(msg.pose_id);
  bytes += sizeof(msg.pose.rows);
  bytes += sizeof(msg.pose.cols);
  bytes += sizeof(msg.pose.values[0]) * msg.pose.values.size();
  return bytes;
}

PoseGraphEdge RelativeMeasurementToMsg(const RelativeSEMeasurement& m) {
  assert(m.R.rows() == 3 && m.R.cols() == 3);
  assert(m.t.rows() == 3 && m.t.cols() == 1);

  PoseGraphEdge msg;
  msg.robot_from = m.r1;
  msg.robot_to = m.r2;
  msg.key_from = m.p1;
  msg.key_to = m.p2;

  // convert rotation to ROS message
  tf::Matrix3x3 rotation(m.R(0, 0), m.R(0, 1), m.R(0, 2), m.R(1, 0), m.R(1, 1),
                         m.R(1, 2), m.R(2, 0), m.R(2, 1), m.R(2, 2));
  tf::Quaternion quat;
  rotation.getRotation(quat);
  tf::quaternionTFToMsg(quat, msg.pose.orientation);

  // convert translation to ROS message
  tf::Vector3 translation(m.t(0), m.t(1), m.t(2));
  tf::pointTFToMsg(translation, msg.pose.position);

  return msg;
}

RelativeSEMeasurement RelativeMeasurementFromMsg(const PoseGraphEdge& msg) {
  size_t r1 = msg.robot_from;
  size_t r2 = msg.robot_to;
  size_t p1 = msg.key_from;
  size_t p2 = msg.key_to;

  // read rotation
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  tf::Matrix3x3 rotation(quat);
  Matrix R(3, 3);
  R << rotation[0][0], rotation[0][1], rotation[0][2], rotation[1][0],
      rotation[1][1], rotation[1][2], rotation[2][0], rotation[2][1],
      rotation[2][2];

  // read translation
  tf::Vector3 translation;
  tf::pointMsgToTF(msg.pose.position, translation);
  Matrix t(3, 1);
  t << translation.x(), translation.y(), translation.z();

  // use hardcoded weight
  double kappa = 1500.0;
  double tau = 100.0;

  return RelativeSEMeasurement(r1, r2, p1, p2, R, t, kappa, tau);
}

geometry_msgs::PoseArray TrajectoryToPoseArray(const unsigned d,
                                               const unsigned n,
                                               const Matrix& T) {
  assert(d == 3);
  assert(T.rows() == d);
  assert(T.cols() == (d + 1) * n);
  geometry_msgs::PoseArray msg;
  msg.header.frame_id = "/world";
  msg.header.stamp = ros::Time::now();
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::Pose pose;
    Matrix Ri = T.block(0, i * (d + 1), d, d);
    Matrix ti = T.block(0, i * (d + 1) + d, d, 1);

    // convert rotation to ROS message
    tf::Matrix3x3 rotation(Ri(0, 0), Ri(0, 1), Ri(0, 2), Ri(1, 0), Ri(1, 1),
                           Ri(1, 2), Ri(2, 0), Ri(2, 1), Ri(2, 2));
    tf::Quaternion quat;
    rotation.getRotation(quat);
    tf::quaternionTFToMsg(quat, pose.orientation);

    // convert translation to ROS message
    tf::Vector3 translation(ti(0), ti(1), ti(2));
    tf::pointTFToMsg(translation, pose.position);

    msg.poses.push_back(pose);
  }
  return msg;
}

nav_msgs::Path TrajectoryToPath(const unsigned d, const unsigned n,
                                const Matrix& T) {
  assert(d == 3);
  assert(T.rows() == d);
  assert(T.cols() == (d + 1) * n);
  nav_msgs::Path msg;
  msg.header.frame_id = "/world";
  msg.header.stamp = ros::Time::now();
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::Pose pose;
    Matrix Ri = T.block(0, i * (d + 1), d, d);
    Matrix ti = T.block(0, i * (d + 1) + d, d, 1);

    // convert rotation to ROS message
    tf::Matrix3x3 rotation(Ri(0, 0), Ri(0, 1), Ri(0, 2), Ri(1, 0), Ri(1, 1),
                           Ri(1, 2), Ri(2, 0), Ri(2, 1), Ri(2, 2));
    tf::Quaternion quat;
    rotation.getRotation(quat);
    tf::quaternionTFToMsg(quat, pose.orientation);

    // convert translation to ROS message
    tf::Vector3 translation(ti(0), ti(1), ti(2));
    tf::pointTFToMsg(translation, pose.position);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = "/world";
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose = pose;

    msg.poses.push_back(poseStamped);
  }
  return msg;
}

bool savePoseArrayToFile(const geometry_msgs::PoseArray& msg,
                         const std::string& filename) {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening log file: " << filename);
    return false;
  }

  file << "pose_index,qx,qy,qz,qw,tx,ty,tz\n";
  for (size_t i = 0; i < msg.poses.size(); ++i) {
    geometry_msgs::Point position = msg.poses[i].position;
    geometry_msgs::Quaternion orientation = msg.poses[i].orientation;
    file << i << ",";
    file << orientation.x << ",";
    file << orientation.y << ",";
    file << orientation.z << ",";
    file << orientation.w << ",";
    file << position.x << ",";
    file << position.y << ",";
    file << position.z << "\n";
  }

  file.close();

  return true;
}

bool saveRelativeMeasurementsToFile(
    const std::vector<RelativeSEMeasurement>& measurements,
    const std::string filename) {
  std::ofstream file;
  file.open(filename);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Error opening file: " << filename);
    return false;
  }

  // Header line
  file << "robot_src,pose_src,robot_dst,pose_dst,qx,qy,qz,qw,tx,ty,tz,kappa,"
          "tau\n";

  for (size_t i = 0; i < measurements.size(); ++ i) {
    RelativeSEMeasurement m = measurements[i];
    tf::Matrix3x3 rotation(m.R(0, 0), m.R(0, 1), m.R(0, 2), 
                           m.R(1, 0), m.R(1, 1), m.R(1, 2), 
                           m.R(2, 0), m.R(2, 1), m.R(2, 2));
    tf::Quaternion quat;
    rotation.getRotation(quat);

    file << m.r1 << ",";
    file << m.p1 << ",";
    file << m.r2 << ",";
    file << m.p2 << ",";
    file << quat.x() << ",";
    file << quat.y() << ",";
    file << quat.z() << ",";
    file << quat.w() << ",";
    file << m.t(0) << ",";
    file << m.t(1) << ",";
    file << m.t(2) << ",";
    file << m.kappa << ",";
    file << m.tau << "\n";
  }

  file.close();

  return true;
}

}  // namespace dpgo_ros