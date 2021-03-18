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

std::vector<double> serializeMatrix(size_t rows, size_t cols,
                                    const Matrix &Mat) {
  assert((size_t) Mat.rows() == rows);
  assert((size_t) Mat.cols() == cols);

  std::vector<double> v;

  for (size_t row = 0; row < rows; ++row) {
    for (size_t col = 0; col < cols; ++col) {
      double scalar = Mat(row, col);
      v.push_back(scalar);
    }
  }

  return v;
}

Matrix deserializeMatrix(size_t rows, size_t cols,
                         const std::vector<double> &v) {
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

MatrixMsg MatrixToMsg(const Matrix &Mat) {
  MatrixMsg msg;
  msg.rows = Mat.rows();
  msg.cols = Mat.cols();
  msg.values = serializeMatrix(msg.rows, msg.cols, Mat);
  return msg;
}

Matrix MatrixFromMsg(const MatrixMsg &msg) {
  return deserializeMatrix(msg.rows, msg.cols, msg.values);
}

Matrix RotationFromPoseMsg(const geometry_msgs::Pose &msg) {
  // read rotation
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.orientation, quat);
  tf::Matrix3x3 rotation(quat);
  Matrix R(3, 3);
  R << rotation[0][0], rotation[0][1], rotation[0][2],
       rotation[1][0], rotation[1][1], rotation[1][2],
       rotation[2][0], rotation[2][1], rotation[2][2];

  return R;
}

Matrix TranslationFromPoseMsg(const geometry_msgs::Pose &msg) {
  tf::Vector3 translation;
  tf::pointMsgToTF(msg.position, translation);
  Matrix t(3, 1);
  t << translation.x(), translation.y(), translation.z();
  return t;
}

geometry_msgs::Quaternion RotationToQuaternionMsg(const Matrix &R) {
  assert(R.rows() == 3);
  assert(R.cols() == 3);

  tf::Matrix3x3 rotation(R(0, 0), R(0, 1), R(0, 2),
                         R(1, 0), R(1, 1), R(1, 2),
                         R(2, 0), R(2, 1), R(2, 2));
  tf::Quaternion quat;
  rotation.getRotation(quat);
  geometry_msgs::Quaternion quat_msg;
  tf::quaternionTFToMsg(quat, quat_msg);
  return quat_msg;
}

geometry_msgs::Point TranslationToPointMsg(const Matrix &t) {
  assert(t.rows() == 3);
  assert(t.cols() == 1);
  // convert translation to ROS message
  tf::Vector3 translation(t(0), t(1), t(2));
  geometry_msgs::Point point_msg;
  tf::pointTFToMsg(translation, point_msg);
  return point_msg;
}

PoseGraphEdge RelativeMeasurementToMsg(const RelativeSEMeasurement &m) {
  assert(m.R.rows() == 3 && m.R.cols() == 3);
  assert(m.t.rows() == 3 && m.t.cols() == 1);

  PoseGraphEdge msg;
  msg.robot_from = m.r1;
  msg.robot_to = m.r2;
  msg.key_from = m.p1;
  msg.key_to = m.p2;

  // convert rotation to ROS message
  msg.pose.orientation = RotationToQuaternionMsg(m.R);

  // convert translation to ROS message
  msg.pose.position = TranslationToPointMsg(m.t);

  // TODO: write covariance to message
  return msg;
}

RelativeSEMeasurement RelativeMeasurementFromMsg(const PoseGraphEdge &msg) {
  size_t r1 = msg.robot_from;
  size_t r2 = msg.robot_to;
  size_t p1 = msg.key_from;
  size_t p2 = msg.key_to;

  // read rotation
  Matrix R = RotationFromPoseMsg(msg.pose);

  // read translation
  Matrix t = TranslationFromPoseMsg(msg.pose);

  // TODO: read covariance from message
  double kappa = 10000;
  double tau = 100;

  return RelativeSEMeasurement(r1, r2, p1, p2, R, t, kappa, tau);
}

geometry_msgs::PoseArray TrajectoryToPoseArray(unsigned d, unsigned n, const Matrix &T) {
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
    pose.orientation = RotationToQuaternionMsg(Ri);

    // convert translation to ROS message
    pose.position = TranslationToPointMsg(ti);

    msg.poses.push_back(pose);
  }
  return msg;
}

nav_msgs::Path TrajectoryToPath(unsigned d, unsigned n, const Matrix &T) {
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
    pose.orientation = RotationToQuaternionMsg(Ri);

    // convert translation to ROS message
    pose.position = TranslationToPointMsg(ti);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = "/world";
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose = pose;

    msg.poses.push_back(poseStamped);
  }
  return msg;
}

sensor_msgs::PointCloud TrajectoryToPointCloud(unsigned d, unsigned n, const Matrix &T) {
  assert(d == 3);
  assert(T.rows() == d);
  assert(T.cols() == (d + 1) * n);
  sensor_msgs::PointCloud msg;
  msg.header.frame_id = "/world";
  msg.header.stamp = ros::Time::now();
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::Point32 point;
    Matrix ti = T.block(0, i * (d + 1) + d, d, 1);
    point.x = ti(0);
    point.y = ti(1);
    point.z = ti(2);
    msg.points.push_back(point);
  }
  return msg;
}

pose_graph_tools::PoseGraph TrajectoryToPoseGraphMsg(unsigned robotID, unsigned d, unsigned n, const Matrix &T) {
  assert(d == 3);
  assert(T.rows() == d);
  assert(T.cols() == (d + 1) * n);
  pose_graph_tools::PoseGraph pose_graph_msg;
  pose_graph_msg.header.frame_id = "/world";
  pose_graph_msg.header.stamp = ros::Time::now();
  for (size_t i = 0; i < n; ++i) {
    pose_graph_tools::PoseGraphNode node_msg;
    node_msg.robot_id = robotID;
    node_msg.key = i;
    node_msg.header.frame_id = "/world";
    node_msg.header.stamp = pose_graph_msg.header.stamp;

    Matrix Ri = T.block(0, i * (d + 1), d, d);
    Matrix ti = T.block(0, i * (d + 1) + d, d, 1);

    // convert rotation to ROS message
    node_msg.pose.orientation = RotationToQuaternionMsg(Ri);

    // convert translation to ROS message
    node_msg.pose.position = TranslationToPointMsg(ti);

    pose_graph_msg.nodes.push_back(node_msg);
  }
  return pose_graph_msg;
}

size_t computePublicPosesMsgSize(const PublicPoses &msg) {
  size_t bytes = 0;
  bytes += sizeof(msg.robot_id);
  bytes += sizeof(msg.cluster_id);
  bytes += sizeof(msg.instance_number);
  bytes += sizeof(msg.iteration_number);
  bytes += sizeof(msg.is_auxiliary);
  bytes += sizeof(msg.pose_ids[0]) * msg.pose_ids.size();
  bytes += sizeof(msg.poses[0]) * msg.poses.size();
  return bytes;
}

Status statusToMsg(const PGOAgentStatus &status) {
  Status msg;
  msg.robot_id = status.agentID;
  msg.state = status.state;
  msg.instance_number = status.instanceNumber;
  msg.iteration_number = status.iterationNumber;
  msg.ready_to_terminate = status.readyToTerminate;
  msg.relative_change = status.relativeChange;
  return msg;
}

PGOAgentStatus statusFromMsg(const Status &msg) {
  PGOAgentStatus status(msg.robot_id,
                        static_cast<PGOAgentState>(msg.state),
                        msg.instance_number,
                        msg.iteration_number,
                        msg.ready_to_terminate,
                        msg.relative_change);
  return status;
}

}  // namespace dpgo_ros