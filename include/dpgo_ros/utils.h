/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <DPGO/DPGO_types.h>
#include <DPGO/PGOAgent.h>
#include <DPGO/RelativeSEMeasurement.h>
#include <dpgo_ros/MatrixMsg.h>
#include <dpgo_ros/PublicPoses.h>
#include <dpgo_ros/Status.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <tf/tf.h>

#include <cassert>
#include <fstream>
#include <vector>

using namespace DPGO;
using pose_graph_tools::PoseGraphEdge;

namespace dpgo_ros {

/**
Serialize a Matrix object into a vector of Float64 messages, in row-major format
*/
std::vector<double> serializeMatrix(size_t rows, size_t cols,
                                    const Matrix &Mat);

/**
Deserialize a vector of Float64 messages into a Matrix object, using row-major
format
*/
Matrix deserializeMatrix(size_t rows, size_t cols,
                         const std::vector<double> &v);

/**
Write a matrix to ROS message
*/
MatrixMsg MatrixToMsg(const Matrix &Mat);

/**
Read a matrix from ROS message
*/
Matrix MatrixFromMsg(const MatrixMsg &msg);

/**
 * @brief Retrieve 3-by-3 rotation matrix from geometry_msgs::Pose
 * @param msg
 * @return
 */
Matrix RotationFromPoseMsg(const geometry_msgs::Pose &msg);

/**
 * @brief Retrieve 3-by-1 translation vector from geometry_msgs::Pose
 * @param msg
 * @return
 */
Matrix TranslationFromPoseMsg(const geometry_msgs::Pose &msg);

/**
Write a relative measurement to ROS message
*/
PoseGraphEdge RelativeMeasurementToMsg(const RelativeSEMeasurement &m);

/**
Read a relative measurement from ROS message
*/
RelativeSEMeasurement RelativeMeasurementFromMsg(const PoseGraphEdge &msg);

/**
Convert an aggregate matrix T \in (SO(d) \times Rd)^n to a ROS PoseArray message
*/
geometry_msgs::PoseArray TrajectoryToPoseArray(unsigned d, unsigned n, const Matrix &T);

/**
Convert an aggregate matrix T \in (SO(d) \times Rd)^n to a ROS Path message
*/
nav_msgs::Path TrajectoryToPath(unsigned d, unsigned n, const Matrix &T);

/**
Compute the number of bytes of a PublicPoses message.
*/
size_t computePublicPosesMsgSize(const PublicPoses &msg);

/**
 * @brief Convert a PGOAgentStatus struct to its corresponding ROS message
 * @param status
 * @return
 */
Status statusToMsg(const PGOAgentStatus &status);

/**
 * @brief Create a PGOAgentStatus struct from its corresponding ROS message
 * @param msg
 * @return
 */
PGOAgentStatus statusFromMsg(const Status &msg);

}  // namespace dpgo_ros
