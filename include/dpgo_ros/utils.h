/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <DPGO/DPGO_types.h>
#include <dpgo_ros/MatrixMsg.h>
#include <dpgo_ros/LiftedPose.h>
#include <dpgo_ros/LiftedPoseArray.h>
#include <tf/tf.h>

#include <cassert>
#include <vector>

using namespace DPGO;

namespace dpgo_ros {

/**
Serialize a Matrix object into a vector of Float64 messages, in row-major format
*/
std::vector<double> serializeMatrix(const size_t rows, const size_t cols,
                                    const Matrix& Mat);

/**
Deserialize a vector of Float64 messages into a Matrix object, using row-major
format
*/
Matrix deserializeMatrix(const size_t rows, const size_t cols,
                         const std::vector<double>& v);

/**
 * Write a matrix to ROS message
 */
MatrixMsg MatrixToMsg(const Matrix& Mat);

/**
 * Read a matrix from ROS message
 */
Matrix MatrixFromMsg(const MatrixMsg& msg);

/**
Helper function to construct a lifted pose message from inputs
*/
LiftedPose constructLiftedPoseMsg(const size_t dimension,
                                  const size_t relaxation_rank,
                                  const size_t cluster_id,
                                  const size_t robot_id, 
                                  const size_t pose_id,
                                  const Matrix pose);

// /**
// Helper function to construct a relative measurement message from inputs
// */
// RelativeMeasurementStamped constructRelativeMeasurementMsg(
//     const size_t robot1, const size_t pose1, const size_t robot2,
//     const size_t pose2, const tf::Quaternion rotation,
//     const tf::Vector3 translation, const double kappa, const double tau);

}  // namespace dpgo_ros
