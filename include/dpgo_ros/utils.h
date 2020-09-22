/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#pragma once

#include <DPGO/DPGO_types.h>
#include <DPGO/RelativeSEMeasurement.h>
#include <dpgo_ros/LiftedPose.h>
#include <dpgo_ros/LiftedPoseArray.h>
#include <dpgo_ros/MatrixMsg.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <tf/tf.h>

#include <cassert>
#include <vector>

using namespace DPGO;
using pose_graph_tools::PoseGraphEdge;

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
Write a matrix to ROS message
*/
MatrixMsg MatrixToMsg(const Matrix& Mat);

/**
Read a matrix from ROS message
*/
Matrix MatrixFromMsg(const MatrixMsg& msg);

/**
Construct a lifted pose message from inputs
*/
LiftedPose constructLiftedPoseMsg(const size_t dimension,
                                  const size_t relaxation_rank,
                                  const size_t cluster_id,
                                  const size_t robot_id, const size_t pose_id,
                                  const Matrix pose);

/**
Write a relative measurement to ROS message
*/
PoseGraphEdge RelativeMeasurementToMsg(const RelativeSEMeasurement& m);

/**
Read a relative measurement from ROS message
*/
RelativeSEMeasurement RelativeMeasurementFromMsg(const PoseGraphEdge& msg);

}  // namespace dpgo_ros
