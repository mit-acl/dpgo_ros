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
Compute the total payload size contained in a lifted pose message
*/
size_t computeLiftedPosePayloadBytes(const LiftedPose& msg);

/**
Write a relative measurement to ROS message
*/
PoseGraphEdge RelativeMeasurementToMsg(const RelativeSEMeasurement& m);

/**
Read a relative measurement from ROS message
*/
RelativeSEMeasurement RelativeMeasurementFromMsg(const PoseGraphEdge& msg);

/**
Convert an aggregate matrix T \in (SO(d) \times Rd)^n to a ROS PoseArray message
*/
geometry_msgs::PoseArray TrajectoryToPoseArray(const unsigned d,
                                               const unsigned n,
                                               const Matrix& T);

/**
Convert an aggregate matrix T \in (SO(d) \times Rd)^n to a ROS Path message
*/
nav_msgs::Path TrajectoryToPath(const unsigned d, const unsigned n,
                                const Matrix& T);

/**
Save a ROS pose array message to CSV file
*/
bool savePoseArrayToFile(const geometry_msgs::PoseArray& msg,
                         const std::string& filename);

/**
Save relative measurements to file
*/
bool saveRelativeMeasurementsToFile(
    const std::vector<RelativeSEMeasurement>& measurements,
    const std::string filename);

}  // namespace dpgo_ros
