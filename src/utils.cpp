/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include <DPGO/DPGO_types.h>
#include <DPGO/DPGO_utils.h>
#include <DPGO/RelativeSEMeasurement.h>
#include <dpgo_ros/utils.h>
#include <tf/tf.h>

#include <map>

using namespace DPGO;
// using dpgo_ros::LiftedPoseArray;

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
                                  const size_t robot_id, 
                                  const size_t pose_id,
                                  const Matrix pose) 
{
  assert(pose.rows() == relaxation_rank);
  assert(pose.cols() == dimension);
  LiftedPose msg;
  msg.cluster_id = cluster_id;
  msg.robot_id = robot_id;
  msg.pose_id = pose_id;
  msg.pose = MatrixToMsg(pose);
  return msg;
}

}  // namespace dpgo_ros