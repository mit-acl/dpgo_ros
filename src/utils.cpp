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

/**
Serialize a Matrix object into a vector of Float64 messages, in row-major format
*/
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

/**
Deserialize a vector of Float64 messages into a Matrix object, using row-major
format
*/
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

/**
 * Write a matrix to ROS message
 */
MatrixMsg MatrixToMsg(const Matrix& Mat) {
  MatrixMsg msg;
  msg.rows = Mat.rows();
  msg.cols = Mat.cols();
  msg.values = serializeMatrix(msg.rows, msg.cols, Mat);
  return msg;
}

/**
 * Read a matrix from ROS message
 */
Matrix MatrixFromMsg(const MatrixMsg& msg) {
  return deserializeMatrix(msg.rows, msg.cols, msg.values);
}

}  // namespace dpgo_ros