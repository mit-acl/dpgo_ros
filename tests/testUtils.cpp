/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#include <dpgo_ros/utils.h>
#include <ros/ros.h>

#include "gtest/gtest.h"

using namespace dpgo_ros;

TEST(UtilsTest, MatrixMsg) {
  DPGO::Matrix Mat(3, 3);
  Mat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  MatrixMsg msg = MatrixToMsg(Mat);
  ASSERT_EQ(msg.rows, 3);
  ASSERT_EQ(msg.cols, 3);

  DPGO::Matrix MatOut = MatrixFromMsg(msg);
  ASSERT_LE((MatOut - Mat).norm(), 1e-6);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dpgo_ros_test_utils");
  return RUN_ALL_TESTS();
}