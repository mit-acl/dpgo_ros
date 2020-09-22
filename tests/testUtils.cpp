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

TEST(UtilsTest, LiftedPose) {
  DPGO::Matrix Mat(3, 3);
  Mat << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;

  size_t cluster_id = 0;
  size_t robot_id = 1;
  size_t pose_id = 2;

  LiftedPose msg =
      constructLiftedPoseMsg(3, 3, cluster_id, robot_id, pose_id, Mat);
  ASSERT_EQ(msg.cluster_id, cluster_id);
  ASSERT_EQ(msg.robot_id, robot_id);
  ASSERT_EQ(msg.pose_id, pose_id);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dpgo_ros_test_utils");
  return RUN_ALL_TESTS();
}