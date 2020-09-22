/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */
#include <DPGO/RelativeSEMeasurement.h>
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

TEST(UtilsTest, PoseGraphEdge) {
  size_t r1 = 0;
  size_t r2 = 1;
  size_t p1 = 2;
  size_t p2 = 3;
  double kappa = 1.0;
  double tau = 1.0;
  DPGO::Matrix R(3,3);
  R << -0.2727695, -0.4248134,  0.8632094,
       -0.5148591,  0.8223981,  0.2420361,
       -0.8127219, -0.3784111, -0.4430441;

  DPGO::Matrix t(3,1);
  t << -1.5, 2.1, 3.9;

  DPGO::RelativeSEMeasurement m(r1, r2, p1, p2, R, t, kappa, tau);
  pose_graph_tools::PoseGraphEdge msg = RelativeMeasurementToMsg(m);
  DPGO::RelativeSEMeasurement mOut = RelativeMeasurementFromMsg(msg);

  ASSERT_EQ(mOut.r1, r1);
  ASSERT_EQ(mOut.r2, r2);
  ASSERT_EQ(mOut.p1, p1);
  ASSERT_EQ(mOut.p2, p2);
  ASSERT_LE((R-mOut.R).norm(), 1e-6);
  ASSERT_LE((t-mOut.t).norm(), 1e-6);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "dpgo_ros_test_utils");
  return RUN_ALL_TESTS();
}