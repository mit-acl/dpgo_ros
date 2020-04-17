/* ----------------------------------------------------------------------------
 * Copyright 2020, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Yulun Tian, et al. (see README for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef DATAUTILS_H
#define DATAUTILS_H

#include <cassert>
#include <tf/tf.h>
#include "DPGO_types.h"
#include <dpgo_ros/LiftedPose.h>
#include <dpgo_ros/RelativeMeasurementStamped.h>

using namespace std;
using namespace DPGO;

namespace DPGO_ROS{


	/**
	Serialize a Matrix object into a vector of Float64 messages, in row-major format
	*/
	vector<double> serializeMatrix(const size_t rows, const size_t cols, const Matrix& Mat)
	{
		assert((size_t) Mat.rows() == rows);
		assert((size_t) Mat.cols() == cols);

		vector<double> v;

		for(size_t row = 0; row < rows; ++row){
			for(size_t col = 0; col < cols; ++col){
				double scalar = Mat(row,col);
				v.push_back(scalar);
			}
		}

		return v;
	}


	/**
	Deserialize a vector of Float64 messages into a Matrix object, using row-major format
	*/
	Matrix deserializeMatrix(const size_t rows, const size_t cols, const vector<double>& v)
	{	
		assert(v.size() == rows * cols);
		Matrix Mat = Matrix::Zero(rows, cols);
		for (size_t row = 0; row < rows; ++row){
			for(size_t col = 0; col < cols; ++col){
				size_t index = row * cols + col;
				Mat(row,col) = v[index];
			}
		}

		return Mat;
	}


	/**
	Helper function to construct a lifted pose message from inputs
	*/
	dpgo_ros::LiftedPose constructLiftedPoseMsg(const size_t dimension,  const size_t relaxation_rank, 
												const size_t cluster_id, const size_t robot_id, const size_t pose_id, 
												const Matrix pose)
	{
		dpgo_ros::LiftedPose msg;
		msg.dimension = dimension;
		msg.relaxation_rank = relaxation_rank;
		msg.cluster_id = cluster_id;
		msg.robot_id = robot_id;
		msg.pose_id = pose_id;
		msg.pose = serializeMatrix(relaxation_rank, dimension+1, pose);
		return msg;
	}



	/**
	Helper function to construct a relative measurement message from inputs
	*/
	dpgo_ros::RelativeMeasurementStamped constructRelativeMeasurementMsg(const size_t robot1, const size_t pose1, 
															   			 const size_t robot2, const size_t pose2,
															   			 const tf::Quaternion rotation, 
															   			 const tf::Vector3    translation, 
															   			 const double kappa,  const double tau)
	{
		dpgo_ros::RelativeMeasurementStamped msg;
		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "world";

		msg.robot1 = robot1;
		msg.robot2 = robot2;
		msg.pose1 = pose1;
		msg.pose2 = pose2;

		tf::quaternionTFToMsg(rotation, msg.rotation);
		tf::vector3TFToMsg(translation, msg.translation);

		msg.kappa = kappa;
		msg.tau = tau;

		return msg;
	} 

}







#endif