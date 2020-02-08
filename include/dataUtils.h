#ifndef DATAUTILS_H
#define DATAUTILS_H

#include <cassert>
#include "DPGO_types.h"
#include <std_msgs/Float64.h>

using namespace std;
using namespace DPGO;

namespace DPGO_ROS{


	/**
	Serialize a Matrix object into a vector of Float64 messages, in row-major format
	*/
	vector<std_msgs::Float64> serializeMatrix(const size_t rows, const size_t cols, const Matrix& Mat)
	{
		assert((size_t) Mat.rows() == rows);
		assert((size_t) Mat.cols() == cols);

		vector<std_msgs::Float64> v;
		std_msgs::Float64 scalar;

		for(size_t row = 0; row < rows; ++row){
			for(size_t col = 0; col < cols; ++col){
				scalar.data = Mat(row,col);
				v.push_back(scalar);
			}
		}

		return v;

	}



	/**
	Deserialize a vector of Float64 messages into a Matrix object, using row-major format
	*/
	Matrix deserializeMatrix(const size_t rows, const size_t cols, const vector<std_msgs::Float64>& v)
	{	
		assert(v.size() == rows * cols);
		Matrix Mat = Matrix::Zero(rows, cols);
		for (size_t row = 0; row < rows; ++row){
			for(size_t col = 0; col < cols; ++col){
				size_t index = row * cols + col;
				Mat(row,col) = v[index].data;
			}
		}

		return Mat;
	}

}







#endif