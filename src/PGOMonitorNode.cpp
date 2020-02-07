#include "PGOMonitorNode.h"
#include <map>
#include "DPGO_utils.h"
#include "DPGO_types.h"
#include "RelativeSEMeasurement.h"
#include "msgUtils.h"

using namespace std;
using namespace DPGO;
using dpgo_ros::LiftedPose;
using dpgo_ros::LiftedPoseArray;

namespace DPGO_ROS{

	PGOMonitorNode::PGOMonitorNode(ros::NodeHandle nh_):nh(nh_){

		string filename;
		nh.getParam("/dataset", filename);

		int num_robots = 0;
		nh.getParam("/num_robots", num_robots);
		ROS_WARN_STREAM("Number of robots: " << num_robots << ".");

		size_t N = 0; 
	    vector<RelativeSEMeasurement> dataset = DPGO::read_g2o_file(filename, N);
	    ROS_WARN_STREAM("Loaded dataset: " << filename << ".");

	    unsigned int num_poses_per_robot = N / num_robots;
	   	ROS_WARN_STREAM("Creating mapping from local to global pose... ");
	    for(unsigned robot = 0; robot < (unsigned) num_robots; ++robot){
	        unsigned startIdx = robot * num_poses_per_robot;
	        unsigned endIdx = (robot+1) * num_poses_per_robot; // non-inclusive
	        if (robot == (unsigned) num_robots - 1) endIdx = N;
	        for(unsigned idx = startIdx; idx < endIdx; ++idx){
	            unsigned localIdx = idx - startIdx; // this is the local ID of this pose
	            PoseID pose = make_pair(robot, localIdx);
	            PoseMap[pose] = idx;
	        }
	    }

	    int d = -1;
	    int r = -1;
		nh.getParam("/dimension", d);
		nh.getParam("/relaxation_rank", r);
		if (d <= 0){
			ROS_ERROR_STREAM("Dimendion cannot be negative!");
			ros::shutdown();
		}
		if (r < d){
			ROS_ERROR_STREAM("Relaxation rank cannot be smaller than dimension!");
			ros::shutdown();
		}

		Y = Matrix::Zero(r, (d+1) * N);
		SparseMatrix Q = constructConnectionLaplacianSE(dataset);
		SparseMatrix G(r, (d+1) * N);
		G.setZero();
		problem = new QuadraticProblem(N, d, r, Q, G);

		string Y_topic;
		nh.getParam("/Y_topic", Y_topic);
		YSubscriber = nh.subscribe(Y_topic, 1, &PGOMonitorNode::YSubscribeCallback, this);
	}


	PGOMonitorNode::~PGOMonitorNode(){ delete problem; }


	void PGOMonitorNode::YSubscribeCallback(const dpgo_ros::LiftedPoseArrayConstPtr& msg){
		unsigned r = problem->relaxation_rank();
		unsigned d = problem->dimension();

		for(size_t i = 0; i < msg->poses.size(); ++i){
			LiftedPose poseMsg = msg->poses[i];

			PoseID localID = make_pair(poseMsg.robot_id.data, poseMsg.pose_id.data);
			unsigned index = PoseMap[localID];

			Y.block(0,index*(d+1),r,d+1) = deserializeMatrix(r,d+1,poseMsg.pose);
		}
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "coordinator_node");
	
	ros::NodeHandle nh;

	DPGO_ROS::PGOMonitorNode node(nh);

	ros::spin();

	return 0;

}