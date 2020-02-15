#include "PGOAgentNode.h"
#include <map>
#include "DPGO_utils.h"
#include "DPGO_types.h"
#include "RelativeSEMeasurement.h"

using namespace std;
using namespace DPGO;

/** 
This script implements the entry point for running a single PGO Agent in ROS
*/

int main(int argc, char **argv) {
	ros::init(argc, argv, "agent_node");
	ros::NodeHandle nh;

	int ID = -1;
	ros::param::get("~agent_id", ID);
	if (ID < 0){
		ROS_ERROR_STREAM("Negative agent id! ");
		return -1;
	}


	/** 
	##########################################################################################
	Read dataset from file
	##########################################################################################
	*/

	int num_robots = 0;
	nh.getParam("/num_robots", num_robots);
	if(num_robots <= 0){
		ROS_ERROR_STREAM("Number of robots must be positive!");
		return -1;
	}
	if(ID >= num_robots){
		ROS_ERROR_STREAM("ID greater than number of robots!");
		return -1;
	}
	
	string filename;
	nh.getParam("/dataset", filename);
	cout << "Reading dataset: " << filename << "..." << endl;


	size_t N = 0; // total number of poses in the dataset
    vector<RelativeSEMeasurement> dataset = DPGO::read_g2o_file(filename, N);
    cout << "Loaded dataset: " << filename << "." << endl;
    if(dataset.empty()){
    	ROS_ERROR_STREAM("Empty dataset!");
    	return -1;
    }


    unsigned int num_poses_per_robot = N / num_robots;
    if(num_poses_per_robot <= 0){
        ROS_ERROR_STREAM("More robot than the total number of poses!");
        return -1;
    }

    cout << "Creating mapping from global to local pose... " << endl;
    map<unsigned, PoseID> PoseMap;
    for(unsigned robot = 0; robot < (unsigned) num_robots; ++robot){
        unsigned startIdx = robot * num_poses_per_robot;
        unsigned endIdx = (robot+1) * num_poses_per_robot; // non-inclusive
        if (robot == (unsigned) num_robots - 1) endIdx = N;
        for(unsigned idx = startIdx; idx < endIdx; ++idx){
            unsigned localIdx = idx - startIdx; // this is the local ID of this pose
            PoseID pose = make_pair(robot, localIdx);
            PoseMap[idx] = pose;
        }
    }


    /** 
	##########################################################################################
	Initialize PGOAgent 
	##########################################################################################
	*/
	int d = -1;
	int r = -1;
	double rate = -1.0;
	double stepsize = -1.0;
	ROPTALG algorithm = ROPTALG::RTR;
	bool verbose = false;

	nh.getParam("/dimension", d);
	nh.getParam("/relaxation_rank", r);
	nh.getParam("/optimization_rate", rate);
	nh.getParam("/rgd_stepsize", stepsize);
	if (d < 0){
		ROS_ERROR_STREAM("Negative dimension!");
		return -1;
	}
	unsigned d_ = (!dataset.empty() ? dataset[0].t.size() : 0);
	if ((unsigned) d != d_){
		ROS_ERROR_STREAM("Given dimension does not match measurement!");
		return -1;
	}
	if (r < d){
		ROS_ERROR_STREAM("Relaxation rank cannot be smaller than dimension!");
		return -1;
	}
	if (rate < 0){
		ROS_ERROR_STREAM("Negative optimization rate!");
		return -1;
	}
	if (stepsize > 0){
		ROS_INFO_STREAM("Using Riemannian gradient descent with stepsize " << stepsize);
		algorithm = ROPTALG::RGD;
	}

	cout << "Initializing PGO Agent: ID = " << ID << ", dimension = " << d << ", relaxation_rank = " << r << ", optimization_rate = " << rate <<  endl;
	
	PGOAgentParameters options(d,r,algorithm,verbose);
	
	DPGO_ROS::PGOAgentNode node(nh, ID, options);

	node.setStepsize(stepsize);


	/** 
	##########################################################################################
	Staic pose graph: add all measurements to node before starting optimization
	##########################################################################################
	*/

   for(size_t k = 0; k < dataset.size(); ++k){
        RelativeSEMeasurement mIn = dataset[k];
        PoseID src = PoseMap[mIn.p1];
        PoseID dst = PoseMap[mIn.p2];

        unsigned srcRobot = get<0>(src);
        unsigned srcIdx = get<1>(src);
        unsigned dstRobot = get<0>(dst);
        unsigned dstIdx = get<1>(dst);

        RelativeSEMeasurement m(srcRobot, dstRobot, srcIdx, dstIdx, mIn.R, mIn.t, mIn.kappa, mIn.tau);
        unsigned mID = (unsigned) ID;

        if (mID == srcRobot && mID == dstRobot){
        	if (srcIdx + 1 == dstIdx){
        		// odometry
        		node.addOdometry(m);
        	}
        	else{
        		// private loop closure
        		node.addPrivateLoopClosure(m);
        	}
        }else if(mID != srcRobot && mID != dstRobot){
        	// discard
        }else{
        	// shared loop closure
        	node.addSharedLoopClosure(m);
        }
    }

 //    nh.getParam("/Yinit", filename);
 //    Matrix Yinit = read_matrix_from_file(filename);
 //    unsigned startIdx = ID * num_poses_per_robot;
 //    unsigned endIdx = (ID+1) * num_poses_per_robot; // non-inclusive
 //    if (ID ==  num_robots - 1) endIdx = N;
 //    node.setY(Yinit.block(0, startIdx*(d+1), r, (endIdx-startIdx)*(d+1)));



    /** 
	##########################################################################################
	Dynamic pose graph: add measurements to queue
	##########################################################################################
	*/

	// size_t n = num_poses_per_robot;
	// if (ID == num_robots){
	// 	n = N - (num_robots-1)*num_poses_per_robot;
	// } 

	// node.initializeLoopClosureQueues(n);

	// for(size_t k = 0; k < dataset.size(); ++k){
        // RelativeSEMeasurement mIn = dataset[k];
        // PoseID src = PoseMap[mIn.p1];
        // PoseID dst = PoseMap[mIn.p2];

        // unsigned srcRobot = get<0>(src);
        // unsigned srcIdx = get<1>(src);
        // unsigned dstRobot = get<0>(dst);
        // unsigned dstIdx = get<1>(dst);

        // RelativeSEMeasurement m(srcRobot, dstRobot, srcIdx, dstIdx, mIn.R, mIn.t, mIn.kappa, mIn.tau);
        // unsigned mID = (unsigned) ID;

        // if (mID == srcRobot && mID == dstRobot){
        	// if (srcIdx + 1 == dstIdx){
        		// // odometry
        		// node.addOdometryToQueue(m);
        	// }
        	// else{
        		// // private loop closure
        		// node.addPrivateLoopClosureToQueue(m);
        	// }
        // }else if(mID != srcRobot && mID != dstRobot){
        	// discard
        // }else{
        	// shared loop closure
        	// node.addSharedLoopClosureToQueue(m);
        // }
    // }

    // node.registerPoseInsertionCallback();

    /** 
	##########################################################################################
	Set up callbacks
	##########################################################################################
	*/

    string pose_update_topic, anchor_topic, solution_topic;
    double pose_update_publish_rate;
	nh.getParam("/pose_update_topic", pose_update_topic);
	nh.getParam("/pose_update_publish_rate", pose_update_publish_rate);
	nh.getParam("/anchor_topic", anchor_topic);
	nh.getParam("/solution_topic", solution_topic);
	node.registerPoseUpdateCallback(pose_update_topic, pose_update_publish_rate);
	node.registerAnchorCallback(anchor_topic);
	node.registerSolutionCallback(solution_topic);
	node.registerTrajectoryCallback();


	/** 
	##########################################################################################
	Kick off optimization
	##########################################################################################
	*/
	cout << "Sleeping..." << endl;
	sleep(2);

	node.startOptimizationLoop(rate);

	ros::spin();

	return 0;
}