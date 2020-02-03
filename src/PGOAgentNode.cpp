#include "PGOAgentNode.h"


using namespace std;
using namespace DPGO;

namespace DPGO_ROS{

PGOAgentNode::PGOAgentNode(ros::NodeHandle nh_):nh(nh_){}

PGOAgentNode::~PGOAgentNode()
{	
	delete agent; // this also ensures that optimization thread is not running
}


void PGOAgentNode::initialize(unsigned ID, const PGOAgentParameters& params){
	agent = new PGOAgent(ID, params);
}

void PGOAgentNode::startOptimizationLoop(double freq){
	agent->startOptimizationLoop(freq);
}


void PGOAgentNode::endOptimizationLoop(){
	agent->endOptimizationLoop();
}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "agent_node");
	ros::NodeHandle nh;

	// Get agent ID
	int ID = -1;
	ros::param::get("~agent_id", ID);
	if (ID < 0){
		ROS_ERROR_STREAM("Error: negative agent id. ");
		return -1;
	}

	// Get dimension and rank relaxation
	int d = -1;
	int r = -1;
	double rate = -1.0;
	nh.getParam("/dimension", d);
	nh.getParam("/relaxation_rank", r);
	nh.getParam("/optimization_rate", rate);
	if (d < 0){
		ROS_ERROR_STREAM("Error: negative dimension.");
		return -1;
	}
	if (r < d){
		ROS_ERROR_STREAM("Error: relaxation rank cannot be smaller than dimension.");
		return -1;
	}
	if (rate < 0){
		ROS_ERROR_STREAM("Error: negative optimization rate.");
		return -1;
	}

	cout << "Initializing PGO Agent: ID = " << ID << ", dimension = " << d << ", relaxation_rank = " << r << ", optimization_rate = " << rate <<  endl;
	
	PGOAgentParameters options(d,r,ROPTALG::RTR,true);
	
	DPGO_ROS::PGOAgentNode node(nh);

	node.initialize(ID, options);

	node.startOptimizationLoop(rate);

	ros::spin();

	return 0;
}