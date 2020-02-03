#include "PGOAgentNode.h"


using namespace std;
using namespace DPGO;

namespace DPGO_ROS{

PGOAgentNode::PGOAgentNode(ros::NodeHandle nh_):nh(nh_){}

PGOAgentNode::~PGOAgentNode()
{
	delete agent;
}


void PGOAgentNode::initialize(unsigned ID, const PGOAgentParameters& params){
	agent = new PGOAgent(ID, params);
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
	nh.getParam("/dimension", d);
	nh.getParam("/relaxation_rank", r);
	if (d < 0 || r < 0){
		ROS_ERROR_STREAM("Error: negative dimension or relaxation rank.");
		return -1;
	}

	cout << "Initializing PGO Agent: ID = " << ID << ", dimension = " << d << ", relaxation_rank = " << r << endl;
	
	PGOAgentParameters options(d,r,ROPTALG::RTR,true);
	
	DPGO_ROS::PGOAgentNode node(nh);

	node.initialize(ID, options);

	ros::spin();

	return 0;
}