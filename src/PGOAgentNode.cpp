#include "PGOAgentNode.h"
#include <map>
#include "DPGO_utils.h"
#include "DPGO_types.h"
#include "RelativeSEMeasurement.h"
#include <tf/tf.h>
#include <dpgo_ros/LiftedPoseStamped.h>

using namespace std;
using namespace DPGO;

namespace DPGO_ROS{

PGOAgentNode::PGOAgentNode(ros::NodeHandle nh_, unsigned ID, const PGOAgentParameters& params):nh(nh_){
	agent = new PGOAgent(ID, params);

	localTrajectoryPublisher = nh.advertise<nav_msgs::Path>("local_trajectory", 1);
	localTrajectoryPublishTimer = nh.createTimer(ros::Duration(0.5), &PGOAgentNode::localTrajectoryPublishCallback, this);

}

PGOAgentNode::~PGOAgentNode()
{	
	delete agent; // this also ensures that optimization thread is not running
}


void PGOAgentNode::localTrajectoryPublishCallback(const ros::TimerEvent&){
	unsigned n = agent->num_poses();
	unsigned d = agent->dimension();
	Matrix T = agent->getTrajectoryInLocalFrame();

	// initialize ROS message
	nav_msgs::Path trajectory;
	trajectory.header.frame_id = "map";

	// populate path message
	for (unsigned i = 0; i < n; ++i){
		Matrix R = T.block(0,i*(d+1),d,d);
		Matrix t = T.block(0,i*(d+1)+d,d,1);

		tf::Point tTF(t(0), t(1), t(2));

		// convert rotation matrix to quaternion
		tf::Matrix3x3 RTF(R(0,0),R(0,1),R(0,2),
						  R(1,0),R(1,1),R(1,2),
						  R(2,0),R(2,1),R(2,2));

		tf::Quaternion quatTF;
		RTF.getRotation(quatTF);

		tf::Pose poseTF(quatTF, tTF);
		geometry_msgs::Pose poseMsg;
		tf::poseTFToMsg(poseTF, poseMsg);

		// create stamped message
		geometry_msgs::PoseStamped poseMsgStamped;
		poseMsgStamped.pose = poseMsg;
		poseMsgStamped.header.stamp = ros::Time::now();
		poseMsgStamped.header.frame_id = "map";

		trajectory.poses.push_back(poseMsgStamped);
	}

	localTrajectoryPublisher.publish(trajectory);
}


}
