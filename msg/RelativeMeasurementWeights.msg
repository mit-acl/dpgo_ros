uint16 robot_id                # ID of the publishing robot
uint16 cluster_id              # ID of the cluster that the publishing robot belongs to
uint16 destination_robot_id    # ID of receiving robot 
uint16[] src_robot_ids
uint16[] dst_robot_ids
uint32[] src_pose_ids
uint32[] dst_pose_ids
float32[] weights
bool[] fixed_weights