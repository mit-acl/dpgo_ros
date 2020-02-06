# pose dimension (2 or 3)
std_msgs/UInt32 dimension

# relaxation rank in the Riemannian problem (equals to the number of rows of the pose matrix)
std_msgs/UInt32 relaxation_rank

# the cluster that this robot belongs to
std_msgs/UInt32 cluster_id  

# the uniquely identifying id of this robot
std_msgs/UInt32 robot_id

# the id of the pose within this robot's trajectory. Together, (robot_id, pose_id) gives a unique identifier of the shared pose in the global pose graph
std_msgs/UInt32 pose_id

# pose matrix stored in row-major format
std_msgs/Float64[] pose     
