# pose dimension (2 or 3)
uint32 dimension

# relaxation rank in the Riemannian problem (equals to the number of rows of the pose matrix)
uint32 relaxation_rank

# the cluster that this robot belongs to
uint32 cluster_id  

# the uniquely identifying id of this robot
uint32 robot_id

# the id of the pose within this robot's trajectory. Together, (robot_id, pose_id) gives a unique identifier of the shared pose in the global pose graph
uint32 pose_id

# pose matrix stored in row-major format
float64[] pose     
