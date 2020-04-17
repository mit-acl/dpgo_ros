# This message encodes a relative SE(3) measurement from pose (robot1, pose1) to (robot2, pose2)

# Used for timestamp
std_msgs/Header header

# ID of first robot
uint32 robot1

# local ID of first pose
uint32 pose1

# ID of second robot
uint32 robot2

# local ID of second pose
uint32 pose2

# relative rotation
geometry_msgs/Quaternion rotation

# relative translation 
geometry_msgs/Vector3 translation

# rotation measurement weight
float64 kappa

# translation measurement weight
float64 tau