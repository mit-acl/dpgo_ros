# This message encodes a relative SE(3) measurement from pose (robot1, pose1) to (robot2, pose2)

# Used for timestamp
std_msgs/Header header

# ID of first robot
std_msgs/UInt32 robot1

# local ID of first pose
std_msgs/UInt32 pose1

# ID of second robot
std_msgs/UInt32 robot2

# local ID of second pose
std_msgs/UInt32 pose2

# relative rotation
geometry_msgs/Quaternion rotation

# relative translation 
geometry_msgs/Vector3 translation

# rotation measurement weight
std_msgs/Float64 kappa

# translation measurement weight
std_msgs/Float64 tau