uint16 robot_id                     # ID of the publishing robot
uint16 destination_robot_id         # ID of the receiving robot
uint16 instance_number
uint16 iteration_number
bool is_auxiliary
uint32[] pose_ids                   # Pose IDs of the publishing robot
dpgo_ros/MatrixMsg[] poses          # Corresponding public poses of the publishing robot