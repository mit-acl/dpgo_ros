uint8 WAIT_FOR_DATA=0
uint8 WAIT_FOR_INITIALIZATION=1
uint8 INITIALIZED=2

std_msgs/Header header
uint16 instance_number
uint16 iteration_number
uint16 robot_id
uint16 cluster_id
uint8 state
bool ready_to_terminate
float32 relative_change