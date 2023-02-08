uint8 REQUEST_POSE_GRAPH=0    # Request pose graph for distributed optimization
uint8 UPDATE=1                # Perform update / iteration
uint8 TERMINATE=2             # Nominal termination (after successful optimization)
uint8 HARD_TERMINATE=3        # Hard termination (e.g., after communication loss)
uint8 INITIALIZE=4            # Perform initialization
uint8 UPDATE_WEIGHT=5         # Update measurement weights (for robust optimization)
uint8 RECOVER=6               # Recover from disconnection
uint8 SET_ACTIVE_ROBOTS=7     # Set the list of active robots that will participate in distributed optimization
uint8 NOOP=8                  # NoOp (used for debugging)

std_msgs/Header header
uint8 command
uint16 cluster_id             # Cluster ID
uint16 publishing_robot       # The robot that publishes this command
uint16 executing_robot        # The robot that is scheduled to update (only used by UPDATE command)
uint16 executing_iteration    # Iteration number of the scheduled update (only used by UPDATE command)
uint16[] active_robots        # List of active robots (only used by SET_ACTIVE_ROBOTS command)