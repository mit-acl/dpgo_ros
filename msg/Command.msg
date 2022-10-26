uint8 REQUESTPOSEGRAPH=0      # Request pose graph
uint8 UPDATE=1                # Perform update / iteration
uint8 TERMINATE=2             # Nominal termination (after successful optimization)
uint8 HARD_TERMINATE=3        # Hard termination (e.g., after communication loss)
uint8 INITIALIZE=4            # Perform initialization
uint8 NOOP=5                  # NoOp (used for debugging)

std_msgs/Header header
uint8 command
uint16 publishing_robot       # The robot that publishes this command
uint16 executing_robot        # The robot that is scheduled to update (only used by UPDATE command)
uint16 executing_iteration    # Iteration number of the scheduled update (only used by UPDATE command)