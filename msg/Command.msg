uint8 REQUESTPOSEGRAPH=0      # Request pose graph
uint8 UPDATE=1                # Perform update / iteration
uint8 TERMINATE=2             # Terminate iterations
uint8 INITIALIZE=3            # Perform initialization

uint8 command
uint16 executing_robot        # The robot that is scheduled to perform update
uint16 executing_iteration    # Iteration number of the scheduled update