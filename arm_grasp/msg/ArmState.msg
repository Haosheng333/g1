# ArmState.msg - Arm joint state information
# Published at 50 Hz on ~arm_state

# Timestamp
time stamp

# Left arm joint states (5 DOF)
float64[] left_position      # Joint positions (rad)
float64[] left_velocity      # Joint velocities (rad/s)

# Right arm joint states (5 DOF)
float64[] right_position     # Joint positions (rad)
float64[] right_velocity     # Joint velocities (rad/s)

# Joint names for reference
string[] left_joint_names
string[] right_joint_names

# Controller state
bool is_active               # Controller is running
float64 weight               # Current weight parameter (0-1)
