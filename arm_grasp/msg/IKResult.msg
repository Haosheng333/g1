# IKResult.msg - Result of inverse kinematics computation

# Header
time stamp

# Status code
int32 status
int32 SUCCESS = 0
int32 CONVERGENCE_FAILED = 1
int32 JOINT_LIMIT_VIOLATED = 2
int32 COLLISION_DETECTED = 3
int32 INVALID_REQUEST = 4
int32 SOLVER_ERROR = 5

# Which arm this result is for ("left" or "right")
string arm

# Solution joint angles (10 DOF: left_arm[5] + right_arm[5])
float64[] joint_angles

# Position error (meters)
float64 position_error

# Orientation error (Frobenius norm of rotation difference)
float64 orientation_error

# Minimum distance to collision (meters)
float64 min_collision_distance

# Solve time (seconds)
float64 solve_time

# Human-readable message
string message
