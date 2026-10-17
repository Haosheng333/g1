# EndEffectorPose.msg - End-effector pose from forward kinematics
# Published at 50 Hz on ~left_ee_pose and ~right_ee_pose

# Timestamp
time stamp

# Arm identifier
string arm                    # "left" or "right"

# Position relative to base frame (meters)
geometry_msgs/Point position

# Orientation as quaternion (xyzw)
geometry_msgs/Quaternion orientation

# Orientation as Euler angles (roll, pitch, yaw in radians)
float64 roll
float64 pitch
float64 yaw
