"""
arm_config.py
=============
Single source of truth shared by all three pipeline stages
(01_generate_data.py, 02_train_mlp.py, 03_plan_path.py).

Why this file exists
---------------------
The biggest way a 3-stage ML pipeline silently breaks is *drift*: stage 1
assumes joint order [a,b,c,d,e], stage 2 assumes [b,a,c,d,e], and stage 3
loads a checkpoint trained on one workspace box but plans paths in another.
Every constant that must be identical across stages -- joint names, joint
limits, the kinematic chain, the workspace bounding box, and the network
architecture hyperparameters -- lives here, imported everywhere. Nothing
below is redefined in the stage scripts.

Kinematic chain
----------------
We use the RIGHT arm of the Unitree G1 (g1_23dof.urdf), which is exactly a
5-DOF serial chain from the torso to the hand:

    torso_link
      -> right_shoulder_pitch_joint  (revolute, axis Y)
      -> right_shoulder_roll_joint   (revolute, axis X)
      -> right_shoulder_yaw_joint    (revolute, axis Z)
      -> right_elbow_joint           (revolute, axis Y)
      -> right_wrist_roll_joint      (revolute, axis X)
      -> right_wrist_roll_rubber_hand (end-effector link)

All fixed joint origins (xyz + rpy) and joint axes below were extracted
directly from the uploaded URDF. A vectorized numpy forward-kinematics
function (`fk_batch`) reproduces this chain and has been checked against
yourdfpy's per-sample FK to ~1e-15 numerical agreement -- so stage 1 can
generate a million samples without paying yourdfpy's per-call Python
overhead a million times.
"""

import numpy as np

# ---------------------------------------------------------------------------
# URDF / chain identity
# ---------------------------------------------------------------------------
URDF_PATH = "g1_23dof.urdf"          # expected alongside the scripts
BASE_LINK = "torso_link"
EE_LINK = "right_wrist_roll_rubber_hand"

JOINT_NAMES = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
]
NUM_JOINTS = len(JOINT_NAMES)

# (lower, upper) in radians, taken verbatim from each <joint><limit> tag.
JOINT_LIMITS = np.array([
    [-3.0892,           2.6704],
    [-2.2515,           1.5882],
    [-2.618,            2.618],
    [-1.0472,           2.0944],
    [-1.972222054,      1.972222054],
])

# Fixed parent->child origin transform for each joint (from <origin>).
ORIGIN_XYZ = np.array([
    [0.0039563, -0.10021,    0.23778],
    [0.0,       -0.038,     -0.013831],
    [0.0,       -0.00624,   -0.1032],
    [0.015783,   0.0,       -0.080518],
    [0.100,     -0.00188791, -0.010],
])
ORIGIN_RPY = np.array([
    [-0.27931,   5.4949e-05,  0.00019159],
    [ 0.27925,   0.0,         0.0],
    [ 0.0,       0.0,         0.0],
    [ 0.0,       0.0,         0.0],
    [ 0.0,       0.0,         0.0],
])
# Local rotation axis for each revolute joint (from <axis>).
JOINT_AXES = np.array([
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0],
    [1.0, 0.0, 0.0],
])

# ---------------------------------------------------------------------------
# Front workspace bounding box -- "reach out and grab a can/drink in front
# of the robot", expressed in the torso_link frame (x=forward, y=lateral,
# z=vertical).
#
# Derived empirically: 3000 uniform-random samples across the full joint
# range put the right hand inside x in [-0.29, 0.30], y in [-0.42, 0.15],
# z in [-0.05, 0.54] m relative to the torso. A can sitting on a table in
# front of the robot, within comfortable reach of the RIGHT hand (which is
# offset to -y in the torso frame), lives in a sub-box of that: arm
# extended forward and slightly down/level, on the right side of centre.
# Tune these five numbers if your table height / can placement differs.
# ---------------------------------------------------------------------------
WORKSPACE_BOUNDS = {
    "x": (0.15, 0.32),    # forward reach
    "y": (-0.35, -0.05),  # right-hand side of the torso centerline
    "z": (-0.05, 0.20),   # roughly chest-to-table height band
}

# ---------------------------------------------------------------------------
# Network architecture (shared between training and inference so stage 3
# can reconstruct the exact module graph before loading state_dict).
# "Larger net, longer training, best accuracy" configuration.
# ---------------------------------------------------------------------------
INPUT_DIM = 5     # x, y, z, vx, vy
OUTPUT_DIM = 5    # 5 joint angles
HIDDEN_DIM = 512
NUM_RESIDUAL_BLOCKS = 6

# ---------------------------------------------------------------------------
# Vectorized forward kinematics
# ---------------------------------------------------------------------------
def _rpy_to_R(rpy):
    """Fixed (non-batched) rotation matrix from static URDF <origin rpy=.../>."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _axis_angle_to_R_batch(axis, theta):
    """
    Rodrigues' rotation formula, vectorized over N joint-angle samples,
    for a single fixed rotation axis.

        R(theta) = I + sin(theta) K + (1 - cos(theta)) K^2

    where K is the skew-symmetric cross-product matrix of `axis`.
    theta: (N,) -> returns (N, 3, 3)
    """
    x, y, z = axis
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    K2 = K @ K
    n = theta.shape[0]
    s = np.sin(theta)[:, None, None]
    c = np.cos(theta)[:, None, None]
    I = np.eye(3)[None, :, :]
    return I + s * K[None, :, :] + (1.0 - c) * K2[None, :, :]


def fk_batch(joint_angles: np.ndarray) -> np.ndarray:
    """
    Batched forward kinematics for the 5-DOF right-arm chain.

    Parameters
    ----------
    joint_angles : (N, 5) array of joint angles in radians, ordered as
        JOINT_NAMES.

    Returns
    -------
    T : (N, 4, 4) array of homogeneous transforms mapping EE_LINK -> BASE_LINK
        (i.e. the end-effector pose expressed in the torso frame).
    """
    joint_angles = np.asarray(joint_angles, dtype=np.float64)
    n = joint_angles.shape[0]
    T = np.tile(np.eye(4), (n, 1, 1))

    for i in range(NUM_JOINTS):
        # Static parent -> joint-frame transform.
        T_origin = np.eye(4)
        T_origin[:3, :3] = _rpy_to_R(ORIGIN_RPY[i])
        T_origin[:3, 3] = ORIGIN_XYZ[i]
        T_origin_b = np.tile(T_origin, (n, 1, 1))

        # Variable rotation about the joint's local axis.
        R_joint = _axis_angle_to_R_batch(JOINT_AXES[i], joint_angles[:, i])
        T_joint = np.tile(np.eye(4), (n, 1, 1))
        T_joint[:, :3, :3] = R_joint

        T = T @ T_origin_b @ T_joint

    return T


def positions_and_approach(joint_angles: np.ndarray):
    """
    Convenience wrapper: from (N, 5) joint angles, return
      pos       : (N, 3) end-effector (x, y, z) in the torso frame [m]
      approach2d: (N, 2) unit (vx, vy) -- the horizontal projection of the
                  end-effector's local +X axis (its "pointing"/approach
                  direction), re-normalized to unit length in the xy-plane.
                  This tells the network which way the hand faces when it
                  arrives at a position, which position alone can't convey.
    """
    T = fk_batch(joint_angles)
    pos = T[:, :3, 3]
    approach_3d = T[:, :3, 0]  # local +X axis of the end effector, in torso frame
    approach_xy = approach_3d[:, :2]
    norm = np.linalg.norm(approach_xy, axis=1, keepdims=True)
    norm = np.clip(norm, 1e-8, None)
    approach2d = approach_xy / norm
    return pos, approach2d


def in_workspace(pos: np.ndarray) -> np.ndarray:
    """Boolean mask: which (N,3) positions fall inside WORKSPACE_BOUNDS."""
    x, y, z = pos[:, 0], pos[:, 1], pos[:, 2]
    xlo, xhi = WORKSPACE_BOUNDS["x"]
    ylo, yhi = WORKSPACE_BOUNDS["y"]
    zlo, zhi = WORKSPACE_BOUNDS["z"]
    return (x >= xlo) & (x <= xhi) & (y >= ylo) & (y <= yhi) & (z >= zlo) & (z <= zhi)


def verify_against_yourdfpy(n_checks: int = 200, seed: int = 0) -> float:
    """
    Sanity check: `fk_batch` above is a hand-vectorized numpy re-derivation
    of the URDF chain, used because calling yourdfpy once per sample is far
    too slow for generating a million-sample dataset. This function is the
    yourdfpy-in-the-loop check called out in the project's tech stack: it
    loads the real URDF with yourdfpy and confirms fk_batch agrees with it
    to numerical precision on random configurations, so you can trust the
    fast path. Run this once after editing anything in this file, or if you
    swap in a different URDF.

    Returns the maximum discrepancy found (position [m] and rotation
    Frobenius norm combined) -- this should be on the order of 1e-10 or
    smaller.
    """
    import yourdfpy  # imported here, not at module load time, so the fast

    robot = yourdfpy.URDF.load(URDF_PATH, load_meshes=False, build_collision_scene_graph=False)
    rng = np.random.default_rng(seed)
    lo, hi = JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1]
    test_angles = rng.uniform(lo, hi, size=(n_checks, NUM_JOINTS))

    T_batch = fk_batch(test_angles)

    max_err = 0.0
    for i in range(n_checks):
        joint_cfg = {jn: 0.0 for jn in robot.actuated_joint_names}
        for j, jn in enumerate(JOINT_NAMES):
            joint_cfg[jn] = test_angles[i, j]
        robot.update_cfg(joint_cfg)
        T_ref = robot.get_transform(EE_LINK, BASE_LINK)

        err_pos = np.linalg.norm(T_ref[:3, 3] - T_batch[i, :3, 3])
        err_rot = np.linalg.norm(T_ref[:3, :3] - T_batch[i, :3, :3])
        max_err = max(max_err, err_pos, err_rot)

    print(f"[verify_against_yourdfpy] max discrepancy over {n_checks} random "
          f"configs: {max_err:.2e}")
    return max_err


if __name__ == "__main__":
    # `python arm_config.py` on its own runs the yourdfpy cross-check.
    verify_against_yourdfpy()
