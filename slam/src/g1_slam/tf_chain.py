"""SE(3) helpers and the G1 pelvis TF chain.

Pure Python: no ROS, no numpy. This is *production* code, imported by
``scripts/g1_tf_publisher.py`` and by ``scripts/slam_health_check.py``, and
exercised by ``test/test_tf_transforms.py``.

Frames
------
``camera_init``  FAST-LIO2 odometry origin (world). Re-created at the pose the
                 Mid-360 IMU had on the first LiDAR frame of each session.
``body``         FAST-LIO2's tracked pose = the Mid-360 **IMU** frame.
``mid360_link``  The Mid-360 **LiDAR** frame, as named by the official URDF.
``torso_link``   G1 torso. Parent of mid360_link through a *fixed* joint.
``pelvis``       Official URDF root. Parent of torso_link through
                 ``waist_yaw_joint``, which is **revolute** - so the transform
                 between the sensor and the robot root is NOT static.

Transform direction (the thing that is easy to get backwards)
-------------------------------------------------------------
FAST-LIO2's ``mapping/extrinsic_T`` in ``config/fastlio.yaml`` is the LiDAR pose
expressed in the IMU frame. In ROS TF terms that is::

    body -> mid360_link :  translation (-0.011, -0.02329, 0.04412), rotation identity
    mid360_link -> body :  the inverse, translation (+0.011, +0.02329, -0.04412)

``extrinsic_R`` is identity, and stays identity even though the Mid-360 is
mounted upside down: it describes the LiDAR-to-IMU relationship *inside* the
sensor housing, which does not change with how the housing is bolted on. The
180 degree roll of the inverted mount lives in the URDF's ``mid360_joint``.

Composition order is ``parent_to_child``: ``multiply(a_to_b, b_to_c)`` gives
``a_to_c``.
"""
import math
import xml.etree.ElementTree as ET

# --- Constants taken from config/fastlio.yaml and the official URDF ----------

#: ``body -> mid360_link`` translation, i.e. FAST-LIO2's mapping/extrinsic_T.
BODY_TO_MID360_XYZ = (-0.011, -0.02329, 0.04412)
#: ``body -> mid360_link`` rotation. Identity (FAST-LIO2's extrinsic_R).
BODY_TO_MID360_RPY = (0.0, 0.0, 0.0)

#: URDF ``waist_yaw_joint`` origin: pelvis -> torso_link, before the rotation.
WAIST_YAW_ORIGIN_XYZ = (-0.0039635, 0.0, 0.044)
#: URDF ``waist_yaw_joint`` axis. Rotation about Z.
WAIST_YAW_AXIS = (0.0, 0.0, 1.0)
#: URDF ``mid360_joint`` origin: torso_link -> mid360_link, fixed.
#: The pi roll is the upside-down mount.
TORSO_TO_MID360_XYZ = (0.0002835, 0.00003, 0.428434)
TORSO_TO_MID360_RPY = (math.pi, 0.05112069379091391, 0.0)

#: URDF joint names this module depends on, and the type each must have.
URDF_REQUIRED_JOINTS = {
    "waist_yaw_joint": "revolute",
    "mid360_joint": "fixed",
}

WORLD_FRAME = "camera_init"
IMU_FRAME = "body"
LIDAR_FRAME = "mid360_link"
TORSO_FRAME = "torso_link"
ROBOT_ROOT_FRAME = "pelvis"

_EPS = 1e-12


# --- Matrix primitives -------------------------------------------------------

def identity():
    """The 4x4 identity as a tuple of row tuples."""
    return ((1.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 0.0),
            (0.0, 0.0, 0.0, 1.0))


def rotation_from_rpy(roll, pitch, yaw):
    """3x3 rotation for URDF/tf2 fixed-axis rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return ((cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp,     cp * sr,               cp * cr))


def rpy_from_rotation(rot):
    """Fixed-axis (roll, pitch, yaw) recovered from a 3x3 rotation."""
    pitch = math.atan2(-rot[2][0], math.hypot(rot[0][0], rot[1][0]))
    return (math.atan2(rot[2][1], rot[2][2]),
            pitch,
            math.atan2(rot[1][0], rot[0][0]))


def transform(xyz, rpy):
    """4x4 homogeneous transform from a translation and a fixed-axis rpy."""
    rot = rotation_from_rpy(*rpy)
    return (rot[0] + (float(xyz[0]),),
            rot[1] + (float(xyz[1]),),
            rot[2] + (float(xyz[2]),),
            (0.0, 0.0, 0.0, 1.0))


def multiply(a, b):
    """Compose two 4x4 transforms. multiply(a_to_b, b_to_c) == a_to_c."""
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4))
                 for i in range(4))


def invert(m):
    """Inverse of a rigid 4x4 transform (transpose the rotation, re-map t)."""
    rot_t = tuple(tuple(m[j][i] for j in range(3)) for i in range(3))
    trans = tuple(m[i][3] for i in range(3))
    neg = tuple(-sum(rot_t[i][k] * trans[k] for k in range(3)) for i in range(3))
    return (rot_t[0] + (neg[0],),
            rot_t[1] + (neg[1],),
            rot_t[2] + (neg[2],),
            (0.0, 0.0, 0.0, 1.0))


def translation_of(m):
    """The (x, y, z) translation of a 4x4 transform."""
    return (m[0][3], m[1][3], m[2][3])


def rotation_of(m):
    """The 3x3 rotation block of a 4x4 transform."""
    return tuple(tuple(m[i][j] for j in range(3)) for i in range(3))


def rotation_from_quaternion(x, y, z, w):
    """3x3 rotation from a (x, y, z, w) quaternion, normalised defensively."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < _EPS:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)),
            (2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)))


def quaternion_of(m):
    """(x, y, z, w) quaternion from a 4x4 transform's rotation block."""
    r = m
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return ((r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s,
                (r[1][0] - r[0][1]) / s, 0.25 * s)
    if r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2]) * 2.0
        return (0.25 * s, (r[0][1] + r[1][0]) / s,
                (r[0][2] + r[2][0]) / s, (r[2][1] - r[1][2]) / s)
    if r[1][1] > r[2][2]:
        s = math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2]) * 2.0
        return ((r[0][1] + r[1][0]) / s, 0.25 * s,
                (r[1][2] + r[2][1]) / s, (r[0][2] - r[2][0]) / s)
    s = math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1]) * 2.0
    return ((r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s,
            0.25 * s, (r[1][0] - r[0][1]) / s)


def transform_from_pose(position, quaternion):
    """4x4 from a (x, y, z) position and an (x, y, z, w) quaternion."""
    rot = rotation_from_quaternion(*quaternion)
    return (rot[0] + (float(position[0]),),
            rot[1] + (float(position[1]),),
            rot[2] + (float(position[2]),),
            (0.0, 0.0, 0.0, 1.0))


def max_abs_difference(a, b):
    """Largest elementwise absolute difference between two 4x4 transforms."""
    return max(abs(a[i][j] - b[i][j]) for i in range(4) for j in range(4))


# --- The G1 chain ------------------------------------------------------------

def body_to_mid360():
    """``body -> mid360_link``: FAST-LIO2's IMU-to-LiDAR extrinsic, identity rotation."""
    return transform(BODY_TO_MID360_XYZ, BODY_TO_MID360_RPY)


def mid360_to_body():
    """``mid360_link -> body``: the inverse of :func:`body_to_mid360`."""
    return invert(body_to_mid360())


def pelvis_to_torso(waist_yaw):
    """``pelvis -> torso_link`` at a given ``waist_yaw_joint`` angle (radians).

    This is the joint that makes the sensor-to-robot transform dynamic.
    """
    return transform(WAIST_YAW_ORIGIN_XYZ, (0.0, 0.0, float(waist_yaw)))


def torso_to_mid360():
    """``torso_link -> mid360_link``: the fixed, upside-down sensor mount."""
    return transform(TORSO_TO_MID360_XYZ, TORSO_TO_MID360_RPY)


def pelvis_to_mid360(waist_yaw):
    """``pelvis -> mid360_link`` through the waist joint and the sensor mount."""
    return multiply(pelvis_to_torso(waist_yaw), torso_to_mid360())


def mid360_to_pelvis(waist_yaw):
    """``mid360_link -> pelvis``: the inverse of :func:`pelvis_to_mid360`."""
    return invert(pelvis_to_mid360(waist_yaw))


def body_to_pelvis(waist_yaw):
    """``body -> pelvis``: from FAST-LIO2's tracked IMU frame to the URDF root."""
    return multiply(body_to_mid360(), mid360_to_pelvis(waist_yaw))


def world_to_pelvis(world_to_body, waist_yaw):
    """``camera_init -> pelvis``, the transform g1_tf_publisher broadcasts.

    ``world_to_body`` is FAST-LIO2's ``camera_init -> body`` pose, normally built
    from a ``nav_msgs/Odometry`` message with :func:`transform_from_pose`.
    """
    return multiply(world_to_body, body_to_pelvis(waist_yaw))


# --- URDF cross-checks -------------------------------------------------------

def _floats(text, count=3):
    parts = text.split()
    if len(parts) != count:
        raise ValueError("expected {} numbers, got {!r}".format(count, text))
    return tuple(float(p) for p in parts)


def read_joint(urdf_path, joint_name):
    """Return a dict describing one top-level ``<joint>``, or None if absent.

    Only direct children of ``<robot>`` are considered, so ``<transmission>``
    blocks that also carry a ``<joint name=...>`` cannot shadow a real joint.
    """
    root = ET.parse(urdf_path).getroot()
    for joint in root.findall("joint"):
        if joint.get("name") != joint_name:
            continue
        origin = joint.find("origin")
        axis = joint.find("axis")
        parent = joint.find("parent")
        child = joint.find("child")
        return {
            "name": joint_name,
            "type": joint.get("type"),
            "parent": parent.get("link") if parent is not None else None,
            "child": child.get("link") if child is not None else None,
            "xyz": _floats(origin.get("xyz", "0 0 0")) if origin is not None else (0.0, 0.0, 0.0),
            "rpy": _floats(origin.get("rpy", "0 0 0")) if origin is not None else (0.0, 0.0, 0.0),
            "axis": _floats(axis.get("xyz", "0 0 0")) if axis is not None else None,
        }
    return None


def joint_edges(urdf_path):
    """``[(parent_link, child_link, joint_name, joint_type), ...]`` for the URDF."""
    root = ET.parse(urdf_path).getroot()
    edges = []
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        edges.append((parent.get("link"), child.get("link"),
                      joint.get("name"), joint.get("type")))
    return edges


def verify_against_urdf(urdf_path, tolerance=1e-9):
    """Check the constants above still match the URDF. Returns a list of problems.

    An empty list means the vendored URDF agrees with this module. Anything else
    means the URDF changed underneath us and the chain math is stale.
    """
    problems = []
    for joint_name, expected_type in sorted(URDF_REQUIRED_JOINTS.items()):
        try:
            joint = read_joint(urdf_path, joint_name)
        except Exception as exc:                      # malformed XML, bad numbers
            problems.append("{}: cannot read joint {}: {}".format(urdf_path, joint_name, exc))
            continue
        if joint is None:
            problems.append("joint {!r} is missing from {}".format(joint_name, urdf_path))
            continue
        if joint["type"] != expected_type:
            problems.append("joint {!r} has type {!r}, expected {!r}".format(
                joint_name, joint["type"], expected_type))

    waist = None
    mid360 = None
    try:
        waist = read_joint(urdf_path, "waist_yaw_joint")
        mid360 = read_joint(urdf_path, "mid360_joint")
    except Exception:
        return problems

    if waist is not None:
        if (waist["parent"], waist["child"]) != (ROBOT_ROOT_FRAME, TORSO_FRAME):
            problems.append("waist_yaw_joint links {} -> {}, expected {} -> {}".format(
                waist["parent"], waist["child"], ROBOT_ROOT_FRAME, TORSO_FRAME))
        if _max_diff(waist["xyz"], WAIST_YAW_ORIGIN_XYZ) > tolerance:
            problems.append("waist_yaw_joint origin xyz {} != WAIST_YAW_ORIGIN_XYZ {}".format(
                waist["xyz"], WAIST_YAW_ORIGIN_XYZ))
        if _max_diff(waist["rpy"], (0.0, 0.0, 0.0)) > tolerance:
            problems.append("waist_yaw_joint origin rpy {} is not zero; the chain math "
                            "assumes the joint rotation is the only rotation".format(waist["rpy"]))
        if waist["axis"] is not None and _max_diff(waist["axis"], WAIST_YAW_AXIS) > tolerance:
            problems.append("waist_yaw_joint axis {} != WAIST_YAW_AXIS {}".format(
                waist["axis"], WAIST_YAW_AXIS))

    if mid360 is not None:
        if (mid360["parent"], mid360["child"]) != (TORSO_FRAME, LIDAR_FRAME):
            problems.append("mid360_joint links {} -> {}, expected {} -> {}".format(
                mid360["parent"], mid360["child"], TORSO_FRAME, LIDAR_FRAME))
        if _max_diff(mid360["xyz"], TORSO_TO_MID360_XYZ) > tolerance:
            problems.append("mid360_joint origin xyz {} != TORSO_TO_MID360_XYZ {}".format(
                mid360["xyz"], TORSO_TO_MID360_XYZ))
        if _max_diff(mid360["rpy"], TORSO_TO_MID360_RPY) > tolerance:
            problems.append("mid360_joint origin rpy {} != TORSO_TO_MID360_RPY {}".format(
                mid360["rpy"], TORSO_TO_MID360_RPY))

    return problems


def _max_diff(a, b):
    return max(abs(float(x) - float(y)) for x, y in zip(a, b))
