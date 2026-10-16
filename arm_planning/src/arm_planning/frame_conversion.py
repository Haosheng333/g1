"""base_link -> the planner's torso_link. Pure maths, no ROS, no TF client.

The caller (arm_planning_node) performs the TF lookup and hands the result in,
so every rule here is testable offline with hand-computed expectations.

=====================================================================
THE TWO torso_link CONVENTIONS -- the 10 mm, resolved explicitly
=====================================================================
The live TF tree is published by g1_slam from
`slam/urdf/g1_23dof_mode_10.urdf`. The planner's numbers in
`arm_planning/config/arm_kinematics.yaml` are derived from
`arm_planning/urdf/g1_23dof.urdf`. They disagree about where `torso_link` sits:

    joint / link chain                 mode_10      g1_23dof
    pelvis -> torso_link  (waist_yaw)   0.044        0.054      <- differ by 10 mm
    torso_link -> shoulder_pitch        0.24778      0.23778    <- differ by 10 mm
    pelvis -> shoulder_pitch            0.29178      0.29178    <- IDENTICAL

Neither URDF is wrong. The arm geometry relative to the pelvis is the same in
both; the two files simply place the `torso_link` DATUM 10 mm apart, and each
compensates in the next link. So this is a convention difference, not a
modelling error, and the fix is a declared convention transform -- NOT a
redefinition of the TF frame and NOT an edit to either URDF.

Both waist_yaw_joint origins have `rpy = 0 0 0` and the joint rotates about z,
so the 10 mm offset is parallel to z at EVERY waist angle and the two torso
frames always share orientation. The correction is therefore a pure translation,
exact at every waist yaw -- not a small-angle approximation.

DIRECTION OF THE CORRECTION. The planner's torso_link origin sits 10 mm HIGHER
than the TF one (0.054 > 0.044). A point's coordinates are measured FROM the
origin, so moving the origin up by 10 mm makes the point's z 10 mm SMALLER:

    p_planner = p_tf_torso - [0, 0, +0.010]

x and y are identical in both URDFs (-0.0039635, 0), so only z is corrected.

=====================================================================
WHAT THIS MODULE DOES NOT ESTABLISH
=====================================================================
It converts a point that is ALREADY correct in base_link. It cannot repair a
point computed with wrong camera extrinsics: perception's `R_base_opt` is
hard-coded for a torso-mounted D435 on a different robot, that camera DOES
rotate with the waist, and the matrix is not orthonormal (det = 1.0003760).
Those are perception's prerequisites and are tracked separately as the
`perception_extrinsics` capability. A correct base->planner transform is
necessary, never sufficient.
"""
from __future__ import annotations

import math

#: Source frame perception reports in. An identity child of pelvis, published
#: once by system_bringup (docs/FRAME_CONTRACT.md).
DEFAULT_SOURCE_FRAME = "base_link"

#: The frame the planner's services accept. Same NAME as the TF frame, different
#: datum -- see the module docstring.
PLANNER_FRAME = "torso_link"

#: waist_yaw_joint origin z in each URDF. Re-derived from both files by
#: test_frame_conversion.py, so a URDF change breaks the test rather than
#: silently shifting every grasp target.
TF_WAIST_Z = 0.044          # slam/urdf/g1_23dof_mode_10.urdf
PLANNER_WAIST_Z = 0.054     # arm_planning/urdf/g1_23dof.urdf

#: Subtracted from a point already expressed in the TF torso_link.
CONVENTION_OFFSET = (0.0, 0.0, PLANNER_WAIST_Z - TF_WAIST_Z)

#: Default guard on pairing an unstamped observation with a later waist pose.
#: ~1.1 degrees. A requested setting, not a measured tolerance.
DEFAULT_MAX_WAIST_DELTA_RAD = 0.02


class ConversionError(ValueError):
    """The observation or the transform cannot be used as given."""


def _finite_triple(values, what):
    try:
        out = [float(v) for v in values]
    except (TypeError, ValueError):
        raise ConversionError("%s is not a list of numbers: %r" % (what, values))
    if len(out) != 3:
        raise ConversionError("%s must be exactly 3 values, got %d" % (what, len(out)))
    if not all(math.isfinite(v) for v in out):
        raise ConversionError("%s contains a non-finite value: %r" % (what, out))
    return out


def rotation_from_quaternion(x, y, z, w):
    """3x3 rotation from an (x, y, z, w) quaternion. Rejects a degenerate one."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ConversionError("transform carries a degenerate quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)),
        (2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)),
    )


def yaw_from_quaternion(x, y, z, w):
    """Z yaw of a quaternion, radians. The waist angle carried by the transform."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def apply_transform(point, translation, quaternion):
    """Map `point` through a target<-source transform. Returns the target point.

    `translation` and `quaternion` are exactly what tf2's
    lookup_transform(target, source, t).transform carries, so
        p_target = R * p_source + t
    """
    p = _finite_triple(point, "point")
    t = _finite_triple(translation, "transform translation")
    rot = rotation_from_quaternion(*quaternion)
    return [sum(rot[i][j] * p[j] for j in range(3)) + t[i] for i in range(3)]


def tf_torso_to_planner(point_tf_torso):
    """Apply ONLY the convention correction, TF torso_link -> planner torso_link."""
    p = _finite_triple(point_tf_torso, "point")
    return [p[i] - CONVENTION_OFFSET[i] for i in range(3)]


def planner_to_tf_torso(point_planner):
    """The inverse convention correction. For tests and diagnostics."""
    p = _finite_triple(point_planner, "point")
    return [p[i] + CONVENTION_OFFSET[i] for i in range(3)]


def convert(point, translation, quaternion, source_frame=DEFAULT_SOURCE_FRAME):
    """`point` in `source_frame` -> the planner's torso_link.

    The transform must be the TF `torso_link <- source_frame` one; this applies
    it and then the fixed convention correction.
    """
    in_tf_torso = apply_transform(point, translation, quaternion)
    return tf_torso_to_planner(in_tf_torso), in_tf_torso


def waist_delta(a_rad, b_rad):
    """Smallest absolute angular difference, handling wrap-around."""
    d = abs(float(a_rad) - float(b_rad)) % (2.0 * math.pi)
    return min(d, 2.0 * math.pi - d)


def check_waist_consistency(waist_at_stamp_rad, waist_latest_rad, max_delta_rad=None,
                            stamp_is_observation_time=True):
    """Refuse to pair an observation with a waist pose it did not belong to.

    Returns "" when the pairing is defensible, else the reason it is not.

    STATELESS BY DESIGN. Both angles come from the SAME TF history -- the
    transform at the observation's stamp and the latest one -- so the verdict
    depends only on this request. An earlier version compared against the waist
    seen by the previous conversion, which coupled unrelated callers and made
    the answer depend on call order.

    When the producer timestamps its output there is nothing to guard: the
    transform at that stamp is the right one. perception does NOT timestamp
    (GetObjectPosition.srv has no stamp field), so its caller must pass an upper
    bound with stamp_is_observation_time=false, and the only honest check is
    that the waist did not move across that window.
    """
    if stamp_is_observation_time:
        return ""
    if waist_at_stamp_rad is None:
        return ("the observation is not timestamped at source and no transform is "
                "available for its receipt time, so the waist angle when it was taken "
                "cannot be established")
    limit = DEFAULT_MAX_WAIST_DELTA_RAD if not max_delta_rad or max_delta_rad <= 0 \
        else float(max_delta_rad)
    delta = waist_delta(waist_at_stamp_rad, waist_latest_rad)
    if delta > limit:
        return ("the waist moved %.4f rad (limit %.4f) between the observation's "
                "receipt time and now. perception's GetObjectPosition.srv carries no "
                "timestamp, so an old observation cannot be paired with a newer waist "
                "pose; refusing rather than guessing." % (delta, limit))
    return ""
