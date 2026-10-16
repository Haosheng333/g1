"""base_link -> planner torso_link: offline tests with independent expectations.

Every expected value here is computed from the URDF numbers by a SEPARATE
derivation written out in the test, not by calling the module under test. A
round-trip identity would pass even if the convention offset were applied in the
wrong direction, or omitted, so round-trips are only a supplement.

THE GEOMETRY, derived here independently:
  base_link is an identity child of pelvis (system_bringup), so p_base == p_pelvis.
  pelvis -> torso_link is the waist_yaw joint: translate by [-0.0039635, 0, Z]
  then rotate about z by the waist angle. Therefore

      p_tf_torso = Rz(-theta) * (p_pelvis - [-0.0039635, 0, 0.044])

  with Z = 0.044 for the TF tree (slam/urdf/g1_23dof_mode_10.urdf). The planner's
  frame uses Z = 0.054 (arm_planning/urdf/g1_23dof.urdf), so

      p_planner  = Rz(-theta) * (p_pelvis - [-0.0039635, 0, 0.054])
                 = p_tf_torso - [0, 0, 0.010]

  the second form holding at EVERY theta because the waist rotates about z.
"""
import math
import os
import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from arm_planning import frame_conversion as fc  # noqa: E402

PKG = Path(os.path.dirname(os.path.abspath(__file__))).parent
REPO = PKG.parent

WAIST_XY = (-0.0039635, 0.0)
TF_Z = 0.044
PLANNER_Z = 0.054


# --- independent reference implementation, written from the URDF numbers -------------
def ref_tf_transform(theta):
    """(translation, quaternion) for TF `torso_link <- base_link` at waist `theta`.

    Mirrors what tf2 would return: p_torso = R*p_base + t.
    pelvis->torso is T = Trans([-0.0039635,0,0.044]) * Rz(theta); the lookup is
    its inverse, so R = Rz(-theta) and t = -Rz(-theta) * [-0.0039635,0,0.044].
    """
    c, s = math.cos(-theta), math.sin(-theta)
    origin = (WAIST_XY[0], WAIST_XY[1], TF_Z)
    t = (-(c * origin[0] - s * origin[1]),
         -(s * origin[0] + c * origin[1]),
         -origin[2])
    quat = (0.0, 0.0, math.sin(-theta / 2.0), math.cos(-theta / 2.0))
    return t, quat


def ref_planner_point(point, theta):
    """p_base -> p_planner, computed directly from the g1_23dof origin."""
    dx = point[0] - WAIST_XY[0]
    dy = point[1] - WAIST_XY[1]
    dz = point[2] - PLANNER_Z
    c, s = math.cos(-theta), math.sin(-theta)
    return [c * dx - s * dy, s * dx + c * dy, dz]


def ref_tf_torso_point(point, theta):
    dx = point[0] - WAIST_XY[0]
    dy = point[1] - WAIST_XY[1]
    dz = point[2] - TF_Z
    c, s = math.cos(-theta), math.sin(-theta)
    return [c * dx - s * dy, s * dx + c * dy, dz]


class TheOffsetComesFromTheUrdfs(unittest.TestCase):
    """Re-derive the 10 mm from both files, so a URDF edit breaks this test."""

    def _waist_origin(self, urdf):
        text = Path(urdf).read_text(encoding="utf-8")
        m = re.search(r'<joint name="waist_yaw_joint"[^>]*>\s*<origin xyz="([^"]+)"', text)
        self.assertIsNotNone(m, "waist_yaw_joint origin not found in %s" % urdf)
        return [float(v) for v in m.group(1).split()]

    def _shoulder_origin(self, urdf):
        text = Path(urdf).read_text(encoding="utf-8")
        m = re.search(r'<joint name="right_shoulder_pitch_joint"[^>]*>\s*<origin xyz="([^"]+)"',
                      text)
        self.assertIsNotNone(m)
        return [float(v) for v in m.group(1).split()]

    def test_the_two_urdfs_place_torso_link_ten_mm_apart(self):
        tf = self._waist_origin(REPO / "slam" / "urdf" / "g1_23dof_mode_10.urdf")
        pl = self._waist_origin(PKG / "urdf" / "g1_23dof.urdf")
        self.assertAlmostEqual(tf[2], fc.TF_WAIST_Z, places=9)
        self.assertAlmostEqual(pl[2], fc.PLANNER_WAIST_Z, places=9)
        self.assertAlmostEqual(pl[2] - tf[2], 0.010, places=9)
        # x and y agree, so the correction is z-only.
        self.assertAlmostEqual(tf[0], pl[0], places=9)
        self.assertAlmostEqual(tf[1], pl[1], places=9)

    def test_neither_urdf_is_wrong_pelvis_to_shoulder_is_identical(self):
        """The evidence that this is a CONVENTION difference, not a modelling error."""
        tf_w = self._waist_origin(REPO / "slam" / "urdf" / "g1_23dof_mode_10.urdf")
        pl_w = self._waist_origin(PKG / "urdf" / "g1_23dof.urdf")
        tf_s = self._shoulder_origin(REPO / "slam" / "urdf" / "g1_23dof_mode_10.urdf")
        pl_s = self._shoulder_origin(PKG / "urdf" / "g1_23dof.urdf")
        self.assertAlmostEqual(tf_w[2] + tf_s[2], pl_w[2] + pl_s[2], places=9)
        self.assertAlmostEqual(tf_w[2] + tf_s[2], 0.29178, places=9)

    def test_the_planner_config_uses_the_g1_23dof_convention(self):
        """arm_kinematics.yaml's shoulder z proves which torso_link it means."""
        import yaml
        cfg = yaml.safe_load((PKG / "config" / "arm_kinematics.yaml").read_text())
        self.assertAlmostEqual(cfg["right"]["origins_xyz"][0][2], 0.23778, places=9)
        self.assertEqual(cfg["base_link"], fc.PLANNER_FRAME)

    def test_module_offset_matches_the_urdf_difference(self):
        self.assertEqual(fc.CONVENTION_OFFSET[0], 0.0)
        self.assertEqual(fc.CONVENTION_OFFSET[1], 0.0)
        self.assertAlmostEqual(fc.CONVENTION_OFFSET[2], 0.010, places=9)


class KnownPointsAtZeroWaist(unittest.TestCase):
    def test_pelvis_origin(self):
        got, in_tf = fc.convert([0.0, 0.0, 0.0], *ref_tf_transform(0.0))
        # Independent: [0 - (-0.0039635), 0, 0 - 0.054]
        self.assertAlmostEqual(got[0], 0.0039635, places=9)
        self.assertAlmostEqual(got[1], 0.0, places=9)
        self.assertAlmostEqual(got[2], -0.054, places=9)
        self.assertAlmostEqual(in_tf[2], -0.044, places=9)

    def test_a_table_height_observation(self):
        """A plausible detection: 0.42 m forward, 0.13 m right, 0.91 m up."""
        point = [0.412, -0.133, 0.907]
        got, _ = fc.convert(point, *ref_tf_transform(0.0))
        expected = ref_planner_point(point, 0.0)
        for axis in range(3):
            self.assertAlmostEqual(got[axis], expected[axis], places=9)
        # Spelled out, so the numbers are visible in the test itself:
        self.assertAlmostEqual(got[0], 0.4159635, places=7)
        self.assertAlmostEqual(got[1], -0.133, places=7)
        self.assertAlmostEqual(got[2], 0.853, places=7)

    def test_the_offset_is_not_silently_omitted(self):
        """Without the correction, z would be 0.863 rather than 0.853."""
        point = [0.412, -0.133, 0.907]
        got, in_tf = fc.convert(point, *ref_tf_transform(0.0))
        self.assertAlmostEqual(in_tf[2] - got[2], 0.010, places=9)
        self.assertNotAlmostEqual(got[2], in_tf[2], places=5)


class NonZeroWaistYaw(unittest.TestCase):
    def test_several_waist_angles_match_the_independent_derivation(self):
        point = [0.412, -0.133, 0.907]
        for deg in (-90.0, -30.0, -5.0, 5.0, 15.0, 30.0, 90.0, 150.0):
            theta = math.radians(deg)
            got, in_tf = fc.convert(point, *ref_tf_transform(theta))
            expected = ref_planner_point(point, theta)
            for axis in range(3):
                self.assertAlmostEqual(got[axis], expected[axis], places=9,
                                       msg="waist %+.1f deg axis %d" % (deg, axis))
            expected_tf = ref_tf_torso_point(point, theta)
            for axis in range(3):
                self.assertAlmostEqual(in_tf[axis], expected_tf[axis], places=9)

    def test_the_offset_stays_exactly_ten_mm_in_z_at_every_angle(self):
        """Because the waist rotates about z, this is exact, not an approximation."""
        point = [0.3, 0.2, 0.8]
        for deg in range(-180, 181, 15):
            theta = math.radians(deg)
            got, in_tf = fc.convert(point, *ref_tf_transform(theta))
            self.assertAlmostEqual(in_tf[0] - got[0], 0.0, places=12, msg=str(deg))
            self.assertAlmostEqual(in_tf[1] - got[1], 0.0, places=12, msg=str(deg))
            self.assertAlmostEqual(in_tf[2] - got[2], 0.010, places=12, msg=str(deg))

    def test_a_nonzero_waist_actually_changes_the_answer(self):
        """Guards against a transform that ignores its rotation."""
        point = [0.412, -0.133, 0.907]
        a, _ = fc.convert(point, *ref_tf_transform(0.0))
        b, _ = fc.convert(point, *ref_tf_transform(math.radians(30.0)))
        self.assertGreater(max(abs(x - y) for x, y in zip(a, b)), 0.05)

    def test_waist_yaw_is_recovered_from_the_quaternion(self):
        for deg in (-120.0, -45.0, 0.0, 45.0, 120.0):
            theta = math.radians(deg)
            _, quat = ref_tf_transform(theta)
            # The lookup's rotation is Rz(-theta), so its yaw is -theta.
            self.assertAlmostEqual(fc.yaw_from_quaternion(*quat), -theta, places=9)


class TransformDirection(unittest.TestCase):
    def test_the_inverse_offset_is_the_opposite_sign(self):
        p = [0.1, 0.2, 0.3]
        self.assertAlmostEqual(fc.tf_torso_to_planner(p)[2], 0.29, places=9)
        self.assertAlmostEqual(fc.planner_to_tf_torso(p)[2], 0.31, places=9)

    def test_offset_round_trip_is_identity_but_is_not_the_real_check(self):
        p = [0.1, 0.2, 0.3]
        back = fc.planner_to_tf_torso(fc.tf_torso_to_planner(p))
        for axis in range(3):
            self.assertAlmostEqual(back[axis], p[axis], places=12)
        # And the two directions genuinely differ, which a round trip alone
        # would not reveal.
        self.assertNotAlmostEqual(fc.tf_torso_to_planner(p)[2],
                                  fc.planner_to_tf_torso(p)[2], places=6)

    def test_applying_the_transform_the_wrong_way_gives_a_different_answer(self):
        """A target<-source transform is not symmetric; prove we notice."""
        point = [0.412, -0.133, 0.907]
        theta = math.radians(40.0)
        t, q = ref_tf_transform(theta)
        right, _ = fc.convert(point, t, q)
        # Deliberately conjugate the quaternion and negate t: the wrong direction.
        wrong = fc.apply_transform(point, [-v for v in t], (q[0], q[1], -q[2], q[3]))
        self.assertGreater(max(abs(a - b) for a, b in zip(right, wrong)), 0.01)


class BadInputIsRefused(unittest.TestCase):
    def test_a_short_point_is_refused(self):
        with self.assertRaises(fc.ConversionError):
            fc.convert([0.1, 0.2], *ref_tf_transform(0.0))

    def test_a_non_finite_point_is_refused(self):
        with self.assertRaises(fc.ConversionError):
            fc.convert([0.1, float("nan"), 0.3], *ref_tf_transform(0.0))

    def test_a_non_numeric_point_is_refused(self):
        with self.assertRaises(fc.ConversionError):
            fc.convert([0.1, "x", 0.3], *ref_tf_transform(0.0))

    def test_a_degenerate_quaternion_is_refused(self):
        with self.assertRaises(fc.ConversionError):
            fc.convert([0.1, 0.2, 0.3], [0, 0, 0], (0.0, 0.0, 0.0, 0.0))

    def test_a_non_finite_translation_is_refused(self):
        with self.assertRaises(fc.ConversionError):
            fc.convert([0.1, 0.2, 0.3], [0, float("inf"), 0], (0, 0, 0, 1))


class StaleOrMissingTransformData(unittest.TestCase):
    """The waist guard for an observation that carries no timestamp.

    Stateless: both angles come from the same request's TF history -- the
    transform at the observation's stamp and the latest one.
    """

    def test_a_still_waist_is_accepted(self):
        self.assertEqual(
            fc.check_waist_consistency(0.30, 0.30, stamp_is_observation_time=False), "")

    def test_a_small_wobble_inside_the_limit_is_accepted(self):
        self.assertEqual(
            fc.check_waist_consistency(0.300, 0.310, stamp_is_observation_time=False), "")

    def test_a_moved_waist_is_refused(self):
        reason = fc.check_waist_consistency(0.10, 0.30, stamp_is_observation_time=False)
        self.assertIn("waist moved", reason)
        self.assertIn("GetObjectPosition", reason)

    def test_the_limit_is_configurable(self):
        self.assertEqual(
            fc.check_waist_consistency(0.10, 0.30, max_delta_rad=1.0,
                                       stamp_is_observation_time=False), "")

    def test_no_transform_at_the_observation_time_is_refused(self):
        reason = fc.check_waist_consistency(None, 0.30, stamp_is_observation_time=False)
        self.assertIn("cannot be established", reason)

    def test_a_properly_stamped_observation_needs_no_guard(self):
        self.assertEqual(
            fc.check_waist_consistency(None, 0.30, stamp_is_observation_time=True), "")
        self.assertEqual(
            fc.check_waist_consistency(0.10, 2.90, stamp_is_observation_time=True), "")

    def test_wrap_around_is_not_counted_as_motion(self):
        """+179 deg to -179 deg is 2 deg of motion, not 358."""
        self.assertEqual(
            fc.check_waist_consistency(math.radians(179.0), math.radians(-179.0),
                                       max_delta_rad=math.radians(5.0),
                                       stamp_is_observation_time=False), "")

    def test_waist_delta_handles_wrap_around(self):
        self.assertAlmostEqual(
            fc.waist_delta(math.radians(179.0), math.radians(-179.0)),
            math.radians(2.0), places=9)
        self.assertAlmostEqual(fc.waist_delta(0.1, 0.4), 0.3, places=9)

    def test_the_guard_is_order_independent(self):
        """Same inputs, same verdict, regardless of any earlier call."""
        for _ in range(3):
            self.assertEqual(
                fc.check_waist_consistency(0.30, 0.30, stamp_is_observation_time=False), "")
            self.assertIn("waist moved", fc.check_waist_consistency(
                0.10, 0.90, stamp_is_observation_time=False))


class ThisDoesNotEstablishPerceptionCorrectness(unittest.TestCase):
    def test_the_module_says_so_explicitly(self):
        """Requirement 7: a correct base->planner transform is not sufficient."""
        doc = fc.__doc__
        self.assertIn("cannot repair", doc)
        self.assertIn("perception_extrinsics", doc)


if __name__ == "__main__":
    unittest.main()
