#!/usr/bin/env python3
"""Transform-direction tests for the G1 TF chain.

The one thing these tests exist to prevent is a silently inverted extrinsic.
In ROS TF convention, with FAST-LIO2's mapping/extrinsic_T read as "the LiDAR
pose expressed in the IMU frame":

    body -> mid360_link  ==  translation (-0.011, -0.02329, 0.04412), identity rotation
    mid360_link -> body  ==  its inverse,  (+0.011, +0.02329, -0.04412)

A sign flip anywhere in that chain puts the robot base on the wrong side of the
sensor, so the direction is pinned here rather than left to a comment.
"""
import math
import os
import unittest

from g1_slam import tf_chain

# The authoritative direction, restated literally so a refactor of tf_chain's
# constants cannot quietly redefine what the test is asserting.
BODY_TO_MID360 = (-0.011, -0.02329, 0.04412)
MID360_TO_BODY = (0.011, 0.02329, -0.04412)

TOL = 1e-12
URDF_RELPATH = os.path.join("urdf", "g1_23dof_mode_10.urdf")
FASTLIO_RELPATH = os.path.join("config", "fastlio.yaml")


def _package_root():
    """The g1_slam source directory (this file lives in <root>/test/)."""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class TestExtrinsicDirection(unittest.TestCase):
    """body -> mid360_link and its inverse."""

    def test_body_to_mid360_translation(self):
        got = tf_chain.translation_of(tf_chain.body_to_mid360())
        for axis, (g, want) in enumerate(zip(got, BODY_TO_MID360)):
            self.assertAlmostEqual(g, want, delta=TOL,
                                   msg="body->mid360_link axis {} is {}, expected {}".format(
                                       axis, g, want))

    def test_body_to_mid360_rotation_is_identity(self):
        rot = tf_chain.rotation_of(tf_chain.body_to_mid360())
        for roll_pitch_yaw in tf_chain.rpy_from_rotation(rot):
            self.assertAlmostEqual(roll_pitch_yaw, 0.0, delta=TOL)
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(rot[i][j], 1.0 if i == j else 0.0, delta=TOL)

    def test_mid360_to_body_is_the_inverse(self):
        got = tf_chain.translation_of(tf_chain.mid360_to_body())
        for axis, (g, want) in enumerate(zip(got, MID360_TO_BODY)):
            self.assertAlmostEqual(g, want, delta=TOL,
                                   msg="mid360_link->body axis {} is {}, expected {}".format(
                                       axis, g, want))

    def test_round_trip_is_identity(self):
        composed = tf_chain.multiply(tf_chain.body_to_mid360(), tf_chain.mid360_to_body())
        self.assertLess(tf_chain.max_abs_difference(composed, tf_chain.identity()), 1e-12)

    def test_not_self_inverse(self):
        """A round-trip test alone would pass even if both directions were equal."""
        forward = tf_chain.translation_of(tf_chain.body_to_mid360())
        backward = tf_chain.translation_of(tf_chain.mid360_to_body())
        self.assertGreater(max(abs(f - b) for f, b in zip(forward, backward)), 1e-6,
                           "body->mid360_link and mid360_link->body must differ in sign")
        for f, b in zip(forward, backward):
            self.assertAlmostEqual(f, -b, delta=TOL)


class TestMatchesFastlioConfig(unittest.TestCase):
    """The constants must not drift from config/fastlio.yaml."""

    def setUp(self):
        try:
            import yaml
        except ImportError:                                   # pragma: no cover
            self.skipTest("PyYAML not available")
        path = os.path.join(_package_root(), FASTLIO_RELPATH)
        if not os.path.isfile(path):
            self.skipTest("fastlio.yaml not found at {}".format(path))
        with open(path) as handle:
            self.mapping = yaml.safe_load(handle)["mapping"]

    def test_extrinsic_t_matches_body_to_mid360(self):
        for axis, (cfg, want) in enumerate(zip(self.mapping["extrinsic_T"], BODY_TO_MID360)):
            self.assertAlmostEqual(float(cfg), want, delta=TOL,
                                   msg="fastlio.yaml extrinsic_T axis {} drifted".format(axis))

    def test_extrinsic_r_is_identity(self):
        """Identity even though the Mid-360 is mounted upside down.

        extrinsic_R is the LiDAR-to-IMU relationship inside the sensor housing,
        which does not change with mounting. The 180 degree roll of the inverted
        mount belongs to the URDF's mid360_joint, checked separately below.
        """
        flat = [float(v) for v in self.mapping["extrinsic_R"]]
        self.assertEqual(len(flat), 9)
        self.assertEqual(flat, [1, 0, 0, 0, 1, 0, 0, 0, 1])


class TestPelvisChain(unittest.TestCase):
    """The waist joint makes body -> pelvis dynamic, not static."""

    def test_body_to_pelvis_depends_on_waist_yaw(self):
        at_zero = tf_chain.body_to_pelvis(0.0)
        at_thirty = tf_chain.body_to_pelvis(math.radians(30.0))
        self.assertGreater(tf_chain.max_abs_difference(at_zero, at_thirty), 1e-3,
                           "body -> pelvis must change with waist_yaw; a static "
                           "transform would be wrong on the real robot")

    def test_waist_yaw_error_is_mostly_heading(self):
        """Freezing waist_yaw at 0 costs ~1 degree of yaw per degree of joint error."""
        at_zero = tf_chain.body_to_pelvis(0.0)
        at_thirty = tf_chain.body_to_pelvis(math.radians(30.0))
        yaw_zero = tf_chain.rpy_from_rotation(tf_chain.rotation_of(at_zero))[2]
        yaw_thirty = tf_chain.rpy_from_rotation(tf_chain.rotation_of(at_thirty))[2]
        self.assertAlmostEqual(abs(math.degrees(yaw_zero - yaw_thirty)), 30.0, delta=0.5)

    def test_pelvis_to_mid360_round_trips(self):
        for degrees in (-90.0, -30.0, 0.0, 15.0, 150.0):
            radians = math.radians(degrees)
            composed = tf_chain.multiply(tf_chain.pelvis_to_mid360(radians),
                                         tf_chain.mid360_to_pelvis(radians))
            self.assertLess(tf_chain.max_abs_difference(composed, tf_chain.identity()), 1e-12)

    def test_world_to_pelvis_composes_odometry(self):
        """camera_init -> pelvis with identity odometry equals body -> pelvis."""
        composed = tf_chain.world_to_pelvis(tf_chain.identity(), 0.25)
        self.assertLess(tf_chain.max_abs_difference(composed, tf_chain.body_to_pelvis(0.25)),
                        1e-12)

    def test_quaternion_round_trip(self):
        """quaternion_of / rotation_from_quaternion must agree with the rpy path."""
        for rpy in ((0.0, 0.0, 0.0), (math.pi, 0.05112069379091391, 0.0), (0.3, -0.2, 1.1)):
            original = tf_chain.transform((1.0, 2.0, 3.0), rpy)
            quat = tf_chain.quaternion_of(original)
            rebuilt = tf_chain.transform_from_pose((1.0, 2.0, 3.0), quat)
            self.assertLess(tf_chain.max_abs_difference(original, rebuilt), 1e-9)


class TestVendoredUrdfAgrees(unittest.TestCase):
    """The vendored URDF must still match tf_chain's joint constants."""

    def setUp(self):
        self.urdf = os.path.join(_package_root(), URDF_RELPATH)
        if not os.path.isfile(self.urdf):
            self.fail("vendored URDF missing at {}".format(self.urdf))

    def test_constants_match_urdf(self):
        problems = tf_chain.verify_against_urdf(self.urdf)
        self.assertEqual(problems, [], "; ".join(problems))

    def test_waist_yaw_joint_is_revolute(self):
        joint = tf_chain.read_joint(self.urdf, "waist_yaw_joint")
        self.assertIsNotNone(joint)
        self.assertEqual(joint["type"], "revolute")
        self.assertEqual((joint["parent"], joint["child"]), ("pelvis", "torso_link"))

    def test_mid360_joint_is_fixed_and_inverted(self):
        joint = tf_chain.read_joint(self.urdf, "mid360_joint")
        self.assertIsNotNone(joint)
        self.assertEqual(joint["type"], "fixed")
        self.assertEqual((joint["parent"], joint["child"]), ("torso_link", "mid360_link"))
        self.assertAlmostEqual(abs(joint["rpy"][0]), math.pi, delta=1e-9,
                               msg="mid360_joint roll should be pi (upside-down mount)")

    def test_urdf_root_is_pelvis(self):
        """pelvis must be the only link that is never a joint child."""
        edges = tf_chain.joint_edges(self.urdf)
        children = {child for _parent, child, _name, _type in edges}
        parents = {parent for parent, _child, _name, _type in edges}
        roots = sorted(parents - children)
        self.assertEqual(roots, ["pelvis"])

    def test_urdf_has_single_parent_per_link(self):
        seen = {}
        for parent, child, name, _type in tf_chain.joint_edges(self.urdf):
            self.assertNotIn(child, seen,
                             "{} is parented by both {} and {} (joint {})".format(
                                 child, seen.get(child), parent, name))
            seen[child] = parent

    def test_urdf_does_not_define_base_link(self):
        """base_link is an unresolved integration decision - nothing may invent it."""
        edges = tf_chain.joint_edges(self.urdf)
        links = {p for p, _c, _n, _t in edges} | {c for _p, c, _n, _t in edges}
        self.assertNotIn("base_link", links)


if __name__ == "__main__":
    unittest.main()
