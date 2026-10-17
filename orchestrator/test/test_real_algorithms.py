#!/usr/bin/env python3
"""Part C: exercise the REAL production algorithms, without hardware.

Cross-package by design: this is an integration-level check that the pipeline's
DATA PROCESSING works, so it reaches into perception, arm_planning and
navigation rather than testing one package in isolation.

WHAT IS REAL HERE AND WHAT IS NOT -- read before quoting any result:

  * perception GEOMETRY  real production functions (backproject_pixel,
                         push_along_ray, optical_to_base). The YOLO detector is
                         NOT run: `ultralytics` is absent, and m1_perception
                         imports it at module scope, so the module is imported
                         with a stub in sys.modules. Detections are supplied as
                         known pixel coordinates. This tests the CONVERSION
                         CODE. It says nothing about camera calibration or
                         physical geometry, which need a real camera.
  * arm_planning FK      real production forward kinematics (numpy only),
                         including the corrected tool point.
  * arm_planning IK      BLOCKED: `torch` is absent, so no checkpoint can be
                         loaded and no IK or collision query can run.
  * navigation           the real sim map/semantic fixture is parsed; the
                         planner itself needs scipy+the node, reported below.
  * handover detector    real, pure python, already covered by handover's own
                         155 tests; here with values explicitly labelled in N.

Requires numpy, so run it in the ROS container:
    docker exec g1-ros1 bash -lc 'cd /root/catkin_ws/src/g1/orchestrator &&
        python3 -m unittest test.test_real_algorithms -v'
"""
import math
import os
import sys
import types
import unittest
from pathlib import Path

REPO = Path(os.path.dirname(os.path.abspath(__file__))).parent.parent

BLOCKED = []          # (what, exact reason)


def _blocked(what, reason):
    BLOCKED.append((what, reason))
    return reason


def _have(module):
    try:
        __import__(module)
        return True
    except ImportError:
        return False


# =====================================================================================
class PerceptionGeometryIsReal(unittest.TestCase):
    """The production back-projection chain, with the detector stubbed out."""

    @classmethod
    def setUpClass(cls):
        if not _have("numpy"):
            raise unittest.SkipTest(_blocked("perception geometry", "numpy absent"))
        sys.path.insert(0, str(REPO / "perception" / "src"))
        if not _have("ultralytics"):
            # Stub ONLY the detector import. Every function exercised below is
            # the real production implementation.
            stub = types.ModuleType("ultralytics")
            stub.YOLO = object
            sys.modules.setdefault("ultralytics", stub)
        from perception import m1_perception as m1
        cls.m1 = m1

    def test_backprojection_is_the_pinhole_inverse(self):
        m1 = self.m1
        K = m1.CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)
        # A pixel exactly at the principal point must land on the optical axis.
        p = m1.backproject_pixel(320, 240, 1.5, K)
        self.assertAlmostEqual(float(p[0]), 0.0, places=9)
        self.assertAlmostEqual(float(p[1]), 0.0, places=9)
        self.assertAlmostEqual(float(p[2]), 1.5, places=9)
        # One focal length off-centre at range z gives exactly z metres offset.
        q = m1.backproject_pixel(320 + 600, 240, 2.0, K)
        self.assertAlmostEqual(float(q[0]), 2.0, places=9)

    def test_push_along_ray_moves_the_point_by_the_radius(self):
        import numpy as np
        m1 = self.m1
        p = np.array([0.0, 0.0, 1.0])
        out = m1.push_along_ray(p, 0.033)
        self.assertAlmostEqual(float(np.linalg.norm(out)), 1.033, places=9)

    def test_the_extrinsics_rotation_is_orthonormal(self):
        """R is built from yaw/pitch/roll Euler matrices, so it is a proper
        rotation: optical_to_base applies no scale or shear."""
        import numpy as np
        R = self.m1._default_R_base_opt()
        self.assertLess(float(np.abs(R.T @ R - np.eye(3)).max()), 1e-12)
        self.assertAlmostEqual(float(np.linalg.det(R)), 1.0, places=12)

    def test_optical_to_base_preserves_distance(self):
        """A rigid transform: distances are preserved."""
        import numpy as np
        m1 = self.m1
        ex = m1.Extrinsics()
        a = m1.optical_to_base(np.array([0.0, 0.0, 1.0]), ex)
        b = m1.optical_to_base(np.array([0.0, 0.0, 2.0]), ex)
        self.assertAlmostEqual(float(np.linalg.norm(b - a)), 1.0, places=12)

    def test_optical_to_base_is_translation_plus_linear_map(self):
        """Linearity holds exactly."""
        import numpy as np
        m1 = self.m1
        ex = m1.Extrinsics()
        p, q = np.array([0.1, 0.2, 1.0]), np.array([-0.3, 0.05, 1.7])
        lhs = m1.optical_to_base(p, ex) - m1.optical_to_base(q, ex)
        rhs = m1._default_R_base_opt() @ (p - q)
        self.assertTrue(np.allclose(lhs, rhs, atol=1e-12))

    def test_the_declared_output_frame_is_torso_link(self):
        """Boundary 3's premise: perception labels its output with the frame
        the camera is mounted on, torso_link, and convert_observation takes it
        from there."""
        self.assertEqual(self.m1.BASE_FRAME, "torso_link")
        self.assertEqual(self.m1.ObjectPositionResult(success=True).frame_id, "torso_link")

    def test_median_nonzero_depth_ignores_holes(self):
        import numpy as np
        m1 = self.m1
        depth = np.zeros((20, 20), dtype=float)
        depth[8:12, 8:12] = 1.25          # a valid patch
        depth[10, 10] = 0.0               # one hole inside it
        z = m1.median_nonzero_depth(depth, 10, 10, patch=2)
        self.assertAlmostEqual(z, 1.25, places=6)

    def test_an_all_zero_depth_patch_returns_none_not_a_guess(self):
        import numpy as np
        z = self.m1.median_nonzero_depth(np.zeros((20, 20), dtype=float), 10, 10, patch=2)
        self.assertIsNone(z, "a depth hole must not be back-projected as a number")


# =====================================================================================
class ArmPlanningForwardKinematicsIsReal(unittest.TestCase):
    """Real production FK over the real config, including the corrected offset."""

    @classmethod
    def setUpClass(cls):
        if not _have("numpy"):
            raise unittest.SkipTest(_blocked("arm_planning FK", "numpy absent"))
        sys.path.insert(0, str(REPO / "arm_planning" / "scripts"))
        from g1_forward_kinematics import forward_kinematics, load_config, pose_5d
        # staticmethod: a plain class attribute would be bound as a method
        # and silently receive `self` as the first joint vector.
        cls.fk = staticmethod(forward_kinematics)
        cls.pose_5d = staticmethod(pose_5d)
        cls.cfg = load_config(REPO / "arm_planning" / "config" / "arm_kinematics.yaml")

    def test_config_carries_the_corrected_tool_point(self):
        self.assertAlmostEqual(self.cfg["left"]["grasp_xyz"][1], 0.06, places=9,
                               msg="the 0.006 -> 0.06 correction is not in the config")
        self.assertAlmostEqual(self.cfg["right"]["grasp_xyz"][1], -0.06, places=9)

    def test_fk_returns_a_valid_rigid_transform(self):
        import numpy as np
        T = self.fk(np.zeros(5), "left", self.cfg)
        self.assertEqual(T.shape, (4, 4))
        self.assertTrue(np.allclose(T[3], [0, 0, 0, 1]))
        self.assertTrue(np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-10))

    def test_pose_is_five_values_in_the_agreed_order(self):
        """[x, y, z, rx, rz] -- pose_order in arm_kinematics.yaml."""
        import numpy as np
        self.assertEqual(list(self.cfg["pose_order"]), ["x", "y", "z", "rx", "rz"])
        for arm in ("left", "right"):
            p = self.pose_5d(np.zeros(5), arm, self.cfg)
            self.assertEqual(p.shape, (5,))
            self.assertTrue(np.all(np.isfinite(p)))

    def test_five_joint_ordering_matches_the_config(self):
        for arm in ("left", "right"):
            names = self.cfg[arm]["joint_names"]
            self.assertEqual(len(names), 5)
            self.assertEqual([n.split("_", 1)[1] for n in names], [
                "shoulder_pitch_joint", "shoulder_roll_joint", "shoulder_yaw_joint",
                "elbow_joint", "wrist_roll_joint"])

    def test_fk_is_finite_across_the_whole_joint_range(self):
        import numpy as np
        rng = np.random.default_rng(0)
        for arm in ("left", "right"):
            lo = np.asarray(self.cfg[arm]["lower_limits"])
            hi = np.asarray(self.cfg[arm]["upper_limits"])
            for q in rng.uniform(lo, hi, size=(40, 5)):
                self.assertTrue(np.all(np.isfinite(self.pose_5d(q, arm, self.cfg))))

    def test_out_of_range_joints_are_rejected(self):
        import numpy as np
        q = np.asarray(self.cfg["left"]["upper_limits"]) + 1.0
        with self.assertRaises(ValueError):
            self.fk(q, "left", self.cfg)

    def test_left_right_mirror_holds_with_the_corrected_offset(self):
        import numpy as np
        l = self.pose_5d(np.zeros(5), "left", self.cfg)
        r = self.pose_5d(np.zeros(5), "right", self.cfg)
        self.assertAlmostEqual(l[0], r[0], delta=2e-3)
        self.assertAlmostEqual(l[1], -r[1], delta=2e-3)
        self.assertAlmostEqual(l[2], r[2], delta=2e-3)


# =====================================================================================
class ArmPlanningIkIsBlocked(unittest.TestCase):
    """Reported as BLOCKED with the exact reason, never faked."""

    def test_torch_absence_blocks_every_ik_and_collision_query(self):
        if _have("torch"):
            self.skipTest("torch is now present; this blocker can be retired")
        reason = _blocked(
            "arm_planning IK + collision",
            "torch is not installed on the host or in the g1-ros1 container, so "
            "GuardedIKSolver cannot load models/g1_*_hybrid_best.pt and no IK, "
            "collision-mask or trajectory query can run. trimesh and python-fcl "
            "are also absent, so g1_collision_probe cannot run either.")
        self.assertIn("torch", reason)

    def test_the_checkpoints_are_still_stale_against_the_corrected_tool_point(self):
        """Readable without torch, so this part is NOT blocked."""
        if not _have("yaml"):
            self.skipTest("PyYAML absent")
        import yaml
        sys.path.insert(0, str(REPO / "arm_planning" / "src"))
        from arm_planning import checkpoint_meta as cm
        cfg = yaml.safe_load((REPO / "arm_planning" / "config"
                              / "arm_kinematics.yaml").read_text(encoding="utf-8"))
        comparisons = cm.compare_all(cfg, REPO / "arm_planning" / "models")
        stale = cm.stale_arms(comparisons)
        self.assertEqual(stale, ["left", "right"],
                         "; ".join(c.describe() for c in comparisons))
        for c in comparisons:
            self.assertAlmostEqual(c.translation_delta_m, 0.054, places=4)


# =====================================================================================
class NavigationFixtureIsReal(unittest.TestCase):
    """The existing sim map/semantic fixture, parsed by real yaml."""

    @classmethod
    def setUpClass(cls):
        cls.sim = REPO / "navigation" / "sim" / "sim_room"
        if not cls.sim.is_dir():
            raise unittest.SkipTest(_blocked("navigation sim", "sim_room fixture absent"))

    def test_the_sim_map_and_semantic_files_exist(self):
        for name in ("map.yaml", "map.pgm", "semantic.yaml"):
            self.assertTrue((self.sim / name).is_file(), "missing %s" % name)

    def test_the_map_yaml_declares_resolution_and_origin(self):
        import yaml
        m = yaml.safe_load((self.sim / "map.yaml").read_text())
        self.assertIn("resolution", m)
        self.assertIn("origin", m)
        self.assertGreater(float(m["resolution"]), 0.0)
        self.assertEqual(len(m["origin"]), 3)

    def test_semantic_waypoints_are_named_and_numeric(self):
        """A navigate request names a class from here, so it must resolve."""
        import yaml
        s = yaml.safe_load((self.sim / "semantic.yaml").read_text())
        self.assertTrue(s, "semantic.yaml is empty: no waypoint could be resolved")
        flat = s.get("objects", s.get("waypoints", s))
        self.assertTrue(isinstance(flat, dict) and flat, "no waypoint mapping found")
        for name, entry in flat.items():
            self.assertTrue(str(name).strip())
            if isinstance(entry, dict):
                for key in ("x", "y"):
                    if key in entry:
                        self.assertTrue(math.isfinite(float(entry[key])),
                                        "%s.%s is not finite" % (name, key))


# =====================================================================================
class HandoverDetectorIsReal(unittest.TestCase):
    """The real detector, with values EXPLICITLY labelled in newtons."""

    @classmethod
    def setUpClass(cls):
        sys.path.insert(0, str(REPO / "handover" / "src"))
        from handover import tactile_detector as td
        from handover import tactile_frame as tf
        cls.td, cls.tf = td, tf

    def _detector(self):
        td, tf = self.td, self.tf
        specs = {n: tf.ChannelSpec(n, f, tf.FORCE_LIKE, "N (TEST FIXTURE)")
                 for n, f in (("ch0", "index"), ("ch1", "middle"), ("ch2", "thumb"))}
        cfg = td.TactileDetectorConfig(channels=specs, post_arm_suppress_sec=0.0)
        return td.TactileReleaseDetector(cfg), cfg

    def _frame(self, t, values_n):
        return self.tf.TactileFrame(
            stamp=t, values=dict(zip(("ch0", "ch1", "ch2"), values_n)))

    def test_requested_threshold_is_three_newtons(self):
        self.assertEqual(self.td.DEFAULT_RELEASE_THRESHOLD, 3.0)

    def test_one_channel_over_three_newtons_detects(self):
        det, _ = self._detector()
        det.arm(0.0)
        det.update(0.0, self._frame(0.0, [10.0, 10.0, 10.0]))      # reference, N
        ev = det.update(0.05, self._frame(0.05, [14.0, 11.0, 9.5]))  # +4.0 / +1.0 / -0.5 N
        self.assertTrue(ev.trigger)
        self.assertEqual(ev.qualifying, ("ch0",))

    def test_all_channels_under_three_newtons_do_not_detect(self):
        det, _ = self._detector()
        det.arm(0.0)
        det.update(0.0, self._frame(0.0, [10.0, 10.0, 10.0]))
        ev = det.update(0.05, self._frame(0.05, [12.0, 11.5, 12.0]))  # +2.0 / +1.5 / +2.0 N
        self.assertFalse(ev.trigger)

    def test_channels_are_not_summed(self):
        """2.0 + 1.5 + 2.0 = 5.5 N total, but no single channel crosses 3 N."""
        det, _ = self._detector()
        det.arm(0.0)
        det.update(0.0, self._frame(0.0, [10.0, 10.0, 10.0]))
        ev = det.update(0.05, self._frame(0.05, [12.0, 11.5, 12.0]))
        self.assertEqual(ev.qualifying, ())

    def test_the_delta_sign_convention_is_undefined_for_real_hardware(self):
        """Recorded as a gap, not invented.

        The detector's default is `either` (magnitude). Whether a human pull
        makes a Revo2 force channel RISE or DROP is not established anywhere,
        so this fixture deliberately proves only that both signs are accepted.
        """
        for step_n in ([14.0, 10.0, 10.0], [6.0, 10.0, 10.0]):   # +4.0 N and -4.0 N
            det, _ = self._detector()
            det.arm(0.0)
            det.update(0.0, self._frame(0.0, [10.0, 10.0, 10.0]))
            self.assertTrue(det.update(0.05, self._frame(0.05, step_n)).trigger, step_n)
        _blocked("handover delta sign",
                 "whether a real pull makes a Revo2 force channel rise or drop is "
                 "unknown; the detector defaults to a magnitude compare and the "
                 "per-channel `direction` must be set once the SDK is read")


def tearDownModule():
    if BLOCKED:
        print("\n=== BLOCKED CHECKS (not failures, not passes) ===")
        for what, reason in BLOCKED:
            print("  * %s:\n      %s" % (what, reason))


if __name__ == "__main__":
    unittest.main(verbosity=2)
