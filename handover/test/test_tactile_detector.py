"""Offline tests of the multi-channel tactile pull detector.

SYNTHETIC SAMPLES ONLY. Every frame below is fabricated in this file to
exercise the state machine. None of it is hardware validation, and none of the
numbers here are calibration values for a real hand: they are chosen so the
arithmetic is obvious.

    cd handover && python3 -m unittest discover -s test -v
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from handover import tactile_frame as tf  # noqa: E402
from handover.tactile_detector import (  # noqa: E402
    BASELINING,
    IDLE,
    MONITORING,
    TRIGGERED,
    TactileDetectorConfig,
    TactileReleaseDetector,
)

#: Two force-like channels plus one of each non-comparable kind.
SPECS = {
    "f1_normal": tf.ChannelSpec("f1_normal", "index", tf.FORCE_LIKE, "counts"),
    "f2_normal": tf.ChannelSpec("f2_normal", "middle", tf.FORCE_LIKE, "counts"),
    "f1_dir": tf.ChannelSpec("f1_dir", "index", tf.ANGULAR, "rad"),
    "f1_prox": tf.ChannelSpec("f1_prox", "index", tf.OPAQUE, "counts"),
}

DT = 1.0 / 30.0          # the framework's ~30 Hz
FORCE = ("f1_normal", "f2_normal")


def cfg(**kw):
    """A fully calibrated config. These numbers are test fixtures, not calibration."""
    base = dict(
        channels=dict(SPECS),
        enabled_channels=FORCE,
        release_threshold={"f1_normal": 10.0, "f2_normal": 10.0},
        stable_tolerance={"f1_normal": 1.0, "f2_normal": 1.0},
        direction={"f1_normal": "drop", "f2_normal": "drop"},
        debounce_sec=0.20,
        baseline_frames=10,
        moving_average_frames=10,
        post_arm_suppress_sec=1.0,
        sample_timeout_sec=0.5,
        monitor_timeout_sec=120.0,
        baseline_timeout_sec=10.0,
        min_channels=1,
    )
    base.update(kw)
    return TactileDetectorConfig(**base)


def frame(t, f1=100.0, f2=100.0, **extra):
    values = {"f1_normal": f1, "f2_normal": f2, "f1_dir": 0.0, "f1_prox": 500.0}
    values.update(extra)
    return tf.TactileFrame(stamp=t, values=values)


def feed(det, t0, n, dt=DT, **kw):
    """Push n identical frames; return (events, next time)."""
    events, t = [], t0
    for _ in range(n):
        events.append(det.update(t, frame(t, **kw)))
        t += dt
    return events, t


def armed_and_monitoring(det, t0=0.0, **kw):
    """Walk a fresh detector into MONITORING and past the suppression window."""
    det.arm(t0)
    _, t = feed(det, t0, det.config.baseline_frames, **kw)
    assert det.state == MONITORING, det.last_reason
    # Clear the post-arm suppression window with in-band frames.
    needed = int(math.ceil(det.config.post_arm_suppress_sec / DT)) + 1
    _, t = feed(det, t, needed, **kw)
    return t


# ----------------------------------------------------------------- calibration gate
class CalibrationIsRequired(unittest.TestCase):
    def test_empty_config_cannot_be_armed(self):
        det = TactileReleaseDetector(TactileDetectorConfig())
        outcome = det.arm(0.0)
        self.assertFalse(outcome.ok)
        self.assertIn("NOT_READY", outcome.reason)
        self.assertEqual(det.state, IDLE)

    def test_no_channels_declared_is_named(self):
        problems = TactileDetectorConfig().validate()
        self.assertTrue(any("no tactile channels declared" in p for p in problems))
        self.assertTrue(any("unconfirmed" in p for p in problems))

    def test_missing_threshold_is_named_per_channel(self):
        c = cfg(release_threshold={"f1_normal": 10.0})
        problems = c.validate()
        self.assertTrue(any("release_threshold['f2_normal']" in p for p in problems))
        self.assertTrue(any("not guessed" in p for p in problems))

    def test_missing_tolerance_is_named_per_channel(self):
        problems = cfg(stable_tolerance={"f1_normal": 1.0}).validate()
        self.assertTrue(any("stable_tolerance['f2_normal']" in p for p in problems))

    def test_missing_direction_is_named_per_channel(self):
        problems = cfg(direction={"f1_normal": "drop"}).validate()
        self.assertTrue(any("direction['f2_normal']" in p for p in problems))

    def test_missing_debounce_is_named(self):
        problems = cfg(debounce_sec=None).validate()
        self.assertTrue(any("debounce_sec is not configured" in p for p in problems))

    def test_tolerance_must_be_below_threshold(self):
        problems = cfg(stable_tolerance={"f1_normal": 10.0, "f2_normal": 1.0}).validate()
        self.assertTrue(any("must be smaller than" in p for p in problems))

    def test_non_positive_threshold_rejected(self):
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            problems = cfg(release_threshold={"f1_normal": bad, "f2_normal": 10.0}).validate()
            self.assertTrue(problems, bad)

    def test_a_fully_calibrated_config_arms(self):
        det = TactileReleaseDetector(cfg())
        self.assertEqual(det.config.validate(), [])
        self.assertTrue(det.arm(0.0).ok)
        self.assertEqual(det.state, BASELINING)


# ------------------------------------------------------------------- channel kinds
class ChannelKindsAreNotInterchangeable(unittest.TestCase):
    def test_force_like_uses_plain_subtraction(self):
        self.assertAlmostEqual(tf.difference(SPECS["f1_normal"], 90.0, 100.0), -10.0)

    def test_angular_wraps(self):
        spec = tf.ChannelSpec("a", "index", tf.ANGULAR, "rad", period=2 * math.pi)
        d = tf.difference(spec, 0.05, 2 * math.pi - 0.05)
        self.assertAlmostEqual(d, 0.10, places=6)

    def test_angular_degrees_period(self):
        spec = tf.ChannelSpec("a", "index", tf.ANGULAR, "deg", period=360.0)
        self.assertAlmostEqual(tf.difference(spec, 1.0, 359.0), 2.0)

    def test_opaque_refuses_to_be_differenced(self):
        with self.assertRaises(ValueError) as ctx:
            tf.difference(SPECS["f1_prox"], 600.0, 500.0)
        self.assertIn("no meaningful difference", str(ctx.exception))

    def test_opaque_and_angular_are_not_triggerable(self):
        self.assertFalse(SPECS["f1_prox"].triggerable)
        self.assertFalse(SPECS["f1_dir"].triggerable)
        self.assertTrue(SPECS["f1_normal"].triggerable)

    def test_active_channels_exclude_non_force_kinds(self):
        c = cfg(enabled_channels=())
        self.assertEqual(c.active_channels(), ["f1_normal", "f2_normal"])
        self.assertIn("f1_prox", c.masked_out())
        self.assertIn("f1_dir", c.masked_out())

    def test_enabling_a_non_force_channel_is_refused(self):
        problems = cfg(enabled_channels=("f1_normal", "f1_prox")).validate()
        self.assertTrue(any("may not be thresholded" in p for p in problems))

    def test_bad_kind_is_rejected_at_construction(self):
        with self.assertRaises(ValueError):
            tf.ChannelSpec("x", "index", "sideways")

    def test_a_huge_proximity_change_never_triggers(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        events, _ = feed(det, t, 40, f1_prox=999999.0)
        self.assertFalse(any(e.trigger for e in events))
        self.assertEqual(det.state, MONITORING)


# ---------------------------------------------------------------------- baselining
class ArmingAndBaseline(unittest.TestCase):
    def test_baseline_is_the_mean_of_ten_frames(self):
        det = TactileReleaseDetector(cfg())
        det.arm(0.0)
        t = 0.0
        for i in range(10):
            det.update(t, frame(t, f1=100.0 + (0.5 if i % 2 else -0.5)))
            t += DT
        self.assertEqual(det.state, MONITORING)
        self.assertAlmostEqual(det.baseline["f1_normal"], 100.0, places=6)

    def test_monitoring_not_entered_before_ten_frames(self):
        det = TactileReleaseDetector(cfg())
        det.arm(0.0)
        feed(det, 0.0, 9)
        self.assertEqual(det.state, BASELINING)
        self.assertEqual(det.baseline, {})

    def test_unstable_frames_do_not_become_a_baseline(self):
        """Arm vibration: spread beyond stable_tolerance keeps us baselining."""
        det = TactileReleaseDetector(cfg())
        det.arm(0.0)
        t = 0.0
        for i in range(10):
            det.update(t, frame(t, f1=100.0 + (5.0 if i % 2 else -5.0)))
            t += DT
        self.assertEqual(det.state, BASELINING)
        self.assertIn("not stable yet", det.last_reason)

    def test_settling_then_stabilising_enters_monitoring(self):
        det = TactileReleaseDetector(cfg())
        det.arm(0.0)
        t = 0.0
        for i in range(10):                       # vibration
            det.update(t, frame(t, f1=100.0 + (5.0 if i % 2 else -5.0)))
            t += DT
        self.assertEqual(det.state, BASELINING)
        _, t = feed(det, t, 10, f1=100.0)         # settled
        self.assertEqual(det.state, MONITORING)

    def test_arming_clears_previous_run(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det, f1=100.0)
        det.disarm(t)
        self.assertTrue(det.arm(t).ok)
        self.assertEqual(det.baseline, {})
        self.assertEqual(det.state, BASELINING)
        t2 = armed_and_monitoring(det, t0=t, f1=50.0)
        self.assertAlmostEqual(det.baseline["f1_normal"], 50.0, places=6)

    def test_arming_twice_is_refused(self):
        det = TactileReleaseDetector(cfg())
        self.assertTrue(det.arm(0.0).ok)
        outcome = det.arm(0.1)
        self.assertFalse(outcome.ok)
        self.assertIn("already armed", outcome.reason)

    def test_baseline_timeout_returns_to_idle(self):
        det = TactileReleaseDetector(cfg(baseline_frames=1000, baseline_timeout_sec=1.0))
        det.arm(0.0)
        feed(det, 0.0, 5)
        event = det.update(1.5, frame(1.5))
        self.assertEqual(event.state, IDLE)
        self.assertIn("TIMEOUT", event.reason)
        self.assertFalse(event.trigger)


# ------------------------------------------------------------- post-arm suppression
class PostArmSuppression(unittest.TestCase):
    def test_a_pull_inside_the_suppression_window_does_not_trigger(self):
        det = TactileReleaseDetector(cfg())
        det.arm(0.0)
        _, t = feed(det, 0.0, 10)                     # baseline, ends ~0.33 s
        self.assertEqual(det.state, MONITORING)
        events, _ = feed(det, t, 15, f1=0.0)          # big drop, still < 1.0 s
        self.assertFalse(any(e.trigger for e in events))
        self.assertTrue(any("post-arm suppression" in e.reason for e in events))

    def test_the_same_pull_triggers_once_the_window_passes(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        events, _ = feed(det, t, 20, f1=0.0)
        self.assertTrue(any(e.trigger for e in events))
        self.assertEqual(det.state, TRIGGERED)

    def test_suppression_is_at_least_one_second_by_default(self):
        self.assertGreaterEqual(TactileDetectorConfig().post_arm_suppress_sec, 1.0)

    def test_framework_frame_counts_are_the_defaults(self):
        c = TactileDetectorConfig()
        self.assertEqual(c.baseline_frames, 10)
        self.assertEqual(c.moving_average_frames, 10)


# --------------------------------------------------------------------- drift vs pull
class DriftAndSuddenChange(unittest.TestCase):
    def test_slow_drift_inside_tolerance_is_absorbed_and_never_triggers(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        value = 100.0
        triggered = False
        for _ in range(600):                    # ~20 s of 0.05/frame drift
            value -= 0.05
            e = det.update(t, frame(t, f1=value, f2=value))
            triggered = triggered or e.trigger
            t += DT
        self.assertFalse(triggered, "slow drift must not be read as a pull")
        self.assertEqual(det.state, MONITORING)
        # The baseline followed the drift rather than accumulating a false diff.
        self.assertLess(abs(det.baseline["f1_normal"] - value), 2.0)

    def test_a_sudden_drop_triggers(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        events, _ = feed(det, t, 20, f1=80.0, f2=80.0)
        self.assertTrue(any(e.trigger for e in events))

    def test_drift_absorption_does_not_erase_a_developing_pull(self):
        """A channel past its tolerance keeps its baseline."""
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        before = det.baseline["f1_normal"]
        # Move well outside the stable band but below threshold.
        _, t = feed(det, t, 5, f1=before - 5.0)
        self.assertLess(abs(det.baseline["f1_normal"] - before), 0.2,
                        "a channel past its tolerance must keep its baseline")

    def test_a_ramp_faster_than_the_drift_gain_can_absorb_still_triggers(self):
        """Absorption must be bounded, or a slow pull would be invisible."""
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        value, triggered = 100.0, False
        for _ in range(400):
            value -= 1.0                     # 1.0/frame: far above drift_gain*threshold
            e = det.update(t, frame(t, f1=value, f2=value))
            triggered = triggered or e.trigger
            if triggered:
                break
            t += DT
        self.assertTrue(triggered, "a steady pull must not be absorbed forever")

    def test_wrong_direction_never_triggers(self):
        det = TactileReleaseDetector(cfg())          # direction = drop
        t = armed_and_monitoring(det)
        events, _ = feed(det, t, 40, f1=1000.0, f2=1000.0)
        self.assertFalse(any(e.trigger for e in events))

    def test_either_direction_accepts_both_signs(self):
        for level in (0.0, 1000.0):
            det = TactileReleaseDetector(
                cfg(direction={"f1_normal": "either", "f2_normal": "either"}))
            t = armed_and_monitoring(det)
            events, _ = feed(det, t, 20, f1=level, f2=level)
            self.assertTrue(any(e.trigger for e in events), level)


# ------------------------------------------------------------------------- debounce
class Debounce(unittest.TestCase):
    def test_a_single_spike_does_not_trigger(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        e = det.update(t, frame(t, f1=0.0, f2=0.0))
        self.assertFalse(e.trigger)
        self.assertTrue(e.candidate)
        events, _ = feed(det, t + DT, 20, f1=100.0, f2=100.0)
        self.assertFalse(any(x.trigger for x in events))

    def test_an_interrupted_candidate_restarts_the_debounce_timer(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        events_a, t = feed(det, t, 3, f1=0.0, f2=0.0)      # ~0.10 s < 0.20 s
        self.assertFalse(any(e.trigger for e in events_a))
        _, t = feed(det, t, 1, f1=100.0, f2=100.0)         # back in band: reset
        events_c, _ = feed(det, t, 3, f1=0.0, f2=0.0)      # ~0.10 s again
        self.assertFalse(any(e.trigger for e in events_c))

    def test_debounce_progress_is_reported(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        e = det.update(t, frame(t, f1=0.0, f2=0.0))
        self.assertIn("candidate pull", e.reason)
        self.assertTrue(e.candidate)
        self.assertEqual(e.qualifying, ("f1_normal", "f2_normal"))

    def test_zero_debounce_triggers_on_the_first_qualifying_frame(self):
        det = TactileReleaseDetector(cfg(debounce_sec=0.0))
        t = armed_and_monitoring(det)
        e = det.update(t, frame(t, f1=0.0, f2=0.0))
        self.assertTrue(e.trigger)


# --------------------------------------------------------------------------- masking
class ChannelMasking(unittest.TestCase):
    def test_a_masked_channel_cannot_trigger(self):
        det = TactileReleaseDetector(cfg(enabled_channels=("f2_normal",)))
        t = armed_and_monitoring(det)
        events, _ = feed(det, t, 40, f1=0.0, f2=100.0)   # only the masked one moves
        self.assertFalse(any(e.trigger for e in events))

    def test_the_unmasked_channel_still_triggers(self):
        det = TactileReleaseDetector(cfg(enabled_channels=("f2_normal",)))
        t = armed_and_monitoring(det)
        events, _ = feed(det, t, 20, f1=100.0, f2=0.0)
        self.assertTrue(any(e.trigger for e in events))

    def test_min_channels_requires_simultaneous_agreement(self):
        det = TactileReleaseDetector(cfg(min_channels=2))
        t = armed_and_monitoring(det)
        events, t = feed(det, t, 20, f1=0.0, f2=100.0)   # only one qualifies
        self.assertFalse(any(e.trigger for e in events))
        events, _ = feed(det, t, 20, f1=0.0, f2=0.0)     # now both
        self.assertTrue(any(e.trigger for e in events))

    def test_min_channels_beyond_the_enabled_count_is_refused(self):
        problems = cfg(min_channels=3).validate()
        self.assertTrue(any("exceeds" in p for p in problems))

    def test_masked_out_lists_excluded_channels(self):
        c = cfg(enabled_channels=("f1_normal",))
        self.assertEqual(c.masked_out(), ["f1_dir", "f1_prox", "f2_normal"])


# ------------------------------------------------------- invalid data is never proof
class InvalidDataIsNeverEvidence(unittest.TestCase):
    def _armed(self):
        det = TactileReleaseDetector(cfg())
        return det, armed_and_monitoring(det)

    def test_missing_frame_is_a_fault_not_a_trigger(self):
        det, t = self._armed()
        e = det.update(t, None)
        self.assertFalse(e.trigger)
        self.assertTrue(e.fault)
        self.assertIn("FAULT", e.reason)

    def test_a_fault_demands_a_fresh_baseline(self):
        det, t = self._armed()
        det.update(t, None)
        self.assertEqual(det.state, BASELINING)
        self.assertEqual(det.baseline, {})

    def test_a_fault_cancels_an_in_progress_debounce(self):
        det, t = self._armed()
        events, t = feed(det, t, 3, f1=0.0, f2=0.0)     # candidate building
        self.assertFalse(any(e.trigger for e in events))
        self.assertTrue(det.update(t, None).fault)       # dropout
        t += DT
        _, t = feed(det, t, 10, f1=0.0, f2=0.0)          # re-baseline on pulled values
        # The new baseline IS the pulled level, so this is no longer a pull.
        events, _ = feed(det, t, 20, f1=0.0, f2=0.0)
        self.assertFalse(any(e.trigger for e in events),
                         "a dropout must not be bridged into a trigger")

    def test_non_finite_channel_is_a_fault(self):
        det, t = self._armed()
        for bad in (float("nan"), float("inf"), float("-inf")):
            det2 = TactileReleaseDetector(cfg())
            t2 = armed_and_monitoring(det2)
            e = det2.update(t2, frame(t2, f1=bad))
            self.assertTrue(e.fault, bad)
            self.assertFalse(e.trigger)

    def test_non_numeric_channel_is_a_fault(self):
        det, t = self._armed()
        e = det.update(t, frame(t, f1="hard"))
        self.assertTrue(e.fault)

    def test_boolean_channel_is_a_fault(self):
        det, t = self._armed()
        self.assertTrue(det.update(t, frame(t, f1=True)).fault)

    def test_a_missing_required_channel_is_a_fault_not_a_mask(self):
        det, t = self._armed()
        bad = tf.TactileFrame(stamp=t, values={"f1_normal": 100.0})   # f2 absent
        e = det.update(t, bad)
        self.assertTrue(e.fault)
        self.assertIn("missing required channel", e.reason)

    def test_source_invalid_flag_is_honoured(self):
        det, t = self._armed()
        bad = tf.TactileFrame(stamp=t, values=frame(t).values, source_valid=False)
        self.assertTrue(det.update(t, bad).fault)

    def test_non_finite_stamp_is_a_fault(self):
        det, t = self._armed()
        self.assertTrue(det.update(t, tf.TactileFrame(stamp=float("nan"),
                                                      values=frame(t).values)).fault)

    def test_stale_gap_is_a_fault(self):
        det, t = self._armed()
        e = det.update(t + 5.0, frame(t + 5.0))
        self.assertTrue(e.fault)
        self.assertIn("stale", e.reason)

    def test_a_silent_sensor_never_triggers_and_times_out(self):
        det = TactileReleaseDetector(cfg(monitor_timeout_sec=2.0))
        det.arm(0.0)
        t = 0.0
        for _ in range(200):
            e = det.update(t, None)
            self.assertFalse(e.trigger)
            t += DT
        self.assertEqual(det.state, IDLE)
        self.assertIn("TIMEOUT", det.last_reason)


# ---------------------------------------------------------------- one-shot & disarm
class OneShotAndDisarm(unittest.TestCase):
    def test_trigger_fires_exactly_once_per_arming(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        events, t = feed(det, t, 30, f1=0.0, f2=0.0)
        self.assertEqual(sum(1 for e in events if e.trigger), 1)
        more, _ = feed(det, t, 30, f1=0.0, f2=0.0)
        self.assertFalse(any(e.trigger for e in more), "TRIGGERED must be terminal")

    def test_disarm_clears_pending_detection(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        feed(det, t, 3, f1=0.0, f2=0.0)          # candidate pending
        det.disarm(t)
        self.assertEqual(det.state, IDLE)
        events, _ = feed(det, t, 40, f1=0.0, f2=0.0)
        self.assertFalse(any(e.trigger for e in events))

    def test_disarm_from_every_state(self):
        for reach in ("baselining", "monitoring", "triggered"):
            det = TactileReleaseDetector(cfg())
            if reach == "baselining":
                det.arm(0.0)
            else:
                t = armed_and_monitoring(det)
                if reach == "triggered":
                    feed(det, t, 30, f1=0.0, f2=0.0)
                    self.assertEqual(det.state, TRIGGERED)
            self.assertTrue(det.disarm(99.0).ok)
            self.assertEqual(det.state, IDLE, reach)

    def test_rearming_after_a_trigger_allows_a_new_detection(self):
        det = TactileReleaseDetector(cfg())
        t = armed_and_monitoring(det)
        feed(det, t, 30, f1=0.0, f2=0.0)
        self.assertEqual(det.state, TRIGGERED)
        det.disarm(t)
        t2 = armed_and_monitoring(det, t0=t + 1.0)
        events, _ = feed(det, t2, 30, f1=0.0, f2=0.0)
        self.assertTrue(any(e.trigger for e in events))

    def test_idle_detector_ignores_frames(self):
        det = TactileReleaseDetector(cfg())
        events, _ = feed(det, 0.0, 40, f1=0.0)
        self.assertFalse(any(e.trigger for e in events))
        self.assertEqual(det.state, IDLE)

    def test_timeout_is_never_reported_as_success(self):
        det = TactileReleaseDetector(cfg(monitor_timeout_sec=1.0))
        t = armed_and_monitoring(det)
        e = det.update(t + 5.0, frame(t + 5.0, f1=0.0, f2=0.0))
        self.assertFalse(e.trigger)
        self.assertEqual(e.state, IDLE)
        self.assertIn("TIMEOUT", e.reason)


class PerHandIsolation(unittest.TestCase):
    """One state machine per hand; they must not share state."""

    def test_two_detectors_are_independent(self):
        left = TactileReleaseDetector(cfg(), hand="left")
        right = TactileReleaseDetector(cfg(), hand="right")
        t = armed_and_monitoring(left)
        self.assertEqual(right.state, IDLE)
        events, _ = feed(left, t, 30, f1=0.0, f2=0.0)
        self.assertTrue(any(e.trigger for e in events))
        self.assertEqual(left.state, TRIGGERED)
        self.assertEqual(right.state, IDLE)
        self.assertEqual(right.baseline, {})


if __name__ == "__main__":
    unittest.main()
