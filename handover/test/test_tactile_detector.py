"""Offline tests of the consecutive-frame force pull detector.

SYNTHETIC SAMPLES ONLY. Every frame below is fabricated in this file to
exercise the rule and the state machine. None of it is hardware validation, and
none of these numbers are measured properties of a real hand.

    cd handover && python3 -m unittest discover -s test -v
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from handover import tactile_frame as tf  # noqa: E402
from handover.tactile_detector import (  # noqa: E402
    DEFAULT_DIRECTION,
    DEFAULT_RELEASE_THRESHOLD,
    IDLE,
    MONITORING,
    REFERENCING,
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

DT = 1.0 / 30.0          # ~30 Hz
FORCE = ("f1_normal", "f2_normal")


def cfg(**kw):
    """Config relying on the requested defaults unless a test overrides them."""
    base = dict(
        channels=dict(SPECS),
        enabled_channels=FORCE,
        post_arm_suppress_sec=1.0,
        sample_timeout_sec=0.5,
        monitor_timeout_sec=120.0,
        reference_timeout_sec=10.0,
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


def monitoring(det, t0=0.0, **kw):
    """Arm, take a reference, and clear the post-arm suppression window."""
    det.arm(t0)
    events, t = feed(det, t0, 1, **kw)
    assert det.state == MONITORING, det.last_reason
    needed = int(math.ceil(det.config.post_arm_suppress_sec / DT)) + 1
    _, t = feed(det, t, needed, **kw)
    assert det.state == MONITORING, det.last_reason
    return t


# ------------------------------------------------------------------ readiness gate
class ReadinessGate(unittest.TestCase):
    def test_no_channels_declared_cannot_be_armed(self):
        det = TactileReleaseDetector(TactileDetectorConfig())
        outcome = det.arm(0.0)
        self.assertFalse(outcome.ok)
        self.assertIn("no force channels declared", outcome.reason)

    def test_declared_channels_arm_on_the_defaults_alone(self):
        """No calibration table is needed: the requested 3 N default applies."""
        det = TactileReleaseDetector(cfg())
        self.assertEqual(det.config.validate(), [])
        self.assertTrue(det.arm(0.0).ok)

    def test_requested_threshold_is_three(self):
        self.assertEqual(DEFAULT_RELEASE_THRESHOLD, 3.0)
        self.assertEqual(cfg().threshold_for("f1_normal"), 3.0)

    def test_default_direction_is_magnitude(self):
        self.assertEqual(DEFAULT_DIRECTION, "either")
        self.assertEqual(cfg().direction_for("f1_normal"), "either")

    def test_explicit_threshold_overrides_the_default(self):
        self.assertEqual(cfg(release_threshold={"f1_normal": 7.5}).threshold_for("f1_normal"), 7.5)

    def test_non_positive_threshold_is_refused(self):
        self.assertTrue(any("finite positive" in p
                            for p in cfg(release_threshold={"f1_normal": 0.0}).validate()))

    def test_unknown_direction_is_refused(self):
        self.assertTrue(any("direction" in p
                            for p in cfg(direction={"f1_normal": "sideways"}).validate()))

    def test_enabling_a_non_force_channel_is_refused(self):
        problems = cfg(enabled_channels=("f1_prox",)).validate()
        self.assertTrue(any("may not be thresholded" in p for p in problems))

    def test_undeclared_enabled_channel_is_refused(self):
        self.assertTrue(any("not declared" in p
                            for p in cfg(enabled_channels=("nope",)).validate()))

    def test_only_force_channels_are_active(self):
        self.assertEqual(cfg(enabled_channels=()).active_channels(), list(FORCE))
        self.assertIn("f1_prox", cfg().masked_out())


# ------------------------------------------------------- the consecutive-frame rule
class ConsecutiveFrameRule(unittest.TestCase):
    def test_first_valid_frame_becomes_the_reference(self):
        det = TactileReleaseDetector(cfg())
        det.arm(0.0)
        self.assertEqual(det.state, REFERENCING)
        det.update(0.0, frame(0.0, f1=100.0, f2=100.0))
        self.assertEqual(det.state, MONITORING)
        self.assertEqual(det.reference["f1_normal"], 100.0)

    def test_a_step_past_threshold_on_one_channel_triggers(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        event = det.update(t, frame(t, f1=104.0))      # delta +4 > 3
        self.assertTrue(event.trigger)
        self.assertEqual(det.state, TRIGGERED)
        self.assertEqual(event.qualifying, ("f1_normal",))

    def test_a_step_below_threshold_advances_the_reference(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        event = det.update(t, frame(t, f1=102.0))      # delta +2 < 3
        self.assertFalse(event.trigger)
        self.assertEqual(det.reference["f1_normal"], 102.0)
        self.assertIn("reference advanced", event.reason)

    def test_delta_is_measured_against_the_previous_frame_not_the_first(self):
        """Two sub-threshold steps in the same direction must not accumulate."""
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        for value in (102.0, 104.0, 106.0):            # +2 each
            event = det.update(t, frame(t, f1=value))
            self.assertFalse(event.trigger, "accumulated to a trigger at %s" % value)
            t += DT
        self.assertEqual(det.reference["f1_normal"], 106.0)

    def test_a_slow_ramp_never_triggers(self):
        """Inherent to the requested rule, recorded so it is not a surprise.

        The reference advances on every non-qualifying frame, so a pull slower
        than the threshold PER FRAME is tracked rather than detected. At 30 Hz
        and 3 N, anything under ~90 N/s is invisible to this rule.
        """
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        value = 100.0
        for _ in range(200):
            value += 2.5                               # 2.5 < 3 every frame
            event = det.update(t, frame(t, f1=value))
            self.assertFalse(event.trigger)
            t += DT
        self.assertEqual(det.state, MONITORING)

    def test_channels_are_never_summed(self):
        """Two channels at 2 each (sum 4) must NOT trigger a 3 N threshold."""
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        event = det.update(t, frame(t, f1=102.0, f2=102.0))
        self.assertFalse(event.trigger)
        self.assertEqual(event.qualifying, ())

    def test_any_single_channel_is_enough(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        event = det.update(t, frame(t, f1=100.0, f2=96.0))   # only f2 moves, by -4
        self.assertTrue(event.trigger)
        self.assertEqual(event.qualifying, ("f2_normal",))

    def test_both_channels_can_qualify_together(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        event = det.update(t, frame(t, f1=110.0, f2=90.0))
        self.assertTrue(event.trigger)
        self.assertEqual(set(event.qualifying), set(FORCE))

    def test_exactly_at_the_threshold_qualifies(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1=103.0)).trigger)

    def test_deltas_are_reported_for_every_active_channel(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        event = det.update(t, frame(t, f1=101.0, f2=99.0))
        self.assertAlmostEqual(event.diffs["f1_normal"], 1.0)
        self.assertAlmostEqual(event.diffs["f2_normal"], -1.0)


# ---------------------------------------------------------------- sign convention
class SignConvention(unittest.TestCase):
    """The project's signed-delta + per-channel direction mechanism, preserved."""

    def test_either_accepts_both_signs(self):
        for value in (104.0, 96.0):
            det = TactileReleaseDetector(cfg(direction={"f1_normal": "either"}))
            t = monitoring(det)
            self.assertTrue(det.update(t, frame(t, f1=value)).trigger, value)

    def test_rise_ignores_a_drop(self):
        det = TactileReleaseDetector(cfg(direction={"f1_normal": "rise",
                                                    "f2_normal": "rise"}))
        t = monitoring(det)
        self.assertFalse(det.update(t, frame(t, f1=90.0, f2=90.0)).trigger)

    def test_rise_accepts_a_rise(self):
        det = TactileReleaseDetector(cfg(direction={"f1_normal": "rise"}))
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1=110.0)).trigger)

    def test_drop_ignores_a_rise(self):
        det = TactileReleaseDetector(cfg(direction={"f1_normal": "drop",
                                                    "f2_normal": "drop"}))
        t = monitoring(det)
        self.assertFalse(det.update(t, frame(t, f1=110.0, f2=110.0)).trigger)

    def test_drop_accepts_a_drop(self):
        det = TactileReleaseDetector(cfg(direction={"f1_normal": "drop"}))
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1=90.0)).trigger)

    def test_direction_is_per_channel(self):
        det = TactileReleaseDetector(cfg(direction={"f1_normal": "rise",
                                                    "f2_normal": "drop"}))
        t = monitoring(det)
        event = det.update(t, frame(t, f1=90.0, f2=90.0))   # both drop
        self.assertEqual(event.qualifying, ("f2_normal",))


# ------------------------------------------------------------------- channel kinds
class ChannelKinds(unittest.TestCase):
    def test_opaque_refuses_to_be_differenced(self):
        with self.assertRaises(ValueError):
            tf.difference(SPECS["f1_prox"], 1.0, 0.0)

    def test_angular_wraps(self):
        spec = SPECS["f1_dir"]
        self.assertAlmostEqual(tf.difference(spec, 0.05, 2 * math.pi - 0.05), 0.1, places=9)

    def test_a_huge_opaque_change_never_triggers(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        event = det.update(t, frame(t, f1_prox=999999.0))
        self.assertFalse(event.trigger)


# ------------------------------------------------------------- post-arm suppression
class PostArmSuppression(unittest.TestCase):
    def test_a_step_inside_the_window_does_not_trigger(self):
        det = TactileReleaseDetector(cfg())
        det.arm(0.0)
        det.update(0.0, frame(0.0))
        event = det.update(0.1, frame(0.1, f1=120.0))
        self.assertFalse(event.trigger)
        self.assertIn("suppression", event.reason)

    def test_the_suppressed_step_is_absorbed_into_the_reference(self):
        """It must not re-fire the instant suppression ends."""
        det = TactileReleaseDetector(cfg())
        det.arm(0.0)
        det.update(0.0, frame(0.0))
        det.update(0.1, frame(0.1, f1=120.0))
        self.assertEqual(det.reference["f1_normal"], 120.0)
        event = det.update(2.0, frame(2.0, f1=120.0))      # same value, past the window
        self.assertFalse(event.trigger)

    def test_the_same_step_triggers_once_the_window_passes(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1=120.0)).trigger)

    def test_suppression_defaults_to_one_second(self):
        self.assertGreaterEqual(TactileDetectorConfig().post_arm_suppress_sec, 1.0)


# ------------------------------------------------------- invalid data is not evidence
class InvalidDataIsNeverEvidence(unittest.TestCase):
    def test_missing_frame_is_a_fault(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        event = det.update(t, None)
        self.assertTrue(event.fault)
        self.assertFalse(event.trigger)
        self.assertEqual(det.state, REFERENCING)

    def test_a_fault_discards_the_reference(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        det.update(t, None)
        self.assertEqual(det.reference, {})

    def test_the_frame_after_a_fault_only_re_references(self):
        """A delta across a fault is not a delta between consecutive frames."""
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        det.update(t, None)
        t += DT
        event = det.update(t, frame(t, f1=200.0))      # huge jump, but no reference
        self.assertFalse(event.trigger)
        self.assertEqual(det.state, MONITORING)
        self.assertEqual(det.reference["f1_normal"], 200.0)

    def test_non_finite_channel_is_a_fault(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1=float("nan"))).fault)

    def test_non_numeric_channel_is_a_fault(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1="hard")).fault)

    def test_boolean_channel_is_a_fault(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1=True)).fault)

    def test_a_missing_required_channel_is_a_fault_not_a_mask(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        bad = tf.TactileFrame(stamp=t, values={"f1_normal": 100.0, "f1_dir": 0.0})
        self.assertTrue(det.update(t, bad).fault)

    def test_source_invalid_flag_is_honoured(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        bad = frame(t)
        bad.source_valid = False
        self.assertTrue(det.update(t, bad).fault)

    def test_non_finite_stamp_is_a_fault(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        self.assertTrue(det.update(t, tf.TactileFrame(
            stamp=float("inf"),
            values={n: 100.0 for n in ("f1_normal", "f2_normal", "f1_dir", "f1_prox")})).fault)

    def test_a_stale_gap_is_a_fault(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        event = det.update(t + 5.0, frame(t + 5.0))
        self.assertTrue(event.fault)
        self.assertIn("not consecutive", event.reason)

    def test_a_silent_sensor_times_out_and_never_triggers(self):
        det = TactileReleaseDetector(cfg(monitor_timeout_sec=1.0))
        det.arm(0.0)
        det.update(0.0, frame(0.0))
        event = det.update(2.0, frame(2.0))
        self.assertFalse(event.trigger)
        self.assertIn("TIMEOUT", event.reason)
        self.assertEqual(det.state, IDLE)

    def test_reference_timeout_returns_to_idle(self):
        det = TactileReleaseDetector(cfg(reference_timeout_sec=1.0))
        det.arm(0.0)
        event = det.update(2.0, None)
        self.assertEqual(det.state, IDLE)
        self.assertIn("TIMEOUT", event.reason)


# ------------------------------------------------------------------ one-shot / disarm
class OneShotAndDisarm(unittest.TestCase):
    def test_trigger_fires_exactly_once_per_arming(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1=120.0)).trigger)
        for _ in range(5):
            t += DT
            self.assertFalse(det.update(t, frame(t, f1=120.0)).trigger)

    def test_disarm_from_every_state(self):
        for prepare in (lambda d: None,
                        lambda d: d.arm(0.0),
                        lambda d: monitoring(d)):
            det = TactileReleaseDetector(cfg())
            prepare(det)
            self.assertTrue(det.disarm(0.0).ok)
            self.assertEqual(det.state, IDLE)
            self.assertEqual(det.reference, {})

    def test_rearming_after_a_trigger_allows_a_new_detection(self):
        det = TactileReleaseDetector(cfg())
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1=120.0)).trigger)
        det.disarm(t)
        t2 = monitoring(det, t + 1.0)
        self.assertTrue(det.update(t2, frame(t2, f1=120.0)).trigger)

    def test_arming_twice_is_refused(self):
        det = TactileReleaseDetector(cfg())
        det.arm(0.0)
        self.assertFalse(det.arm(0.1).ok)

    def test_idle_detector_ignores_frames(self):
        det = TactileReleaseDetector(cfg())
        event = det.update(0.0, frame(0.0, f1=1e6))
        self.assertEqual(event.state, IDLE)
        self.assertFalse(event.trigger)

    def test_timeout_is_never_reported_as_a_detection(self):
        det = TactileReleaseDetector(cfg(monitor_timeout_sec=0.5))
        det.arm(0.0)
        det.update(0.0, frame(0.0))
        event = det.update(1.0, frame(1.0, f1=120.0))
        self.assertFalse(event.trigger)
        self.assertFalse(det.fired)


# ------------------------------------------------------------------ masking / isolation
class MaskingAndIsolation(unittest.TestCase):
    def test_a_masked_channel_cannot_trigger(self):
        det = TactileReleaseDetector(cfg(enabled_channels=("f1_normal",)))
        t = monitoring(det)
        self.assertFalse(det.update(t, frame(t, f2=50.0)).trigger)

    def test_the_unmasked_channel_still_triggers(self):
        det = TactileReleaseDetector(cfg(enabled_channels=("f1_normal",)))
        t = monitoring(det)
        self.assertTrue(det.update(t, frame(t, f1=50.0)).trigger)

    def test_two_detectors_are_independent(self):
        left = TactileReleaseDetector(cfg(), hand="left")
        right = TactileReleaseDetector(cfg(), hand="right")
        t = monitoring(left)
        monitoring(right)
        self.assertTrue(left.update(t, frame(t, f1=120.0)).trigger)
        self.assertFalse(right.fired)
        self.assertEqual(right.state, MONITORING)


if __name__ == "__main__":
    unittest.main()
