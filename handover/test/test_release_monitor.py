"""Offline tests of the adaptive-baseline release state machine.

Simulated samples only, no ROS and no hardware. Time is injected, so debounce,
minimum-hold and timeout behaviour are exact rather than wall-clock dependent.

    cd handover && python3 -m unittest discover -s test -v
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))

from handover.release_monitor import (  # noqa: E402
    BASELINING,
    DIRECTIONS,
    IDLE,
    MONITORING,
    STATES,
    TRIGGERED,
    MonitorConfig,
    ReleaseMonitor,
)

#: A fully specified config. The two hardware values are set HERE, in the test,
#: which is the only place in this repository allowed to pick a magnitude.
def cfg(**kw):
    base = dict(
        release_delta=1.0,
        direction="drop",
        baseline_samples=5,
        baseline_timeout_s=5.0,
        debounce_samples=3,
        min_hold_s=0.20,
        sample_timeout_s=0.50,
        monitor_timeout_s=10.0,
    )
    base.update(kw)
    return MonitorConfig(**base)


def feed(monitor, start_t, values, dt=0.05):
    """Push `values` at `dt` spacing; return (events, next timestamp)."""
    events = []
    t = start_t
    for v in values:
        events.append(monitor.update(t, v))
        t += dt
    return events, t


def baseline_at(monitor, start_t, level=10.0, dt=0.05):
    """Walk a fresh monitor through BASELINING at `level`; return next timestamp."""
    _, t = feed(monitor, start_t, [level] * monitor.config.baseline_samples, dt)
    assert monitor.state == MONITORING, monitor.last_reason
    return t


class ConfigurationGate(unittest.TestCase):
    """The monitor must refuse to arm on a guessed threshold (or none)."""

    def test_default_config_cannot_be_armed(self):
        monitor = ReleaseMonitor(config=MonitorConfig())
        outcome = monitor.arm(0.0)
        self.assertFalse(outcome.ok)
        self.assertIn("NOT_READY", outcome.reason)
        self.assertEqual(monitor.state, IDLE)

    def test_missing_release_delta_is_named(self):
        monitor = ReleaseMonitor(config=cfg(release_delta=None))
        outcome = monitor.arm(0.0)
        self.assertFalse(outcome.ok)
        self.assertIn("release_delta is not configured", outcome.reason)
        self.assertIn("not guessed", outcome.reason)

    def test_missing_direction_is_named(self):
        monitor = ReleaseMonitor(config=cfg(direction=None))
        outcome = monitor.arm(0.0)
        self.assertFalse(outcome.ok)
        self.assertIn("direction is not configured", outcome.reason)

    def test_invalid_direction_is_rejected(self):
        monitor = ReleaseMonitor(config=cfg(direction="sideways"))
        self.assertFalse(monitor.arm(0.0).ok)

    def test_non_positive_release_delta_is_rejected(self):
        for bad in (0.0, -1.0, float("nan"), float("inf")):
            self.assertFalse(ReleaseMonitor(config=cfg(release_delta=bad)).arm(0.0).ok)

    def test_valid_config_arms(self):
        monitor = ReleaseMonitor(config=cfg())
        outcome = monitor.arm(0.0)
        self.assertTrue(outcome.ok, outcome.reason)
        self.assertEqual(monitor.state, BASELINING)

    def test_states_and_directions_are_the_documented_sets(self):
        self.assertEqual(STATES, ("IDLE", "BASELINING", "MONITORING", "TRIGGERED"))
        self.assertEqual(DIRECTIONS, ("rise", "drop", "either"))


class Baselining(unittest.TestCase):
    def test_baseline_is_the_mean_of_the_collected_samples(self):
        monitor = ReleaseMonitor(config=cfg(baseline_samples=4))
        monitor.arm(0.0)
        feed(monitor, 0.0, [10.0, 12.0, 8.0, 10.0])
        self.assertEqual(monitor.state, MONITORING)
        self.assertAlmostEqual(monitor.baseline, 10.0)

    def test_monitoring_is_not_entered_early(self):
        monitor = ReleaseMonitor(config=cfg(baseline_samples=5))
        monitor.arm(0.0)
        feed(monitor, 0.0, [10.0] * 4)
        self.assertEqual(monitor.state, BASELINING)
        self.assertIsNone(monitor.baseline)

    def test_baseline_timeout_returns_to_idle(self):
        monitor = ReleaseMonitor(config=cfg(baseline_samples=50, baseline_timeout_s=1.0))
        monitor.arm(0.0)
        feed(monitor, 0.0, [10.0] * 5, dt=0.05)
        event = monitor.update(1.5, 10.0)
        self.assertEqual(event.state, IDLE)
        self.assertEqual(monitor.state, IDLE)
        self.assertIn("timeout", event.reason)
        self.assertIn("baseline samples", event.reason)

    def test_a_gap_discards_a_partial_baseline(self):
        """Averaging across a sensor gap would not be a baseline."""
        monitor = ReleaseMonitor(config=cfg(baseline_samples=4, baseline_timeout_s=5.0))
        monitor.arm(0.0)
        feed(monitor, 0.0, [10.0, 10.0], dt=0.05)
        stale = monitor.update(1.0, 10.0)              # gap 0.95 s > sample_timeout_s 0.5 s
        self.assertTrue(stale.wait)
        self.assertEqual(monitor.state, BASELINING)
        feed(monitor, 1.05, [20.0] * 4, dt=0.05)       # a clean run of four, still inside the budget
        self.assertEqual(monitor.state, MONITORING, monitor.last_reason)
        self.assertAlmostEqual(monitor.baseline, 20.0, msg="the pre-gap 10.0 samples must be discarded")


class NoFalseTrigger(unittest.TestCase):
    def test_noise_within_the_threshold_never_triggers(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        noise = [10.0, 10.9, 9.1, 10.5, 9.5, 10.8, 9.2, 10.99, 9.01] * 10
        events, _ = feed(monitor, t, noise)
        self.assertFalse(any(e.trigger for e in events))
        self.assertEqual(monitor.state, MONITORING)

    def test_excursion_in_the_wrong_direction_never_triggers(self):
        monitor = ReleaseMonitor(config=cfg(direction="drop"))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events, _ = feed(monitor, t, [50.0] * 40)      # a large RISE, direction is drop
        self.assertFalse(any(e.trigger for e in events))
        self.assertEqual(monitor.state, MONITORING)

    def test_exactly_at_the_threshold_counts_but_still_needs_debounce_and_hold(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events, _ = feed(monitor, t, [9.0, 9.0])       # delta == -release_delta, only 2 samples
        self.assertFalse(any(e.trigger for e in events))


class Debounce(unittest.TestCase):
    def test_single_spike_does_not_trigger(self):
        monitor = ReleaseMonitor(config=cfg(debounce_samples=3))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events, _ = feed(monitor, t, [0.0, 10.0, 10.0, 10.0])
        self.assertFalse(any(e.trigger for e in events))
        self.assertEqual(monitor.state, MONITORING)

    def test_interrupted_run_resets_the_debounce_count(self):
        monitor = ReleaseMonitor(config=cfg(debounce_samples=3, min_hold_s=0.0))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        # two over, one back inside, two over: never three consecutive
        events, _ = feed(monitor, t, [0.0, 0.0, 10.0, 0.0, 0.0])
        self.assertFalse(any(e.trigger for e in events))

    def test_debounce_progress_is_reported(self):
        monitor = ReleaseMonitor(config=cfg(debounce_samples=3))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events, _ = feed(monitor, t, [0.0, 0.0])
        self.assertIn("debounce 1/3", events[0].reason)
        self.assertIn("debounce 2/3", events[1].reason)


class MinimumHold(unittest.TestCase):
    def test_debounce_met_but_not_held_long_enough_does_not_trigger(self):
        monitor = ReleaseMonitor(config=cfg(debounce_samples=3, min_hold_s=1.0))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events, _ = feed(monitor, t, [0.0] * 5, dt=0.05)   # 0.20 s of excursion < 1.0 s
        self.assertFalse(any(e.trigger for e in events))
        self.assertIn("held", events[-1].reason)

    def test_held_long_enough_triggers(self):
        monitor = ReleaseMonitor(config=cfg(debounce_samples=3, min_hold_s=0.20))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events, _ = feed(monitor, t, [0.0] * 8, dt=0.05)
        self.assertTrue(any(e.trigger for e in events))
        self.assertEqual(monitor.state, TRIGGERED)

    def test_hold_timer_restarts_after_an_interruption(self):
        monitor = ReleaseMonitor(config=cfg(debounce_samples=1, min_hold_s=0.30))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events_a, t = feed(monitor, t, [0.0] * 4, dt=0.05)    # 0.15 s held
        self.assertFalse(any(e.trigger for e in events_a))
        events_b, t = feed(monitor, t, [10.0], dt=0.05)       # back inside: timer resets
        events_c, _ = feed(monitor, t, [0.0] * 4, dt=0.05)    # only 0.15 s again
        self.assertFalse(any(e.trigger for e in events_b + events_c))


class VerifiedTrigger(unittest.TestCase):
    def test_drop_direction_triggers_once_and_is_terminal(self):
        monitor = ReleaseMonitor(config=cfg(direction="drop"))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events, t = feed(monitor, t, [0.0] * 10, dt=0.05)
        self.assertEqual(sum(1 for e in events if e.trigger), 1)
        self.assertEqual(monitor.state, TRIGGERED)
        more, _ = feed(monitor, t, [0.0] * 10, dt=0.05)
        self.assertFalse(any(e.trigger for e in more), "TRIGGERED must be terminal")

    def test_rise_direction_triggers(self):
        monitor = ReleaseMonitor(config=cfg(direction="rise"))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events, _ = feed(monitor, t, [20.0] * 10, dt=0.05)
        self.assertTrue(any(e.trigger for e in events))

    def test_either_direction_accepts_both_signs(self):
        for level in (0.0, 20.0):
            monitor = ReleaseMonitor(config=cfg(direction="either"))
            monitor.arm(0.0)
            t = baseline_at(monitor, 0.0, level=10.0)
            events, _ = feed(monitor, t, [level] * 10, dt=0.05)
            self.assertTrue(any(e.trigger for e in events), "level %s" % level)

    def test_trigger_event_reports_baseline_and_delta(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events, _ = feed(monitor, t, [0.0] * 10, dt=0.05)
        trigger = [e for e in events if e.trigger][0]
        self.assertAlmostEqual(trigger.baseline, 10.0)
        self.assertAlmostEqual(trigger.delta, -10.0)
        self.assertIn("release verified", trigger.reason)


class MissingAndStaleData(unittest.TestCase):
    def test_none_sample_is_a_wait_not_a_trigger(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        event = monitor.update(t, None)
        self.assertTrue(event.wait)
        self.assertFalse(event.trigger)
        self.assertIn("WAIT", event.reason)
        self.assertEqual(monitor.state, MONITORING)

    def test_non_numeric_sample_is_a_wait(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        for bad in ("x", object(), [1, 2]):
            event = monitor.update(t, bad)
            self.assertTrue(event.wait, repr(bad))
            self.assertFalse(event.trigger)

    def test_non_finite_sample_is_a_wait(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        for bad in (float("nan"), float("inf")):
            event = monitor.update(t, bad)
            self.assertTrue(event.wait)
            self.assertFalse(event.trigger)

    def test_stale_gap_is_a_wait(self):
        monitor = ReleaseMonitor(config=cfg(sample_timeout_s=0.5))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        feed(monitor, t, [10.0], dt=0.05)
        event = monitor.update(t + 2.0, 10.0)      # 2 s gap
        self.assertTrue(event.wait)
        self.assertIn("stale", event.reason)

    def test_missing_data_cancels_an_in_progress_detection(self):
        """A dropout mid-excursion must not be bridged into a trigger."""
        monitor = ReleaseMonitor(config=cfg(debounce_samples=3, min_hold_s=0.0))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        events_a, t = feed(monitor, t, [0.0, 0.0], dt=0.05)     # debounce 2/3
        self.assertFalse(any(e.trigger for e in events_a))
        gap = monitor.update(t, None)                           # dropout resets
        self.assertTrue(gap.wait)
        events_b, _ = feed(monitor, t + 0.05, [0.0, 0.0], dt=0.05)
        self.assertFalse(any(e.trigger for e in events_b), "count must restart after a dropout")

    def test_stale_data_cancels_an_in_progress_detection(self):
        monitor = ReleaseMonitor(config=cfg(debounce_samples=3, min_hold_s=0.0))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        feed(monitor, t, [0.0, 0.0], dt=0.05)
        stale = monitor.update(t + 3.0, 0.0)
        self.assertTrue(stale.wait)
        events, _ = feed(monitor, t + 3.05, [0.0, 0.0], dt=0.05)
        self.assertFalse(any(e.trigger for e in events))

    def test_silent_sensor_never_triggers_and_eventually_times_out(self):
        monitor = ReleaseMonitor(config=cfg(monitor_timeout_s=2.0))
        monitor.arm(0.0)
        t = 0.0
        for _ in range(40):
            event = monitor.update(t, None)
            self.assertFalse(event.trigger)
            t += 0.1
        self.assertEqual(monitor.state, IDLE)
        self.assertIn("timeout", monitor.last_reason)


class Timeout(unittest.TestCase):
    def test_monitor_timeout_returns_to_idle(self):
        monitor = ReleaseMonitor(config=cfg(monitor_timeout_s=1.0))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        event = monitor.update(t + 2.0, 10.0)
        self.assertEqual(event.state, IDLE)
        self.assertIn("no verified release", event.reason)

    def test_zero_timeout_disables_the_budget(self):
        monitor = ReleaseMonitor(config=cfg(monitor_timeout_s=0.0))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        event = monitor.update(t + 10000.0, 10.0)
        self.assertEqual(event.state, MONITORING)

    def test_timeout_does_not_trigger_a_release(self):
        monitor = ReleaseMonitor(config=cfg(monitor_timeout_s=1.0))
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        event = monitor.update(t + 5.0, 0.0)     # over threshold AND past the budget
        self.assertFalse(event.trigger, "a timeout must never be reported as a release")
        self.assertEqual(event.state, IDLE)


class Disarm(unittest.TestCase):
    def test_disarm_from_baselining(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        self.assertTrue(monitor.disarm(0.1).ok)
        self.assertEqual(monitor.state, IDLE)

    def test_disarm_from_monitoring_clears_the_baseline(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        baseline_at(monitor, 0.0, level=10.0)
        monitor.disarm(1.0)
        self.assertEqual(monitor.state, IDLE)
        self.assertIsNone(monitor.baseline)

    def test_disarm_from_triggered(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        feed(monitor, t, [0.0] * 10, dt=0.05)
        self.assertEqual(monitor.state, TRIGGERED)
        self.assertTrue(monitor.disarm(5.0).ok)
        self.assertEqual(monitor.state, IDLE)

    def test_updates_after_disarm_do_nothing(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        monitor.disarm(t)
        events, _ = feed(monitor, t, [0.0] * 20, dt=0.05)
        self.assertFalse(any(e.trigger for e in events))
        self.assertTrue(all(e.state == IDLE for e in events))

    def test_disarm_is_idempotent(self):
        monitor = ReleaseMonitor(config=cfg())
        self.assertTrue(monitor.disarm(0.0).ok)
        self.assertTrue(monitor.disarm(0.0).ok)
        self.assertEqual(monitor.state, IDLE)

    def test_rearm_after_disarm_rebaselines(self):
        monitor = ReleaseMonitor(config=cfg())
        monitor.arm(0.0)
        t = baseline_at(monitor, 0.0, level=10.0)
        monitor.disarm(t)
        self.assertTrue(monitor.arm(t).ok)
        self.assertEqual(monitor.state, BASELINING)
        t2 = baseline_at(monitor, t, level=50.0)
        self.assertAlmostEqual(monitor.baseline, 50.0)
        events, _ = feed(monitor, t2, [49.5] * 10, dt=0.05)
        self.assertFalse(any(e.trigger for e in events), "new baseline must be used")

    def test_arming_twice_is_refused(self):
        monitor = ReleaseMonitor(config=cfg())
        self.assertTrue(monitor.arm(0.0).ok)
        outcome = monitor.arm(0.1)
        self.assertFalse(outcome.ok)
        self.assertIn("already armed", outcome.reason)

    def test_idle_monitor_ignores_samples(self):
        monitor = ReleaseMonitor(config=cfg())
        events, _ = feed(monitor, 0.0, [0.0] * 20)
        self.assertFalse(any(e.trigger for e in events))
        self.assertEqual(monitor.state, IDLE)


if __name__ == "__main__":
    unittest.main()
