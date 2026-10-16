"""Multi-channel tactile pull detector. One instance per hand. No ROS, no SDK.

Reuses the state names and result types of `release_monitor` (which stays as
the single-scalar case) and adds what the framework's detector section
requires: a moving average, an adaptive baseline, post-arm suppression,
channel masks, and per-channel-kind comparison.

    IDLE --arm()--> BASELINING --10 stable frames--> MONITORING --verified--> TRIGGERED
      ^                 |                                |                       |
      +--- timeout -----+------------ timeout -----------+----- disarm() --------+
                        ^                                |
                        +--------- invalid frame --------+   (fault: fresh baseline required)

CALIBRATION, NOT DEFAULTS. `release_threshold`, `stable_tolerance`,
`direction` (per channel) and `debounce_sec` have NO defaults. They are
physical properties of a specific hand and object and must come from real
recordings (see handover/README.md). `validate()` refuses to arm without them,
so the detector cannot fire on a guessed number.

Frame counts and suppression duration DO have defaults, because the framework
states them: 10 baseline frames, 10-frame moving average, >= 1.0 s suppression.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from handover import tactile_frame as tf
from handover.release_monitor import (
    BASELINING,
    DIRECTIONS,
    IDLE,
    MONITORING,
    TRIGGERED,
    Event,
    Outcome,
)

__all__ = [
    "TactileDetectorConfig", "TactileReleaseDetector",
    "IDLE", "BASELINING", "MONITORING", "TRIGGERED",
]


@dataclass
class TactileDetectorConfig:
    """Per-hand detector settings. The four calibration items have no defaults."""

    #: channel name -> ChannelSpec. Declared by configuration, not assumed.
    channels: dict = field(default_factory=dict)
    #: Channel mask: names the detector may use. Empty = every triggerable channel.
    enabled_channels: tuple = ()

    # --- calibration: NO defaults ---
    release_threshold: dict = field(default_factory=dict)   # name -> float
    stable_tolerance: dict = field(default_factory=dict)    # name -> float
    direction: dict = field(default_factory=dict)           # name -> rise|drop|either
    debounce_sec: float | None = None

    # --- framework-stated, so defaulted ---
    baseline_frames: int = 10
    moving_average_frames: int = 10
    post_arm_suppress_sec: float = 1.0

    # --- signal processing ---
    sample_timeout_sec: float = 0.5
    monitor_timeout_sec: float = 120.0
    baseline_timeout_sec: float = 10.0
    #: How many enabled channels must qualify simultaneously to trigger.
    min_channels: int = 1
    #: Fraction of the in-band difference folded into the baseline each frame.
    #: Dimensionless, so unlike the thresholds it may carry a default. Snapping
    #: the baseline straight onto the moving average (gain 1.0) would absorb any
    #: ramp slower than stable_tolerance per frame, which erases a slow pull;
    #: a small gain leaves a persistent difference that still crosses the
    #: threshold. Validate the (drift_gain, release_threshold) PAIR together on
    #: the stationary-hold and deliberate-pull recordings.
    drift_gain: float = 0.05

    # ------------------------------------------------------------------ helpers
    def active_channels(self) -> list:
        """Enabled channels the detector is allowed to trigger on, in order."""
        triggerable = [n for n, s in sorted(self.channels.items()) if s.triggerable]
        if not self.enabled_channels:
            return triggerable
        return [n for n in triggerable if n in self.enabled_channels]

    def masked_out(self) -> list:
        """Declared channels excluded by the mask or by their kind."""
        active = set(self.active_channels())
        return [n for n in sorted(self.channels) if n not in active]

    def validate(self) -> list:
        """Reasons this detector cannot be armed (empty = ready)."""
        problems = []

        if not self.channels:
            problems.append(
                "no tactile channels declared (the SDK schema is unconfirmed; see "
                "handover/README.md)"
            )
        for name in sorted(self.enabled_channels):
            if name not in self.channels:
                problems.append("enabled channel %r is not declared" % (name,))
            elif not self.channels[name].triggerable:
                problems.append(
                    "enabled channel %r has kind %r and may not be thresholded"
                    % (name, self.channels[name].kind)
                )

        active = self.active_channels()
        if self.channels and not active:
            problems.append(
                "no force-like channel is enabled, so there is nothing a release "
                "decision could legitimately be based on"
            )

        for name in active:
            t = self.release_threshold.get(name)
            if t is None:
                problems.append(
                    "release_threshold[%r] is not configured (must be measured on the "
                    "real hand, not guessed)" % (name,)
                )
            elif not isinstance(t, (int, float)) or isinstance(t, bool) \
                    or not math.isfinite(float(t)) or float(t) <= 0.0:
                problems.append("release_threshold[%r] must be a finite positive number" % (name,))

            s = self.stable_tolerance.get(name)
            if s is None:
                problems.append(
                    "stable_tolerance[%r] is not configured (baseline-drift band; must "
                    "come from a stationary-hold recording)" % (name,)
                )
            elif not isinstance(s, (int, float)) or isinstance(s, bool) \
                    or not math.isfinite(float(s)) or float(s) < 0.0:
                problems.append("stable_tolerance[%r] must be a finite non-negative number" % (name,))

            if t is not None and s is not None:
                try:
                    if float(s) >= float(t):
                        problems.append(
                            "stable_tolerance[%r]=%s must be smaller than "
                            "release_threshold[%r]=%s, otherwise drift absorption would "
                            "swallow every pull" % (name, s, name, t)
                        )
                except (TypeError, ValueError):
                    pass

            d = self.direction.get(name)
            if d is None:
                problems.append(
                    "direction[%r] is not configured (does a pull make this channel rise "
                    "or drop? must be confirmed on the real hand)" % (name,)
                )
            elif d not in DIRECTIONS:
                problems.append(
                    "direction[%r] must be one of %s" % (name, ", ".join(DIRECTIONS)))

        if self.debounce_sec is None:
            problems.append(
                "debounce_sec is not configured (how long a qualifying change must "
                "persist; must come from real pull recordings)"
            )
        elif not isinstance(self.debounce_sec, (int, float)) or \
                not math.isfinite(float(self.debounce_sec)) or float(self.debounce_sec) < 0.0:
            problems.append("debounce_sec must be a finite non-negative number")

        if self.baseline_frames < 1:
            problems.append("baseline_frames must be at least 1")
        if self.moving_average_frames < 1:
            problems.append("moving_average_frames must be at least 1")
        if self.post_arm_suppress_sec < 0.0:
            problems.append("post_arm_suppress_sec must not be negative")
        if self.sample_timeout_sec <= 0.0:
            problems.append("sample_timeout_sec must be positive")
        if self.baseline_timeout_sec <= 0.0:
            problems.append("baseline_timeout_sec must be positive")
        if self.monitor_timeout_sec < 0.0:
            problems.append("monitor_timeout_sec must not be negative (0 disables)")
        if not (0.0 < float(self.drift_gain) <= 1.0):
            problems.append("drift_gain must be in (0, 1]")
        if self.min_channels < 1:
            problems.append("min_channels must be at least 1")
        elif active and self.min_channels > len(active):
            problems.append(
                "min_channels=%d exceeds the %d enabled channel(s)"
                % (self.min_channels, len(active)))
        return problems


@dataclass
class TactileEvent(Event):
    """An Event plus per-channel detail, for diagnostics and recordings."""

    #: channel -> signed difference of the moving average from the baseline.
    diffs: dict = field(default_factory=dict)
    #: channels currently past their threshold in the configured direction.
    qualifying: tuple = ()
    #: True when a candidate is being debounced but has not yet qualified.
    candidate: bool = False
    #: True when the frame was rejected (missing/stale/malformed/non-finite).
    fault: bool = False
    held_sec: float = 0.0


class TactileReleaseDetector(object):
    """One hand's detector. Time is injected; it never reads a clock."""

    def __init__(self, config: TactileDetectorConfig, hand: str = "right"):
        self.config = config
        self.hand = hand
        self._state = IDLE
        self._last_reason = "never armed"
        self._reset_run()

    # ------------------------------------------------------------------ internals
    def _reset_run(self) -> None:
        """Clear samples, baseline, debounce state and pending triggers."""
        self._frames = deque(maxlen=max(1, self.config.moving_average_frames))
        self._baseline_buffer = deque(maxlen=max(1, self.config.baseline_frames))
        self._baseline = {}
        self._armed_at = None
        self._last_stamp = None
        self._over_since = None
        self._fired = False

    @property
    def state(self) -> str:
        return self._state

    @property
    def baseline(self) -> dict:
        return dict(self._baseline)

    @property
    def last_reason(self) -> str:
        return self._last_reason

    @property
    def armed(self) -> bool:
        return self._state in (BASELINING, MONITORING)

    @property
    def fired(self) -> bool:
        return self._fired

    def moving_average(self) -> dict:
        """Mean of the retained valid frames, per active channel."""
        if not self._frames:
            return {}
        names = self.config.active_channels()
        return {n: sum(f[n] for f in self._frames) / float(len(self._frames)) for n in names}

    # ------------------------------------------------------------------ commands
    def arm(self, now: float) -> Outcome:
        problems = self.config.validate()
        if problems:
            self._last_reason = "NOT_READY: " + "; ".join(problems)
            return Outcome(False, self._last_reason)
        if self.armed:
            self._last_reason = "already armed (state %s)" % self._state
            return Outcome(False, self._last_reason)

        self._reset_run()
        self._armed_at = float(now)
        self._state = BASELINING
        self._last_reason = "armed; collecting %d stable baseline frames" % (
            self.config.baseline_frames,)
        return Outcome(True, self._last_reason)

    def disarm(self, now: float = 0.0, reason: str = "disarmed by request") -> Outcome:
        was = self._state
        self._reset_run()
        self._state = IDLE
        self._last_reason = reason
        return Outcome(True, "%s (was %s)" % (reason, was))

    # ------------------------------------------------------------------ sampling
    def update(self, now: float, frame=None) -> TactileEvent:
        now = float(now)

        if self._state in (IDLE, TRIGGERED):
            return TactileEvent(state=self._state, reason=self._last_reason)

        # Budgets first, so a silent sensor cannot hold the detector armed.
        if self.config.monitor_timeout_sec > 0.0 and self._armed_at is not None:
            if now - self._armed_at > self.config.monitor_timeout_sec:
                reason = "TIMEOUT: no verified pull within %.1fs of arming" % (
                    self.config.monitor_timeout_sec,)
                self.disarm(now, reason)
                return TactileEvent(state=IDLE, reason=reason)
        if self._state == BASELINING and self._armed_at is not None:
            if now - self._armed_at > self.config.baseline_timeout_sec:
                reason = "TIMEOUT: only %d of %d stable baseline frames within %.1fs" % (
                    len(self._baseline_buffer), self.config.baseline_frames,
                    self.config.baseline_timeout_sec)
                self.disarm(now, reason)
                return TactileEvent(state=IDLE, reason=reason)

        names = self.config.active_channels()
        check = tf.validate_frame(frame, self.config.channels, names)
        if not check.ok:
            return self._fault(now, check.reason)

        stale = (
            self._last_stamp is not None
            and (now - self._last_stamp) > self.config.sample_timeout_sec
        )
        self._last_stamp = now
        if stale:
            return self._fault(
                now, "tactile data went stale (gap > %.3fs)" % self.config.sample_timeout_sec)

        if self._state == BASELINING:
            return self._baselining(now, check.clean, names)
        return self._monitoring(now, check.clean, names)

    # ------------------------------------------------------------------ fault path
    def _fault(self, now: float, reason: str) -> TactileEvent:
        """Invalid data is never evidence, and costs us the baseline.

        Any in-progress debounce is cancelled and the detector drops back to
        BASELINING, so detection resumes only after a fresh valid baseline.
        """
        self._over_since = None
        self._frames.clear()
        self._baseline_buffer.clear()
        self._baseline = {}
        self._state = BASELINING
        self._last_reason = (
            "FAULT: %s; a fresh baseline is required before detection resumes" % reason)
        return TactileEvent(state=BASELINING, wait=True, fault=True, reason=self._last_reason)

    # ------------------------------------------------------------------ baselining
    def _baselining(self, now: float, clean: dict, names) -> TactileEvent:
        self._baseline_buffer.append(clean)
        have = len(self._baseline_buffer)
        need = self.config.baseline_frames
        if have < need:
            self._last_reason = "baselining %d/%d" % (have, need)
            return TactileEvent(state=BASELINING, reason=self._last_reason)

        # "Stable" means the spread across the collected frames is inside each
        # channel's calibrated tolerance. A hand still vibrating is not a baseline.
        unstable = []
        for n in names:
            series = [f[n] for f in self._baseline_buffer]
            spread = max(series) - min(series)
            tol = float(self.config.stable_tolerance[n])
            if spread > tol:
                unstable.append("%s spread %.6g > tolerance %.6g" % (n, spread, tol))
        if unstable:
            # Keep sliding: the deque already dropped the oldest frame.
            self._last_reason = "baselining: not stable yet (%s)" % "; ".join(unstable)
            return TactileEvent(state=BASELINING, reason=self._last_reason)

        self._baseline = {
            n: sum(f[n] for f in self._baseline_buffer) / float(have) for n in names
        }
        self._frames.clear()
        self._state = MONITORING
        self._last_reason = "monitoring; baseline %s" % (
            {k: round(v, 6) for k, v in self._baseline.items()},)
        return TactileEvent(state=MONITORING, reason=self._last_reason)

    # ------------------------------------------------------------------ monitoring
    def _passes(self, name: str, diff: float) -> bool:
        threshold = float(self.config.release_threshold[name])
        want = self.config.direction[name]
        if want == "rise":
            return diff >= threshold
        if want == "drop":
            return diff <= -threshold
        return abs(diff) >= threshold

    def _qualifies(self, name: str, avg_diff: float, latest_diff: float) -> bool:
        """Both the smoothed and the instantaneous difference must pass.

        The moving average alone is not enough: a 10-frame window stretches a
        single-frame spike into ~0.33 s at 30 Hz, which can out-last a shorter
        debounce window and trigger on a transient. Requiring the latest frame
        to agree makes "persist for debounce_sec" mean the pull is still
        happening, not merely still inside the averaging window.
        """
        return self._passes(name, avg_diff) and self._passes(name, latest_diff)

    def _monitoring(self, now: float, clean: dict, names) -> TactileEvent:
        self._frames.append(clean)
        average = self.moving_average()

        diffs, latest_diffs = {}, {}
        for n in names:
            spec = self.config.channels[n]
            diffs[n] = tf.difference(spec, average[n], self._baseline[n])
            latest_diffs[n] = tf.difference(spec, clean[n], self._baseline[n])

        qualifying = tuple(n for n in names if self._qualifies(n, diffs[n], latest_diffs[n]))

        # Adaptive baseline: absorb slow drift ONLY on channels sitting inside
        # their stable band. A channel past its tolerance keeps its baseline, so
        # a developing pull is never erased by the drift update.
        gain = float(self.config.drift_gain)
        for n in names:
            if abs(diffs[n]) <= float(self.config.stable_tolerance[n]):
                self._baseline[n] = self._baseline[n] + gain * diffs[n]

        suppressed = (
            self._armed_at is not None
            and (now - self._armed_at) < self.config.post_arm_suppress_sec
        )

        if len(qualifying) < self.config.min_channels:
            self._over_since = None
            self._last_reason = "monitoring; %d/%d channels qualifying" % (
                len(qualifying), self.config.min_channels)
            return TactileEvent(state=MONITORING, reason=self._last_reason,
                                diffs=diffs, qualifying=qualifying)

        if suppressed:
            # Post-arm settling: the arm's own deceleration can look like a pull.
            self._over_since = None
            self._last_reason = (
                "monitoring; %d channel(s) qualifying but still inside the %.2fs post-arm "
                "suppression window" % (len(qualifying), self.config.post_arm_suppress_sec))
            return TactileEvent(state=MONITORING, reason=self._last_reason,
                                diffs=diffs, qualifying=qualifying)

        if self._over_since is None:
            self._over_since = now
        held = now - self._over_since

        if held < float(self.config.debounce_sec):
            self._last_reason = "candidate pull on %s, held %.3fs of %.3fs" % (
                ",".join(qualifying), held, self.config.debounce_sec)
            return TactileEvent(state=MONITORING, reason=self._last_reason, diffs=diffs,
                                qualifying=qualifying, candidate=True, held_sec=held)

        self._state = TRIGGERED
        self._fired = True
        self._last_reason = "pull verified on %s after %.3fs (diffs %s)" % (
            ",".join(qualifying), held, {k: round(diffs[k], 6) for k in qualifying})
        return TactileEvent(state=TRIGGERED, trigger=True, reason=self._last_reason,
                            diffs=diffs, qualifying=qualifying, held_sec=held)
