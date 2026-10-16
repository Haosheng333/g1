"""Multi-channel force pull detector. One instance per hand. No ROS, no SDK.

DETECTION RULE -- consecutive frames, any channel (requested 2026-10-14)
Each valid frame is compared against the one before it, per channel:

    delta[c] = difference(frame[c], reference[c])     for each enabled channel c

  * if ANY enabled channel's delta passes its threshold, ONE release request is
    generated and the detector latches TRIGGERED;
  * if NO channel passes, the reference frame is replaced by this frame and
    monitoring continues.

It is an ANY-CHANNEL condition. Deltas are never summed across fingers: a sum
would let several sub-threshold fingers add up to a release nobody pulled, and
would also let one real pull be diluted by quiet channels.

This replaces an earlier design built on a 10-frame moving average plus an
adaptive baseline with a drift gain. That machinery is gone, not merely
bypassed: with a one-frame reference there is no window to average over and no
slow-drift term to absorb, because the reference is rewritten on every
non-qualifying frame -- which IS the drift tracking, exactly as requested.

    IDLE --arm()--> REFERENCING --1 valid frame--> MONITORING --delta--> TRIGGERED
      ^                  |                             |                     |
      +---- timeout -----+----------- timeout ---------+----- disarm() ------+
                         ^                             |
                         +------- invalid frame -------+  (fault: fresh reference)

SIGN CONVENTION -- preserved from this project, not reinvented.
`tactile_frame.difference()` returns a SIGNED delta for force-like channels
(`new - reference`), and `release_monitor.DIRECTIONS` defines how a signed delta
is compared: `rise` needs delta >= +threshold, `drop` needs delta <= -threshold,
`either` needs abs(delta) >= threshold. That mechanism is kept verbatim.

What this project never defined is which of the three applies to a human pull,
because that depends on how the hand's force channels are wired. `either` is the
default here, since "delta exceeds 3 N" reads as a magnitude, and it is the only
choice that cannot miss a real pull by guessing the sign wrong. It stays
per-channel configurable. CONFIRM THE SIGN ON THE ROBOT and set `direction`
explicitly once the SDK's channel semantics are known.

THRESHOLD. `release_threshold` defaults to DEFAULT_RELEASE_THRESHOLD (3.0) per
enabled channel: the requested setting, in the SDK's own force unit. It is NOT a
measured hardware figure, and the unit is unconfirmed -- see handover/README.md.
The detector still refuses to arm with no channels declared, because a threshold
is meaningless without knowing what it is thresholding.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from handover import tactile_frame as tf
from handover.release_monitor import (
    DIRECTIONS,
    IDLE,
    MONITORING,
    TRIGGERED,
    Event,
    Outcome,
)
from handover.release_monitor import BASELINING as REFERENCING

__all__ = [
    "TactileDetectorConfig", "TactileReleaseDetector", "DEFAULT_RELEASE_THRESHOLD",
    "DEFAULT_DIRECTION", "IDLE", "REFERENCING", "MONITORING", "TRIGGERED",
]

#: Requested per-channel release threshold, in the SDK's force unit. A requested
#: setting, not a measured hardware characteristic.
DEFAULT_RELEASE_THRESHOLD = 3.0

#: Compare the magnitude of the signed delta. See "SIGN CONVENTION" above.
DEFAULT_DIRECTION = "either"


@dataclass
class TactileDetectorConfig:
    """Per-hand detector settings for the consecutive-frame rule."""

    #: channel name -> ChannelSpec. Declared by configuration, never assumed.
    channels: dict = field(default_factory=dict)
    #: Channel mask: names the detector may use. Empty = every triggerable channel.
    enabled_channels: tuple = ()

    #: channel -> delta that counts as a pull. Missing entries take
    #: DEFAULT_RELEASE_THRESHOLD (the requested 3 N).
    release_threshold: dict = field(default_factory=dict)
    #: channel -> rise | drop | either. Missing entries take DEFAULT_DIRECTION.
    direction: dict = field(default_factory=dict)

    #: No trigger within this long of arming: the arm's own deceleration can
    #: produce a frame-to-frame force step that is not a human pull. The
    #: reference still advances while suppressed, so a suppressed step is not
    #: re-counted afterwards.
    post_arm_suppress_sec: float = 1.0

    #: A gap longer than this between frames is a FAULT, never evidence: across
    #: a long gap "consecutive" no longer means consecutive.
    sample_timeout_sec: float = 0.5
    #: Overall arm-to-trigger budget. 0 disables.
    monitor_timeout_sec: float = 120.0
    #: Give up waiting for the first valid reference frame after this long.
    reference_timeout_sec: float = 10.0

    # ------------------------------------------------------------------ helpers
    def threshold_for(self, name: str) -> float:
        value = self.release_threshold.get(name)
        return DEFAULT_RELEASE_THRESHOLD if value is None else float(value)

    def direction_for(self, name: str) -> str:
        value = self.direction.get(name)
        return DEFAULT_DIRECTION if value is None else value

    def active_channels(self) -> list:
        """Enabled channels the detector may trigger on, in order."""
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
                "no force channels declared (the SDK force schema is unconfirmed; see "
                "handover/README.md). A 3 N threshold is meaningless until we know "
                "which channels it applies to and in what unit."
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
            threshold = self.release_threshold.get(name)
            if threshold is not None:
                if isinstance(threshold, bool) or not isinstance(threshold, (int, float)) \
                        or not math.isfinite(float(threshold)) or float(threshold) <= 0.0:
                    problems.append(
                        "release_threshold[%r] must be a finite positive number" % (name,))
            configured = self.direction.get(name)
            if configured is not None and configured not in DIRECTIONS:
                problems.append(
                    "direction[%r] must be one of %s" % (name, ", ".join(DIRECTIONS)))

        if self.post_arm_suppress_sec < 0.0:
            problems.append("post_arm_suppress_sec must not be negative")
        if self.sample_timeout_sec <= 0.0:
            problems.append("sample_timeout_sec must be positive")
        if self.reference_timeout_sec <= 0.0:
            problems.append("reference_timeout_sec must be positive")
        if self.monitor_timeout_sec < 0.0:
            problems.append("monitor_timeout_sec must not be negative (0 disables)")
        return problems


@dataclass
class TactileEvent(Event):
    """An Event plus per-channel detail, for diagnostics and recordings."""

    #: channel -> signed delta of this frame from the reference frame.
    diffs: dict = field(default_factory=dict)
    #: channels whose delta passed their threshold in the configured direction.
    qualifying: tuple = ()
    #: True when the frame was rejected (missing/stale/malformed/non-finite).
    fault: bool = False


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
        """Clear the reference frame and any latched trigger."""
        self._reference = {}
        self._armed_at = None
        self._last_stamp = None
        self._fired = False

    @property
    def state(self) -> str:
        return self._state

    @property
    def reference(self) -> dict:
        """The frame the next delta will be measured against."""
        return dict(self._reference)

    @property
    def last_reason(self) -> str:
        return self._last_reason

    @property
    def armed(self) -> bool:
        return self._state in (REFERENCING, MONITORING)

    @property
    def fired(self) -> bool:
        return self._fired

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
        self._state = REFERENCING
        self._last_reason = "armed; waiting for the first valid reference frame"
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
        if self._state == REFERENCING and self._armed_at is not None:
            if now - self._armed_at > self.config.reference_timeout_sec:
                reason = "TIMEOUT: no valid reference frame within %.1fs" % (
                    self.config.reference_timeout_sec,)
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
                now,
                "force data went stale (gap > %.3fs), so the next frame is not "
                "consecutive with the reference" % self.config.sample_timeout_sec)

        if self._state == REFERENCING:
            return self._take_reference(check.clean, names)
        return self._monitoring(now, check.clean, names)

    # ------------------------------------------------------------------ fault path
    def _fault(self, now: float, reason: str) -> TactileEvent:
        """Invalid data is never evidence, and costs us the reference frame.

        The detector drops back to REFERENCING, so a delta is only ever measured
        between two frames that were both valid and genuinely consecutive.
        """
        self._reference = {}
        self._state = REFERENCING
        self._last_reason = (
            "FAULT: %s; a fresh reference frame is required before detection resumes"
            % reason)
        return TactileEvent(state=REFERENCING, wait=True, fault=True, reason=self._last_reason)

    # ------------------------------------------------------------------ reference
    def _take_reference(self, clean: dict, names) -> TactileEvent:
        """The first valid frame after arming or a fault becomes the reference."""
        self._reference = dict(clean)
        self._state = MONITORING
        self._last_reason = "monitoring; reference %s" % (
            {k: round(clean[k], 6) for k in names},)
        return TactileEvent(state=MONITORING, reason=self._last_reason)

    # ------------------------------------------------------------------ monitoring
    def _passes(self, name: str, delta: float) -> bool:
        """Compare one signed delta using this project's direction convention."""
        threshold = self.config.threshold_for(name)
        want = self.config.direction_for(name)
        if want == "rise":
            return delta >= threshold
        if want == "drop":
            return delta <= -threshold
        return abs(delta) >= threshold

    def _monitoring(self, now: float, clean: dict, names) -> TactileEvent:
        diffs = {
            n: tf.difference(self.config.channels[n], clean[n], self._reference[n])
            for n in names
        }
        # ANY channel, independently. Never a sum across fingers.
        qualifying = tuple(n for n in names if self._passes(n, diffs[n]))

        suppressed = (
            self._armed_at is not None
            and (now - self._armed_at) < self.config.post_arm_suppress_sec
        )

        if not qualifying:
            # No channel qualifies: this frame becomes the new reference. That
            # single line is the whole drift-tracking story.
            self._reference = dict(clean)
            self._last_reason = "monitoring; no channel past threshold, reference advanced"
            return TactileEvent(state=MONITORING, reason=self._last_reason,
                                diffs=diffs, qualifying=())

        if suppressed:
            # Advance the reference anyway, so a suppressed step is absorbed
            # rather than re-detected the moment suppression ends.
            self._reference = dict(clean)
            self._last_reason = (
                "monitoring; %d channel(s) past threshold but still inside the %.2fs "
                "post-arm suppression window; reference advanced"
                % (len(qualifying), self.config.post_arm_suppress_sec))
            return TactileEvent(state=MONITORING, reason=self._last_reason,
                                diffs=diffs, qualifying=qualifying)

        self._state = TRIGGERED
        self._fired = True
        self._last_reason = "pull detected on %s (deltas %s)" % (
            ",".join(qualifying), {k: round(diffs[k], 6) for k in qualifying})
        return TactileEvent(state=TRIGGERED, trigger=True, reason=self._last_reason,
                            diffs=diffs, qualifying=qualifying)
