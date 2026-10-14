"""Adaptive-baseline release detection, with no ROS and no hardware knowledge.

    IDLE --arm()--> BASELINING --enough samples--> MONITORING --verified--> TRIGGERED
      ^                  |                              |                       |
      +-- timeout -------+------------- timeout --------+--- disarm() ----------+

The state machine consumes a single scalar per sample. Turning a real tactile
message into that scalar is `tactile_adapter`'s job, and is deliberately kept
out of here: this module contains no message type, no field name and no
hardware magnitude.

Two settings have NO default on purpose, because they are physical properties
of a hand nobody has characterised yet:

  * ``release_delta``  how far from baseline counts as a release
  * ``direction``      whether a release makes the signal rise or drop

`MonitorConfig.validate()` refuses to arm without both, so the monitor can
never fire on a guessed threshold. Everything else (sample counts, debounce,
hold and timeout durations) is signal-processing, not hardware, and does have
defaults.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

# --- states -------------------------------------------------------------------------

IDLE = "IDLE"
BASELINING = "BASELINING"
MONITORING = "MONITORING"
TRIGGERED = "TRIGGERED"

STATES = (IDLE, BASELINING, MONITORING, TRIGGERED)

#: Release direction relative to the baseline. "either" accepts both signs.
DIRECTIONS = ("rise", "drop", "either")


@dataclass
class MonitorConfig:
    """Tunables. `release_delta` and `direction` are REQUIRED (see module docstring)."""

    # --- hardware, no defaults ---
    release_delta: float | None = None
    direction: str | None = None

    # --- signal processing, safe defaults ---
    baseline_samples: int = 20          # samples averaged into the baseline
    baseline_timeout_s: float = 5.0     # give up baselining after this long
    debounce_samples: int = 3           # consecutive over-threshold samples required
    min_hold_s: float = 0.20            # ... and it must stay over for this long
    sample_timeout_s: float = 0.50      # older than this = stale, never a trigger
    monitor_timeout_s: float = 120.0    # overall arm-to-trigger budget; 0 disables

    def validate(self) -> list:
        """Return the list of reasons this config cannot be armed (empty = fine)."""
        problems = []
        if self.release_delta is None:
            problems.append(
                "release_delta is not configured (hardware threshold; must be measured "
                "on the real hand, not guessed)"
            )
        elif not (isinstance(self.release_delta, (int, float)) and not isinstance(self.release_delta, bool)):
            problems.append("release_delta must be a number, got %r" % (self.release_delta,))
        elif not math.isfinite(float(self.release_delta)) or float(self.release_delta) <= 0.0:
            problems.append("release_delta must be a finite positive number, got %r" % (self.release_delta,))

        if self.direction is None:
            problems.append(
                "direction is not configured (does a release make the signal rise or "
                "drop? must be confirmed on the real hand)"
            )
        elif self.direction not in DIRECTIONS:
            problems.append(
                "direction must be one of %s, got %r" % (", ".join(DIRECTIONS), self.direction)
            )

        if self.baseline_samples < 1:
            problems.append("baseline_samples must be at least 1")
        if self.debounce_samples < 1:
            problems.append("debounce_samples must be at least 1")
        if self.min_hold_s < 0.0:
            problems.append("min_hold_s must not be negative")
        if self.sample_timeout_s <= 0.0:
            problems.append("sample_timeout_s must be positive")
        if self.baseline_timeout_s <= 0.0:
            problems.append("baseline_timeout_s must be positive")
        if self.monitor_timeout_s < 0.0:
            problems.append("monitor_timeout_s must not be negative (0 disables the budget)")
        return problems


@dataclass
class Outcome:
    """Result of arm()/disarm(): did it happen, and why not."""

    ok: bool
    reason: str = ""


@dataclass
class Event:
    """Result of one update() call."""

    state: str
    #: True exactly once, on the update that verifies a release.
    trigger: bool = False
    #: WAIT when data is missing or stale; the caller must not treat it as success.
    wait: bool = False
    reason: str = ""
    baseline: float | None = None
    delta: float | None = None


@dataclass
class ReleaseMonitor:
    """Adaptive-baseline monitor. Time is injected; it never reads a clock."""

    config: MonitorConfig = field(default_factory=MonitorConfig)

    def __post_init__(self) -> None:
        self._state = IDLE
        self._reset_run()
        self._last_reason = "never armed"

    # ------------------------------------------------------------------ internals
    def _reset_run(self) -> None:
        self._baseline_buffer: list = []
        self._baseline: float | None = None
        self._armed_at: float | None = None
        self._last_sample_t: float | None = None
        self._over_count = 0
        self._over_since: float | None = None
        self._triggered = False

    def _exceeds(self, value: float) -> bool:
        """Is `value` a release-sized excursion from the baseline?"""
        if self._baseline is None:
            return False
        delta = float(value) - self._baseline
        magnitude = float(self.config.release_delta)
        if self.config.direction == "rise":
            return delta >= magnitude
        if self.config.direction == "drop":
            return delta <= -magnitude
        return abs(delta) >= magnitude

    # ------------------------------------------------------------------ properties
    @property
    def state(self) -> str:
        return self._state

    @property
    def baseline(self):
        return self._baseline

    @property
    def last_reason(self) -> str:
        return self._last_reason

    @property
    def armed(self) -> bool:
        return self._state in (BASELINING, MONITORING)

    # ------------------------------------------------------------------ commands
    def arm(self, now: float) -> Outcome:
        """Begin baselining. Refuses while already armed or mis-configured."""
        problems = self.config.validate()
        if problems:
            self._last_reason = "NOT_READY: " + "; ".join(problems)
            return Outcome(False, self._last_reason)
        if self._state in (BASELINING, MONITORING):
            self._last_reason = "already armed (state %s)" % self._state
            return Outcome(False, self._last_reason)

        self._reset_run()
        self._armed_at = float(now)
        self._state = BASELINING
        self._last_reason = "armed; collecting %d baseline samples" % self.config.baseline_samples
        return Outcome(True, self._last_reason)

    def disarm(self, now: float = 0.0, reason: str = "disarmed by request") -> Outcome:
        """Return to IDLE from any state. Always succeeds; never moves anything."""
        was = self._state
        self._reset_run()
        self._state = IDLE
        self._last_reason = reason
        return Outcome(True, "%s (was %s)" % (reason, was))

    # ------------------------------------------------------------------ sampling
    def update(self, now: float, sample=None) -> Event:
        """Feed one sample (or None for "no data this tick") and advance the state.

        `sample` must already be the extracted scalar. A None sample, or a
        sample older than `sample_timeout_s`, yields a WAIT event and can never
        produce a trigger.
        """
        now = float(now)

        if self._state in (IDLE, TRIGGERED):
            return Event(state=self._state, reason=self._last_reason, baseline=self._baseline)

        # Overall budget, checked before anything else so a silent sensor cannot
        # hold the monitor armed forever.
        if self.config.monitor_timeout_s > 0.0 and self._armed_at is not None:
            if now - self._armed_at > self.config.monitor_timeout_s:
                reason = "timeout: no verified release within %.1fs of arming" % (
                    self.config.monitor_timeout_s,
                )
                self.disarm(now, reason)
                return Event(state=IDLE, reason=reason)

        # Baselining has its own, shorter budget. Checked here rather than in
        # _baselining() so a sensor that goes quiet mid-baseline still times
        # out instead of waiting for the overall budget.
        if self._state == BASELINING and self._armed_at is not None:
            if now - self._armed_at > self.config.baseline_timeout_s:
                reason = "timeout: only %d of %d baseline samples within %.1fs" % (
                    len(self._baseline_buffer),
                    self.config.baseline_samples,
                    self.config.baseline_timeout_s,
                )
                self.disarm(now, reason)
                return Event(state=IDLE, reason=reason)

        if sample is None:
            return self._on_missing(now, "no tactile sample available")
        try:
            value = float(sample)
        except (TypeError, ValueError):
            return self._on_missing(now, "tactile sample is not numeric: %r" % (sample,))
        if not math.isfinite(value):
            return self._on_missing(now, "tactile sample is not finite: %r" % (sample,))

        stale = (
            self._last_sample_t is not None
            and (now - self._last_sample_t) > self.config.sample_timeout_s
        )
        self._last_sample_t = now
        if stale:
            return self._on_missing(
                now, "tactile data went stale (gap > %.2fs)" % self.config.sample_timeout_s
            )

        if self._state == BASELINING:
            return self._baselining(now, value)
        return self._monitoring(now, value)

    def _on_missing(self, now: float, reason: str) -> Event:
        """Missing or stale data: WAIT, drop any in-progress detection."""
        self._over_count = 0
        self._over_since = None
        if self._state == BASELINING:
            # A gap invalidates a partially collected baseline: an average over
            # samples straddling a gap is not a baseline.
            self._baseline_buffer = []
        self._last_reason = "WAIT: " + reason
        return Event(state=self._state, wait=True, reason=self._last_reason, baseline=self._baseline)

    def _baselining(self, now: float, value: float) -> Event:
        self._baseline_buffer.append(value)
        if len(self._baseline_buffer) < self.config.baseline_samples:
            self._last_reason = "baselining %d/%d" % (
                len(self._baseline_buffer),
                self.config.baseline_samples,
            )
            return Event(state=BASELINING, reason=self._last_reason)

        self._baseline = sum(self._baseline_buffer) / float(len(self._baseline_buffer))
        self._baseline_buffer = []
        self._state = MONITORING
        self._last_reason = "monitoring; baseline %.6g" % self._baseline
        return Event(state=MONITORING, reason=self._last_reason, baseline=self._baseline)

    def _monitoring(self, now: float, value: float) -> Event:
        delta = value - float(self._baseline)
        if not self._exceeds(value):
            self._over_count = 0
            self._over_since = None
            self._last_reason = "monitoring; delta %.6g within threshold" % delta
            return Event(
                state=MONITORING, reason=self._last_reason, baseline=self._baseline, delta=delta
            )

        self._over_count += 1
        if self._over_since is None:
            self._over_since = now

        if self._over_count < self.config.debounce_samples:
            self._last_reason = "candidate release, debounce %d/%d" % (
                self._over_count,
                self.config.debounce_samples,
            )
            return Event(
                state=MONITORING, reason=self._last_reason, baseline=self._baseline, delta=delta
            )

        held = now - self._over_since
        if held < self.config.min_hold_s:
            self._last_reason = "candidate release, held %.3fs of %.3fs" % (
                held,
                self.config.min_hold_s,
            )
            return Event(
                state=MONITORING, reason=self._last_reason, baseline=self._baseline, delta=delta
            )

        self._state = TRIGGERED
        self._triggered = True
        self._last_reason = (
            "release verified: delta %.6g beyond %.6g (%s), %d consecutive samples, held %.3fs"
            % (delta, float(self.config.release_delta), self.config.direction,
               self._over_count, held)
        )
        return Event(
            state=TRIGGERED,
            trigger=True,
            reason=self._last_reason,
            baseline=self._baseline,
            delta=delta,
        )
