# handover

Tactile-triggered handover: detect a human taking the object the robot is
holding, and ask the **orchestrator** to release it.

**Detection only.** This package never opens the hand and never moves the arm.
Detection and actuation are split so a sensor glitch cannot drop an object, and
so the release passes the orchestrator's own state validation.
`test/test_node_contract.py` and `test/test_detection_only.py` enforce it.

---

## ⚠ Status: NOT_READY — the force schema is unconfirmed

The detector logic is complete and tested. The **hardware path is disabled**,
and the reason is now narrower than it was: the hand and the SDK are settled,
the *data shape* is not.

### Settled as of 2026-10-14 — no longer open questions

| Fact | Consequence |
|---|---|
| The robot is a **Unitree G1 23-DOF** and the installed hand is a **BrainCo Revo2** | "identify the robot/hand model for SDK installation" is **closed**. Do not re-raise it. |
| The **SDK and runtime are available**, and the team has already used this hardware configuration for IK | SDK availability and interpreter version are **closed**. The former Python 3.8-vs-3.9 concern is moot. |
| Force feedback will be read **directly through the SDK** | No separate ROS tactile-publisher implementation is requested at this stage. |

Settling the hand model does **not** settle two other things, which stay open on
their own evidence: **collision-model compatibility** (the URDFs model the stock
Unitree `*_wrist_roll_rubber_hand`, not a Revo2 — see
`arm_planning/config/arm_kinematics.yaml`) and **orchestrator-to-SDK
integration** (nothing has run against the real SDK yet).

### Still blocking — the schema, not the hardware

| Unknown | Why it blocks |
|---|---|
| Which SDK call returns force, and its field names and types | the adapter cannot be pointed at anything |
| The channel → finger mapping | a per-channel threshold needs channels |
| The **unit** | a number from the device does not state its own unit. It could be newtons, raw counts or a normalised fraction, and **nothing on the device says which**. Units need vendor documentation or a calibration — never inference from observed values. The requested 3 N threshold is uninterpretable until the unit is known. |
| The frame rate | the consecutive-frame rule's sensitivity is per frame, so the rule's effective N/s depends on it |

Because of this, **no force publisher was written against a guessed API**. That
would be fabricated hardware support.

### What is needed to lift NOT_READY

Read on the real robot and report:

1. **The SDK force call** — its name, and the type it returns.
2. **One sample frame, printed raw** — field names, types, how the fingers are
   indexed, and the observed rate.
3. **The unit**, from vendor documentation or a calibration. Not inferred.
4. **The sign a human pull produces** on each channel, which fixes `direction`
   (see "Sign convention" below).

Items 1–2 are what the `tactile:` block in `config/auto_release.yaml` needs;
item 3 makes the 3 N threshold meaningful; item 4 lets `direction` be set
explicitly instead of defaulting to a magnitude compare.

---

## Interface

| Service | Type | Action |
|---|---|---|
| `/auto_release_monitor/arm` | `handover/ArmMonitor` | start monitoring: `side` + `session_token` in, the token echoed back |
| `/auto_release_monitor/disarm` | `std_srvs/Trigger` | back to `IDLE`; always safe |
| `/auto_release_monitor/<side>/arm` | `handover/ArmMonitor` | side-qualified alias |
| `/auto_release_monitor/<side>/disarm` | `std_srvs/Trigger` | side-qualified alias |

| Topic | Type | Meaning |
|---|---|---|
| `/auto_release_monitor/state` | `std_msgs/String` (latched) | state + reason |
| `/auto_release_monitor/ready` | `std_msgs/Bool` (latched) | can it be armed |
| `/auto_release_monitor/event` | `std_msgs/String` | `confirmed\|…`, `fault\|…`. There is no `candidate\|…`: the rule decides on one frame, so there is no debounce window to report progress through. The `confirmed\|` prefix is unchanged. |

One node per hand, launched `side:=left|right`, the same shape as
`hand_control`. The unqualified names are kept so the orchestrator's existing
preflight and tests are unaffected.

With `release_integration:=true`, a verified pull makes **one** call to
`/orchestrator/run_auto_release` (`orchestrator/RunRoute`, the type already on
this branch). That service only **validates and records** the event for the task
that owns the execution lock — see `orchestrator/README.md`.

## Ownership

| Package | Responsibility |
|---|---|
| `hand_control` | own the SDK connection to the hand. Force acquisition is **not implemented** — blocked on the schema above, not on the hardware |
| `handover` | detect a human taking the object |
| `orchestrator` | own task state, validate release requests, sequence release and retraction |

`handover` never opens the hand or moves the arm.

**Motor position, motor current and `expect_contact` are not tactile force.**
`hand_control`'s `expect_contact` infers "something is in the hand" from fingers
stopping short of their target. That is a grasp-success check, not a force
measurement, and it cannot see a human pulling on a held object. It is not used
here and must not be substituted.

## Detector

```
IDLE --arm()--> REFERENCING --1 valid frame--> MONITORING --delta--> TRIGGERED
  ^                  |                             |                     |
  +---- timeout -----+----------- timeout ---------+----- disarm() ------+
                     ^                             |
                     +------- invalid frame -------+  (fault: fresh reference)
```

### The rule: consecutive frames, any channel

Each valid frame is compared with **the frame before it**, per channel:

* if **any** enabled channel's delta passes its threshold, **one** release
  request is produced and the detector latches `TRIGGERED`;
* if **no** channel passes, that frame becomes the new reference and monitoring
  continues.

It is an **any-channel** condition. Deltas are **never summed across fingers**: a
sum would let several sub-threshold fingers add up to a release nobody asked
for, and would equally let one real pull be diluted by quiet channels. The test
`test_channels_are_never_summed` pins this — two channels at 2 each do not
cross a 3 threshold.

Advancing the reference on every non-qualifying frame **is** the drift tracking.
There is no separate baseline to maintain and nothing to tune for drift.

`src/handover/tactile_detector.py` is pure python with time injected, so every
timing rule is tested exactly.

* **Arming** clears the reference and any latched trigger, and requires fresh
  valid data.
* **Referencing** takes the first valid frame as the reference; that is all it
  needs.
* **Post-arm suppression** blocks triggering for ≥ 1.0 s after arming, because
  the arm's own deceleration can produce a frame-to-frame step that is not a
  pull. The reference still advances while suppressed, so a suppressed step is
  absorbed rather than re-detected the moment the window ends.
* **Masks**: `enabled_channels` selects what may trigger.

This **replaces** an earlier design built on a 10-frame moving average plus an
adaptive baseline with a drift gain, and the earlier `debounce_sec` and
`min_channels`. That machinery is gone, not bypassed.

### Threshold and sign convention

**Threshold: 3 N per channel**, `tactile_detector.DEFAULT_RELEASE_THRESHOLD`.
This is the **requested setting**, not a measured hardware figure, and it is in
whatever unit the SDK reports — which is not yet confirmed. `release_threshold`
overrides it per channel.

**Sign: a signed delta plus a per-channel `direction`**, preserved from this
project rather than reinvented. `tactile_frame.difference()` returns
`new - reference` for force-like channels, and `release_monitor.DIRECTIONS`
defines the comparison:

| `direction` | passes when |
|---|---|
| `rise` | `delta >= +threshold` |
| `drop` | `delta <= -threshold` |
| `either` | `abs(delta) >= threshold` |

Which one a human pull produces depends on how the hand's force channels are
wired, and **this project never defined it**. `either` is the default, because
"delta exceeds 3 N" reads as a magnitude and it is the only choice that cannot
miss a real pull by guessing the sign wrong. **Set `direction` explicitly once
the SDK semantics are confirmed.**

### Two properties worth knowing

**A channel's *kind* decides how it may be compared.** `force_like` is
subtractable and thresholdable; `angular` differences are wrapped into the
period; `opaque` (proximity, raw counts, capacitance) is recorded and published
but **never thresholded**, because its units, sign and linearity are unknown.
Asking the detector to difference an `opaque` channel raises.

**A pull slower than the threshold per frame is invisible to this rule.** The
reference advances every non-qualifying frame, so at 30 Hz and 3 N anything
under roughly 90 N/s is tracked rather than detected. That is inherent to the
requested rule, not a defect; `test_a_slow_ramp_never_triggers` records it so it
cannot surprise anyone later. Whether real taking actions clear that rate is a
question for the first real recordings.

### Invalid data is never evidence

Missing, stale, malformed, non-finite, or source-flagged-invalid frames all:
never count toward a pull; discard the reference frame; publish a `fault|…`
diagnostic; and drop the detector back to `REFERENCING`, so detection resumes
only after a **fresh valid reference**. That matters more under this rule than
under the old one: a delta measured across a dropout is not a delta between
consecutive frames, so the frame after a fault can only re-reference, never
trigger (`test_the_frame_after_a_fault_only_re_references`). A gap longer than
`sample_timeout_sec` is itself a fault, for the same reason. A missing finger is
a fault, not a zero reading. `TRIGGERED` is one-shot per arming session. A
timeout reports `TIMEOUT`, never a successful handover.

## Handover sequence (orchestrator-owned)

1. Orchestrator confirms an object is held and which hand.
2. Arm reaches the handover pose, hand stays closed.
3. Wait ~1 s for deceleration vibration to settle.
4. **Only then** arm the monitor.
5. Monitor takes one reference frame, then compares each frame with the one
   before it.
6. A qualifying delta sends **one** request.
7. Orchestrator validates its own handover state before allowing release.

The detector is never armed during arm movement: the orchestrator only accepts
release requests in its `AWAITING_PULL` phase, and rejects them in
`MOVING_TO_POSE`. Nothing releases because a timer expired.

## Detection-only mode (the default)

With `release_integration:=false`:

* real tactile readings are consumed and the detector runs normally;
* state, candidate, confirmed and fault events are published;
* **no release client of any kind is constructed** — the node cannot open the
  hand and cannot ask the orchestrator to;
* `orchestrator.srv` is not even imported.

`test_detection_only.py` drives a synthetic pull to a confirmed trigger and
asserts that zero service clients were built. Calibrate in this mode.

## Run

```bash
roslaunch handover auto_release_monitor.launch side:=right
rostopic echo -n1 /auto_release_monitor/ready     # false until the blockers above are cleared
rostopic echo /auto_release_monitor/event
# Arming needs the side and the orchestrator's session token; it is refused
# without one, and refused again while the detector is NOT_READY.
rosservice call /auto_release_monitor/arm "{side: right, session_token: abc123}"
```

## Recording and checking

The threshold is **given** (3 N) rather than derived, so this is no longer a
calibration exercise — it is a check that the requested setting behaves on real
data, plus the two things that genuinely must be read off the hardware: the
**unit** and the **sign**.

Prerequisite: the `tactile:` block in `config/auto_release.yaml` filled in from
items 1–2 above. Then, with the object held and the arm at the handover pose:

```bash
# 1. stationary hold, >= 60 s. Gives you the noise floor and the drift rate.
rosrun handover record_tactile.py --out /tmp/hold.csv  --seconds 60 --note "stationary hold"

# 2. arm settling: start recording BEFORE the arm reaches the pose.
rosrun handover record_tactile.py --out /tmp/settle.csv --seconds 20 --note "settling"

# 3. deliberate pulls, several directions, one per recording.
rosrun handover record_tactile.py --out /tmp/pull_up.csv    --seconds 20 --note "pull up"
rosrun handover record_tactile.py --out /tmp/pull_out.csv   --seconds 20 --note "pull outward"
rosrun handover record_tactile.py --out /tmp/pull_side.csv  --seconds 20 --note "pull sideways"

# 4. dropouts: unplug the sensor / power-cycle the hand mid-recording.
rosrun handover record_tactile.py --out /tmp/dropout.csv --seconds 30 --note "disconnect"
```

Each row carries `wall_sec`, the frame stamp, `source_valid` and every declared
channel. From them, read off:

| What | From |
|---|---|
| the **unit** | vendor documentation or a calibration. **Not** the recordings — a number does not state its own unit. |
| `direction`, per channel | the sign the deliberate-pull recordings move in |
| whether **3 N** is right | the stationary hold and settling recordings must stay inside it frame-to-frame; the deliberate pulls must cross it on at least one channel |
| the effective rate | the frame stamps. 3 N per frame at 30 Hz is ~90 N/s; a slower taking action will not be detected. |

Set `direction` in the `calibration:` block, override `release_threshold` only
if the recordings show 3 N is wrong, then **check on recordings you did not look
at while deciding**:

```bash
rosrun handover record_tactile.py --out /tmp/verify.csv --seconds 60 --detector --note "verify"
```

`--detector` adds `reference_*`, `delta_*`, `state`, `fault`, `qualifying` and
`event` columns, so a false trigger is traceable to the channel and frame that
caused it.

### Acceptance targets — to measure, not achieved

The framework's intended targets, neither of which has been measured because
there is no tactile data:

* **zero** false releases/detections during a 60 s stationary hold;
* detection **within 1 s** of a deliberate taking action.

Treat them as the gate for enabling `release_integration`, not as a claim.

## Tests

```bash
cd handover && python3 -m unittest discover -s test -v
```

Offline, no ROS and no hardware. **Every sample in them is synthetic and
fabricated in the test file; none of it is hardware validation**, and the
numbers in them are fixtures, not measured values. Covered: the readiness gate
and the requested defaults, the consecutive-frame rule (step past threshold,
step below it advancing the reference, non-accumulation of sub-threshold steps,
the slow-ramp blind spot, never summing channels, any-single-channel
sufficiency), the signed-delta/`direction` convention in all three modes,
channel kinds, post-arm suppression and the absorption of a suppressed step,
stale/missing/non-finite/malformed data, that a delta is never measured across a
fault, timeouts, disarm and re-arm, one-shot triggering, per-hand isolation, and
that detection-only mode constructs no release client.
