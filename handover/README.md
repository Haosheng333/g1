# handover

Tactile-triggered handover: detect a human taking the object the robot is
holding, and ask the **orchestrator** to release it.

**Detection only.** This package never opens the hand and never moves the arm.
Detection and actuation are split so a sensor glitch cannot drop an object, and
so the release passes the orchestrator's own state validation.
`test/test_node_contract.py` and `test/test_detection_only.py` enforce it.

---

## ⚠ Status: NOT_READY — the tactile API could not be verified

The detector logic is complete and tested. The **hardware path is disabled**,
because the schema it would need is unconfirmed. These are the verified facts:

| Fact | Evidence |
|---|---|
| `bc_stark_sdk` is **not installed** here (host or container), and no vendor SDK source is on disk | `python3 -c "import bc_stark_sdk"` fails in both |
| This repo's existing SDK contract uses **no tactile call** | `src/hand_control/revo_hand_link.py` header: only `auto_detect`, `init_from_detected`, `get_hand_type`, `set_finger_positions`, `get_finger_positions`, `close_device_handler` |
| Revo2 ships in **four variants**, and **Revo2 Basic has no sensor at all** | vendor docs: Basic / Touch (capacitive) / TouchForce3D / TouchArrayPressure |
| Capability probes exist, so the variant **can** be identified at runtime | `is_capacitive_touch()`, `is_pressure_touch()`, `is_force3d_touch()`, `is_array_pressure_touch()` |
| **`bc-stark-sdk` 1.x DOES support Python 3.8** — the version conflict is *not* a blocker | PyPI's **per-release** JSON gives `requires_python: ">=3.8"` for the whole 1.x line, with `cp38` wheels published. The project-level `Requires-Python >=3.9` that an earlier version of this README cited is the **latest** release's value, not 1.x's. `requirements.txt`'s `pip3 install "bc-stark-sdk<2"` is therefore correct for Noetic. |
| Exact field list, units, channel layout and rate are **not published** | the authoritative source is the SDK's `main_mod.pyi`, which we do not have |

**The schema is the blocking one, and units are part of it.** A raw reading from
the device does **not** establish a physical unit: a number arriving over the
wire could be a count, a normalised fraction or a calibrated force, and nothing
on the device says which. Units require **vendor documentation or a
calibration** — never inference from observed values. Until the field list and
units are established, tactile acquisition in `hand_control` cannot be written.

Because of this, **no tactile publisher was added to `hand_control`**: writing
one against an unverified API, for a hand whose variant is unconfirmed, and
thresholding values whose units nobody can state, would be fabricated hardware
support.

### What is needed to lift NOT_READY

1. **The hand variant.** Run the capability probes on the real device and report
   which of the four it is. If it is **Revo2 Basic there is no tactile sensor**
   and tactile-triggered handover is impossible with that hardware.
2. **`main_mod.pyi`** from the SDK's Advanced Reference (the authoritative field
   list), or the `python/revo2_tactile_grasp/` examples from
   `BrainCoTech/brainco-hand-sdk`.
3. **One sample frame**, printed raw: the field names, their types, how the five
   fingers are indexed, the units, and the observed rate.
4. **Confirmation that the 1.x wheel installs on this robot's 3.8**, which the
   per-release metadata says it should (`pip3 install "bc-stark-sdk<2"`). Only
   if that fails does the interpreter question return.

Items 1–3 are what the `tactile:` block in `config/auto_release.yaml` needs.
Item 4 decides where the publisher can live.

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
| `/auto_release_monitor/event` | `std_msgs/String` | `candidate\|…`, `confirmed\|…`, `fault\|…` |

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
| `hand_control` | acquire and publish tactile data via the vendor SDK — **not implemented, blocked above** |
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
IDLE --arm()--> BASELINING --10 stable frames--> MONITORING --verified--> TRIGGERED
  ^                 |                                 |                      |
  +--- timeout -----+------------ timeout ------------+----- disarm() -------+
                    ^                                 |
                    +--------- invalid frame ---------+   (fault: fresh baseline)
```

`src/handover/tactile_detector.py` is pure python with time injected, so every
timing rule is tested exactly. Per the framework:

* **Arming** clears samples, baseline, debounce state and pending triggers, and
  requires fresh valid data.
* **Baselining** needs 10 frames whose spread is inside each channel's
  `stable_tolerance`. A hand still vibrating is not a baseline, so this doubles
  as the settling check.
* **Post-arm suppression** blocks triggering for ≥ 1.0 s after arming, because
  the arm's own deceleration looks like a pull.
* **Monitoring** keeps a 10-frame moving average and compares it to the baseline.
* **Adaptive baseline** absorbs slow drift only on channels inside their stable
  band, and only at `drift_gain` per frame. A channel past its tolerance keeps
  its baseline, so a developing pull is never erased.
* **Debounce**: a qualifying change must persist for `debounce_sec`.
* **Masks**: `enabled_channels` and `min_channels` select what may trigger.

### Two design points worth knowing

**A channel's *kind* decides how it may be compared.** `force_like` is
subtractable and thresholdable; `angular` differences are wrapped into the
period; `opaque` (proximity, raw counts, capacitance) is recorded and published
but **never thresholded**, because its units, sign and linearity are unknown.
Asking the detector to difference an `opaque` channel raises. Promote a channel
to `force_like` only once calibration shows subtraction is meaningful.

**Triggering needs both the smoothed and the instantaneous value past
threshold.** A 10-frame average stretches a single-frame spike into ~0.33 s at
30 Hz, which can out-last a shorter debounce window and fire on a transient.
Requiring the latest frame to agree makes "persist for `debounce_sec`" mean the
pull is *still happening*.

### Invalid data is never evidence

Missing, stale, malformed, non-finite, or source-flagged-invalid frames all:
never count toward a pull; cancel any in-progress debounce; publish a
`fault|…` diagnostic; and drop the detector back to `BASELINING`, so detection
resumes only after a **fresh valid baseline**. A missing finger is a fault, not
a zero reading. `TRIGGERED` is one-shot per arming session. A timeout reports
`TIMEOUT`, never a successful handover.

## Handover sequence (orchestrator-owned)

1. Orchestrator confirms an object is held and which hand.
2. Arm reaches the handover pose, hand stays closed.
3. Wait ~1 s for deceleration vibration to settle.
4. **Only then** arm the monitor.
5. Monitor baselines and watches.
6. A verified pull sends **one** request.
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

## Calibration

**Nothing may be guessed.** `release_threshold`, `stable_tolerance`, `direction`
and `debounce_sec` have no defaults; the detector refuses to arm without them.
Derive them from recordings, then validate on *separate* recordings.

Prerequisite: the `tactile:` block in `config/auto_release.yaml` filled in from
items 1–3 above. Then, with the object held and the arm at the handover pose:

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
channel. Derive, per channel:

| Value | From |
|---|---|
| `stable_tolerance` | peak-to-peak spread over the stationary hold, with margin. Must also cover the settling recording. |
| `direction` | the sign the pull recordings move in |
| `release_threshold` | comfortably above the settling excursion and below the smallest deliberate pull. Must be **larger** than `stable_tolerance` (validated). |
| `debounce_sec` | how long the pull signature persists, minus margin, while staying under the 1 s detection target |
| `drift_gain` | validate the (gain, threshold) pair: drift tracked over 60 s, slow pull still crossing |

Fill them into the `calibration:` block, then **validate on recordings you did
not tune on**:

```bash
rosrun handover record_tactile.py --out /tmp/verify.csv --seconds 60 --detector --note "verify"
```

`--detector` adds `baseline_*`, `avg_*`, `diff_*`, `state`, `candidate`,
`fault`, `qualifying` and `event` columns, so a false trigger is traceable to
the channel and frame that caused it.

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
numbers in them are fixtures, not calibration values. Covered: arming and
baseline collection, settling/unstable frames, drift versus sudden change,
bounded drift absorption, debounce and post-arm suppression, masked and
non-force channels, stale/missing/non-finite/malformed data, timeout, disarm,
duplicate-trigger prevention, per-hand isolation, and that detection-only mode
constructs no release client.
