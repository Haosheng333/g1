# handover

Auto-release handover monitor: watches a tactile signal for the moment a human
takes the object the robot is holding, and then asks the **orchestrator** to run
the release.

**Detection only.** This package never commands the arm and never opens the
hand. Detection and actuation are split on purpose, so a sensor glitch cannot
drop an object by itself, and so the release goes through the orchestrator's
own safety gates. `test/test_node_contract.py` enforces that: the node's code
may not reference a hand, arm, planning, locomotion or navigation endpoint, and
its only service client is the orchestrator route.

## Interface

| Service | Type | Action |
|---|---|---|
| `/auto_release_monitor/arm` | `std_srvs/Trigger` | start baselining, then monitor |
| `/auto_release_monitor/disarm` | `std_srvs/Trigger` | back to `IDLE`; always safe, always succeeds |

| Topic | Type | Meaning |
|---|---|---|
| `/auto_release_monitor/state` | `std_msgs/String` (latched) | current state + reason |
| `/auto_release_monitor/ready` | `std_msgs/Bool` (latched) | can it be armed at all |

On a verified detection it calls, exactly once:

| Service | Type |
|---|---|
| `/orchestrator/run_auto_release` | `orchestrator/RunRoute` |

That type is imported **lazily**, at trigger time, so this node still starts
when the orchestrator package has not been built.

## State machine

```
IDLE --arm()--> BASELINING --enough samples--> MONITORING --verified--> TRIGGERED
  ^                  |                              |                       |
  +--- timeout ------+--------- timeout ------------+------ disarm() -------+
```

`src/handover/release_monitor.py` is pure python with time injected — no ROS,
no clock, no hardware knowledge — so every timing rule is tested exactly:

* **BASELINING** averages `baseline_samples` samples into an adaptive
  baseline. A data gap discards a partially collected baseline, because an
  average across a gap is not a baseline.
* **MONITORING** requires `debounce_samples` *consecutive* over-threshold
  samples **and** that the excursion persists for `min_hold_s`. Any sample back
  inside the threshold resets both.
* Missing, stale, non-numeric or non-finite data yields a **WAIT** event, never
  a trigger, and cancels any detection in progress.
* `baseline_timeout_s` and `monitor_timeout_s` both return the monitor to
  `IDLE`. A timeout is never reported as a release.
* `TRIGGERED` is terminal until `disarm`, so the release is requested once.

## Why this is NOT_READY out of the box

Two things are deliberately missing, and the monitor **refuses to arm** until
both are supplied — `arm` returns `success=false` with the exact list.

**1. There is no tactile topic in this workspace.** `hand_control` drives the
BrainCo Revo2 over serial, uses finger-position feedback internally
(`expect_contact` in `hand_control/config/finger_presets.yaml`) and publishes no
sensor topic at all. So the message type, field layout and units are unknown.
`src/handover/tactile_adapter.py` is therefore fully config-driven and
`enabled: false`:

```yaml
tactile:
  enabled: false
  topic: ""             # e.g. /hand_left/tactile
  message_type: ""      # "pkg/Msg", imported at runtime
  value_field: ""       # dotted path, e.g. "data" or "wrench.force.z"
  reduction: mean       # mean | max | min | sum | first | norm
```

**2. The release threshold and direction are unset.** These are physical
properties of the hand that nobody has characterised:

```yaml
release_delta:        # REQUIRED, no default
direction:            # REQUIRED, no default: rise | drop | either
```

Whether a release makes the signal rise or drop, and by how much, must be
**measured on the real hand**. Shipping a default would mean shipping a guess,
so there is none. The hand owner confirms the schema, someone measures the
magnitudes, then both blocks get filled in and the offline tests re-run.

Everything else (`baseline_samples`, `debounce_samples`, `min_hold_s`,
`sample_timeout_s`, `monitor_timeout_s`, `rate`) is signal processing rather
than hardware, and does have defaults.

## Run

```bash
roslaunch handover auto_release_monitor.launch
rostopic echo -n1 /auto_release_monitor/ready    # false until the two blocks above are filled
rosservice call /auto_release_monitor/arm        # refused, with the reason
```

## Tests

Simulated samples, no ROS and no hardware:

```bash
cd handover && python3 -m unittest discover -s test -v
```

Covers no-false-trigger (noise, wrong direction), debounce, minimum hold,
stale and missing data, both timeouts, disarm from every state, re-arm
re-baselining, trigger-exactly-once, the adapter's field extraction and
reductions, and that the **shipped config cannot be armed**.
