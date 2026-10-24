# orchestrator

Declarative sequencing for the grasp-and-deliver pipeline. It composes the
other modules' published interfaces and **contains no robotics of its own** —
no IK, no TF maths, no path planning, no perception, no arm or hand control, no
navigation. `test/test_node_safety.py` enforces that: no source file in this
package may import numpy, scipy, torch or tf2_ros, or mention a costmap, a
planner, a quaternion conversion or a Unitree topic.

## Interface

| Service | Type | Route |
|---|---|---|
| `/orchestrator/run_navigate_report` | `orchestrator/RunRoute` | navigate → perceive → report |
| `/orchestrator/run_grasp` | `orchestrator/RunRoute` | full grasp (**NOT_READY**, see below) |
| `/orchestrator/run_deliver` | `orchestrator/RunRoute` | carry to a waypoint, arm the handover monitor |
| `/orchestrator/run_deliver_task` | `orchestrator/RunRoute` | grasp then deliver, one lock (**NOT_READY**) |
| `/orchestrator/run_fetch_deliver` | `orchestrator/RunRoute` | **the spoken task** on arm_grasp + handover_node, see below |
| `/orchestrator/run_auto_release` | `orchestrator/RunRoute` | validate and record a release request |

`/orchestrator/status` (`std_msgs/String`, latched) reports the current phase.
`/orchestrator/run_auto_release` is what `handover`'s monitor calls after it
verifies a release.

Request: `route` (optional guard), `target`, `destination`, `label`,
`start_step` (resume point), `dry_run`, `event_token`. Response: `success`,
`message`, `steps_completed`, `steps_skipped`, `blockers`, `final_step`,
`motion_issued`, `dry_run`, `object_state`, `transfer_verified`.

## `fetch_deliver`: the spoken task

```
"go to the desk, pick up a can, go to the chair, and pass me, over"
    voice_control -> /orchestrator/run_fetch_deliver
        {target: table_01, destination: chair_01, label: bottle}
```

This is g1_intellect `main_process`'s navigate -> grasp -> navigate -> handover
plan, sequenced through DIP's own interfaces, and it is a **separate route**
because it runs on a different stack from `deliver_task`: the modules that have
actually been exercised on this robot.

| Step | Interface | Owner |
|---|---|---|
| `await_localization` | `/slam/manager/state` | slam |
| `enable_motion` | `/navigation/enable_motion` | navigation |
| `navigate_to_pickup` | **standing pose** `target` -> `/move_base_simple/goal`, saw_active guard on `/navigation/status` | slam (`standing_waypoints.yaml`) + navigation |
| `settle_at_pickup` | sleep `timeouts.nav_settle_sec` (1.5 s) | |
| `detect_object` | `/perception/get_object_position(label)`, in `torso_link` | perception |
| `plan_grasp_point` | `/g1_arm_simple/plan_grasp_from_point`, **no motion** | arm_grasp |
| `execute_grasp` | `/g1_arm_simple/execute_grasp_from_point`: pregrasp, hand `pre_grasp`, approach, `grasp_5f`, lift, retract, home | arm_grasp (+ g1_hands on PC2) |
| `record_held_object` | `HandoverState.begin()` | this package |
| `hand_back_arms` | `/g1_arm_simple/release_arm_control` (best effort, see below) | arm_grasp |
| `navigate_to_recipient` | standing pose `destination` | slam + navigation |
| `settle_at_recipient` | sleep | |
| `handover_start` | `/handover/start`: held arm to the handover pose, g1_hands' tactile monitor armed | handover (`handover_node.py`) |
| `await_handover` | `/handover/state` until `done`; timeout -> `/handover/abort`, **nothing released** | handover |
| `hand_back_arms_after_handover` | `/g1_arm_simple/release_arm_control` | arm_grasp |

What is different from the older routes, and why:

* **No frame conversion.** perception reports in `torso_link`, which is the
  frame arm_grasp plans in. The `perception_extrinsics` / frame-contract
  blockers of `grasp` do not apply; the extrinsics were fitted and measured for
  this camera (`perception/README.md`).
* **Standing poses, not `semantic.yaml`.** `target` and `destination` are names
  from `slam/maps/<map>/standing_waypoints.yaml` (recorded with
  `rosrun g1_slam record_waypoints.py`), driven through navigation's exact-pose
  topic with **no approach offset**, via `g1_slam.standing_goal` -- the same
  code `goto_waypoint.py` uses. The node resolves both names **before
  preflight** and checks the file's map identity against the live manifest, so
  a missing record or a map mismatch is a blocker, not a surprise after the
  first drive. `~map_name` (launch arg `map_name`) selects the map.
* **The grasp is arm_grasp's.** `plan_grasp_point` plans everything with the
  arm parked; `execute_grasp` then runs the whole pipeline inside
  `g1_arm_simple`, exactly as `grasp_point.py grasp` does. A failed grasp
  leaves the arm under SDK control; the message says to park it with
  `rosrun arm_grasp grasp_point.py home`.
* **Arm hand-back before walking.** `main_process` releases the arms to the
  G1's own controller before every walking leg (`@walking_task`) and after
  every arm task. `release_arm_control` does the same; arm_grasp refuses it
  unless both arms are at home, in which case the step reports that and the
  robot walks with the SDK holding the arm, as main_process does. It is the
  one step that never fails the route.
* **The handover is handover_node's.** `/handover/start` returns once the arm
  is at the pose and the monitor armed; `await_handover` then only *watches*
  `/handover/state`. The release (hand open, settle, `GoHomeArm`) runs inside
  handover_node when the monitor detects the pull. `done` records a
  **commanded** release and closes the task record; `error:*` marks the object
  `UNCERTAIN`; a timeout (`timeouts.handover_sec`, 120 s) aborts the handover
  -- monitor disarmed, hand still closed, arm left at the pose -- and the route
  fails with the manual command (`rosrun handover handover_cli.py release`).
  Cleanup aborts an open handover on every other exit path too.
* **One arm, declared twice.** The orchestrator's `side` is the grasping arm;
  `handover.launch arm:=` must be the same. The node reads `/handover/arm` and
  reports a mismatch as a blocker.
* **Preflight** requires `localization`, `map`, `joint_states`, `navigation`,
  `motion_gate`, `perception`, `arm_grasp` (the three `g1_arm_simple`
  services), `remote_hands` (`/g1_hand_<side>/{pre_grasp,grasp_5f,release}`,
  started with `rosrun hand_control start_remote_hands.sh`) and
  `handover_node` (`/handover/{start,abort}`, `/auto_release_monitor/{arm,disarm}`,
  `/handover/state`). None of them is an unresolved decision, so with every
  node up the route is **ready in principle**; it has not yet been run end to
  end on the robot through this orchestrator.

```bash
roslaunch system_bringup fetch_deliver.launch map_name:=lab_map_DIP joint_mode:=relay joint_input_topic:=/arm/joint_states
rosrun perception start_remote_camera.sh                        # D455 driver on PC2
rosrun hand_control start_remote_hands.sh                       # hands on PC2
rosservice call /orchestrator/run_fetch_deliver "{target: table_01, destination: chair_01, label: bottle}"   # dry run
# ... clear the blockers it lists, then relaunch with enable_motion:=true
```

`tools/fetch_deliver_pipeline.sh` starts each of these as its own `roslaunch`
(logs and PIDs under `~/.ros/fetch_deliver/`), `tools/fetch_deliver_check.py`
prints the readiness checklist (PASS / WAIT / FAIL per interface, the two
standing waypoints, the handover arm, a perception read) and ends with the dry
run; exit code 0 means a real run is possible.

The real backend for this route is `ArmGraspBackend`; the older routes keep
`RosBackend` (arm_planning / arm_control / hand_control). Both share
`RosCommon`, and neither is constructed for a dry run.

## Safety model

**Dry run by default.** With `enable_motion:=false` (the default) every route
runs on `DryRunBackend`, which **constructs no robot service client of any
kind** and performs nothing, while still traversing every step. So a route can
be rehearsed with the robot powered off, and dry-run safety is a property of
the object graph rather than a flag checked in a dozen places.

```bash
roslaunch orchestrator orchestrator.launch
rosservice call /orchestrator/run_navigate_report "{target: table}"
```

`dry_run: true` in a request forces a dry run even on a motion-enabled node.
There is deliberately **no request field that can force motion on** — a test
pins the request field list to exactly `route, target, start_step, dry_run`.

**Preflight before anything moves.** Every capability a route declares is
checked first, and preflight runs to completion to report *every* blocker, not
just the first — so one pass tells an operator everything to fix. On a real
run, a single blocker means **zero steps and zero motion**, with the full list
returned.

**Cleanup is unconditional.** `cancel_navigation` and `disable_motion` run in
the sequencer's `finally` block on every exit path: success, step failure,
raised exception, unknown step. A route cannot leave the motion gate open.

**Fail-fast.** The first failing step ends the route. A step that raises is a
failed step, not a crash.

**Non-blocking lock.** A second route is refused immediately rather than queued
— two routes interleaving would mean two owners of the arm and the motion gate,
and a blocking lock would hold a service call open for a minutes-long
navigation.

**The grasp is planned in full before anything moves.** The route order puts
`plan_grasp` ahead of every arm and hand step (doc 13.3), and a test asserts it.

## Release ownership: why `run_auto_release` is not a route

`/orchestrator/run_auto_release` is the one service here that does **not** run a
route and does **not** take the execution lock.

The reason is concrete. `ExecutionLock` is non-reentrant and non-blocking, and a
handover task holds it for the whole route — including while it waits at the
handover pose for a human to take the object. If the tactile monitor's release
request tried to run a route, it would call `ExecutionLock.hold()`, find the lock
held by the very task it is trying to serve, and be **refused outright**. Not a
deadlock, because the lock is non-blocking — but refusing every legitimate
release is just as broken.

So the callback only **validates and records**:

1. `handover_state.HandoverState.request_release()` checks the task state under
   its **own** small mutex, never the execution lock. **Implemented.**
2. On success the validated pull is recorded as a numbered event.
   **Implemented.**
3. The task that owns the execution lock would call `consume_pending()` and
   perform the release under its existing ownership. **NOT IMPLEMENTED — see
   the next section.**

There is exactly one motion path, and the service callback is not on it. The
callback's response always carries `motion_issued=False`.

## The delivery route and the handover event

`deliver` is now 14 steps and **does** consume the validated pull event:

```
require_held_object -> await_localization -> enable_motion -> navigate
  -> plan_handover_pose -> execute_handover_pose -> settle_handover
  -> arm_monitor -> await_pull -> disarm_monitor -> release_hand
  -> confirm_released -> settle_recipient -> execute_retract_home
```

`await_pull` runs **inside the route, while it holds the execution lock**, and
polls `HandoverState.consume_pending()`. The polling loop lives in
`sequencer.Backend.await_pull`, so the real and fake backends share it and the
integration tests drive the production code rather than a copy.

| Guarantee | Where |
|---|---|
| a delivery cannot start, or resume, without a grasp having recorded an object | `require_held_object` → `HandoverState.require_held_object()` |
| the detector is armed only after the arm moved and settled | step order: `execute_handover_pose` → `settle_handover` → `arm_monitor` |
| disarm strictly before release | step order, pinned by a test |
| the hand cannot open without a consumed pull, even on resume | `release_permitted()` |
| a failed or unclear release marks `UNCERTAIN` and **blocks retraction** | `release_hand`, `confirm_released`, `retraction_allowed()` |
| a timeout or cancel releases nothing and keeps the object held | `await_pull` returns failure; cleanup disarms |
| no route can exit leaving the detector armed | `disarm_monitor_cleanup()` in the `finally` |

### A successful release is not a verified transfer

`hand_control`'s `release` preset sets neither `require_settle` nor
`expect_contact`, so `/hand_{side}/release` returns success as soon as the open
command is accepted — `hand_node._execute` explicitly returns
*"release commanded, did not settle ... expected if holding an object"*. So
`confirm_released` records **`RELEASE_COMMANDED`**, never a verified transfer,
and the response's `transfer_verified` field stays false. Upgrading that
evidence needs `require_settle: true` on the release preset, which is
hand_control's call.

### Event identity: one authoritative token

`handover_state.new_session_token()` is the **only** place a token is created.
The orchestrator calls it once per arming, sends it to the monitor in
`handover/ArmMonitor.srv` (`side` + `session_token`), checks the monitor echoed
that exact value, and only then enters `AWAITING_PULL`. The monitor stores it
and echoes it in `event_token` on its release request; it never generates one.

So there is exactly one token per handover session, and a request quoting an
old one — a pull detected before a re-arm and delivered late — is refused as
stale. A tokenless request is refused outright: the request carries no origin
time, so stamping its receipt time would show only when it arrived, not which
handover detected it.

Ordering matters and is deliberate: the token is created, the monitor confirms
it, and `mark_awaiting_pull` happens last, so there is no window in which a
release could be validated against a monitor that is not yet armed.

`test_monitor_integration.py` exercises this through the **real** monitor and
the **real** validator in one process: a synthetic pull drives the real
detector, the real monitor sends a real request, and the real validator accepts
it — plus old-token-after-rearm, duplicates, disarm and timeout.

### Release completion: three different things

| State | What established it | Retraction |
|---|---|---|
| `RELEASE_COMMANDED` | `/hand_{side}/release` returned success — an **actuation acknowledgement**. `revo_hand_link.wait_until_settled` *does* poll fresh `read_positions()` against the target within `settle_tolerance` (deadline `settle_timeout_s`), but the result is not exposed; the release preset leaves `require_settle` **absent**, so `_execute` does **not** fail on a settling timeout and returns *"release commanded, did not settle"* as success. | **Refused, with no override.** A command ack is not physical completion, and retracting next to a person who may still hold the object needs better evidence. |
| `RELEASE_SETTLED` | Settling **confirmed from checkable evidence** in the service result — a structured flag, or feedback positions the orchestrator compared itself. **Unreachable today**, and deliberately not settable from an orchestrator config flag: that would be this package asserting a fact about hand_control's YAML it cannot observe. | Allowed. |
| **object transfer verified** | **Nothing in this workspace can observe this.** There is no state for it; the `transfer_verified` response field exists and is **always false**. | — |

`UNCERTAIN` (release failed, raised, or outcome unknown) refuses retraction
outright and asks for human inspection.

### Known gaps are PREFLIGHT blockers, not step-time surprises

Every known-missing requirement is declared in the route's `requires`, so
preflight refuses before any motion. Finding out there is no home pose at
`execute_retract_home` would mean finding out **after the hand had already
opened**.

`deliver` requires `handover_pose`, `release_completion` and `home_pose`;
`grasp` requires `stow_pose`. With every service and flag present, preflight
still reports 4 blockers for `deliver` and 2 for `grasp`.

The step-level checks are kept as well, for runtime failure handling: a
capability can be satisfied at preflight and still fail mid-route, and the step
must refuse rather than proceed.

### The handshake window

The monitor's arm handler can return before the orchestrator calls
`mark_awaiting_pull`, so a pull detected in that gap would once have been
rejected as "not waiting" and silently lost, leaving the task waiting for a pull
that already happened. The `ARMING` phase closes it: the token is issued and the
phase entered **before** the monitor is called, a correctly-tokened request is
**queued** there, and `mark_awaiting_pull` preserves it. If arming then fails,
the echo mismatches, or the task is cancelled, `abandon()` drops the queued
event and clears the token, so nothing can be released against a monitor that is
not armed. The detector's own post-arm suppression (>= 1.0 s) is a second line
of defence, not the mechanism.

## The `saw_active` guard

`/navigation/status` is latched and re-published every tick, so a `SUCCESS`
left over from the previous goal is still on the topic when a new goal is
accepted. `nav_guard.py` therefore only accepts a terminal state **after** an
ACTIVE one has been seen. Without that rule the orchestrator would instantly
"succeed" on a stale message and move to the next step while the robot had not
moved. This is the client side of navigation's own contract
(`navigation/scripts/nav_client.py`, doc 4.3) with time injected so it is
testable; it adds no navigation logic.

## Why the full grasp is NOT_READY

The `grasp` route requires the `frame_contract` capability, which is declared
**unresolved** — no runtime state can clear it:

Navigation's half is **settled** (`docs/FRAME_CONTRACT.md`, PR #10):
`base_link` is an identity alias of `pelvis` at **every** waist angle — the
pelvis is the waist's *parent*, so it never rotates with the waist — published
once by `system_bringup` with `with_base_link` on by default (`g1_slam` never
publishes it). What remains:

* `perception` reports object positions in `base_link`; `arm_planning` takes
  targets in `torso_link` (doc 2.3, 13.4), and **nobody has implemented the
  conversion**. It is one TF lookup, `torso_link <- base_link`, which carries
  the live waist yaw, and it belongs in `arm_planning`, not here.
* The two vendored URDFs place `torso_link` **10 mm** apart
  (`waist_yaw_joint` z = 0.044 in slam's mode-10 URDF vs 0.054 in
  arm_planning's) — the size of arm_planning's own 0.01 m tolerance. This may
  be two different *definitions* of the frame rather than a wrong URDF, so the
  TF `torso_link` must be shown to match the planner's convention before a
  target is routed through it; run `tools/check_urdf_agreement.py`.

A **separate**, independent requirement, not part of the frame contract:
`perception`'s camera extrinsics are hard-coded for a torso-mounted D435 that
**does** rotate with the waist, so perception's output is only valid near zero
waist yaw regardless of how `base_link` is defined. The zero-waist limitation
belongs to those extrinsics, never to `base_link`.

Converting a perception point into a planner target would be a guess, so this
package does not. A dry run still traverses the whole grasp route, so the
sequence can be reviewed; a real run is refused with that explanation.

`navigate_report` deliberately does **not** require the frame contract: it
reports the detection verbatim and never converts it, so it is the one
grasp-adjacent route that can run today.

## Configuration

`config/orchestrator.yaml`: `enable_motion` (false), `side`, `timeouts`
(localization, navigate, service_call), `service_timeout_s`, and per-action
`retries`. Anything not listed in `retries` gets exactly one attempt, because
retrying a motion step is not safe by default; `close_gripper` is 3 because
`hand_control` documents the caller as retrying the grasp.

## Tests

```bash
cd orchestrator/test && python3 -m unittest discover -s . -v
```

Offline tests, no ROS master: route order and resume, the blocked grasp,
the fetch_deliver order and its standing-waypoint blockers
(`test_fetch_deliver_node.py`, against the real `g1_slam` resolver),
preflight blocker collection, WAIT vs false flags, the `saw_active` guard
(including that a stale SUCCESS can never be rescued by a timeout), the
non-blocking lock, dry-run traversal of every route, fail-fast, retries,
unconditional cleanup on every exit path, and the no-robotics-inside rule.
