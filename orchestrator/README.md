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
| `/orchestrator/run_auto_release` | `orchestrator/RunRoute` | open the hand, disarm the monitor |

`/orchestrator/status` (`std_msgs/String`, latched) reports the current phase.
`/orchestrator/run_auto_release` is what `handover`'s monitor calls after it
verifies a release.

Request: `route` (optional guard), `target`, `start_step` (resume point),
`dry_run`. Response: `success`, `message`, `steps_completed`, `steps_skipped`,
`blockers`, `final_step`, `motion_issued`, `dry_run`.

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

95 offline tests, no ROS master: route order and resume, the blocked grasp,
preflight blocker collection, WAIT vs false flags, the `saw_active` guard
(including that a stale SUCCESS can never be rescued by a timeout), the
non-blocking lock, dry-run traversal of every route, fail-fast, retries,
unconditional cleanup on every exit path, and the no-robotics-inside rule.
