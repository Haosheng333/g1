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

## ⚠ The delivery route does not consume handover events

**End-to-end delivery is incomplete.** An earlier version of this section
described step 3 above in the present tense, which overstated it. The
consumer half was never written:

| `HandoverState` method | callers outside its own module and tests |
|---|---|
| `begin()` | **0** |
| `mark_awaiting_pull()` | **0** |
| `mark_releasing()` | **0** |
| `mark_released()` | **0** |
| `consume_pending()` | **0** |
| `abandon()` | **0** |
| `request_release()` | 2 (the monitor, and this service) |

Consequences, all verified rather than inferred:

* Nothing ever puts the task into `AWAITING_PULL`, so `request_release()`
  **always** returns `rejected: no object is recorded as held`. That is
  structural, not a transient state.
* `consume_pending()` is unreachable from the production path.
* The `deliver` route is four steps and stops at `arm_handover`:
  `await_localization → enable_motion → navigate → arm_handover`. It has no
  await-pull, release, retract or home step.
* The `auto_release` route (`release_hand`, `disarm_handover`) is **orphaned**:
  `/orchestrator/run_auto_release` is bound to `handle_release_request`, and
  `make_handler` covers only `navigate_report`, `grasp` and `deliver`, so no
  service invokes that route. Its two steps are also in the reverse of the
  specified order (release before disarm).
* `final_step="await_pull"` in the release response is a label, not a step that
  exists in any route.

Of the specified release sequence `disarm → release → confirm → retract →
home`, **none of the five is implemented in the delivery path**.

`docs/DELIVERY_FLOW.md` holds the status classification, the proposed route,
the unresolved inputs it depends on, and the integration tests that would
demonstrate it. Until that lands, real delivery stays NOT_READY.

A request is rejected when:

| Condition | Message |
|---|---|
| nothing is held | `no object is recorded as held` |
| the task is not at the handover pose | `not waiting for a handover (phase MOVING_TO_POSE…)` |
| a release was already accepted this session | `duplicate; already accepted` |
| one is already pending | `duplicate; already pending` |
| release not authorized (detection-only, or motion off) | `release authorization absent` |
| the handover has been waiting too long | `stale` |

The detector therefore cannot be acted on during arm movement: `MOVING_TO_POSE`
rejects, and only `AWAITING_PULL` accepts. Nothing releases on a timer.

`test/test_handover_state.py` pins all of it, including that validation succeeds
*while* the execution lock is held, that concurrent requests accept exactly one,
and — by AST inspection — that `handle_release_request` references no lock, no
route and no service proxy.

**This patch does not execute the release sequence on hardware.** The
validation half is implemented and tested; the producer and consumer halves
are absent, so the hand-opening and retraction steps are unreachable for a
real run.

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

* `perception` reports object positions in `base_link`, documented as `pelvis`
  at zero waist yaw.
* `arm_planning` takes targets in `torso_link` (doc 2.3, 13.4).
* `slam` refuses to publish `pelvis -> base_link` at all
  (`g1_tf.yaml`: `base_link_alias: ""`, marked UNRESOLVED pending the
  navigation team).
* The two vendored URDFs place `torso_link` **10 mm** apart
  (`waist_yaw_joint` z = 0.044 in slam's mode-10 URDF vs 0.054 in
  arm_planning's) — the size of arm_planning's own 0.01 m tolerance.

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
