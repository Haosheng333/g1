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
