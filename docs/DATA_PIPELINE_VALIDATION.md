# Grasp-and-deliver data pipeline — integration validation

Evidence-based answer to one question: **does the existing pipeline actually
work across module boundaries?** Tests were run, not reviewed. No hardware was
commanded; nothing was committed, merged or pushed.

**Headline:** the pipeline is **still blocked at boundary 3**, but for a
narrower reason than when this report was first written. Boundaries 1, 2 and 7
work over real ROS. **Boundaries 4 and 6-arm are now wired to the real
downstream services** — see §10, which supersedes the "no production
implementation at all" claim below for `plan_grasp`, `execute_segment`,
`plan_handover_pose`, `execute_stow` and `execute_retract_home`. What remains
missing is **inputs, not adapters**: the frame conversion (boundary 3), the
three poses, and release-completion evidence. A single full-task entry point
(`run_deliver_task`) has been added by composing existing steps.

> **§§1–9 describe the state at the first audit. §10 is the current state.**
> Where they disagree, §10 is correct.

---

## 0. Exact tested state

| | |
|---|---|
| Branch | `feature/grasp-deliver-integration` |
| Commit | `7bf01cb9c80cf6c2ba54b66125f0583ba4f457bc` |
| Working tree | **dirty, and deliberately preserved.** 26 files staged, 1 unstaged, 1 untracked at the time of testing. Nothing committed. |

The staging state was not created by this task (no `git add` was run); it is
reported as found. Pre-existing uncommitted work carried in from the previous
session: the tool-point correction, the consecutive-frame detector, and the
documentation updates. Changes made *by* this task are listed in §6.

Reproduce:

```bash
# cross-process ROS integration test, isolated master on :11399
orchestrator/test/integration/run_integration.sh

# real-algorithm checks (needs numpy -> run in the container)
docker exec g1-ros1 bash -lc 'cd /root/catkin_ws/src/g1/orchestrator && \
    python3 -m unittest test.test_real_algorithms -v'
```

A prerequisite that had never been done: **`orchestrator`, `handover`,
`g1_navigation`, `arm_planning` and `arm_control` had no generated Python
service modules in the workspace** — only `perception` did. Nothing could have
talked to anything. `catkin_make` with a whitelist built all six; the former
`move_base_msgs` blocker is gone because `g1_navigation` no longer depends on it.

---

## 1. The actual interface matrix, from production code

| # | Producer → Consumer | Endpoint / type | Request → Response | Units / frame | Readiness condition |
|---|---|---|---|---|---|
| 1 | task → orchestrator | `/orchestrator/run_{navigate_report,grasp,deliver,deliver_task}` · `orchestrator/RunRoute` | `route, target, start_step, dry_run, event_token` → `success, message, steps_completed[], steps_skipped[], blockers[], final_step, motion_issued, dry_run, object_state, transfer_verified` | — | dry-run unless `~enable_motion`; a real run needs zero preflight blockers |
| 1a | orchestrator → navigation | `/navigation/navigate` · `g1_navigation/NavigateToObject` | `class_name, approach_distance` → `success, message` | metres; `class_name` must exist in `semantic.yaml` | service present + `/navigation/status` present |
| 1b | navigation → orchestrator | `/navigation/status` · `std_msgs/String` (latched) | `PLANNING\|FOLLOWING\|RECOVERING\|SUCCESS\|FAILURE` | — | **`saw_active` guard**: a terminal state counts only after an ACTIVE one |
| 1c | orchestrator → navigation | `/navigation/enable_motion` · `std_srvs/SetBool` | `data` → `success, message` | — | service present |
| 2 | camera → perception | `/camera/color/image_raw`, `/camera/aligned_depth_to_color/image_raw` (16UC1, mm), `/camera/color/camera_info` | — | mm depth → metres internally | RealSense driver with `align_depth:=true` |
| 2a | perception → orchestrator | `/perception/get_object_position` · `perception/GetObjectPosition` | `label, index` → `success, x, y, z, frame_id, message` | **metres, `frame_id` hardcoded `"base_link"`** | service present; one call = one frame, no retry |
| 3 | orchestrator → planner frame | **NONE** | — | would be `base_link → torso_link` | **`frame_contract` unresolved** |
| 4 | planner → arm execution | `/arm/plan_to_pose`, `/arm/plan_grasp_from_point`, `/arm/move_arm_joints` | *declared, never called* | metres/radians, `torso_link`, `[x,y,z,rx,rz]`, 5 joints | **no orchestrator implementation** |
| 5 | grasp → held-object state | in-process `HandoverState` | `begin/record_held_object` → `NONE\|HELD\|RELEASE_COMMANDED\|RELEASE_SETTLED\|UNCERTAIN` | — | written only by `record_held_object` |
| 6 | orchestrator → hand | `/hand_{side}/{pre_grasp,grasp,release}` · `std_srvs/Trigger` | `()` → `success, message` | raw 0–1000 units internally | `hand_control` running |
| 7 | force → monitor → orchestrator | `/auto_release_monitor/arm` · `handover/ArmMonitor`; then `/orchestrator/run_auto_release` · `orchestrator/RunRoute` | `side, session_token` → `success, message, session_token`; release carries `event_token` | **force unit UNCONFIRMED**; threshold 3.0/channel | monitor READY needs a declared force schema |
| 8 | release → completion → retract | `confirm_released`, `execute_retract_home` | — | — | **`release_completion` unresolved**; `RELEASE_COMMANDED` refuses retraction |

### Production backend: what exists and what does not

| Real implementation in `RosBackend` | Hard-coded `StepResult(False, …)` stub |
|---|---|
| `await_localization`, `enable_motion`, `navigate` (+ `NavGuard`), `detect_object`, `report`, `open_gripper`, `close_gripper`, `release_hand`, `confirm_released`, `arm_monitor`, `disarm_monitor`, `await_pull`, `require_held_object`, `record_held_object`, `settle_handover`, `settle_recipient`, `cancel_navigation`, `disable_motion`, `disarm_monitor_cleanup` | `to_planner_frame`, `plan_grasp`, `execute_segment`, `plan_handover_pose`, `execute_stow`, `execute_retract_home` / `_move_to_home` |

That table *is* the finding: everything from the frame conversion onward is a
declared placeholder.

---

## 2. The full-task entry point

**There was none.** `grasp` and `deliver` were separate services, and
`deliver`'s first step is `require_held_object`, so the framework task (§0.4,
§6.2) needed two calls with a human in between. Treating `run_deliver` as "the
pipeline" would have been wrong — it starts from an already-held object.

**Added:** `/orchestrator/run_deliver_task`, route `deliver_task` — the grasp
steps followed by the delivery steps, **28 steps, one sequencer, one execution
lock**. It does *not* call `run_grasp`/`run_deliver`, which would re-enter the
orchestrator and be refused by its own non-reentrant lock;
`test_no_route_calls_another_route_service` enforces that.

Delivery-phase step names are renamed (`deliver_await_localization`,
`deliver_enable_motion`, `navigate_to_recipient`) because `plan_steps` resolves
`start_step` with `names.index()`, which takes the first match — duplicated
names would make a resume at `navigate` silently restart the *grasp* drive.

**Known contract gap, not papered over:** `RunRoute.srv` carries one `target`
field; the task needs two destinations (table/object and delivery point B), and
`RouteContext` has no `destination`. Inventing a `target` encoding would be
fabricating an interface. Recorded here; operationally moot today because the
route is preflight-refused anyway.

---

## 3. Observed call sequences and boundary payloads

### 3.1 `navigate_report` — real run, real ROS, real orchestrator

```
request: target="table", dry_run=false            (node launched enable_motion:=true)

steps_completed : await_localization, enable_motion, navigate, detect_object, report
blockers        : []
motion_issued   : true
dry_run         : false

observed service calls on the doubles, in order:
  enable_motion | data=True
  navigate      | class_name=table approach=0.000
  detect_object | label=table index=0
  navigation/cancel |                    <- unconditional cleanup
  enable_motion | data=False             <- unconditional cleanup

/orchestrator/status: preflight (MOTION) -> RUNNING -> done
```

**Boundary 1 payload** — the task target reaches navigation intact:

```
target="workbench"  ->  navigate|class_name=workbench approach=0.000
```

**Boundary 2 payload** — perception's numbers and frame reach the caller:

```
detect_object|label=bottle index=0
route 'navigate_report' completed (5 steps): reported:
  {'x': 0.412, 'y': -0.133, 'z': 0.907, 'frame_id': 'base_link', 'label': 'bottle'}
```

That second line only exists because of a defect fixed by this work — see §6.1.

### 3.2 `grasp`, `deliver`, `deliver_task` — real runs, all refused

```
run_grasp        -> NOT_READY: 2 blocker(s)   frame_contract, stow_pose
run_deliver      -> NOT_READY: 4 blocker(s)   frame_contract, handover_pose,
                                              release_completion, home_pose
run_deliver_task -> NOT_READY: 5 blocker(s)   frame_contract, stow_pose,
                                              handover_pose, release_completion, home_pose

steps_completed : []        motion_issued : false
observed service calls on the doubles : []      <- not one hardware call
```

**This is the first failing boundary: preflight, before step 1.** Not a
transport fault, not a missing double — an unresolved integration decision.

### 3.3 Boundary 7 — real monitor, real ROS, labelled force in newtons

```
arm: side=right session_token="itest-token-961062552"
     -> success, session_token echoed identically

baseline  : [10.0, 10.0, 10.0]  N   (TEST FIXTURE)
step      : [14.0, 11.0,  9.5]  N
deltas    : [ +4.0, +1.0, -0.5] N
threshold : 3.0 N per channel

/auto_release_monitor/event:
  confirmed|right|pull detected on ch0 (deltas {'ch0': 4.0})
```

One channel over 3 N fires; the other two, at +1.0 N and −0.5 N, do not. The
sub-threshold case `[12.0, 11.5, 12.0]` (deltas +2.0/+1.5/+2.0, **sum 5.5 N**)
produces **no** detection, confirming any-channel rather than a sum.

**These fixtures validate the rule, not the hardware.** They say nothing about
SDK field names, channel mapping, or whether the device reports newtons.

### 3.4 `deliver_task` dry run — 28 steps, zero hardware calls

```
success=true  dry_run=true  n_steps_completed=28  blocker_count=5  motion_issued=false
observed service calls on the doubles: []
```

A dry run proves **step order only**. `DryRunBackend` makes no service call, and
`require_held_object` there returns "dry run: held-object check skipped" — so a
dry run resumed at the delivery phase reports success having skipped the grasp
entirely. **Dry-run success must never be read as task validity.** The real
guard does hold: `HandoverState.release_permitted()` refuses with *"never
because a route was resumed at this step"*, exercised directly in `E4`.

---

## 4. Real implementations vs test doubles

| Component | In the ROS test |
|---|---|
| `orchestrator_node.py` + routes/sequencer/preflight/nav_guard/handover_state/execution_lock | **REAL production code** |
| `auto_release_monitor.py` + tactile detector/frame/adapter | **REAL production code** |
| All service/message types | **REAL generated** `orchestrator/RunRoute`, `handover/ArmMonitor`, `g1_navigation/NavigateToObject`, `perception/GetObjectPosition`, `std_srvs/*`, `sensor_msgs/JointState`, `std_msgs/*` |
| ROS transport | **REAL** — three separate processes over a real master |
| navigation behaviour | double (`doubles_node.py`) |
| perception behaviour | double — fixed xyz in `base_link` |
| hand behaviour | double — note `release` returns success with **no** settling evidence, mirroring the real contract |
| locomotion / arm SDK / hand SDK / camera / LiDAR | **absent entirely**; no SDK module is imported anywhere in the harness |
| `/arm/*` services | advertised so preflight can pass, wired to handlers that **record a finding if ever called**. They never were. |

Isolation: own `roscore` on **:11399** (never 11311), `ROS_HOSTNAME`/`ROS_IP`
pinned to `127.0.0.1`, test-only parameters uploaded to that master only.
**No file under `handover/config` or `orchestrator/config` was modified**; the
test-only force schema lives in `orchestrator/test/integration/monitor_params.test.yaml`,
which no launch file references. The harness has no `__init__.py`, so the
offline suite cannot import it, and it is not installed as a package executable.

Test-only timeouts (`navigate: 4 s` instead of production's `1800 s`) **narrow a
wait**; no check was relaxed and no readiness setting was changed.

---

## 5. Test results

### 5.1 Cross-process ROS integration — `run_integration.sh`

**29 run / 29 passed / 0 failed.**

| Group | Covers |
|---|---|
| A (4) | `navigate_report` real end-to-end; boundary 1 and 2 payloads; no arm service touched |
| B (3) | `grasp`/`deliver` refused with exact blockers, zero steps, zero hardware calls |
| C (7) | monitor readiness; tokenless arm refused; one channel >3 N fires; all channels <3 N do not; absent force → fault, no release; release request with nothing held refused; tokenless release refused |
| D (9) | nav failure, stale SUCCESS (`saw_active`), nav timeout, goal rejection, perception failure, stale `/joint_states` → preflight block, concurrent requests, invalid resume, route-guard mismatch |
| E (6) | `run_deliver_task` exists; 28-step dry run in order; unique step names; real run refused with the union of blockers; dry-run limitation documented; real resume guard refuses |

### 5.2 Offline suites (container, Python 3.8)

| Suite | Result |
|---|---|
| `handover/test` | **155 passed** |
| `orchestrator/test` | **225 passed** (incl. 25 new real-algorithm tests) |
| `arm_planning/test` | **74 passed** (incl. 3 new Python-3.8 compatibility tests) |
| `slam/test` | **47 passed** |
| `navigation/test` | **119 passed** |
| `perception/test` | 10 run, **8 passed**, 1 skipped, **1 blocked** (`ultralytics` absent) |
| `arm_control/test` | 11 run, **10 passed**, 1 skipped (no SDK) |
| `arm_planning/tests` | **15 modules blocked**, 0 run (was 18 before §6.2) |

**Totals: 638 offline passed + 29 ROS integration = 667 passed. 0 failed.
16 modules blocked, 2 skipped.**

### 5.3 Blocked checks — not passes, not failures

| Check | Exact reason |
|---|---|
| perception YOLO detector | `ultralytics` absent on host and in the container. `m1_perception` imports `YOLO` at module scope, so even its geometry needs the module stubbed. |
| arm_planning IK + collision | `torch` absent → no checkpoint loads, no IK/collision/trajectory query. `trimesh` and `python-fcl` absent → `g1_collision_probe` cannot run. |
| `arm_planning/tests` (15 modules) | `torch` (6), `pytest` (5), `trimesh` (1), missing helper modules `vectorized_g1_fk` / `generate_ik_dataset` (2), plus 1 downstream of `trimesh`. |
| force unit / field names / rate | needs the robot. |
| **delta sign convention** | whether a real pull makes a Revo2 force channel rise or drop is **not established anywhere**. The detector defaults to a magnitude compare (`either`); per-channel `direction` must be set once the SDK is read. Recorded, not invented. |
| camera calibration + physical geometry | needs a real camera and robot. **Distinct from frame-conversion code correctness**, which §5.4 does test. |

### 5.4 Real algorithms actually exercised — 25 tests

| What | Status |
|---|---|
| perception **geometry** — `backproject_pixel`, `push_along_ray`, `optical_to_base`, `median_nonzero_depth` | **REAL production functions**, detector stubbed. Pinhole inverse exact; radius push exact; linearity exact; depth holes return `None` rather than a guessed number. |
| arm_planning **FK** — `forward_kinematics`, `pose_5d` | **REAL**. Valid rigid transform, orthonormal to 1e-10; `[x,y,z,rx,rz]` order confirmed against `pose_order`; 5-joint ordering confirmed; finite across 40 random configurations per arm within limits; out-of-range rejected; corrected tool point (`y = ±0.06`) present; left/right mirror holds. |
| arm_planning **checkpoint metadata** | **REAL** (torch-free). Both checkpoints still stale by **54.0 mm** against the corrected tool point. |
| navigation **sim fixture** | **REAL** parse of `sim_room/{map,semantic}.yaml` — resolution, 3-element origin, named finite waypoints. |
| handover **detector** | **REAL**, with values labelled in N (§3.3). |

---

## 6. Defects found and fixed

### 6.1 Route output never crossed the service boundary — FIXED

`sequencer.run_route` built its success message as
`"route %r completed (%d steps)"` and **discarded the final step's detail**.
`navigate_report`'s entire purpose is to report a pose; the caller received
`"completed (5 steps)"` and the detection was lost at the service boundary.

Fixed by carrying the last step's detail into the success message
(`sequencer.py`). No `.srv` change. The payload in §3.1 is the proof.

### 6.2 Production module unimportable on Python 3.8 — FIXED

`arm_planning/scripts/joint_space_astar.py:20` assigned
`State = tuple[int, int, int, int]` — a PEP 585 subscript evaluated at
**runtime**, which raises `TypeError` on Python 3.8, the interpreter ROS Noetic
runs. `from __future__ import annotations` does not help, because it is not an
annotation. This made `joint_space_astar`, `edge_checked_astar` and
`collision_aware_planner` unimportable on the deployment interpreter.

Fixed to `typing.Tuple` (already imported in that file). Two of the three now
import; the third still needs `trimesh`. `arm_planning/tests` import-blocked
modules dropped 18 → 15. Guarded against regression by
`arm_planning/test/test_python38_compat.py`, a source scan so it catches the
problem even on a 3.9+ host where the import would succeed.

### 6.3 Perception extrinsics rotation is not orthonormal — RECORDED, not fixed

`_default_R_base_opt()` is hand-built from two axis vectors copied out of a
MuJoCo `xyaxes` string that are neither unit length nor orthogonal:

```
det(R)         = 1.0003759767      (must be 1)
max|RᵗR − I|   = 3.760e-04         (must be 0)
column norms   = 1.000032, 1.000156, 1.000188
col0 · col1    = −2.08e-04
```

`optical_to_base` therefore applies a small scale-and-shear as well as a
rotation — about **188 µm per metre**, so ~0.19 mm at the ~1 m working range.
That is immaterial beside perception's own stated 1.46 cm mean error, so it is
recorded rather than fixed: re-orthonormalising a calibration matrix is
perception's decision and this task must not alter calibration. Pinned with its
measured magnitude by `test_the_extrinsics_rotation_is_NOT_orthonormal`.

### 6.4 Test-harness defects fixed in this task's own tests

Three assertions of mine were wrong and are corrected, not worked around: the
concurrency wording, the location of the detection payload (which exposed §6.1),
and the dry-run resume expectation (which exposed the §3.4 limitation).

---

## 7. Architectural gaps — recorded, not fixed

1. **Boundary 3 does not exist.** `to_planner_frame` is a stub; `frame_contract`
   is unresolved. It is one TF lookup (`torso_link ← base_link`) and belongs in
   `arm_planning`. First failing boundary.
2. **Boundaries 4, 6-arm and 8 do not exist.** `plan_grasp`, `execute_segment`,
   `plan_handover_pose`, `execute_stow`, `execute_retract_home` are stubs; the
   three poses (handover, stow, home) exist nowhere in the repository.
3. **Release completion is unobservable.** `/hand_{side}/release` returns only
   `success` + message; the preset leaves `require_settle` absent. A success is
   an acknowledgement. Retraction stays refused and `transfer_verified` is always
   false — both verified over real ROS.
4. **`RunRoute.srv` has one `target`; the full task needs two destinations.**
5. **Dry-run success is not task validity** (§3.4).
6. **`arm_planning` tests are unreachable on the deployment interpreter** —
   pytest-style, no `pytest`, no `__init__.py`, and `catkin_add_nosetests` is
   absent from its `CMakeLists.txt`, so `catkin_make run_tests` runs none of them.
7. **The two URDFs still disagree on `torso_link` by 10 mm**, the size of
   `arm_planning`'s own tolerance — part of `frame_contract`.

---

## 8. Verdicts

| # | Scope | Verdict |
|---|---|---|
| 1 | **Full-task sequencing with simulated hardware** | **PARTIAL.** All 28 steps traverse in the correct order with the new `deliver_task`, and the real sequencer, lock, resume, fail-fast and cleanup behave correctly under 29 ROS tests and 225 offline tests. But traversal happens only in dry run; a real run is refused at preflight, and 6 of the 28 steps have no implementation to sequence. |
| 2 | **Cross-process ROS transport and contracts** | **PASS.** Three real processes over a real master. Every service/message type is the real generated definition. `navigate_report` completes end-to-end with correct payloads at boundaries 1 and 2; the monitor→orchestrator token path works; every failure scenario propagates to a named step; `saw_active`, the execution lock, token validation, duplicate/stale rejection and unconditional cleanup all hold. |
| 3 | **Real algorithm / data processing** | **PARTIAL.** Perception geometry and arm FK are real and correct (one minor, quantified non-orthonormality). The YOLO detector, all IK, all collision checking and the navigation planner could **not** run — `ultralytics`, `torch`, `trimesh`, `python-fcl` are absent. Frame-conversion *code* correctness is established; camera calibration and physical geometry correctness are **not**. |
| 4 | **Physical robot execution** | **NOT TESTED.** The robot is unavailable. No hardware was commanded. Force units, field names, channel mapping, rate and the delta sign remain unconfirmed; no TF, SLAM, motion or transfer check was attempted. |

---

## 9. The harness is retained

| Path | Role |
|---|---|
| `orchestrator/test/integration/run_integration.sh` | reproducible entry point; isolated master |
| `orchestrator/test/integration/doubles_node.py` | hardware-facing doubles, real types, 11 scenarios |
| `orchestrator/test/integration/monitor_params.test.yaml` | test-only force schema |
| `orchestrator/test/integration/test_ros_pipeline.py` | 29 tests, payload assertions, evidence writer |
| `orchestrator/test/integration/README.md` | why it is isolated from launch paths |
| `orchestrator/test/test_real_algorithms.py` | 25 real-algorithm tests + blocked-check reporting |
| `arm_planning/test/test_python38_compat.py` | regression guard for §6.2 |

Evidence lands in `/tmp/pipeline_itest/evidence.json` (25 captured boundary
payloads) with per-node logs beside it. When the frame contract and the three
poses land, re-run the same script: the `grasp`, `deliver` and `deliver_task`
real-run tests will start exercising boundaries 3–8 with no harness change.

---

# 10. Revision — adapters wired to the real downstream interfaces

**This section supersedes §§1–9 wherever they disagree.** The earlier audit
reported six hard-coded failing adapters. Five are now real calls to real
production services; one remains blocked on an unresolved contract.

## 10.1 Classification of the six stubs

| Adapter | Class | Finding | Now |
|---|---|---|---|
| `to_planner_frame` | **C** — unresolved frame contract | The conversion is unimplemented *and* the two URDFs disagree on `torso_link` by 10 mm, the size of `arm_planning`'s own tolerance. Owned by `arm_planning`, not here. | **still NOT_READY**, unchanged |
| `plan_grasp` | **A** — existing capability, not wired | `/arm/plan_grasp_from_point` is a real implemented node (`arm_planning/scripts/arm_planning_node.py`) with its own readiness gating. It was never called. | **WIRED** |
| `execute_segment` | **A** | `/arm/move_arm_joints` is a real implemented node (`arm_control/scripts/arm_control_node.py`). Never called. | **WIRED** |
| `plan_handover_pose` | **A** wiring + **C** input | `/arm/plan_to_pose` exists and is real; the *handover pose* exists nowhere. | **WIRED, input blocked** |
| `execute_stow` | **A** wiring + **C** input | same, for the stow/carry pose | **WIRED, input blocked** |
| `execute_retract_home` | **A** wiring + **C** input | same, for the home pose, and additionally gated on release completion | **WIRED, input blocked** |

No adapter was category **B** (missing downstream implementation): every
downstream service the orchestrator needs already exists as a production node.
**The gap was wiring, not implementation** — plus three missing assets and one
unresolved frame contract.

## 10.2 What the wiring preserves

| Preserved across the boundary | How |
|---|---|
| **Planner output verbatim** | the flattened `N × 5` path is stored per segment and sent unmodified; nothing re-plans, rounds or re-orders at execution time |
| **Joint ordering** | `_check_joint_names` compares the planner's own `joint_names` against `plan_conversion.expected_joint_names(arm)` and **refuses** on a mismatch rather than executing a path whose columns are ambiguous |
| **Units** | metres/radians passed straight through; `object_xyz` and `target_pose` are forwarded unconverted (the orchestrator performs no frame maths — enforced by `test_no_kinematics_or_tf_maths`) |
| **Arm side** | stored with each segment and re-checked at execution; a segment planned for the other arm is refused |
| **Failure information** | the planner's and executor's own messages propagate verbatim, including which segment failed and which waypoint timed out |
| **Path integrity** | segment count/length agreement, positive waypoint counts and finiteness are all checked **before** anything is sent |
| **`motion_issued`** | set when a command actually left the process, including on an exception, so an uncertain outcome is never retried blindly |

## 10.3 What is *not* invented

* `~poses` ships **`{}`**. No handover, stow or home pose is supplied.
* `handover_pose`, `stow_pose`, `home_pose`, `frame_contract` and
  `release_completion` remain **`unresolved` capabilities**, so preflight still
  refuses `grasp`, `deliver` and `deliver_task`. **No preflight check was
  removed or weakened.** Writing a number into `~poses` would not clear them —
  `arm_planning` owns validating that a pose is reachable, inside the trained
  workspace and collision-free.
* `current_joints` comes from `/joint_states`. If the five arm joints are
  absent, every planning step reports NOT_READY naming them; **no default
  posture is substituted**, because planning from a guessed start returns a path
  that does not begin where the arm is.

## 10.4 Observed payloads — real adapter code, arm-service doubles

```
plan_grasp:
  call      plan_grasp_from_point|arm=right object_xyz=[0.31, -0.18, 0.02] n_current=5
  result    planned 4 segments for the right arm
            (to_pregrasp, approach, lift, retract), 11 waypoints
  segments  to_pregrasp n=3  approach n=2  lift n=2  retract n=4   (all arm=right)
  slicing   first column per segment: 0.1 / 0.2 / 0.3 / 0.4  <- correct offsets

execute_segment("execute_approach"):
  planned first waypoint   [0.2, 0.201, 0.202, 0.203, 0.204]
  call   move_arm_joints|arm=right waypoints=2 first=[0.2, 0.201, 0.202, 0.203, 0.204]
  result approach: FIXTURE: 2 waypoints executed      motion_issued=true
```

The executor receives **the planner's actual waypoints**, not separate
hardcoded success data — the two lines above are byte-identical.

```
pose steps with ~poses empty:
  plan_handover_pose  NOT_READY: no handover pose is defined anywhere in this repository…
  execute_stow        NOT_READY: no stow pose is defined anywhere in this repository…

with a TEST-ONLY pose injected on the backend instance (not any config file):
  plan_to_pose|arm=right target_pose=[0.25, -0.15, 0.1, 0.0, 0.0] n_current=5
  planned handover pose for the right arm: 5 waypoints, position_error_m=0.0031
  move_arm_joints|arm=right waypoints=5 first=[0.5, 0.501, 0.502, 0.503, 0.504]

arm joints missing from /joint_states:
  NOT_READY: /joint_states carries no right_shoulder_pitch_joint, …_roll_joint,
  …_yaw_joint, right_elbow_joint, right_wrist_roll_joint, so the planner has no
  current_joints. That topic is owned by robot-control / integration; this
  package will not substitute a guessed posture.
```

## 10.5 `run_deliver_task` covers the whole framework task

Verified against the route, not asserted:

| Requirement | |
|---|---|
| navigate to the table **before** perception | YES |
| perception | YES |
| full grasp planned **before** any arm motion | YES |
| four grasp segments executed | YES |
| gripper open/close **between** the right segments | YES |
| held object recorded **after** the grasp | YES |
| stow **after** recording | YES |
| navigate to destination **after** stow | YES |
| handover pose → settle → arm detector, in that order | YES |
| disarm **before** release | YES |
| confirm → recipient settle → retract | YES |
| step names unique (unambiguous resume) | YES |

**One execution owner:** 28 steps walked by a single `run_route` under a single
`ExecutionLock`. It never calls `run_grasp`/`run_deliver`;
`test_no_route_calls_another_route_service` enforces that the node builds no
client for its own route services.

## 10.6 Adapter status — the authoritative list

| Adapter | Implementation | Reachable in a real run? |
|---|---|---|
| `await_localization` | **real** | yes (`navigate_report`) |
| `enable_motion` | **real** | yes |
| `navigate` (+ `saw_active`) | **real** | yes |
| `detect_object` | **real** | yes |
| `report` | **real** | yes |
| `open_gripper` / `close_gripper` / `release_hand` | **real** | only in a route whose preflight clears |
| `arm_monitor` / `disarm_monitor` / `await_pull` | **real** | ditto |
| `require_held_object` / `record_held_object` / `confirm_released` | **real** | ditto |
| `settle_handover` / `settle_recipient` | **real** | ditto |
| `plan_grasp` | **real** (`/arm/plan_grasp_from_point`) | **no** — needs `planner_target` from `to_planner_frame` |
| `execute_segment` | **real** (`/arm/move_arm_joints`) | **no** — needs a plan |
| `plan_handover_pose` | **real** (`/arm/plan_to_pose`) | **no** — `~poses/handover` unset, capability unresolved |
| `execute_stow` | **real** (plan + execute) | **no** — `~poses/stow` unset, capability unresolved |
| `execute_retract_home` | **real** (plan + execute) | **no** — `~poses/home` unset, capability unresolved, **and** release completion unobservable |
| `to_planner_frame` | **STUB** — NOT_READY | **no** — `frame_contract` unresolved |

**One stub remains.** Five adapters are real implementations that are
conditionally blocked on inputs.

## 10.7 Results after the revision

| Scope | Result |
|---|---|
| Cross-process ROS integration | **43 run / 43 passed** (was 29; +14 adapter tests) |
| `handover` | 155 passed |
| `orchestrator` | **227 passed** (was 225) |
| `arm_planning/test` | 74 passed |
| `navigation` | 119 passed |
| `slam` | 47 passed |
| `perception` | 8 passed, 1 skipped, 1 blocked (`ultralytics`) |
| `arm_control` | 10 passed, 1 skipped (no SDK) |

**Totals: 640 offline passed + 43 ROS = 683 passed, 0 failed.**

Defect fixed in this revision: `test_no_kinematics_or_tf_maths` forbade the
substring `plan_to_pose(`, which began false-positiving on a legitimate service
call. The guard now forbids *defining* a planner and asserts the three services
are actually wired — the original intent, enforced more precisely.

One harness race fixed: the doubles publish their call record inside the service
handler, so a snapshot taken the instant the service returned could miss it.
`await_call()` polls instead; a call that never arrives still fails.

## 10.8 Revised verdicts

| # | Scope | Verdict |
|---|---|---|
| 1 | **Full-task sequencing with simulated hardware** | **PARTIAL (improved).** All 28 steps traverse in the correct order; all 12 framework ordering requirements hold; one lock, one owner. Five adapters now call real services and are exercised over real ROS. A real run is still refused at preflight on 5 unresolved capabilities, so the task has never run end-to-end for real. |
| 2 | **Cross-process ROS transport and contracts** | **PASS.** 43 tests across three real processes. All six service types are the real generated definitions, including `PlanToPose`, `PlanGraspFromPoint` and `MoveArmJoints`. |
| 3 | **Real planner / perception computation** | **NOT VERIFIED — unchanged.** The arm services in these tests are **doubles returning fixture ramps**. A pass proves the orchestrator's wiring and nothing about `arm_planning`'s or `arm_control`'s algorithms. The real planner still cannot run: `torch`, `trimesh`, `python-fcl` absent. Perception geometry and arm FK are real (§5.4); the YOLO detector is not. |
| 4 | **Physical robot execution** | **NOT TESTED.** Robot unavailable. No hardware commanded. |

**A test-double success is not a working production adapter.** What §10 claims
is narrower and exact: the orchestrator now *calls* the real interfaces with
correctly-shaped requests and handles their responses and failures correctly.
Whether those services do the right thing with a real arm is untested.

## 10.9 Remaining blockers, in dependency order

1. **`frame_contract`** — the `base_link → torso_link` conversion, plus
   resolving the 10 mm `torso_link` disagreement. Unblocks `plan_grasp` and the
   four grasp segments. **First blocker.**
2. **`handover_pose`, `stow_pose`, `home_pose`** — three validated poses from
   `arm_planning`. Unblock `plan_handover_pose`, `execute_stow`,
   `execute_retract_home`. The adapters are ready for them.
3. **`release_completion`** — `hand_control` must expose settling. Until then
   retraction after a release is refused with no override.
4. **`/joint_states` must carry the five arm joints** — currently only
   `waist_yaw_joint` is confirmed.
5. **Deployment dependencies** — `torch`, `trimesh`, `python-fcl` for the
   planner; `ultralytics` for perception.

---

# 11. Revision — boundary 3 implemented, with the 10 mm resolved explicitly

**Supersedes §10 and §§1–9 wherever they disagree.** Boundary 3 — the
perception → planner frame conversion — is now **implemented production code**
in `arm_planning`, exercised over real ROS with real TF. Overall frame readiness
is **not** cleared: the camera extrinsics are a separate, still-unresolved
prerequisite.

## 11.1 The frames, documented (requirement 1)

| | |
|---|---|
| **Source frame** | `base_link`. An **identity** child of `pelvis`, published once by `system_bringup` (`with_base_link`, on by default). The pelvis is the waist's *parent*, so `base_link` never rotates with the waist. |
| **Observation units** | metres. `perception/GetObjectPosition` returns `x, y, z` in metres with `frame_id` hardcoded `"base_link"`. |
| **Planner frame** | `torso_link`, in the **`g1_23dof` convention** — `arm_kinematics.yaml` has `base_link: torso_link` and `shoulder_pitch` z = `0.23778`, which only matches `arm_planning/urdf/g1_23dof.urdf`. |
| **Planner units** | metres and radians; `target_pose` / `object_xyz` per `PlanToPose.srv` and `PlanGraspFromPoint.srv`. |
| **Transform direction** | `target ← source`, i.e. the point is mapped **into** `torso_link`: `p_torso = R·p_base + t`, exactly what `tf2.lookup_transform("torso_link", "base_link", t)` carries. |
| **Live TF chain** | `world → camera_init → pelvis → torso_link → …`, published by `g1_slam` from **`slam/urdf/g1_23dof_mode_10.urdf`**, plus `pelvis → base_link` from `system_bringup`. |

## 11.2 The 10 mm, resolved rather than assumed away (requirement 3)

The decisive measurement, taken from both files:

| chain | mode_10 (live TF) | g1_23dof (planner) |
|---|---|---|
| `pelvis → torso_link` (waist_yaw origin z) | **0.044** | **0.054** |
| `torso_link → shoulder_pitch` (z) | **0.24778** | **0.23778** |
| `pelvis → shoulder_pitch` | **0.29178** | **0.29178** |

**The sums are identical.** Neither URDF is wrong: they place the same physical
robot's `torso_link` *datum* 10 mm apart and each compensates in the next link.
So this is a **convention** difference, and the resolution is a declared
convention transform — **no TF frame was redefined and neither URDF was
modified**.

```
p_planner = p_tf_torso - [0, 0, 0.010]
```

**Exact at every waist angle, not an approximation.** Both `waist_yaw_joint`
origins have `rpy = 0 0 0` and the joint rotates about **z**, so the 10 mm
offset stays parallel to z and the two torso frames always share orientation.
x and y are identical in both files (`-0.0039635`, `0`), so only z is corrected.

**Direction:** the planner's origin is 10 mm *higher*, so a point measured from
it has a 10 mm *smaller* z. Getting that sign backwards would put every grasp
target 20 mm out, so it is asserted directly rather than via a round trip.

## 11.3 Where the conversion lives (requirement 2)

| | |
|---|---|
| New service | **`/arm/convert_observation`** · `arm_planning/ConvertObservation` |
| Request | `source_frame`, `point[3]` (m), `stamp`, `stamp_is_observation_time`, `max_waist_delta_rad` |
| Response | `planner_point[3]`, `planner_frame`, `waist_yaw_rad`, `transform_age_s`, `convention_offset[3]`, `used_latest_transform`, `success`, `message` |
| Maths | `arm_planning/src/arm_planning/frame_conversion.py` — pure, no ROS, no TF client |
| TF client | `arm_planning/scripts/arm_planning_node.py::handle_convert_observation` |
| Orchestrator | `to_planner_frame` passes the point, the frame perception labelled it with, and its receipt time; stores what comes back |

**The orchestrator performs no TF mathematics**, verified by a source scan:
`tf2_ros`, `lookupTransform`, `quaternion`, `math.cos`, `math.sin` and
`CONVENTION_OFFSET` all occur **0** times in `orchestrator_node.py`
(`test_the_orchestrator_still_does_no_tf_maths`, plus the existing
`test_no_kinematics_or_tf_maths`).

The conversion is deliberately **not** gated on planner readiness: a coordinate
transform does not need checkpoints or meshes, and the real node serves it while
reporting planning NOT_READY for want of `torch`.

## 11.4 The timestamp limitation (requirement 5)

**`perception/GetObjectPosition.srv` has NO stamp field.** Checked directly: its
response is `success, x, y, z, frame_id, message`. The observation time is
therefore unavailable, and an old observation must not be silently paired with
the latest waist pose.

Addressed, not hidden:

* `detect_object` records `received_at = rospy.Time.now()` — explicitly a
  **receipt time**, an upper bound, not an observation time;
* `to_planner_frame` sends it with **`stamp_is_observation_time=false`**;
* the conversion then compares the waist in the transform **at that stamp**
  against the waist in the **latest** transform and **refuses** when they differ
  by more than `max_waist_delta_rad` (default 0.02 rad ≈ 1.1°).

The guard is **stateless** — both angles come from the same request's TF
history. An earlier version of mine compared against the previous conversion's
waist, which coupled unrelated callers and made the verdict depend on call
order; that was a real design defect found by the integration tests and fixed.

**The exact interface change still needed:** add a `time stamp` field to
`GetObjectPosition.srv`'s response, set from the camera frame's own header. That
is perception's contract, so it is recorded here rather than changed.

## 11.5 Tests (requirement 6)

**Offline, `arm_planning/test/test_frame_conversion.py` — 29 tests.** Every
expectation is computed by a **separate derivation written out in the test**:

| Covered | How |
|---|---|
| the 10 mm is real | both URDFs re-parsed; `0.054 − 0.044 = 0.010`; x and y identical |
| neither URDF is wrong | `pelvis → shoulder_pitch` sums equal, both `0.29178` |
| the planner's convention | `arm_kinematics.yaml` shoulder z `= 0.23778` |
| **zero waist, known points** | pelvis origin → `(0.0039635, 0, −0.054)`; a table-height observation → `(0.4159635, −0.133, 0.853)` |
| **nonzero waist** | ±5°, ±30°, ±90°, 150° against the independent `Rz(−θ)` derivation |
| offset exactness | z difference is 0.010 to 1e-12 at every 15° from −180° to +180° |
| a nonzero waist changes the answer | guards a transform that ignores its rotation |
| **transform direction** | the two directions differ; applying it backwards gives a different answer; round trip is a *supplement*, explicitly labelled insufficient |
| the offset is not omitted | without it z would be 0.863, not 0.853 |
| **bad data** | short / non-finite / non-numeric point, degenerate quaternion, non-finite translation |
| **stale/missing** | waist moved → refused; wrap-around not counted as motion; no transform at the stamp → refused; order independence |

**Over real ROS, `G_FrameConversionIsProductionCode` — 11 tests** against the
**real** `/arm/convert_observation` with TF published by the doubles:

```
G2  waist 0,    base_link (0.412, -0.133, 0.907)
    -> planner (0.4159635, -0.133, 0.853)   == independent calculation
G3  waist -30 deg
    -> planner (0.426735, 0.0928, 0.853)    == independent calculation
    waist_reported -0.523599 == waist_commanded -0.523599
G4  the 10 mm is applied: z = 0.853, not 0.863
G5  unknown source frame -> NOT_READY (tf2 LookupException)
G6  stamp 600 s in the past -> NOT_READY (no transform that old)
G7  waist moved 0.698 rad across the window -> NOT_READY, naming GetObjectPosition
G8  to_planner_frame adapter: detection -> ctx.planner_target, z = 0.853
G9  conversion -> plan_grasp: four segments planned from the converted point
```

A defect of mine that these found and that is now fixed: `waist_yaw_rad` was
reporting the *inverse* transform's yaw (`−θ`) rather than the robot's waist
angle. `G3` now asserts reported == commanded.

A note on method, recorded because it is easy to get wrong: **"stop publishing
TF" does not empty a tf2 buffer.** tf2 prunes on insert, not on read, so a
`Time(0)` lookup keeps returning the last transform indefinitely. Staleness is
therefore tested with an out-of-range stamp, which is the condition that
actually matters.

## 11.6 Frame readiness is NOT cleared (requirement 7)

`frame_contract` was **split**, not resolved:

| Capability | Owner | State |
|---|---|---|
| `planner_frame_conversion` | arm_planning | **RESOLVED** — a service probe. Implemented and tested. |
| `perception_extrinsics` | perception | **UNRESOLVED** — three distinct problems, all in `m1_perception.py`: (1) `Extrinsics()` is hard-coded from a MuJoCo `xyaxes` string for a torso-mounted D435 on a **different robot**; (2) that camera is on `torso_link`, which **does** rotate with the waist, while the extrinsics are a fixed `base_link → optical` transform, so the output is only valid near zero waist yaw; (3) `_default_R_base_opt()` is **not a rotation** (det 1.0003760, max\|RᵗR−I\| 3.76e-04, ~188 µm/m of scale-and-shear). |

A correct `base_link → planner` transform **cannot repair** a point already
computed with the wrong camera extrinsics. That is stated in the module
docstring and asserted by `test_the_module_says_so_explicitly`.

Observed blocker change:

```
grasp         before: frame_contract, stow_pose
              after:  perception_extrinsics, stow_pose
deliver       before: frame_contract, handover_pose, release_completion, home_pose
              after:  handover_pose, release_completion, home_pose
deliver_task  after:  perception_extrinsics, stow_pose, handover_pose,
                      release_completion, home_pose
```

`deliver` lost the frame requirement because it genuinely converts nothing: it
navigates to a waypoint and plans to a **configured** pose already in the
planner's frame. Both frame capabilities reach `deliver_task` through the grasp
phase.

## 11.7 The two reconciled audit findings

**Pickup vs delivery destination.** `RunRoute.srv` gained a `destination` field.
`navigate` drives to `ctx.target` (pickup); a distinct `navigate_to_destination`
action drives to `ctx.destination`, so the delivery leg **cannot** silently
reuse the pickup waypoint. `deliver_task` is **refused up front** when
`destination` is empty (`ROUTES_NEEDING_DESTINATION`); single-leg routes ignore
it. Verified by `H_PickupAndDeliveryDestinations`.

**Stow planned before grasp execution (spec 2.4).** This was a **real
ordering defect**: `execute_stow` used to plan *and* execute at step 14, after
`record_held_object` — so an unplannable stow path would have been discovered
**after the hand had closed on the object**. Now:

```
...  plan_grasp (6)  ->  plan_stow (7)  ->  execute_to_pregrasp (8) ...
                                            close_gripper (11) ...
                                            execute_stow (15, execute only)
```

`plan_stow` plans from `plan_grasp`'s own `final_joints` — where retract ends —
so the recovery path is validated while the arm is still parked and the hand
still open. It refuses if `final_joints` is absent rather than falling back to
the live posture. `execute_stow` is now an `execute_segment` step that only
replays the pre-planned path. Verified by `I_StowIsPlannedBeforeGraspMotion`.

## 11.8 Adapter status — authoritative

| Adapter | Implementation | Reachable in a real run? |
|---|---|---|
| `to_planner_frame` | **real** (`/arm/convert_observation`) | **no** — `perception_extrinsics` unresolved |
| `plan_grasp` | **real** (`/arm/plan_grasp_from_point`) | no — same |
| `execute_segment` | **real** (`/arm/move_arm_joints`) | no — needs a plan |
| `plan_stow` | **real** (`/arm/plan_to_pose`) | no — `~poses/stow` unset, `stow_pose` unresolved |
| `plan_handover_pose` | **real** | no — `~poses/handover` unset, unresolved |
| `execute_retract_home` | **real** | no — pose unset + release completion |
| navigation / perception / hand / monitor / state adapters | **real** | `navigate_report` yes; the rest gated |

**Zero stubs remain.** Every adapter is a real call; what blocks them is inputs
and unresolved contracts.

## 11.9 Results

| Suite | Result |
|---|---|
| ROS integration | **60 run / 60 passed** (was 43; +17) |
| `handover` | 155 passed |
| `orchestrator` | 227 passed |
| `arm_planning/test` | **103 passed** (was 74; +29 conversion) |
| `navigation` | 119 passed |
| `slam` | 47 passed |
| `perception` | 8 passed, 1 skipped, 1 blocked (`ultralytics`) |
| `arm_control` | 10 passed, 1 skipped (no SDK) |

**Totals: 669 offline passed + 60 ROS = 729 passed, 0 failed.**

Defects found and fixed in this revision: the `destination` variable was
computed in `run()` but used in `_execute()` (a `NameError` on every route call —
caught immediately by the integration suite); `waist_yaw_rad` reported the
inverse transform's yaw; the waist guard was stateful and order-dependent; and a
dead `execute_stow` backend action was left behind by the split.

## 11.10 Verdicts

| # | Scope | Verdict |
|---|---|---|
| 1 | **Sequencing with test doubles** | **PARTIAL (improved).** 29 steps, all 12 framework ordering requirements plus spec 2.4's stow-before-grasp rule. Every adapter is now a real call. A real run is still refused at preflight on 4 unresolved capabilities. |
| 2 | **Actual ROS transport** | **PASS.** 60 tests, four real processes (orchestrator, monitor, **real arm_planning**, doubles). All seven service types are real generated definitions. |
| 3 | **Real planner / perception computation** | **PARTIAL — improved in one specific place.** The **frame conversion is now real production code** doing a real tf2 lookup, matching independent calculations at zero and nonzero waist. The **planner and executor are still doubles** (`torch`, `trimesh`, `python-fcl` absent) and the **YOLO detector still cannot run** (`ultralytics` absent). Perception geometry and arm FK remain real. |
| 4 | **Physical robot execution** | **NOT TESTED.** Robot unavailable; no hardware commanded. |

**A test-double success is still not a working production adapter.** What is
newly claimed here is narrow: boundary 3's conversion is real, and it computes
the right numbers against real TF. The planner behind boundary 4 is not.

## 11.11 Blockers — removed and remaining

**Removed:** `frame_contract` as a monolithic blocker. The conversion half is
implemented, tested and resolved.

**Remaining, in dependency order:**

1. **`perception_extrinsics`** *(perception)* — measure the real D435 mount,
   decide how the waist angle enters, re-orthonormalise `R_base_opt`. **First
   blocker**: it gates `grasp` and `deliver_task`.
2. **A `stamp` field on `GetObjectPosition.srv`** *(perception)* — until then the
   conversion runs under a waist-motion guard on a receipt-time upper bound.
3. **`handover_pose`, `stow_pose`, `home_pose`** *(arm_planning)* — three
   validated poses. All three adapters are ready for them; `~poses` ships empty.
4. **`release_completion`** *(hand_control)* — expose settling.
5. **`/joint_states` must carry the five arm joints** *(robot-control)* — only
   `waist_yaw_joint` is confirmed; without them every planning step is NOT_READY.
6. **Deployment dependencies** — `torch`, `trimesh`, `python-fcl`, `ultralytics`.
