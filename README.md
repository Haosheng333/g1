# g1 — grasp-and-deliver on a Unitree G1

ROS 1 Noetic. A G1 23-DOF humanoid with a BrainCo Revo2 hand: navigate to a
table, find an object, grasp it, carry it to a person, hand it over on a
verified pull, return to a stowed pose.

**Status: the pipeline is wired end to end and tested in simulation. It has
never run on the robot.** Four unresolved contracts still block a real run, and
they are listed in [Blockers](#blockers). Nothing in this repository has ever
commanded hardware.

Branch: `feature/grasp-deliver-integration`. Detailed history and evidence live
in [`docs/`](#documents); this file is the current-state summary.

---

## Modules

Every directory is a catkin package. The package name differs from the
directory for two of them.

| Directory | Package | Role | State |
|---|---|---|---|
| `slam/` | `g1_slam` | Livox Mid-360 + FAST-LIO2 mapping, robot TF tree, relocalizer, **standing-pose waypoints** | working |
| `navigation/` | `g1_navigation` | semantic waypoint navigation, costmaps, motion gate | working |
| `perception/` | `perception` | RGB-D object detection → position in `base_link` | service works; **extrinsics unresolved** |
| `arm_planning/` | `arm_planning` | IK, grasp segmentation, collision masks, **frame conversion** | conversion works; planner needs `torch` |
| `arm_control/` | `arm_control` | joint-trajectory execution | service exists; untested on hardware |
| `hand_control/` | `hand_control` | Revo2 hand presets over the vendor SDK | services exist; untested on hardware |
| `handover/` | `handover` | force-based pull detection | logic complete; **force schema unconfirmed** |
| `orchestrator/` | `orchestrator` | sequences the above; implements none of them | working |
| `system_bringup/` | `system_bringup` | top-level launch, `pelvis → base_link` alias | working |
| `tools/` | — | dataset/training helpers, acceptance scripts | — |

The orchestrator **sequences** capabilities and deliberately implements no
perception, IK, TF maths or SDK control. A source scan enforces that.

---

## The task, as one command

```bash
rosservice call /orchestrator/run_deliver_task \
  "{target: table, destination: person, dry_run: true}"
```

**29 steps, one sequencer, one execution lock.** It never calls another public
route service, which would deadlock against its own lock.

```
 1 await_localization        11 execute_lift             21 execute_handover_pose
 2 enable_motion             12 execute_retract          22 settle_handover
 3 navigate          ◄ pickup 13 record_held_object      23 arm_monitor
 4 detect_object             14 plan_stow (planned at 7) 24 await_pull
 5 to_planner_frame          15 execute_stow             25 disarm_monitor
 6 plan_grasp                16 require_held_object      26 release_hand
 7 plan_stow        ◄ before 17 deliver_await_localization 27 confirm_released
 8 execute_to_pregrasp  any  18 deliver_enable_motion    28 settle_recipient
 9 open_gripper       motion 19 navigate_to_recipient ◄ delivery
10 execute_approach          20 plan_handover_pose       29 execute_retract_home
```

Four routes are advertised: `run_navigate_report`, `run_grasp`, `run_deliver`,
`run_deliver_task`. `run_auto_release` is not a route — it validates and records
a release request for the task that already owns the lock.

**Dry run is the default.** With `enable_motion:=false` no robot service client
is constructed at all, so a route can be rehearsed with the robot powered off.
No request field can turn motion on.

---

## Recording standing poses

Teleoperate the robot somewhere, press `A`, and navigation can send it back to
that exact position *and heading* later. Useful for the capture spot at the
table and the handover spot by the person.

```bash
roslaunch g1_slam localization.launch map_name:=<map>   # /slam/manager/state must be True

rosrun g1_slam record_waypoints.py _prefix:=capture     # drive, stop, square up, press A
rosrun g1_slam record_waypoints.py _prefix:=handover    # then the handover spot

rosrun g1_slam goto_waypoint.py _list:=true
rosrun g1_slam goto_waypoint.py _name:=capture_01
```

`A` records and **nothing else** — the recorder builds no publisher, no goal
client and no motion-gate client. Keys come from the terminal's own stdin via
`termios`, so no root and no `/dev/input`. A press is refused unless
localisation is ready, the `map → pelvis` transform is fresh, its frames are the
configured ones (so an odometry pose can never be written into a map-frame
file), and the file's map identity matches the live `manifest.yaml`.

Records live in `<maps_dir>/<map_name>/standing_waypoints.yaml`, written
atomically, preserving anything already there.

**Two kinds of target, two goal paths — do not merge them:**

| | file | goal path | offset |
|---|---|---|---|
| object centre | `semantic.yaml` | `/navigation/navigate` | yes, `approach_distance` |
| standing pose | `standing_waypoints.yaml` | `/move_base_simple/goal` | **none** |

A standing pose is already the chosen spot; offsetting it again would park the
robot a metre away. **Not yet wired into `run_deliver_task`** — its `target` /
`destination` still resolve through `semantic.yaml`, so standing poses are
driven to with `goto_waypoint.py` for now. See `slam/README.md` section 7b.

---

## What actually runs today

| Route | Real run | Why |
|---|---|---|
| `navigate_report` | ✅ **executes end to end** | navigation + perception only; no unresolved contract |
| `grasp` | ❌ refused at preflight | `perception_extrinsics`, `stow_pose` |
| `deliver` | ❌ refused at preflight | `handover_pose`, `home_pose`, `release_completion` |
| `deliver_task` | ❌ refused at preflight | all five of the above |

A blocked route runs **zero steps** and issues **zero hardware calls** — the
refusal happens before step 1, not after the hand has opened.

### Adapter status

**No stubs remain.** Every orchestrator adapter is a real call to a real
service. What blocks them is missing *inputs*, not missing code.

| Adapter | Calls | Reachable? |
|---|---|---|
| `await_localization`, `enable_motion`, `navigate`, `detect_object`, `report` | slam / navigation / perception | ✅ yes |
| `to_planner_frame` | `/arm/convert_observation` | no — `perception_extrinsics` |
| `plan_grasp` | `/arm/plan_grasp_from_point` | no — needs the converted target |
| `execute_segment` | `/arm/move_arm_joints` | no — needs a plan |
| `plan_stow`, `plan_handover_pose`, `execute_retract_home` | `/arm/plan_to_pose` + `/arm/move_arm_joints` | no — `~poses` ships empty |
| `open_gripper`, `close_gripper`, `release_hand` | `/hand_{side}/*` | gated by route |
| `arm_monitor`, `disarm_monitor`, `await_pull`, state adapters | handover / in-process | gated by route |

---

## Interfaces

| Endpoint | Type | Notes |
|---|---|---|
| `/orchestrator/run_*` | `orchestrator/RunRoute` | `target` = pickup, `destination` = delivery |
| `/navigation/navigate` | `g1_navigation/NavigateToObject` | semantic class name + approach distance |
| `/navigation/status` | `std_msgs/String` | latched; **`saw_active` guard** means a stale `SUCCESS` cannot complete a new goal |
| `/perception/get_object_position` | `perception/GetObjectPosition` | metres, `frame_id` = `base_link`; **no timestamp** |
| `/arm/convert_observation` | `arm_planning/ConvertObservation` | `base_link` → planner `torso_link` |
| `/arm/plan_to_pose`, `/arm/plan_grasp_from_point` | `arm_planning/*` | planning only; never moves anything |
| `/arm/move_arm_joints` | `arm_control/MoveArmJoints` | N × 5 flattened joint path |
| `/hand_{side}/{pre_grasp,grasp,release}` | `std_srvs/Trigger` | success = command **acknowledged**, not completed |
| `/auto_release_monitor/arm` | `handover/ArmMonitor` | carries the session token |

Joint order everywhere: `shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
wrist_roll`. Units: metres and radians.

---

## Two things that are easy to get wrong

### The two `torso_link` conventions (the 10 mm)

The live TF tree comes from `slam/urdf/g1_23dof_mode_10.urdf`; the planner's
numbers come from `arm_planning/urdf/g1_23dof.urdf`. They disagree about where
`torso_link` sits:

| chain | mode_10 (TF) | g1_23dof (planner) |
|---|---|---|
| `pelvis → torso_link` | 0.044 | 0.054 |
| `torso_link → shoulder_pitch` | 0.24778 | 0.23778 |
| `pelvis → shoulder_pitch` | **0.29178** | **0.29178** |

The sums are identical, so **neither URDF is wrong** — they place the same datum
10 mm apart. `/arm/convert_observation` applies a declared `−0.010 m` z
correction after the TF lookup, exact at every waist angle because the waist
rotates about z. **Do not "fix" either URDF, and do not redefine the TF frame.**

### A command acknowledgement is not completion

`/hand_{side}/release` returns only `success` and a message, and the `release`
preset leaves `require_settle` absent — so a success does **not** establish that
the fingers opened. Object states are deliberately distinct:
`HELD` → `RELEASE_COMMANDED` → `RELEASE_SETTLED`, and `transfer_verified` is
**always false** because nothing in this workspace can observe the object
leaving. Retraction after a release is refused with no override.

---

## Tests

**729 passing, 0 failing.** No hardware, no robot, no SDK.

```bash
# cross-process ROS integration: isolated master on :11399, real orchestrator +
# monitor + arm_planning, hardware-facing doubles with real generated types
orchestrator/test/integration/run_integration.sh            # 60 tests

# offline suites (run in the ROS container; some need numpy)
cd <pkg> && PYTHONPATH=src python3 -m unittest discover -s test -t .
```

| Suite | Result |
|---|---|
| ROS integration | **60 passed** |
| `orchestrator` | 227 passed |
| `handover` | 155 passed |
| `navigation` | 119 passed |
| `arm_planning` | 103 passed |
| `slam` | 47 passed |
| `arm_control` | 10 passed, 1 skipped (no SDK) |
| `perception` | 8 passed, 1 skipped, **1 blocked** (`ultralytics` absent) |

**16 modules blocked**, all on absent dependencies: `torch`, `trimesh`,
`python-fcl`, `ultralytics`, `pytest`.

### What the tests do and do not prove

| | |
|---|---|
| ✅ **real production code** | orchestrator, sequencer, preflight, nav guard, handover state, execution lock, the tactile detector, and **the frame conversion** |
| ✅ **real ROS transport** | four separate processes over a real master; all service types are the real generated definitions |
| ⚠️ **test doubles** | navigation, perception, hand, **the arm planner and executor** behaviour |
| ❌ **never run** | any hardware. No arm, hand, camera or LiDAR code has executed |

**A test-double success is not a working production adapter.** A green suite
means the orchestrator calls the real interfaces correctly and handles their
responses and failures correctly — not that those services do the right thing
with a real robot.

---

## Blockers

In dependency order. Each is owned by one module.

1. **`perception_extrinsics`** *(perception)* — **first blocker.** `Extrinsics()`
   is hard-coded from a MuJoCo string for a D435 on a *different robot*; that
   camera is on `torso_link` and **does** rotate with the waist while the
   extrinsics are a fixed transform (so output is only valid near zero waist
   yaw); and `_default_R_base_opt()` is not a rotation (det 1.0003760,
   ~188 µm/m of scale-and-shear). Gates `grasp` and `deliver_task`.
2. **A `stamp` field on `GetObjectPosition.srv`** *(perception)* — until then the
   conversion runs under a waist-motion guard on a receipt-time upper bound.
3. **Three validated poses — `handover`, `stow`, `home`** *(arm_planning)* —
   must be reachable, inside the trained workspace and collision-free. The
   adapters are ready; `orchestrator/config/orchestrator.yaml` ships
   `poses: {}` on purpose. Writing a number there does **not** clear the
   capability.
4. **`release_completion`** *(hand_control)* — expose finger settling (a
   structured flag, or the feedback positions).
5. **`/joint_states` must carry the five arm joints** *(robot-control)* — only
   `waist_yaw_joint` is confirmed. Without them every planning step is NOT_READY;
   no default posture is substituted.
6. **Deployment dependencies** — `torch`, `trimesh`, `python-fcl` for the
   planner; `ultralytics` for perception.

### Also outstanding, not blocking a dry run

- **IK checkpoints are stale by 54.0 mm.** The confirmed tool point is
  `(0.215, 0.06, 0.025)`; both checkpoints embed the pre-correction
  `±0.006`, which is 12–13× their own recorded validation error. Retraining is
  required. Collision masks are provably **unaffected**
  (`g1_collision_probe` never reads `grasp_xyz`).
- **Revo2 force schema unconfirmed** — the SDK call, field names, channel→finger
  mapping, **unit** and rate, plus which sign a pull produces. The detector's
  3 N threshold is the *requested setting*, not a measured figure.
- **Revo2 collision geometry** — both URDFs model the stock Unitree
  `wrist_roll_rubber_hand`, and that `.STL` is still missing from
  `arm_planning/meshes/`.

---

## Environment

Two machines with different layouts; do not mix them up.

| | Machine A | Machine B (`CLAUDE.md`) |
|---|---|---|
| Repo | `~/g1_ros1_ws/src/g1` | `~/Workspace/DIP` |
| Container | `g1-ros1` | `g1_intellect` via `docker/g1.sh` |
| Source space | the whole repo | the nested `g1_ws` |

On machine A the whole repo is the catkin source space, so it also descends into
`g1_ws` and finds `g1_slam` twice. Fix with an **untracked** `g1_ws/CATKIN_IGNORE`
plus a `.git/info/exclude` entry — never by editing a tracked file, because
`g1_ws/src/livox_ros_driver2` encodes machine B's layout on purpose.

`rospack` caches: after changing package visibility, delete
`~/.ros/rospack_cache_*` or it keeps returning the stale path.

---

## Documents

| File | Contents |
|---|---|
| [`docs/DATA_PIPELINE_VALIDATION.md`](docs/DATA_PIPELINE_VALIDATION.md) | the boundary-by-boundary validation report. **§11 is current**; §§1–10 are earlier snapshots, superseded where they disagree |
| [`docs/GRASP_DELIVER_HANDOFF.md`](docs/GRASP_DELIVER_HANDOFF.md) | session-by-session handoff notes; §14 is the latest, with a PR-ready summary |
| [`docs/DELIVERY_FLOW.md`](docs/DELIVERY_FLOW.md) | the handover sequence, session token, release states |
| [`docs/FRAME_CONTRACT.md`](docs/FRAME_CONTRACT.md) | `base_link` / `pelvis` decision |
| [`slam/README.md`](slam/README.md) | SLAM topics, TF tree, map pipeline, health checks |
| [`handover/README.md`](handover/README.md) | the detection rule, what the SDK still has to supply |
| [`orchestrator/README.md`](orchestrator/README.md) | routes, preflight, execution lock |
| [`CLAUDE.md`](CLAUDE.md) | machine B's build and run procedure |

---

## Conventions worth keeping

- **Readiness is three-valued**: `PASS` / `WAIT` / `FAIL`. A missing *hardware*
  or cross-team input is **WAIT**, never FAIL; a malformed config or an
  unresolvable launch is FAIL. `--require-hardware` promotes WAIT to FAIL so a
  check can gate an on-robot bring-up.
- **Never substitute a guess for a missing input.** No invented poses, no
  default postures, no frozen joint angles, no assumed units.
- **An unresolved cross-team decision is a preflight blocker**, not a runtime
  surprise — so a route is refused before it navigates, not after it opens the
  hand.
- **Record provenance when vendoring.** `slam/urdf/SOURCE.md` carries the
  upstream URL, commit and SHA-256, and the health check re-verifies both digests.
