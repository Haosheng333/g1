# g1 — grasp-and-deliver on a Unitree G1

ROS 1 Noetic. A G1 23-DOF humanoid with BrainCo Revo2 hands, a Livox Mid-360
LiDAR and a RealSense D455. One spoken sentence makes it walk to a desk, pick up
a can, walk to a chair and hand the can to the person who pulls on it:

```
"go to the desk, pick up a can, go to the chair, and pass me, over"
```

Every module below runs on the robot. The orchestrator's `fetch_deliver` route
chains them; the voice frontend speaks to the orchestrator. This file is the
entry point: what each module does, how the pipeline fits together, and how to
run it. Per-package detail is in each package's README ([Documents](#documents)).

---

## The pipeline

The orchestrator is the top of the system. It subscribes to the voice module's
confirmed command, then drives every other module through its service and waits
on its status topic. One route, one execution lock, one command at a time.

```
orchestrator                      /orchestrator/run_fetch_deliver   (preflight, 14 steps, one lock)
    │
    ├── voice_control             /voice/command  (g1_voice_control/TaskCommand)   voice_control
    │       microphone -> Vosk (English small model, lexicon grammar)
    │       final results joined until the end word "over"
    │       slot grammar: navigate -> grasp -> navigate -> handover
    │           desk/table -> table_01    chair -> chair_01    can -> bottle
    │       "confirm" within 8 s          (or cancel_window: silence means go)
    │       -> {fetch_deliver, target: table_01, destination: chair_01, label: bottle}
    │       <- /orchestrator/command_result   (TaskResult, same seq) when the route ends
    │
    ├── resolve both standing waypoints   slam/maps/<map>/standing_waypoints.yaml
    ├── preflight                         every service, topic and flag below; one blocker -> zero steps
    │
    ├── 1  await_localization        /slam/manager/state                       slam
    ├── 2  enable_motion             /navigation/enable_motion(true)           navigation
    ├── 3  navigate_to_pickup        /move_base_simple/goal  <- table_01       navigation
    │                                wait on /navigation/status -> SUCCESS
    ├── 4  settle_at_pickup          1.5 s, the base stops rocking
    ├── 5  detect_object             /perception/get_object_position(bottle)   perception
    │                                -> (x, y, z) in torso_link
    ├── 6  plan_grasp_point          /g1_arm_simple/plan_grasp_from_point      arm_grasp
    │                                whole grasp planned, nothing moves
    ├── 7  execute_grasp             /g1_arm_simple/execute_grasp_from_point   arm_grasp
    │                                    pregrasp -> /g1_hand_<side>/pre_grasp   hand_control (g1_hands, PC2)
    │                                    approach -> /g1_hand_<side>/grasp_5f
    │                                    lift -> retract -> home
    ├── 8  record_held_object        the ONLY writer of held-object state
    ├── 9  hand_back_arms            /g1_arm_simple/release_arm_control        arm_grasp
    │                                arms back to the G1's own controller before walking
    ├── 10 navigate_to_recipient     /move_base_simple/goal  <- chair_01       navigation
    ├── 11 settle_at_recipient       1.5 s
    ├── 12 handover_start            /handover/start                           handover
    │                                    elbow-up pose -> plan_to_pose -> move_arm_joints
    │                                    settle -> /auto_release_monitor/arm   (g1_hands tactile)
    ├── 13 await_handover            /handover/state -> done                   handover
    │                                    pull detected -> /handover/run_auto_release
    │                                        /g1_hand_<side>/release -> 1.5 s -> GoHomeArm
    │                                    timeout (120 s) -> /handover/abort, can stays in hand
    └── 14 hand_back_arms_after_handover   /g1_arm_simple/release_arm_control   arm_grasp

    finally, on every exit:  /navigation/cancel, enable_motion(false), handover abort if armed
```

---

## Modules

Every directory is a catkin package, built in the nested `g1_ws` through
relative symlinks (see [`CLAUDE.md`](CLAUDE.md)). The package name differs from
the directory for `slam`, `navigation` and `voice_control`.

| Directory | Package | Role |
|---|---|---|
| `orchestrator/` | `orchestrator` | the top of the system: subscribes to the spoken command, sequences the modules through their services, implements none of them |
| `voice_control/` | `g1_voice_control` | the command: Vosk, slot grammar, confirmation, one `TaskCommand` on `/voice/command` |
| `slam/` | `g1_slam` | Mid-360 + FAST-LIO2 mapping, relocalization, robot TF, standing-pose waypoints |
| `navigation/` | `g1_navigation` | costmap planning and path following to a pose or a semantic object, motion gate, locomotion bridge |
| `perception/` | `perception` | D455 bottle detection → object centre in `torso_link` |
| `arm_grasp/` | `arm_grasp` | the `g1_arm_simple` arm node: NN IK, voxel A*, the complete grasp, arm primitives, SDK hand-over |
| `hand_control/` | `hand_control` | the Revo2 hands: the PC2 nodes via `start_remote_hands.sh`, plus a local driver |
| `handover/` | `handover` | hand the held object to a person on a tactile pull |
| `system_bringup/` | `system_bringup` | top-level launch files |
| `tools/` | — | pipeline start/stop script, readiness checklist, acceptance and dataset scripts |
| `arm_planning/`, `arm_control/` | `arm_planning`, `arm_control` | the earlier manipulation stack, kept for the `grasp` / `deliver` routes |

### `orchestrator/` — the top of the system

Subscribes to `voice_control`'s `/voice/command` (and serves the same routes
as typed `rosservice call`s) and runs each command as a declarative route over
the other modules' interfaces: no IK, no TF maths, no planning, no SDK, and a
source scan enforces it. The answer goes back on `/orchestrator/command_result`. A route is preflighted first
(every service, topic and flag it needs, reported as one complete list), runs
under one non-blocking execution lock, fails fast, and cleans up on every exit
path: cancel navigation, shut the gate, disarm the handover. With
`enable_motion:=false` a route is a dry run in which no robot client is even
constructed. Routes: `fetch_deliver` (the spoken task), `navigate_report`, and
`grasp` / `deliver` / `deliver_task` on the earlier stack. `/orchestrator/status`
is latched.

### `voice_control/` — the spoken command

Vosk on the workstation microphone (English small model, constrained to the
lexicon vocabulary, resampled from the codec's 48 kHz), an utterance buffer
closed by the end word "over", a slot grammar (`config/lexicon.yaml`: verbs,
objects → perception label, places → standing waypoints, priors) that accepts
exactly the navigate → grasp → navigate → handover shape, a confirmation step
("confirm" within 8 s, or `cancel_window` mode where silence means go), and
one `TaskCommand` published on `/voice/command` per confirmed task, answered by
the orchestrator on `/orchestrator/command_result`. The package depends on no
other DIP package; the orchestrator depends on it. Exact phrases cover
`status`, `cancel task` and the single-route starts; `input_mode:=scripted`
replays transcripts without audio; `recognize_wav.py` runs the recogniser over
a WAV file.

### `slam/` — mapping, localization, robot TF, standing poses

The Livox driver and FAST-LIO2 produce the map and the odometry
(`mapping.launch`, `save_map.py`, `pcd_to_map`). For a run, `localization.launch`
starts `relocalizer.py`, which matches live scans to the saved `map.pcd` and
publishes `map → camera_init`; `slam_manager.py`, which publishes the
localization verdict on `/slam/manager/state` and a readable line on
`/slam/manager/status`; `grid_composer.py`, which publishes the tiered `/map`;
and `g1_tf_publisher.py`, which completes the tree to `pelvis` from
`/joint_states`. Maps live in `slam/maps/<name>/`; `current` points at the
last one. **Standing poses** are the pipeline's navigation targets: drive the
robot to a spot, press `A` in `record_waypoints.py`, and the exact pose is
stored in `standing_waypoints.yaml` together with the map's identity.
`goto_waypoint.py` sends one back by hand; the orchestrator uses the same
`g1_slam.standing_goal` code. `slam_health_check.py` checks the driver, the
host IP and the TF chain. `lab_map_DIP` holds `table_01` and `chair_01`.

### `navigation/` — planning, following, motion gate

`navigation_node.py` accepts an exact pose on `/move_base_simple/goal` or a
semantic object through `/navigation/navigate` (centre from `semantic.yaml` plus
an approach distance), plans on the costmap (`global_planner.py`,
`costmap.py`), follows the path (`local_planner.py`) and reports
`PLANNING | FOLLOWING | RECOVERING | SUCCESS | FAILURE` on `/navigation/status`.
`obstacle_cloud_node.py` feeds live LiDAR obstacles into the costmap.
`cmd_vel_gate.py` forwards velocity commands only while
`/navigation/enable_motion` is true and publishes the gate state on
`/navigation/motion_enabled`; `locomotion_bridge.py` turns them into G1 walking
commands; `g1_joint_state_bridge.py` supplies `/joint_states`. `sim.launch`
runs the real nodes against a simulated world. Launch on the robot:
`system_bringup/launch/navigation.launch` (slam + bridges + navigation, gate shut).

### `perception/` — where is the can

`perception_node.py` pairs the RealSense colour and aligned-depth streams,
runs the fine-tuned detector (`models/bottle_finetuned.pt`, class `bottle` = the
14 cm can), back-projects the detection and reports the object's geometric
centre in `torso_link` through `/perception/get_object_position`
(label, index → x, y, z). The D455 extrinsics are a ChArUco calibration refitted
on this robot against a tape measure, with a constant residual offset in
`perception.launch`. The driver runs on PC2 (`scripts/start_remote_camera.sh`)
or on this host (`local_camera:=true`); `perception_viewer.py` shows the
detections and the depth the node reads; `stop_perception.sh` tears it down.

### `arm_grasp/` — the grasp (node `g1_arm_simple`)

The g1_intellect arm node inside DIP: per-arm neural-network IK, an
obstacle-aware voxel A* planner fed by the depth stream, and the complete grasp
pipeline in one node. `plan_grasp_from_point` plans with no motion;
`execute_grasp_from_point` drives pregrasp → hand `pre_grasp` → approach → hand
`grasp_5f` → lift → retract → home. `move_arm_joints`, `plan_to_pose`,
`GoHomeArm` and `GoHome` are the primitives the handover uses;
`acquire_arm_control` / `release_arm_control` switch the arms between the SDK
and the G1's own controller, the hand-back being accepted only with both arms at
home. Weights and the voxel bitmaps are shared with g1_intellect through
`models/`. `grasp_point.py plan|grasp|release|home` is the CLI; the launch is
`arm_node_abs.launch` (bottle-only by default).

### `hand_control/` — the Revo2 hands

Both hands are plugged into PC2, so the nodes that drive them are
g1_intellect's `g1_hands`, started from this workstation with
`start_remote_hands.sh` and registered with this master. They provide
`/g1_hand_{left,right}/{pre_grasp,grasp_5f,release}` and the tactile topics
that `arm_grasp` and the handover monitor use. `hand_node.py` is DIP's own
driver for a hand plugged into the workstation (`/hand_{side}/*`, presets in
`config/finger_presets.yaml`), used by the earlier stack.

### `handover/` — give the can to a person

`handover_node.py` is g1_intellect's handover flow: `/handover/start` lifts the
holding arm through an "elbow up" pose, plans to the handover pose with
`plan_to_pose`, drives it in short `move_arm_joints` steps, settles, and arms
g1_hands' tactile auto-release monitor. A detected pull calls
`/handover/run_auto_release`: open the hand, wait 1.5 s, `GoHomeArm`.
`/handover/state` (latched) goes `idle → moving → armed → releasing → done`;
`/handover/abort` disarms and leaves the arm where it is; `handover_cli.py`
drives all of it by hand. Launch: `handover.launch arm:=<side>`, which also
starts the monitor. The package also keeps the earlier detection-only monitor
and detector (`auto_release_monitor.launch`) used by the `deliver` route.

### `arm_planning/`, `arm_control/` — the earlier manipulation stack

DIP's own planner (IK, grasp segmentation, collision masks, the
`base_link → torso_link` conversion service) and joint-trajectory executor, with
`hand_control`'s local driver and the detection-only monitor. They serve the
`grasp`, `deliver` and `deliver_task` routes, which the orchestrator still
refuses at preflight on the open items listed under [Known limits](#known-limits).
The spoken task does not use them.

### `system_bringup/`, `tools/`

`fetch_deliver.launch` brings up the whole spoken-task stack in one roslaunch;
`navigation.launch`, `manipulation.launch` and `full_system.launch` are the
per-stack compositions. `tools/fetch_deliver_pipeline.sh` starts every
component as its own `roslaunch` with a log and PID each and stops them again;
`tools/fetch_deliver_check.py` prints the readiness checklist. The rest of
`tools/` is acceptance scripts (`nav_acceptance_test.py`, `ab_live_check.py`,
`check_saved_map.py`, `survey_semantic.py`) and the dataset and training
helpers behind the bottle detector.

---

## Running the spoken task

Inside the container (`~/Workspace/g1_docker/g1.sh enter`), with the robot
cable up, PC2 reachable, a can on the desk and a person at the chair:

```bash
tools/fetch_deliver_pipeline.sh start                 # roscore, navigation, camera (PC2), perception,
                                                      # arm_grasp, hands (PC2), handover, orchestrator, voice
tools/fetch_deliver_pipeline.sh check --until-ready   # relocalization + every module, PASS / WAIT / FAIL,
                                                      # ends with "READY TO SPEAK" or what is still open
# say the sentence, end with "over", then say "confirm" within 8 s
tools/fetch_deliver_pipeline.sh stop
```

The checklist reports, in route order: relocalization (`/slam/manager/status`,
the relocalizer's status and fit), map, joint states, navigation, the motion
gate, both standing waypoints, the camera streams and a live perception read,
the `g1_arm_simple` services, the hand services, the handover node and its arm,
the orchestrator's mode and the voice node. `WAIT` is a latched topic that has
not arrived yet, usually localization settling; `FAIL` names the launch or
script to start.

Motion is on by default in the script (`ENABLE_MOTION=true`) and in
`fetch_deliver.launch`; `ENABLE_MOTION=false` rehearses everything as a dry run.
A typed request works without the voice node:

```bash
rosservice call /orchestrator/run_fetch_deliver "{target: table_01, destination: chair_01, label: bottle}"
```

If the grasp fails the arm stays under SDK control: `rosrun arm_grasp
grasp_point.py home` parks it. If nobody pulls within 120 s the monitor is
disarmed and the can stays in the hand: `rosrun handover handover_cli.py release`
opens it. Every exit shuts the motion gate. Details in
[`orchestrator/README.md`](orchestrator/README.md); the grammar and the
microphone notes in [`voice_control/README.md`](voice_control/README.md).

---

## Recording standing poses

Teleoperate the robot somewhere, press `A`, and navigation can send it back to
that exact position and heading later. This is how `table_01` and `chair_01`
were made.

```bash
roslaunch g1_slam localization.launch map_name:=<map>   # /slam/manager/state must be True

rosrun g1_slam record_waypoints.py _prefix:=table       # drive, stop, square up, press A
rosrun g1_slam record_waypoints.py _prefix:=chair

rosrun g1_slam goto_waypoint.py _list:=true
rosrun g1_slam goto_waypoint.py _name:=table_01
```

`A` records and nothing else: the recorder has no publisher, no goal client and
no motion-gate client. A press is refused unless localization is ready, the
`map → pelvis` transform is fresh and in the configured frames, and the file's
map identity matches the live `manifest.yaml`. Records live in
`<maps_dir>/<map_name>/standing_waypoints.yaml`, written atomically.

**Two kinds of target, two goal paths:**

| | file | goal path | offset |
|---|---|---|---|
| object centre | `semantic.yaml` | `/navigation/navigate` | yes, `approach_distance` |
| standing pose | `standing_waypoints.yaml` | `/move_base_simple/goal` | none |

A standing pose is already the chosen spot; offsetting it again would park the
robot a metre away. `fetch_deliver` drives to standing poses; `deliver_task`
resolves its names through `semantic.yaml`. Edit
`voice_control/config/lexicon.yaml` if the poses are re-recorded under other
names.

---

## Routes

| Route | Service | Stack | Status |
|---|---|---|---|
| `fetch_deliver` | `/orchestrator/run_fetch_deliver` | standing waypoints, perception, `arm_grasp`, `handover_node` | **the spoken task** |
| `navigate_report` | `/orchestrator/run_navigate_report` | navigation + perception | runs; reports the detection |
| `grasp`, `deliver`, `deliver_task` | `/orchestrator/run_{grasp,deliver,deliver_task}` | `arm_planning` / `arm_control` / `hand_control` | refused at preflight, see Known limits |

`run_auto_release` is not a route: it validates and records a release request
for the task that owns the lock (the `deliver` design). A blocked route runs
zero steps and issues zero hardware calls.

---

## Interfaces

| Endpoint | Type | Notes |
|---|---|---|
| `/voice/command` → `/orchestrator/command_result` | `g1_voice_control/TaskCommand`, `TaskResult` | the spoken task and its answer, matched by `seq`; the orchestrator subscribes |
| `/orchestrator/run_*` | `orchestrator/RunRoute` | the same routes typed: `target` = pickup, `destination` = delivery, `label` = perception class, `dry_run` |
| `/orchestrator/status` | `std_msgs/String` | latched: idle / preflight / RUNNING / done / FAILED, and the motion mode |
| `/slam/manager/state`, `/slam/manager/status` | `std_msgs/Bool`, `String` | latched localization verdict and its readable line |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | exact standing pose in `map`, no approach offset |
| `/navigation/navigate` | `g1_navigation/NavigateToObject` | semantic class name + approach distance |
| `/navigation/status` | `std_msgs/String` | latched; a terminal state counts only after an ACTIVE one (`saw_active` guard) |
| `/navigation/enable_motion`, `/navigation/motion_enabled` | `std_srvs/SetBool`, `std_msgs/Bool` | the motion gate and its state |
| `/perception/get_object_position` | `perception/GetObjectPosition` | metres in `torso_link` |
| `/g1_arm_simple/{plan,execute}_grasp_from_point` | `arm_grasp/*` | plan with no motion, then the whole grasp |
| `/g1_arm_simple/{move_arm_joints,plan_to_pose,GoHomeArm,release_arm_control}` | `arm_grasp/*`, `std_srvs/Trigger` | arm primitives and the SDK hand-back |
| `/g1_hand_{side}/{pre_grasp,grasp_5f,release}` | `std_srvs/Trigger` | the Revo2 hands on PC2 |
| `/handover/start`, `/handover/abort`, `/handover/run_auto_release` | `std_srvs/Trigger` | handover_node |
| `/handover/state` | `std_msgs/String` | latched: `idle / moving / armed / releasing / done / error:…` |
| `/auto_release_monitor/{arm,disarm}` | `g1_hands/*` | the tactile monitor handover_node arms |

Joint order everywhere: `shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
wrist_roll`. Units: metres and radians. Frames: `map` (z-up) → `camera_init` →
`pelvis`; `base_link` is an identity alias of `pelvis`; arm targets and
perception points are in `torso_link`.

---

## Two things that are easy to get wrong

### The two `torso_link` conventions (the 10 mm)

The live TF tree comes from `slam/urdf/g1_23dof_mode_10.urdf`; the
`arm_planning` numbers come from `arm_planning/urdf/g1_23dof.urdf`. They place
`torso_link` 10 mm apart (`pelvis → torso_link` 0.044 vs 0.054) while
`pelvis → shoulder_pitch` is 0.29178 in both, so neither URDF is wrong.
`/arm/convert_observation` applies a declared −0.010 m z correction after the TF
lookup. `arm_grasp` and `perception` share one `torso_link` datum and one D455
calibration file, so the spoken task never crosses this boundary. Do not "fix"
either URDF.

### A standing pose is not an object

`/navigation/navigate` stands `approach_distance` away from an object centre;
a standing pose is sent through `/move_base_simple/goal` and must not be
offset. The orchestrator keeps the two actions apart (`navigate` vs
`navigate_standing`), and `goto_waypoint.py` refuses a record from another map.

---

## Tests

```bash
# offline suites, no ROS master
cd orchestrator/test  && python3 -m unittest discover -s .
cd voice_control      && PYTHONPATH=src python3 -m unittest discover -s test
cd handover           && python3 -m unittest discover -s test
cd slam/test          && python3 -m unittest discover -s .        # numpy needed for some
cd navigation         && python3 -m unittest discover -s test

# cross-process ROS integration of the arm_planning stack, isolated master on :11399
orchestrator/test/integration/run_integration.sh
```

| Suite | Covers |
|---|---|
| `orchestrator` (246) | route order and resume, preflight, the execution lock, dry-run traversal, cleanup, the `fetch_deliver` route and node against the real waypoint resolver, the delivery handover state machine |
| `g1_voice_control` (90) | the slot grammar on the spoken sentence and its refusals, the utterance buffer, confirmation and cancel-window dispatch, request mapping, the resampler |
| `handover` (155) | the tactile detector and the detection-only monitor |
| `navigation` (119) | planners, costmap, gate, status contract |
| `slam` (61+) | standing store, waypoint scripts, map store, TF chain |
| ROS integration (60) | real orchestrator + monitor + `arm_planning` over a real master with hardware doubles |

The pipeline itself is checked live with `tools/fetch_deliver_pipeline.sh check`.

---

## Known limits

- **The earlier stack is refused at preflight.** `grasp`, `deliver` and
  `deliver_task` wait on validated `handover`, `stow` and `home` poses from
  `arm_planning`, a settling signal from `hand_control`'s release, verified
  extrinsics for the `base_link` conversion path, and `torch` in the
  deployment. None of this is on the spoken task's path.
- **A release is a command, not a verified transfer.** No sensor in the system
  observes the object leaving the hand; `transfer_verified` in the route
  response is therefore always false. The tactile pull is the trigger, the
  hand opening is the action.
- **One arm per run.** `side` selects the grasping arm and must be passed to
  both the orchestrator and `handover.launch`; the checklist flags a mismatch.
- **Vocabulary is the map.** The voice lexicon names standing waypoints
  (`table_01`, `chair_01`) and one detector class (`bottle`); another map or
  object means editing `lexicon.yaml`.

---

## Environment

| | |
|---|---|
| Repo | `~/Workspace/DIP`, bind-mounted into the container at `/home/g1/Workspace/DIP` |
| Container | `g1_intellect` via `~/Workspace/g1_docker/g1.sh start|enter|stop` |
| Source space | the nested `g1_ws` (symlinks to each package); build with the `catkin_make` line in `CLAUDE.md` |
| Robot network | `G1_NETWORK_INTERFACE` (workstation `192.168.123.100`, LiDAR `.120`, PC2 `.164`) |
| Audio | workstation microphone = sounddevice index 4 (`ALC245 Analog`, 48 kHz) |

`rospack` caches: after changing package visibility, delete
`~/.ros/rospack_cache_*`.

---

## Documents

| File | Contents |
|---|---|
| [`CLAUDE.md`](CLAUDE.md) | build and run procedure for this workstation and the container |
| [`orchestrator/README.md`](orchestrator/README.md) | routes incl. `fetch_deliver`, preflight, execution lock, the delivery state machine |
| [`voice_control/README.md`](voice_control/README.md) | the spoken task grammar, confirmation modes, microphone notes |
| [`arm_grasp/README.md`](arm_grasp/README.md) | the `g1_arm_simple` services, the grasp pipeline, the voxel planner |
| [`perception/README.md`](perception/README.md) | the detector, the D455 extrinsics and offsets, PC2 vs local camera |
| [`handover/README.md`](handover/README.md) | `handover_node` and the detection-only monitor |
| [`slam/README.md`](slam/README.md) | SLAM topics, TF tree, map pipeline, standing waypoints, health checks |
| [`navigation/README.md`](navigation/README.md) | frames, interfaces, parameters |
| [`docs/FRAME_CONTRACT.md`](docs/FRAME_CONTRACT.md) | the `base_link` / `pelvis` decision |
| [`docs/DELIVERY_FLOW.md`](docs/DELIVERY_FLOW.md) | the `deliver` route's handover sequence, session token, release states |
| [`docs/DATA_PIPELINE_VALIDATION.md`](docs/DATA_PIPELINE_VALIDATION.md), [`docs/GRASP_DELIVER_HANDOFF.md`](docs/GRASP_DELIVER_HANDOFF.md) | history of the `arm_planning` stack's validation |

---

## Conventions worth keeping

- **Readiness is three-valued**: `PASS` / `WAIT` / `FAIL`. A latched topic that
  has not arrived is `WAIT`; a missing node or a malformed config is `FAIL`.
- **Never substitute a guess for a missing input.** No invented poses, no
  default postures, no made-up waypoints: a missing one is a blocker.
- **Check before moving.** Preflight runs to completion and reports every
  blocker; one blocker means zero steps.
- **Record provenance when vendoring.** `slam/urdf/SOURCE.md` carries the
  upstream URL, commit and SHA-256, and the health check re-verifies them.
