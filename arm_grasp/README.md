# arm_grasp

ROS1 (Noetic) package for absolute arm control with NN-based grasp inference
on Unitree G1. One node, `g1_arm_simple`, owns all low-level motion: NN IK,
voxel-based obstacle-aware A* planning, per-arm pose compensation, dual-arm
home, and hand-service coordination.

Higher-level orchestrators (e.g. `main_process`) sit on top of this node
and drive composite behaviours via the services below. `arm_grasp` never
talks to voice or perception topics directly.

## Node

- Script: `scripts/arm_node_abs.py`
- Node name: `g1_arm_simple` (all services live under `/g1_arm_simple/...`).

## Build

Built as part of DIP's `g1_ws` inside the g1_intellect container (see the
repo-root `CLAUDE.md`), through the relative symlink
`g1_ws/src/arm_grasp -> ../../arm_grasp`:

```bash
cd ~/Workspace/g1_docker && ./g1.sh enter 'cd ~/Workspace/DIP/g1_ws && PATH="$CONDA_PREFIX/bin:$PATH" catkin_make -j8 -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release'
```

`g1_camera` (for `GetObjectPosition` / `GetBerries`) resolves from the
g1_intellect workspace that every container shell already sources.

`models/` is a tracked relative symlink to
`../../g1_intellect/g1_arm_abs/models`: the MLP weights and the ~28 GB voxel
bitmaps are shared with g1_intellect, not copied.

The package is named `arm_grasp` so it never shadows g1_intellect's
`g1_arm_abs`. The node name (`g1_arm_simple`) and every service name are the
same, so never run both nodes at once.

## Run

Overlay DIP first in every shell: `source ~/Workspace/DIP/g1_ws/devel/setup.bash --extend`.

```bash
roslaunch arm_grasp arm_node_abs.launch
```

or

```bash
rosrun arm_grasp arm_node_abs.py
```

The A* planner is the **obstacle-aware voxel planner** (the grasp
pre-path avoids obstacles read live from the head camera). It needs a
one-time provisioning step for the per-arm bitmaps and MLPs; see
"Obstacle-aware voxel A* backend" below for setup. If any pinned weight
is missing or its sha256 drifts at startup, the node aborts via
`weight_verify.verify_pinned_weights()` with a hint to re-run
`rosrun arm_grasp fetch_weights.py`.

If `~/.ros` is on a read-only mount the node falls back to
`ROS_HOME=/tmp/ros_home` automatically.

Optional env:

```bash
export G1_NETWORK_INTERFACE=eno1
```

## Grasp a typed point (no camera)

Coordinates are `torso_link`: x forward, y left, z up (m). The grasp gate is
`x <= 0.39`, `z >= -0.02`. The hands are g1_intellect's `g1_hands` nodes on
PC2; start them from here with `rosrun arm_grasp start_remote_hands.sh`
(a DIP copy of g1_intellect's script; `scripts/start_remote_hands.sh`), then
check `rosservice list | grep g1_hand_` shows BOTH sides.

```bash
roslaunch arm_grasp arm_node_abs.launch network_interface:=enp196s0
rosrun arm_grasp grasp_point.py plan  0.33 -0.08 0.05   # plan only, never moves
rosrun arm_grasp grasp_point.py grasp 0.33 -0.08 0.05   # plan, confirm y/N, execute
rosrun arm_grasp grasp_point.py release --arm right     # open the hand
rosrun arm_grasp grasp_point.py home                    # open both hands, arms home,
                                                        # hand arms back to the robot
```

`--arm` defaults to `left` when `y > 0`, else `right`. `grasp` runs pregrasp
-> `pre_grasp` hand -> grasp -> `grasp_5f` hand -> lift `~lift_dz`. Without
the depth stream the voxel A* plans WITHOUT obstacles: the table is invisible
to it, so keep the swept space clear.

## Services

With node name `g1_arm_simple`, services are:

| Name | Type | Purpose |
|---|---|---|
| `/g1_arm_simple/move_arm_joints`              | `arm_grasp/MoveArmJoints`             | Drive one arm to 5 joint angles |
| `/g1_arm_simple/infer_grasp_from_point`       | `arm_grasp/InferGraspFromPoint`       | NN-infer pregrasp + grasp joints from xyz |
| `/g1_arm_simple/plan_grasp_from_point`        | `arm_grasp/PlanGraspFromPoint`        | Full plan (joints + A* pre_path + Cartesian target_path + lift), no motion |
| `/g1_arm_simple/execute_grasp_from_point`     | `arm_grasp/ExecuteGraspFromPoint`     | Plan + move + hand services + lift (all inside this node) |
| `/g1_arm_simple/execute_astar_path_from_point`| `arm_grasp/ExecuteAstarPathFromPoint` | Same as above, forces A* on |
| `/g1_arm_simple/grasp_bottle`                 | `std_srvs/Trigger`                     | Reads `~grasp_bottle_label` xyz from camera, then full pipeline |
| `/g1_arm_simple/GoHome`                       | `std_srvs/Trigger`                     | Releases BOTH hands, then dual-arm A* home |
| `/g1_arm_simple/acquire_arm_control`          | `std_srvs/Trigger`                     | Take the arms over from the robot's own controller (weight 0->1 ramp) |
| `/g1_arm_simple/release_arm_control`          | `std_srvs/Trigger`                     | Hand the arms back to the robot's own controller (weight 1->0 ramp) |

`plan_grasp_from_point` is what `main_process_node` uses — it returns the
waypoint lists so the orchestrator can drive each hop with `move_arm_joints`
and interleave the correct hand services per arm.

### 1) `move_arm_joints`

```bash
rosservice call /g1_arm_simple/move_arm_joints "arm: 'right'
joint_angles: [0.1, -0.2, 0.3, 0.0, 0.0]
wait: true
timeout: 5.0"
```

### 2) `infer_grasp_from_point`

```bash
rosservice call /g1_arm_simple/infer_grasp_from_point "arm: 'right'
x: 0.35
y: -0.05
z: 0.20"
```

### 3) `plan_grasp_from_point`

Returns the full plan (same NN + A* + Cartesian stack used by
`execute_grasp_from_point`) without executing anything. Response includes
`pre_path_flat` (start→pregrasp waypoints), `target_path_flat`
(pregrasp→grasp waypoints) and `lift_joint_angles`.

```bash
rosservice call /g1_arm_simple/plan_grasp_from_point "arm: 'right'
x: 0.35
y: -0.05
z: 0.20
pre_to_target_waypoint_count: 0
astar_override: 0"   # 0=use ~use_astar_for_grasp, 1=force on, 2=force off
```

### 4) `execute_grasp_from_point`

End-to-end pipeline, all inside `g1_arm_simple`:

1. NN-infer pregrasp/grasp joints (per-arm model + per-arm `{x,y,z}_offset`)
2. A*-plan start→pregrasp (voxel obstacle-aware, per-arm)
3. Cartesian interpolate pregrasp→grasp
4. move to pregrasp
5. call `/g1_hand_{arm}/pre_grasp`
6. move to grasp
7. call `/g1_hand_{arm}/grasp_5f`
8. pure-vertical lift by `~lift_dz` from the grasp pose

```bash
rosservice call /g1_arm_simple/execute_grasp_from_point "arm: 'right'
x: 0.35
y: -0.05
z: 0.20
wait: true
timeout: 6.0"
```

### 5) `execute_astar_path_from_point`

Same pipeline as above but `use_astar` is forced on regardless of
`~use_astar_for_grasp`.

### 6) `grasp_bottle`

Trigger that pulls the xyz from `/perception/get_object_position` using
the node's own `~grasp_bottle_label` / `~grasp_bottle_index` / `~grasp_bottle_arm`
and runs the grasp pipeline. Kept for backwards compatibility — prefer
camera-in-orchestrator + `plan_grasp_from_point` for new code.

```bash
rosservice call /g1_arm_simple/grasp_bottle "{}"
```

### 7) `GoHome`

Releases both hands (`/g1_hand_right/release` and `/g1_hand_left/release`,
best-effort — unavailable sides are skipped), then plans a dual-arm A*
path to the configured home joints and executes both arms in lock-step.

```bash
rosservice call /g1_arm_simple/GoHome "{}"
```

### 8) `acquire_arm_control` / `release_arm_control`

The node only owns the arms while it is executing a motion. At startup (after
the optional init-to-home move) and after every `release_arm_control` call the
`rt/arm_sdk` weight is 0, so the G1's own controller drives the arms (natural
arm swing while walking). Every motion service (`move_arm_joints`, `GoHome`,
`GoHomeArm`, `execute_*`, `grasp_bottle`, `pick_berry`,
`press_elevator_button`) takes the arms over automatically before moving;
`acquire_arm_control` does the same without moving. Each switch ramps the
weight over `~control_switch_delay` seconds (default 0.5 s) while the arms
hold their measured pose, then holds for `~control_switch_settle` seconds
(default 0.5 s) before the service replies, so no further command starts
until the handoff has settled. The orchestrator (`main_process`) calls
`release_arm_control` at the first safe point after an arm task or before
a walking leg, but only when both arms are back at home (holding an object does not matter);
the node itself never hands back on its own. As a last line of defence the
service measures both arms against `~{left,right}_home_joints_deg` and
refuses (`success=false`, control kept) when any joint is more than
`~release_arm_control_home_tolerance` rad away from home
(`~release_arm_control_require_home`, default true).

```bash
rosservice call /g1_arm_simple/release_arm_control "{}"
```

## External services this node consumes

- Hand triggers (`std_srvs/Trigger`) — per side:
  - `/g1_hand_right/pre_grasp`, `/g1_hand_right/grasp_5f`, `/g1_hand_right/release`
  - `/g1_hand_left/pre_grasp`,  `/g1_hand_left/grasp_5f`,  `/g1_hand_left/release`
- Camera (`g1_camera/GetObjectPosition`):
  - `/perception/get_object_position` (used only by `grasp_bottle`)

Unavailable hand services don't crash the node — the grasp pipeline fails
with a clear message; `GoHome` skips missing sides.

## Per-arm selection

The `arm` field on every grasp-related request ("left" / "right") routes
through **one** string that selects **all four** arm-specific resources
simultaneously:

| Resource | Selection |
|---|---|
| NN IK model + scaler         | `self._{arm}_model` / `self._{arm}_scaler` |
| Voxel A* planner             | `self._voxel_adapter` (per-arm bitmap inside) |
| Pose-compensation offsets    | `self._{r,l}_{x,y,z}_offset` |
| Hand Trigger services        | `/g1_hand_{arm}/*` (via `_pre_grasp_service_for(arm)` etc.) |

So a request with `arm='left'` will automatically use the left NN model,
the left voxel bitmap, the `l_{x,y,z}_offset` triple, and the left hand's services.

## Main parameters

### Motion / controller

- `~network_interface` (default: env `G1_NETWORK_INTERFACE` or `eth0`)
- `~simulation_mode` (default: `false`)
- `~motion_mode` (default: `true`)
- `~control_switch_delay` (default: `0.5`): duration of the `rt/arm_sdk` weight ramp on every SDK takeover (0->1) and hand-back (1->0).
- `~control_switch_settle` (default: `0.5`): extra hold after each ramp before the takeover / hand-back call returns and the next command may start.
- `~release_arm_control_after_init_home` (default: `true`): hand the arms back to the robot's own controller once the startup home move is done.
- `~release_arm_control_require_home` (default: `true`): `release_arm_control` refuses unless both arms measure at their home joints.
- `~release_arm_control_home_tolerance` (default: `0.15`): per-joint max abs tolerance (rad) for that home check.
- `~control_frequency` (default: `250.0`)
- `~velocity_limit` (default: `20.0`)
- `~default_timeout` (default: `5.0`)
- `~position_tolerance` (default: `0.07`)

### NN / grasp geometry

- `~left_model_path`, `~left_scaler_path`
- `~right_model_path`, `~right_scaler_path`
- `~left_hidden_dims`, `~right_hidden_dims`
- `~left_dropout`, `~right_dropout`
- `~pre_dist` (default: `0.12`)
- `~delta_rz_deg` (default: `45.0`)

### Pose compensation

Per-arm xyz offsets are subtracted from the raw target before NN-IK, so a positive offset pulls the NN's effective target back toward the body.

- `~use_pose_compensation` (default: `true`)
- `~tilt_deg` (default: `0.0`)
- `~r_x_offset` (default: `0.03`), `~l_x_offset` (default: `0.02`)
- `~r_y_offset` (default: `0.015`), `~l_y_offset` (default: `-0.03`)
- `~r_z_offset` (default: `0.0`), `~l_z_offset` (default: `0.0`)

### Lift and retract

Lift is pure-vertical from the grasp pose (xy and orientation preserved).

- `~lift_dz` (default: `0.10`): vertical lift distance after grasp_5f.
- `~lift_after_grasp` (default: `true`)
- `~retract_after_grasp` (default: `true`): solve a retraction joint target at lift z, sliding along the opposite-yaw direction until x reaches `~retract_x_target`.
- `~retract_x_target` (default: `0.15`)

### Home pose

- `~left_home_joints_deg`  (default: `[20.1, 13.0, 0.0, 49.8, 0.0]`)
- `~right_home_joints_deg` (default: `[20.1, -13.0, 0.0, 49.8, 0.0]`)
- `~init_to_home_on_start` (default: `true`)

### Hand services (per arm + legacy)

- `~pre_grasp_service_name_right` (default: `/g1_hand_right/pre_grasp`)
- `~pre_grasp_service_name_left`  (default: `/g1_hand_left/pre_grasp`)
- `~grasp_5f_service_name_right`  (default: `/g1_hand_right/grasp_5f`)
- `~grasp_5f_service_name_left`   (default: `/g1_hand_left/grasp_5f`)
- `~release_service_name_right`   (default: `/g1_hand_right/release`)
- `~release_service_name_left`    (default: `/g1_hand_left/release`)
- `~pre_grasp_service_name` / `~grasp_5f_service_name` / `~release_service_name`
  — legacy single-bus fallback. Setting any of these overrides BOTH sides.
- `~hand_service_timeout` (default: `3.0`)
- `~hand_settle_time` (default: `0.0`)
- `~grasp_5f_retries` (default: `3`), `~grasp_retry_interval` (default: `0.4`)
- `~post_grasp_hold_time` (default: `1.0`)

### `grasp_bottle` (legacy path)

- `~object_position_service_name` (default: `/perception/get_object_position`)
- `~object_position_timeout` (default: `2.0`)
- `~grasp_bottle_arm` (default: `right`)
- `~grasp_bottle_label` (default: `bottle`)
- `~grasp_bottle_index` (default: `0`)
- `~grasp_bottle_wait` (default: `true`)
- `~grasp_bottle_timeout` (default: `~default_timeout`)

### A* reachable map

- `~use_astar_for_grasp` (default: `true`)
- `~use_astar_only_to_pregrasp` (default: `true`)
- `~use_astar_for_go_home` (default: `false`)
- `~astar_start_from_current` (default: `true`)
- `~astar_simplify_path` (default: `true`)
- `~astar_pregrasp_fine_step_deg` (default: `1.0`)
- `~astar_append_exact_target_goal` (default: `true`)
- `~astar_merge_joint_thresh_rad` (default: `0.03`)

## Obstacle-aware voxel A* backend

The A* backend avoids obstacles read live from the head camera's
aligned-depth stream. It sits in `src/arm_grasp/voxel_planner_deploy/`
and is wrapped by `src/arm_grasp/voxel_planner_adapter.py`.

**Scope:** the voxel backend takes over both the grasp **pre_path**
(current pose → pregrasp) and joint-to-joint planning (e.g. `GoHome`).
The pregrasp→grasp segment is still the Cartesian horizontal
interpolation.

### One-time setup

All 22 weight files (per-arm MLPs for grasp + elevator-button, plus the
voxel A* bitmaps and sidecars under `models/abstraction_map/`) are pinned
in `config/model_weights.yaml` to a specific revision of the private HF
Hub repo `g1-intellect/g1-arm-abs-models`. The recommended provisioning
path is to pull them via the bundled downloader (requires a valid HF
token, either `hf auth login` or `HF_TOKEN` in the environment):

```bash
conda activate g1_intellect_env
rosrun arm_grasp fetch_weights.py
```

`fetch_weights.py` is idempotent (sha256-skip when the on-disk file
already matches the pinned digest) and preserves HF's nested in-repo
layout under `models/<filename>`, so re-running is a no-op verification
pass.

Alternative path when the `.uint8.gz` archives are already present
locally (offline / no HF access): the deploy directory ships with a
decompressor that produces the same `.uint8` layout from the compressed
sources:

```bash
conda activate g1_intellect_env
python3 $(rospack find arm_grasp)/src/arm_grasp/voxel_planner_deploy/decompress_once.py
```

Either way, after the provisioning step the workspace holds ~28 GB on
disk (`voxel_config_bitmap_{left,right}.uint8` are ~14 GB each). RAM
usage with memmap at runtime is ~67 MB per arm (~130 MB both). The
`models/` tree is package-`.gitignore`'d, so the raw bitmaps and the
per-arm MLPs stay out of git.

At startup the node verifies every pinned weight against the manifest
sha256 via `src/arm_grasp/weight_verify.py::verify_pinned_weights()`.
Manifest entries with `verify_sha256: false` get existence-only checks;
today those are just the two ~14 GB `voxel_config_bitmap_{left,right}.uint8`
files, because reading ~28 GB at every startup would add ~30 s of
latency. Set `~force_full_weight_sha256:=true` to override every opt-out
and verify all 22 entries. Missing manifest, missing weight, or sha256
drift aborts the node with a clear message and a hint to re-run
`fetch_weights.py`.

### How inputs reach the planner

The adapter reuses the **same two topics** that `g1_camera` consumes for
`GetObjectPosition` — no new PointCloud2 publisher is required:

- `/camera/aligned_depth_to_color/image_raw` (`sensor_msgs/Image`, 16UC1 mm)
- `/camera/color/camera_info` (`sensor_msgs/CameraInfo`)

Per planning call, the latest depth frame is deprojected through the
intrinsics K, transformed into the planner's obstacle frame using the
configured static extrinsics + `~voxel_obstacle_frame_offset`, clipped
to the planner grid bounds, voxelized (0.02 m), deduped, and handed to
`VoxelObstacleAStar.astar_staged(voxel_indices=...)`. The full staged
planner path (joint-staged + hyperrect shortcut) is then returned to
`_plan_grasp_from_target`, so `main_process` and every downstream
consumer are oblivious to the planner internals.

### Failure handling

- If any pinned weight in `config/model_weights.yaml` is missing or its
  sha256 has drifted, the startup verifier aborts the node with a hint
  to re-run `rosrun arm_grasp fetch_weights.py`.
- If no depth frame / `CameraInfo` has arrived yet when a grasp is
  requested, the planner runs with an empty obstacle set (self-collision
  only) and logs a warning.
- If the bitmaps exist and pass the verifier's existence check but a
  per-arm planner load fails (e.g. one side's `.uint8` is truncated
  after a partial copy that did not change file presence), that arm
  will raise on first plan attempt; the other arm keeps working. Enable
  `~force_full_weight_sha256:=true` to catch this kind of corruption at
  startup instead of mid-mission.

### Voxel A* backend parameters

Topic + camera wiring:
- `~voxel_depth_topic` (default: `/camera/aligned_depth_to_color/image_raw`)
- `~voxel_camera_info_topic` (default: `/camera/color/camera_info`)
- `~voxel_depth_scale` (default: `0.001`) — `float32` images: use `1.0`.
- `~voxel_min_depth` (default: `0.15`) — ignore points closer than this (m).
- `~voxel_max_depth` (default: `2.5`) — ignore points farther than this (m).
- `~voxel_subsample_stride` (default: `4`) — image-grid stride before
  deprojection. Stride 4 on an 848×480 frame yields ~25k candidate
  points; after the voxelize+dedupe step typically a few hundred voxels.

Frames / extrinsics (defaults mirror `g1_camera/config/params.yaml` so the A*
obstacle voxels land where `object_detector_node.py` places detected objects):
- `~voxel_camera_translation` (default: `[0.0576235, 0.01753, 0.42987]`) — camera origin in the robot's root frame (metres). Matches `g1_camera/config/params.yaml:camera_translation`.
- `~voxel_camera_rotation_ypr` (default: `[0.0, 0.8307767239493009, 0.0]`) — yaw, pitch, roll (rad) of `camera_link` in the robot root frame. Matches `g1_camera/config/params.yaml:camera_rotation_ypr`.
- `~voxel_obstacle_frame_offset` (default: `[0.0, 0.0, 0.0]`) — translation applied **after** the camera→root transform, to shift the cloud from root-link coords into the planner's `torso_link` frame. In this project's convention `g1_camera`'s `base_link` IS the URDF's `torso_link` (same physical frame, different label), so the offset stays zero. Set non-zero only if the camera extrinsics start being measured from a different root frame.

Planner tuning (see `voxel_planner_deploy/README.md` for the math):
- `~voxel_defer_joint` (default: `2`) — joint index held fixed during stage-1 of `astar_staged`; joint 2 is `shoulder_yaw` on the G1.
- `~voxel_defer_cost_mult` (default: `2.0`) — soft yaw penalty when the strict two-stage plan is infeasible.
- `~voxel_h_weight` (default: `1.0`) — weighted-A* heuristic multiplier; raise for faster suboptimal plans.
- `~voxel_shortcut` (default: `false`) — 5D hyperrect shortcut compression on the returned path. Off by default until the backend's `compress_path_linecheck` uses a conservative sweep check; see `launch/arm_node_abs.launch` for the failure mode.
- `~voxel_warmup_on_start` (default: `false`) — preload both arm planners at node init (adds ~1 s startup but kills the first-grasp latency spike).
- `~voxel_output_dir` (default: `""`) — override the voxel bitmaps directory. Empty = `models/abstraction_map/` under the package root.

RViz visualization:
- `~voxel_publish_obstacles` (default: `true`) — publish the voxel centers the A* is treating as occupied.
- `~voxel_obstacles_topic` (default: `/g1_arm_simple/voxel_obstacles`) — `sensor_msgs/PointCloud2`, latched, one message per plan call.
- `~voxel_obstacles_frame` (default: `torso_link`) — `frame_id` on the published cloud.

### Visualizing the obstacle cloud in RViz

Standalone, the default launch starts RViz automatically with a
preconfigured display (`config/voxel_obstacles.rviz`): Fixed Frame =
`torso_link`, PointCloud2 on `/g1_arm_simple/voxel_obstacles` as red
0.02 m boxes. No manual setup required — just roslaunch.

For headless runs (SSH with no X forwarding, CI, production), disable
RViz with:

```bash
roslaunch arm_grasp arm_node_abs.launch rviz:=false
```

To use a custom RViz layout:

```bash
roslaunch arm_grasp arm_node_abs.launch rviz_config:=/path/to/your.rviz
```

When this package is launched as a leg of `main_process.launch`, the
voxel RViz and the standalone TF stubs are OFF by default (`arm_voxel_rviz=false`,
`arm_publish_tf_stubs=false`): SLAM owns the canonical TF chain and
the orchestrator-wide visualisation is opt-in. Bring the voxel cloud
up on demand via the orchestrator-wide launcher:

```bash
roslaunch main_process visualization.launch voxel_viz:=true
# or, per-launch only:
roslaunch main_process main_process.launch arm_voxel_rviz:=true
```

The publisher itself is latched — RViz picks up the most recent cloud
even if you open it after the plan/grasp call completes.

What each failure mode looks like:
- **Boxes appear where the world is empty** → extrinsic error (check `~voxel_camera_rotation_ypr` / `~voxel_camera_translation`).
- **Boxes land on the floor but you want the floor ignored** → narrow `~voxel_max_depth` or raise the obstacle frame with `~voxel_obstacle_frame_offset`.
- **No cloud in RViz but log says `voxels=0`** → cloud was fully clipped out of the planner's workspace box; it's a frame-offset problem, not a depth problem.
- **Cloud fires only on grasp calls, not continuously** → by design. Each `latest_voxel_obstacles()` publish is triggered by a plan request.

### Camera extrinsics for the obstacle map

The depth stream is placed in `torso_link` with the SAME calibration file
DIP perception uses, `perception/config/d455_extrinsics_20261017_fitted_20261021.yaml`
(`~voxel_calibration_file`, launch arg `voxel_calibration_file`,
`voxel_use_calibration_file:=false` falls back to the old
`voxel_camera_translation` / `voxel_camera_rotation_ypr` D435i mount). Change
the file in both launches together; see `perception/README.md`
"Camera extrinsics" for how it was fitted.

### Calibrating the obstacle frame

If a grasp starts avoiding "obstacles" that don't exist (the robot's
own arms, the floor, a table that's actually safe), the most common
cause is a frame misalignment. Two knobs to inspect first:

1. **Extrinsics** — `~voxel_camera_translation` and `~voxel_camera_rotation_ypr` need to match your robot. The defaults are correct for the standard G1 head-mount; a re-mounted camera will drift.
2. **Obstacle-frame offset** — the voxel planner's workspace box is `[0, 0.6] × [-0.7, 0.7] × [-0.4, 0.4]` metres in `torso_link`. If your root frame is `base_link` and `torso_link` sits e.g. 6 cm below it, set `~voxel_obstacle_frame_offset:="[0.0, 0.0, -0.06]"`.

To verify the cloud is landing in a sane place, add
`~voxel_warmup_on_start:=true` and watch the `[A* backend=voxel]
cloud_points=... voxels=... stride=4 waypoints=N` log line on the first
grasp. `voxels=0` usually means everything was clipped out of the
workspace box — an extrinsic/offset problem, not a depth problem.

## Verifying per-arm model

At startup you should see:

```
pose compensation: tilt_deg=0.0 | right(x=+0.0300, y=+0.0150, z=+0.0000) | left (x=+0.0200, y=-0.0300, z=+0.0000)
lift: pure-vertical dz=+0.1000 m
left model path:  .../models/grasp_bottle/left/best_model.pth
right model path: .../models/grasp_bottle/right/best_model.pth
[A* backend=voxel] adapter ready: depth='...' info='...' stride=4 depth_range=[0.15, 2.5]
```

To stress-test left vs right planning, call the plan service with mirrored
targets:

```bash
rosservice call /g1_arm_simple/plan_grasp_from_point "arm: 'right'
x: 0.30
y: -0.10
z: 0.00
pre_to_target_waypoint_count: 0
astar_override: 1"   # force A* on

rosservice call /g1_arm_simple/plan_grasp_from_point "arm: 'left'
x: 0.30
y:  0.10
z:  0.00
pre_to_target_waypoint_count: 0
astar_override: 1"
```

Plans should differ — different `pre_waypoint_count`, different joint
trajectories, different log lines (`[plan] arm=...`).

## Quick sanity checklist

```bash
# services
rosservice list | grep g1_arm_simple

# camera reachable?
rosservice call /perception/get_object_position "label: 'bottle'
index: 0"

# home both arms
rosservice call /g1_arm_simple/GoHome "{}"

# grasp by legacy trigger
rosservice call /g1_arm_simple/grasp_bottle "{}"
```

## Notes

- `arm_grasp` depends on `g1_camera` only because `grasp_bottle` imports
  `GetObjectPosition.srv`. The rest of the node never talks to the camera.
- `execute_grasp_from_point` and `plan_grasp_from_point` share the same
  internal planner (`_plan_grasp_from_target`), so any parameter tweak
  (A*, workspace, clipping, waypoint count) affects both paths.
- For orchestration from voice intents, see `main_process/readme.md`.
