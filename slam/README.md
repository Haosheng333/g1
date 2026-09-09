# g1_slam — SLAM & map handoff for navigation

ROS 1 (Noetic) package. LiDAR‑inertial mapping on the Unitree G1 with a Livox
Mid‑360, plus the offline tooling to turn a map into something `map_server` can
serve. This document is the interface contract for the **navigation** teammate:
what g1_slam publishes, in which frames, and what is still missing before
saved‑map navigation can work.

- Package name: `g1_slam`  ·  source dir: `src/g1/slam/`
- External deps (not modified here): `livox_ros_driver2`, `fast_lio` (vanilla
  hku‑mars FAST‑LIO2), `robot_state_publisher`, `tf2_ros`, `map_server`, `nav_msgs`,
  `roslib`, `libpcl` (`common`, `io`).
- Vendored: `urdf/g1_23dof_mode_10.urdf`, an exact copy of Unitree's official
  robot description (`g1_description` is a plain directory in `unitree_ros` with no
  `package.xml`, so it cannot be a ROS dependency). Provenance and SHA‑256 in
  `urdf/SOURCE.md`. **Kinematics and TF only — no meshes, so no visualization.**
- Shared Python: `src/g1_slam/tf_chain.py` (`import g1_slam.tf_chain`), tests in
  `test/`, run with `catkin_make run_tests_g1_slam`.
- Consumed external interface: `/joint_states` carrying `waist_yaw_joint`, owned by
  the robot-control / integration team — see §4a.

---

## 1. SLAM data flow

```
 Livox Mid-360
   │  /livox/lidar  (livox_ros_driver2/CustomMsg, frame livox_frame)
   │  /livox/imu    (sensor_msgs/Imu,            frame livox_frame)
   ▼
 /laserMapping  (fast_lio, node name "laserMapping")
   │
   ├── /Odometry              nav_msgs/Odometry     header camera_init, child body
   ├── /path                  nav_msgs/Path         camera_init
   ├── /cloud_registered      sensor_msgs/PointCloud2  camera_init  (dense world scan)
   ├── /cloud_registered_body sensor_msgs/PointCloud2  body         (per-scan, sensor frame)
   ├── /Laser_map             sensor_msgs/PointCloud2  camera_init
   ├── /cloud_effected        sensor_msgs/PointCloud2  camera_init
   └── TF  camera_init ─▶ body   (dynamic, broadcast every odometry cycle)

 /joint_states  (sensor_msgs/JointState)  — EXTERNAL, owned by robot-control
   │  must carry waist_yaw_joint; g1_slam never synthesizes it
   ▼
 /g1_tf_publisher  (g1_slam)  +  /robot_state_publisher  (urdf/g1_23dof_mode_10.urdf)
   │
   ├── TF  camera_init ─▶ pelvis                        (dynamic, every odometry cycle)
   └── TF  pelvis ─▶ torso_link ─▶ mid360_link, …   (robot_state_publisher)
        — publishes NOTHING until /joint_states arrives, see §4

 On clean shutdown, with pcd_save.pcd_save_en: true (config/fastlio.yaml):
   <fast_lio pkg>/PCD/scans.pcd     binary PCD, PointXYZINormal, frame camera_init
        │  scripts/save_map.py --name <name>
        ▼
   slam/maps/<name>.pcd
        │  pcd_to_map --input slam/maps/<name>.pcd --name <name>
        ▼
   slam/maps/<name>.pgm  +  slam/maps/<name>.yaml
        │  launch/map_server.launch  map:=slam/maps/<name>.yaml
        ▼
   /map, /map_metadata   nav_msgs/OccupancyGrid + MapMetaData   (latched, frame camera_init)
```

### Launch commands

| Command | Starts | Use |
| --- | --- | --- |
| `roslaunch g1_slam sensors.launch` | Livox Mid‑360 driver only (`/livox_lidar_publisher2`) | driver bring‑up / diagnostics |
| `roslaunch g1_slam fastlio_mapping.launch` | FAST‑LIO2 only (`/laserMapping`) | needs `sensors.launch` already running |
| `roslaunch g1_slam slam_bringup.launch` | driver **+** FAST‑LIO2 **+** `g1_tf.launch` | **normal live mapping** |
| `roslaunch g1_slam g1_tf.launch` | `/robot_state_publisher` **+** `/g1_tf_publisher` | robot TF tree — dynamic `camera_init → pelvis` (needs `/joint_states`, §4) |
| `roslaunch g1_slam map_server.launch map:=<yaml>` | `/map_server` only — **no** SLAM, **no** driver | replay a previously saved map |

`slam_bringup.launch` args: `rviz:=true` (default `false`, headless),
`robot_tf:=false` (default `true`, skip `g1_tf.launch`),
`robot_state_publisher:=false` (default `true`; set false if integration already
runs one — two publishers would fight over `/tf`),
`urdf_file:=<path>` (default the vendored `urdf/g1_23dof_mode_10.urdf`).

---

## 2. Published topics

All topics below are what a downstream consumer can rely on. Rates are
approximate and follow the LiDAR frame rate (~10 Hz for the Mid‑360 default).

### Livox driver — `/livox_lidar_publisher2` (from `sensors.launch`)

| Topic | Type | Frame |
| --- | --- | --- |
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` | `livox_frame` |
| `/livox/imu` | `sensor_msgs/Imu` | `livox_frame` |

### FAST‑LIO2 — `/laserMapping` (from `fastlio_mapping.launch`)

| Topic | Type | Frame | Meaning |
| --- | --- | --- | --- |
| `/Odometry` | `nav_msgs/Odometry` | header `camera_init`, child `body` | pose of the Mid‑360 **IMU** (`body`) in `camera_init`; has covariance |
| `/path` | `nav_msgs/Path` | `camera_init` | accumulated trajectory (decimated) |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | `camera_init` | current scan, transformed into the world frame |
| `/cloud_registered_body` | `sensor_msgs/PointCloud2` | `body` | current scan in the IMU/sensor frame |
| `/Laser_map` | `sensor_msgs/PointCloud2` | `camera_init` | map cloud publisher (usually empty in this config) |
| `/cloud_effected` | `sensor_msgs/PointCloud2` | `camera_init` | points used in the last update |
| `/tf` (`camera_init` → `body`) | `tf2_msgs/TFMessage` | — | dynamic transform, every odometry cycle |

### map_server — `/map_server` (from `map_server.launch`, only when run)

| Topic / service | Type | Frame |
| --- | --- | --- |
| `/map` | `nav_msgs/OccupancyGrid` | `camera_init` (launch arg `frame_id`, default `camera_init`) — latched |
| `/map_metadata` | `nav_msgs/MapMetaData` | `camera_init` — latched |
| `/static_map` | `nav_msgs/GetMap` (service) | — |

### g1_slam robot TF — `/g1_tf_publisher` + `/robot_state_publisher` (from `g1_tf.launch`)

| Topic | Type | Status |
| --- | --- | --- |
| `/tf` (`camera_init` → `pelvis`) | `tf2_msgs/TFMessage` | dynamic, per odometry cycle — **only while `/joint_states` carries `waist_yaw_joint`** (§4) |
| `/tf` (`pelvis` → `torso_link` → `mid360_link`, …) | `tf2_msgs/TFMessage` | from `robot_state_publisher` and the vendored URDF |
| `robot_description` (parameter, not a topic) | `string` | the vendored URDF — **kinematics/TF only, no meshes**; see `urdf/SOURCE.md` |

**Consumed, not provided:** `/joint_states` (`sensor_msgs/JointState`) carrying
`waist_yaw_joint`. That topic is owned by the robot-control / integration team;
g1_slam subscribes to it and never synthesizes it.

g1_slam does **not** publish `/cmd_vel`, costmaps, plans, goals, or `base_link` —
those belong to the navigation package. **`base_link` is deliberately absent**, see §4.

---

## 3. TF tree

```
camera_init                                  FAST-LIO2 odometry origin (world)
├── body                                     /laserMapping        dynamic  /tf
└── pelvis                                   /g1_tf_publisher     dynamic  /tf
    ├── pelvis_contour_link, imu_in_pelvis   robot_state_publisher  fixed
    ├── left|right_hip_pitch_link → … → ankles    rsp, revolute
    └── torso_link                           rsp, waist_yaw_joint (REVOLUTE)
        ├── mid360_link                      rsp, fixed
        ├── head_link / logo_link / d435_link / imu_in_torso    rsp, fixed
        └── left|right_shoulder_* → … → wrists                  rsp, revolute

base_link                                    NOT PUBLISHED — unresolved (§4)
```

`camera_init` has two children and every other frame has exactly one parent, so
the tree stays valid.

| Frame | Meaning | Source |
| --- | --- | --- |
| `camera_init` | FAST‑LIO2 odometry origin. Created at the IMU pose at the **first LiDAR frame** of the session (identity initial state). Fixed for the lifetime of that `laserMapping` process. | FAST‑LIO2 |
| `body` | FAST‑LIO2's tracked pose = the Mid‑360 **IMU** frame. Moves with the robot. | FAST‑LIO2 (`camera_init → body`) |
| `pelvis` | The official URDF root, and the frame g1_slam publishes. | `/g1_tf_publisher` (`camera_init → pelvis`) |
| `torso_link` | G1 torso; carries the Mid‑360. Child of `pelvis` through **revolute** `waist_yaw_joint`. | `robot_state_publisher` |
| `mid360_link` | The Mid‑360 **LiDAR** frame, per the URDF. Fixed to `torso_link`; the `rpy` roll of π is the **upside‑down mount**. | `robot_state_publisher` |
| `base_link` | **Not published.** An unresolved integration decision — see §4. | — |

### The chain, and the one transform deliberately not broadcast

```
camera_init → pelvis = (camera_init → body)        from /Odometry
                     · (body → mid360_link)        FAST-LIO2 extrinsic, const
                     · (mid360_link → pelvis)      URDF chain at waist_yaw

body → mid360_link        xyz [-0.011, -0.02329, 0.04412], rotation identity
mid360_link → body        xyz [+0.011, +0.02329, -0.04412]   (the inverse)
pelvis → torso_link       xyz [-0.0039635, 0, 0.044],  rpy(0, 0, waist_yaw)
torso_link → mid360_link  xyz [0.0002835, 0.00003, 0.428434], rpy(π, 0.05112069379091391, 0)
```

`body → mid360_link` is FAST‑LIO2's `mapping/extrinsic_T` — the LiDAR pose
expressed in the IMU frame — and is **never broadcast as TF**: `mid360_link`
already has `torso_link` as its parent from `robot_state_publisher`, and `body`
already has `camera_init` from FAST‑LIO2, so broadcasting it would give a frame
two parents and break TF. It is a math constant in `src/g1_slam/tf_chain.py`, and
its direction is pinned by `test/test_tf_transforms.py`.

`extrinsic_R` is identity, **and stays identity despite the upside‑down mount** —
it describes the LiDAR‑to‑IMU relationship *inside* the sensor housing, which does
not change with how the housing is bolted on. The 180° roll of the inverted mount
lives in the URDF's `mid360_joint`.

`body` and `mid360_link` describe the same physical rigid body reached by two
branches; they agree by construction because `pelvis` is *derived* from `body`.

The vendored URDF is `urdf/g1_23dof_mode_10.urdf` — an exact copy of Unitree's
official description, provenance and SHA‑256 recorded in `urdf/SOURCE.md`. It is
used for **kinematics and TF only**: its mesh references are relative and the
meshes are not vendored, so RViz renders no robot geometry.


---

## 4. What is still missing: `/joint_states` and `base_link`

Two things block a complete `camera_init → … → robot` chain. Neither is g1_slam's
to fix, and neither is faked here.

### 4a. `/joint_states` carrying `waist_yaw_joint` — EXTERNAL

**Owner: the robot-control / integration team.** g1_slam consumes this topic and
never synthesizes it.

The Mid‑360 is bolted to `torso_link`, and `torso_link` hangs off `pelvis` through
`waist_yaw_joint`, which is **revolute**. The sensor‑to‑robot transform is
therefore *not* static — an earlier version of this package published a static
`body → base_link`, and that was simply wrong.

While `/joint_states` is missing, absent `waist_yaw_joint`, or staler than
`joint_states_timeout` (default 0.5 s), `/g1_tf_publisher` stays up, logs a
throttled warning, and **publishes nothing**. It does not fall back to a frozen
waist, because measured on the real URDF chain the cost of that fallback is
almost pure heading error:

| True `waist_yaw` | `pelvis` position error | `pelvis` heading error |
| --- | --- | --- |
| 5° | 0.03 cm | **5.0°** |
| 15° | 0.10 cm | **15.0°** |
| 30° | 0.21 cm | **30.0°** |

Roughly one degree of yaw per degree of waist rotation. A silent default would
hand navigation a confidently wrong heading — worse than no transform at all.

Required interface:

| | |
| --- | --- |
| Topic | `/joint_states` (configurable: `joint_states_topic`) |
| Type | `sensor_msgs/JointState` |
| Must contain | `waist_yaw_joint` in `name[]`, with the matching entry in `position[]` (radians) |
| Freshness | within `joint_states_timeout` of the `/Odometry` stamp |

`slam_health_check.py --component tf` reports **WAIT** while it is absent, never
FAIL — its absence is a pending integration step, not a defect.

### 4b. `pelvis → base_link` — UNRESOLVED with the navigation team

`pelvis` is the official URDF root and the frame this package publishes. Whether
navigation wants `base_link` as an alias of `pelvis`, as a ground‑projected frame
beneath it, or under a different name entirely is **their decision, and it has not
been made.**

Nothing here invents it. `config/g1_tf.yaml` carries `base_link_alias: ""` purely
as a documented placeholder; `""` is the only supported value, and the node
**rejects a non-empty value outright** rather than publishing a guessed transform.
`test/test_tf_transforms.py` asserts `base_link` appears nowhere in the URDF, and
the health check FAILs if it ever shows up in the TF tree.

This decision also affects `perception/`, which already documents its results as
being expressed in `base_link` (`perception/README.md`), so that team should be in
the conversation.

Until 4a and 4b are both settled, **navigation cannot place the robot in a map.**


---

## 5. Offline map pipeline

All outputs land in `slam/maps/` and are git‑ignored (`*.pcd`, `*.pgm`,
`*.yaml`). None of these steps overwrite an existing file.

### 5a. Save the accumulated PCD

Requires a completed FAST‑LIO2 run with `pcd_save.pcd_save_en: true` (already
set in `config/fastlio.yaml`) that was **shut down cleanly** (Ctrl‑C), so
FAST‑LIO2 flushes `<fast_lio pkg>/PCD/scans.pcd`.

```bash
rosrun g1_slam save_map.py --name lab_room
#   -> copies <fast_lio>/PCD/scans.pcd  ->  slam/maps/lab_room.pcd
#   --source <path>     override the PCD source
#   --maps-dir <dir>    override the destination
#   --dry-run           validate name/paths, copy nothing
```

### 5b. Convert PCD → 2D occupancy grid

```bash
rosrun g1_slam pcd_to_map \
  --input  $(rospack find g1_slam)/maps/lab_room.pcd \
  --name   lab_room
#   -> slam/maps/lab_room.pgm  (8-bit binary P5)
#      slam/maps/lab_room.yaml (image, resolution, origin, negate, thresholds)
```

Reads ASCII, binary and `binary_compressed` PCD (FAST‑LIO2 writes uncompressed
binary). Classification is conservative:

- **occupied** – ≥ `--occupied-threshold` points in the slab `[--z-min, --z-max]`
- **free** – not occupied **and** ≥ `--free-threshold` points in the ground band
  `[--ground-z-min, --z-min)`
- **unknown** – no evidence either way (never guessed)

| Option | Default | |
| --- | --- | --- |
| `--output-dir DIR` | `<g1_slam>/maps` | must exist and be writable |
| `--resolution M` | `0.05` | metres per pixel |
| `--z-min` / `--z-max` | `-0.5` / `2.0` | obstacle height slab (metres, in `camera_init`) |
| `--ground-z-min M` | `z-min − 1.0` | bottom of the ground band |
| `--padding M` | `1.0` | free border around the cloud |
| `--occupied-threshold N` / `--free-threshold N` | `1` / `1` | min points per cell |
| `--negate` / `--occupied-thresh` / `--free-thresh` | `0` / `0.65` / `0.196` | written straight into the YAML for `map_server` |
| `--max-megapixels F` | `25` | refuses to allocate a larger grid |

Exit codes: `0` ok · `1` usage · `2` validation (bad name/path/param) · `3`
refuse‑overwrite · `4` PCD load failure · `5` degenerate (empty / nothing
classified / grid too large) · `6` I/O.

Heights are in the `camera_init` frame (≈ the IMU height at session start), so
tune `--z-min/--z-max` to the actual floor and ceiling of that run.

### 5c. Serve the map

```bash
roslaunch g1_slam map_server.launch \
  map:=$(rospack find g1_slam)/maps/lab_room.yaml
#   map:=<path>          REQUIRED (roslaunch errors if omitted)
#   frame_id:=<frame>    default camera_init  (see §7 before changing)
```

Publishes latched `/map` + `/map_metadata` and the `/static_map` service. Does
**not** start FAST‑LIO2 or the driver.

---

## 6. Same‑session navigation interface

"Same‑session" = navigation runs **inside the same `laserMapping` process** that
is doing the mapping. This is the only configuration g1_slam currently supports
end‑to‑end.

- **Navigation frame:** `camera_init`. For a single FAST‑LIO2 run it is stable
  and drift‑bounded (LIO is locally consistent; there is no loop closure), so
  treat `map ≡ odom ≡ camera_init`. No AMCL, no separate localization node.
- **Robot pose:** TF `camera_init → pelvis` (available once §4a is done), or
  `/Odometry` (`camera_init → body`) composed with `body → pelvis` from
  `src/g1_slam/tf_chain.py`. `base_link` is not published (§4b).
- **Odometry topic:** `/Odometry` (`nav_msgs/Odometry`). Its `child_frame_id` is
  `body`, not `pelvis`.
- **Obstacle input for costmaps:** `/cloud_registered` (world, `camera_init`) or
  `/cloud_registered_body` (sensor, `body`). There is **no `sensor_msgs/LaserScan`** —
  a consumer needs a 3D costmap layer or its own `pointcloud_to_laserscan`.
- **2D map (optional):** generate it from **this session's** run via §5, then
  `map_server.launch` with `frame_id:=camera_init`.
- **Suggested `move_base` wiring:** `global_frame: camera_init`,
  `robot_base_frame: pelvis`, `odom_topic: /Odometry` — revisit `robot_base_frame`
  once §4b is decided.

Not provided by g1_slam and **not currently installed** in the workspace:
`move_base`, `costmap_2d`, global/local planners, `pointcloud_to_laserscan`,
`robot_localization`, `amcl`. All are available as `ros-noetic-*` packages.

---

## 7. ⚠️ Saved‑map navigation still needs `map → camera_init` relocalization

**Do not point `map_server.launch` at an old map and expect it to line up.**

- `camera_init` is re‑created at the robot's **power‑on pose every time**
  `laserMapping` starts. A map saved in session 1 is expressed in session 1's
  `camera_init` origin.
- On restart, the live `camera_init` is a **different origin with the same
  name**. `map_server.launch`'s `frame_id: camera_init` default is correct
  **only within the session that generated the map**.
- Nothing in this repo publishes a corrective `map → camera_init` (or
  `map → odom`) transform. Vanilla FAST‑LIO2 has **no relocalization and no
  prior‑map loading**.
- Result across sessions: either the map is silently pinned to the wrong origin
  (if the map frame stays `camera_init`) or `map → base_link` cannot be looked
  up at all (if the map frame is renamed to `map`). Navigation cannot localize.

**What saved‑map navigation requires (not yet implemented):** a relocalization
node that matches the live `/cloud_registered` against the saved
`slam/maps/<name>.pcd` (3D ICP / NDT) and broadcasts TF `map → camera_init`,
giving `map → camera_init → pelvis`.

**AMCL is not recommended here:** the Mid‑360 produces a sparse, non‑repetitive
3D scan and there is no 2D laser; a synthesized `LaserScan` is a poor AMCL
input. Use 3D relocalization. AMCL is only a fallback if the deployment commits
to a flat, wall‑rich indoor space with a `pointcloud_to_laserscan` slice.

---

## 8. Health checks

```bash
rosrun g1_slam slam_health_check.py --component <name> [--timeout N] [--json] [--require-hardware]
```

| `--component` | Scope | Needs a ROS master / hardware? |
| --- | --- | --- |
| `mid360` | Livox driver package, `mid360_config.json`, `sensors.launch`, then network + live topics | live checks WAIT without hardware |
| `fastlio` | `fast_lio` package, `fastlio.yaml` extrinsics, `fastlio_mapping.launch`, then live topics | live checks WAIT without a running node |
| `tf` | vendored‑URDF SHA‑256, `SOURCE.md` provenance, `tf_chain` direction + URDF agreement, `g1_tf.yaml`, `g1_tf_publisher.py --check`, `robot_state_publisher` presence, `g1_tf.launch` / `slam_bringup.launch` resolution, FAST‑LIO2 broadcaster parse, whole‑tree single‑parent guard, then live `/joint_states` | live `/joint_states` check WAITs without it |
| `mapsave` | `maps/` dir, `save_map.py`, `fast_lio` package, whether an accumulated PCD exists | PCD‑present check WAITs until a run completes |
| `pcdmap` | **offline** — `maps/` dir, `.gitignore` patterns, `pcd_to_map --help`, a bad‑input battery, synthetic ASCII+binary conversion round‑trip | no |
| `mapserver` | **offline** — `map_server` package, `map_server.launch` XML / required‑arg / node resolution, `frame_id` default | no |

Common flags: `--timeout N` (seconds for network/topic waits, default 5),
`--json` (machine‑readable), `--require-hardware` (see below).

### PASS / WAIT / FAIL

| Status | Exit | Meaning |
| --- | --- | --- |
| **PASS** | `0` | The check is satisfied. |
| **WAIT** | `2` | Cannot be verified **yet** — hardware not connected, no ROS master / node not running, an un-provisioned package, or a pending cross‑team interface or decision (e.g. `/joint_states`, `pelvis → base_link`). Expected in offline / CI runs. With `--require-hardware` these become **FAIL** so the check can gate an on‑robot bring‑up. |
| **FAIL** | `1` | A real defect — missing file, malformed config, launch that will not resolve, wrong frame wiring, a tool that accepted bad input, etc. Software/structure checks always FAIL on error regardless of `--require-hardware`. |

The **overall** result (and process exit code) is the worst status across all
checks: `FAIL` > `WAIT` > `PASS`.

Typical offline run today: `tf`, `mapsave`, `mid360`, `fastlio` report
**WAIT overall** (hardware / measurement pending); `pcdmap` and `mapserver`
report **PASS overall**.
