# DIP - working in the g1_intellect Docker container

DIP (this repo, upstream `Haosheng333/g1`) is built and run inside the Docker
container that belongs to the sibling project `~/Workspace/g1_intellect`.
DIP has no Docker setup of its own. Read this before touching the build or
launching anything.

## Paths and container

| Where | Host path | Container path |
|-------|-----------|----------------|
| DIP | `~/Workspace/DIP` | `/home/g1/Workspace/DIP` |
| g1_intellect | `~/Workspace/g1_intellect` | `/home/g1/Workspace/g1_intellect` |

`~/Workspace` on the host is bind-mounted at `/home/g1/Workspace`, so files
edited on the host are visible in the container immediately. The container
user is `g1`, the host user is `g1-robot`; never hard-code either home
directory. Use relative symlinks or `~`.

All container commands are run from the g1_intellect checkout via its wrapper:

```bash
cd ~/Workspace/g1_intellect
docker/g1.sh start                 # start container (once), allows X11 for rviz
docker/g1.sh enter                 # interactive shell inside
docker/g1.sh enter '<command>'     # run one command inside and return
docker/g1.sh status                # container / image state
```

Every `g1.sh enter` shell already has ROS Noetic, the g1_intellect
`catkin_ws/devel/setup.bash` and the `g1_intellect_env` conda env sourced.
The DIP workspace is NOT sourced automatically; see "Run".

## Build (catkin workspace `g1_ws`)

`g1_ws` builds `slam/` (package `g1_slam`) plus two external packages:
`livox_ros_driver2` (borrowed from g1_intellect) and `fast_lio`
(hku-mars FAST-LIO2, cloned in, gitignored). See `g1_ws/README.md`.

Procedure, in order. Steps 1 and 2 run on the host, step 3 in the container.

1. `g1_ws/src/livox_ros_driver2` must be a RELATIVE symlink so it resolves on
   both sides of the bind mount:
   ```bash
   cd ~/Workspace/DIP/g1_ws/src
   ln -sfn ../../../g1_intellect/external/livox_ros_driver2 livox_ros_driver2
   ```
   The upstream repo ships an absolute host-only link
   (`/home/g1-robot/g1_intellect/...`), which is broken inside the container.
   Keep the relative form; it is a tracked file, so it shows up in `git status`.

2. Clone and patch FAST-LIO (skip if `g1_ws/src/fast_lio` exists):
   ```bash
   cd ~/Workspace/DIP/g1_ws/src
   git clone --recursive https://github.com/hku-mars/FAST_LIO.git fast_lio
   cd fast_lio && sed -i 's/livox_ros_driver/livox_ros_driver2/g' \
       CMakeLists.txt package.xml src/laserMapping.cpp src/preprocess.cpp src/preprocess.h
   ```

3. Build inside the container. Put the conda `bin` first on PATH exactly as
   g1_intellect's `docker/setup_workspace.sh` does, otherwise catkin picks the
   wrong python/cmake:
   ```bash
   cd ~/Workspace/g1_intellect
   docker/g1.sh enter 'cd ~/Workspace/DIP/g1_ws && PATH="$CONDA_PREFIX/bin:$PATH" catkin_make -j8 -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release'
   ```
   `-DROS_EDITION=ROS1` is required by `livox_ros_driver2`. PCL deprecation
   warnings are expected; errors are not. Use `-j4` on a low-memory host.

All build dependencies (libpcl-dev, libeigen3-dev, pcl_ros, eigen_conversions,
map_server, robot_state_publisher, tf2_ros) are already in the image. Do not
apt-get anything for this workspace.

Verify:
```bash
docker/g1.sh enter 'source ~/Workspace/DIP/g1_ws/devel/setup.bash --extend && rospack find g1_slam fast_lio livox_ros_driver2'
```
Expected: all three resolve under `/home/g1/Workspace/DIP/g1_ws/src/`.

`navigation/` (package `g1_navigation`) and `system_bringup/` are built in
`g1_ws` too, through relative symlinks (create once, they are tracked):

```bash
cd ~/Workspace/DIP/g1_ws/src
ln -s ../../navigation g1_navigation
ln -s ../../system_bringup system_bringup
```

`arm_grasp/` (package `arm_grasp`, the g1_intellect `g1_arm_abs` arm node
ported into DIP, node `g1_arm_simple`) is built the same way through
`g1_ws/src/arm_grasp -> ../../arm_grasp`. Its `models/` is a tracked relative
symlink into `../../g1_intellect/g1_arm_abs/models` (weights + 28 GB voxel
bitmaps, shared, never copied). Grasp a typed `torso_link` point with
`rosrun arm_grasp grasp_point.py plan|grasp X Y Z`; hands come from
g1_intellect on PC2 (`rosrun hand_control start_remote_hands.sh`). See
`arm_grasp/README.md`.

`voice_control/` (package `g1_voice_control`) is built through the tracked
relative symlink `g1_ws/src/g1_voice_control -> ../../voice_control`. It is an
optional ROS1 frontend to the orchestrator and remains disabled by default in
`full_system.launch`. Vosk models live under `voice_control/models/` (gitignored)
and are downloaded explicitly; never add model weights to Git. Install its
Python/audio dependencies only inside the designated ROS Noetic container, not
into the host Python or another ROS environment. See `voice_control/README.md`.

`perception/` (package `perception`, node `perception_node.py`, service
`/perception/get_object_position` in `torso_link`) is built in `g1_ws` through
`g1_ws/src/perception -> ../../perception` and started by
`system_bringup/launch/manipulation.launch perception:=true`. Its default
extrinsics are the D455 ChArUco calibration in `perception/config/`, still to
be confirmed with a tape measure (see `perception/README.md`).

## Run (navigation)

Simulator, no robot (real `navigation_node`, `grid_composer`, `cmd_vel_gate`):

```bash
roslaunch g1_navigation sim.launch                      # rviz: "2D Nav Goal"
python3 ~/Workspace/DIP/tools/nav_acceptance_test.py --dynamic
```

Robot (motion stays disabled until `/navigation/enable_motion`):

```bash
roslaunch system_bringup navigation.launch map_name:=<map> joint_mode:=relay joint_input_topic:=<topic>
python3 ~/Workspace/DIP/tools/ab_live_check.py          # never moves the robot
```

Frames: `map` (z-up, saved map) -> `camera_init` (relocalizer) -> `pelvis`
(g1_tf_publisher, needs REAL `/joint_states`). `base_link` is an OPTIONAL identity alias of
`pelvis` published by `system_bringup` (`with_base_link`, ON by default; see docs/FRAME_CONTRACT.md).

## Run (the spoken task: fetch and deliver)

"go to the desk, pick up a can, go to the chair, and pass me, over" runs as
`/orchestrator/run_fetch_deliver` on standing waypoints + perception +
arm_grasp + handover_node (see `orchestrator/README.md`). Dry run by default:

```bash
roslaunch system_bringup fetch_deliver.launch map_name:=lab_map_DIP joint_mode:=relay joint_input_topic:=<topic>
rosrun perception start_remote_camera.sh                       # D455 driver on PC2, separate shell
rosrun hand_control start_remote_hands.sh                      # hands on PC2, separate shell
rosservice call /orchestrator/run_fetch_deliver "{target: table_01, destination: chair_01, label: bottle}"
# voice frontend (Vosk, English model, grammar from voice_control/config/lexicon.yaml):
roslaunch system_bringup fetch_deliver.launch ... voice_control:=true voice_dry_run_backend:=false
```

Or, one `roslaunch` per component in the background with logs and a
PASS/WAIT/FAIL checklist (inside the container):

```bash
tools/fetch_deliver_pipeline.sh start          # roscore, navigation, camera (PC2), perception, arm_grasp,
                                               # hands (PC2), handover, orchestrator (MOTION ON), voice
tools/fetch_deliver_pipeline.sh check --until-ready   # relocalization + every module; ends with
                                                      # "READY TO SPEAK" or what is still missing
tools/fetch_deliver_pipeline.sh stop
```

`side` (default left) is passed to both the orchestrator and handover_node.
Motion is ON by default in `fetch_deliver.launch` and in the script
(`ENABLE_MOTION=false` for a dry run); the voice node starts by default and
dispatches for real (`VOICE=false` / `VOICE_DRY_RUN=true` to change that). The Vosk model is
downloaded explicitly with `voice_control/scripts/download_model.sh en`
(the container has no `unzip`; the script falls back to python). The
workstation microphone is sounddevice index 4 (`ALC245 Analog`, hw:1,0).

## Run (SLAM)

One `docker/g1.sh enter` shell per ROS process. In every shell that runs a
DIP node, overlay the DIP workspace first:

```bash
source ~/Workspace/DIP/g1_ws/devel/setup.bash --extend
```

Terminal 1:
```bash
roscore
```

Terminal 2 (driver + FAST-LIO2 + robot TF):
```bash
source ~/Workspace/DIP/g1_ws/devel/setup.bash --extend
roslaunch g1_slam slam_bringup.launch rviz:=true       # rviz:=false for headless
```
Args: `robot_tf:=false` skips the TF nodes; `robot_state_publisher:=false` if
another process already publishes the robot TF; `world_tf:=false` drops the
static `world -> camera_init` (roll pi) frame that gives RViz a z-up view of
the upside-down Mid-360's `camera_init`. RViz uses `slam/rviz/slam.rviz`
(Fixed Frame `world`); override with `rviz_config:=<file>`.

Terminal 3 (checks):
```bash
source ~/Workspace/DIP/g1_ws/devel/setup.bash --extend
rosrun g1_slam slam_health_check.py --component mid360
rostopic hz /livox/lidar /Odometry
```

Map save pipeline (overlay sourced; stop the launch with Ctrl+C first so
FAST-LIO writes its PCD):
```bash
rosrun g1_slam save_map.py --name <name>
rosrun g1_slam pcd_to_map --input ~/Workspace/DIP/slam/maps/<name>.pcd --name <name>
roslaunch g1_slam map_server.launch map:=$HOME/Workspace/DIP/slam/maps/<name>.yaml
```

Full topic / TF / health-check contract: `slam/README.md`.

## Pitfalls

* **Livox host IP.** `slam/config/mid360_config.json` sets the host-side
  addresses (`cmd_data_ip`, `push_msg_ip`, `point_data_ip`, `imu_data_ip`) to
  `192.168.123.100`, which must match the workstation's robot-facing interface
  (`G1_NETWORK_INTERFACE`, default `enp196s0`). The driver binds to the
  configured address and receives nothing if the machine does not own it, so
  if the workstation IP ever changes, update those four entries and
  `EXPECTED_HOST_IP` / `EXPECTED_HOST_IFACE` in
  `slam/scripts/slam_health_check.py` together. Check with
  `ip -4 -o addr show dev enp196s0`. The LiDAR itself is `192.168.123.120`.
* **Two packages named `g1_slam`.** g1_intellect has its own `g1_slam`
  (RTAB-Map based). After sourcing the DIP overlay in a shell, `g1_slam`
  resolves to DIP's copy there. Never source the DIP overlay in a shell meant
  for the g1_intellect demo (`docker/demo_launch.sh`), and do not run both
  SLAM stacks against the same LiDAR at the same time.
* **`/joint_states`.** The `camera_init -> pelvis` TF is only published once
  something external publishes `/joint_states` carrying `waist_yaw_joint`.
  Without it, odometry and mapping still work; the `tf` health check reports
  WAIT, not FAIL.
* **Container has no systemd.** Anything that expects chrony / systemctl
  will not work inside it.
* **ROS networking.** The container uses host networking. `ROS_IP` and
  `ROS_MASTER_URI` are derived from the IPv4 on `G1_NETWORK_INTERFACE` when
  the robot cable is connected; they fall back to localhost otherwise.

## Tearing down

Ctrl+C in each terminal. If node names are still claimed by stale processes:
```bash
docker/g1.sh enter 'killall -9 roscore rosmaster roslaunch rviz fastlio_mapping livox_ros_driver2_node 2>/dev/null; true'
```
`docker/g1.sh stop` stops the whole container; the build outputs in
`g1_ws/{build,devel}` live on the host disk and survive.
