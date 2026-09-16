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

`navigation/` and `perception/` are also catkin packages but are not part of
`g1_ws` and are not referenced by any launch file. Leave them unbuilt unless
asked.

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
