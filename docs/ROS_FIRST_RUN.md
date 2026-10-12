# First run on ROS + the robot (what could not be verified offline)

Run in the Noetic container, workspace `g1_ws`, repo root `~/Workspace/DIP` (adjust paths).

## 1. Build and launch resolution
```bash
cd ~/Workspace/DIP && bash tools/cd_apply.sh          # once, after unzipping CD_reviewed.zip
cd g1_ws && catkin_make && source devel/setup.bash --extend
cd .. && bash tools/cd_preflight.sh                   # part 1 static, part 2 headless sim + acceptance (5-10 min)
```
Expect all `[PASS]`. Anything `[FAIL]` -> send the line and `/tmp/cd_sim.log`.

## 2. Motor-index table (robot on the network, damping mode, nobody commanding it)
```bash
python3 tools/verify_joint_bridge.py enp2s0
```
Must print `TABLE CONFIRMED`. Otherwise run the bridge with `joint_mode:=relay` and fix `G1_MOTOR_INDEX`.

## 3. Live TF timing (real FAST-LIO2 + relocalizer + bridge running, robot localized)
```bash
roslaunch system_bringup navigation.launch map_name:=lab joint_mode:=dds network_interface:=enp2s0
python3 tools/check_tf_timing.py --seconds 20
python3 slam/scripts/slam_health_check.py --component tf
```
Key line: `|Odometry stamp - joint_states stamp| < timeout`. If it fails with a large constant skew, FAST-LIO2/Livox is
stamping with the LiDAR clock; set the Livox driver to ROS time (or raise `joint_states_timeout` in `g1_tf.yaml`
only after understanding why).

## 4. Walking dead-band (robot standing in a clear 2 m x 2 m, e-stop in hand, navigation stopped)
```bash
python3 tools/measure_deadband.py --yes
```
Targets: dead-band <= 0.03 m/s and gain >= 0.8 (the simulated tolerance). If the measured dead-band exceeds
`dwa.terminal_limits` (0.06) in `navigation/config/navigation.yaml`, raise `terminal_limits`/`approach_limits`.

## 5. First closed-loop motion
`rosservice call /navigation/enable_motion "data: true"`, then a short goal 1 m ahead in rviz. Keep the e-stop.
