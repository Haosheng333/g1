#!/usr/bin/env bash
# Preflight for sections C and D.  Run INSIDE the container; needs NO robot.
#   docker/g1.sh enter 'bash ~/Workspace/DIP/tools/cd_preflight.sh'
# Part 1: static checks (build, packages, launch files, removed files, unit tests).
# Part 2: a HEADLESS SIMULATION of the whole navigation stack + the acceptance test (sim_world replaces the robot).
REPO=${REPO:-~/Workspace/DIP}
source "$REPO/g1_ws/devel/setup.bash" --extend 2>/dev/null
FAILED=0
ok()  { echo "[PASS] $1"; }
bad() { echo "[FAIL] $1  -- $2"; FAILED=1; }
run() { local n="$1"; shift; local o; if o=$("$@" 2>&1); then ok "$n"; else bad "$n" "$(echo "$o" | tail -3 | tr '\n' ' ')"; fi; }

echo "== part 1: static"
run "rospack finds g1_slam"        rospack find g1_slam
run "rospack finds g1_navigation" rospack find g1_navigation
run "rospack finds system_bringup" rospack find system_bringup
run "python libs (numpy scipy yaml)"                    python3 -c "import numpy, scipy, yaml"
for f in navigation/scripts/navigation_node.py navigation/scripts/obstacle_cloud_node.py navigation/scripts/sim_world.py \
         navigation/scripts/g1_joint_state_bridge.py navigation/scripts/cmd_vel_gate.py tools/nav_acceptance_test.py; do
  [ -x "$REPO/$f" ] && ok "executable: $f" || bad "executable: $f" "chmod +x $f"
done
run "launch resolves: g1_navigation navigation.launch" roslaunch --nodes g1_navigation navigation.launch map_name:=x obstacle_cloud:=false
run "launch resolves: g1_navigation sim.launch"        roslaunch --nodes g1_navigation sim.launch rviz:=false
run "launch resolves: system_bringup navigation.launch" roslaunch --nodes system_bringup navigation.launch map_name:=x joint_mode:=fake allow_fake_zeros:=true
for gone in navigation/launch/nav_bringup.launch navigation/launch/g1_tf.launch navigation/scripts/fake_g1_sensors.py \
            navigation/config/costmap_common_params.yaml navigation/config/global_costmap_params.yaml \
            navigation/config/local_costmap_params.yaml navigation/config/dwa_local_planner_params.yaml navigation/config/nav.rviz; do
  [ ! -e "$REPO/$gone" ] && ok "removed: $gone" || bad "removed: $gone" "git rm $gone"
done
if grep -rqE "pkg=.(amcl|move_base).|odom_frame_id|base_link.*laser_frame" "$REPO/navigation/launch" "$REPO/system_bringup/launch/navigation.launch" 2>/dev/null; then
  bad "no AMCL / move_base / duplicate TF publishers in navigation launches" "grep found one"; else ok "no AMCL / move_base / duplicate TF publishers in navigation launches"; fi
run "unit tests: slam"        bash -c "cd $REPO/slam && PYTHONPATH=src python3 -m unittest discover -s test -p 'test_*.py'"
run "unit tests: navigation"  bash -c "cd $REPO/navigation/test && python3 -m unittest test_navigation_node test_costmap_planner_dwa test_sim_room test_robustness test_launch_files test_slam_nav_contract test_joint_bridge_table test_section_e"

echo; echo "== part 2: headless simulation + acceptance test (takes ~5-10 min)"
pgrep -x rosmaster >/dev/null || { roscore >/tmp/cd_roscore.log 2>&1 & ROSCORE=$!; sleep 4; }
roslaunch g1_navigation sim.launch rviz:=false >/tmp/cd_sim.log 2>&1 & SIM=$!
sleep 8
run "sim stack is up (status + tf + map)" bash -c "rostopic list | grep -q /navigation/status && rostopic list | grep -q /map && rosnode list | grep -q navigation_node"
python3 "$REPO/tools/nav_acceptance_test.py" --dynamic --timeout 240; RC=$?
[ $RC -eq 0 ] && ok "acceptance test (pose goals, semantic goals, cancel, dynamic obstacle, collisions)" || bad "acceptance test" "see output above and /tmp/cd_sim.log"
kill $SIM 2>/dev/null; wait $SIM 2>/dev/null; [ -n "$ROSCORE" ] && kill $ROSCORE 2>/dev/null

echo; [ $FAILED -eq 0 ] && echo "CD PREFLIGHT: ALL PASS" || echo "CD PREFLIGHT: FAILED - fix the [FAIL] lines first"
exit $FAILED
