#!/usr/bin/env bash
# Robot-free preflight for sections A and B. Run INSIDE the container:
#   docker/g1.sh enter 'bash ~/Workspace/DIP/tools/ab_preflight.sh'
# Prints PASS/FAIL per item; exit code 1 if anything failed.
REPO=${REPO:-~/Workspace/DIP}
source "$REPO/g1_ws/devel/setup.bash" --extend 2>/dev/null
FAILED=0
ok()   { echo "[PASS] $1"; }
bad()  { echo "[FAIL] $1  -- $2"; FAILED=1; }
run()  { local name="$1"; shift; local out; if out=$("$@" 2>&1); then ok "$name"; else bad "$name" "$(echo "$out" | tail -3 | tr '\n' ' ')"; fi; }

run "python libs (numpy scipy yaml rospkg)" python3 -c "import numpy, scipy, yaml, rospkg"
run "rospack finds g1_slam"       rospack find g1_slam
run "rospack finds g1_navigation" rospack find g1_navigation
run "rossrv NavigateToObject generated"       rossrv show g1_navigation/NavigateToObject
run "g1_slam python package importable"       python3 -c "from g1_slam import cloud_utils, grid_utils"
run "pcd_to_map binary runs"                  rosrun g1_slam pcd_to_map --help

for f in slam/scripts/slam_manager.py slam/scripts/relocalizer.py slam/scripts/grid_composer.py \
         navigation/scripts/navigation_node.py navigation/scripts/cmd_vel_gate.py \
         navigation/scripts/send_goal_node.py navigation/scripts/send_waypoints_node.py; do
  [ -x "$REPO/$f" ] && ok "executable: $f" || bad "executable: $f" "run chmod +x $f"
done

run "launch resolves: g1_slam mapping.launch"      roslaunch --nodes g1_slam mapping.launch
run "launch resolves: g1_slam localization.launch" roslaunch --nodes g1_slam localization.launch map_name:=x
run "launch resolves: g1_navigation navigation.launch" roslaunch --nodes g1_navigation navigation.launch map_name:=x

for old in rtabmap amcl move_base map_manager; do
  if grep -rqi "pkg=\"$old\|type=\"$old\|name=\"$old" "$REPO/slam/launch" "$REPO/navigation/launch/navigation.launch" 2>/dev/null; then
    bad "no $old in the new launch files" "found"; else ok "no $old in the new launch files"; fi
done
for gone in navigation/scripts/map_manager_node.py navigation/launch/offline_mapping.launch navigation/launch/online_navigation.launch; do
  [ ! -e "$REPO/$gone" ] && ok "removed: $gone" || bad "removed: $gone" "still exists: git rm $gone"
done

run "unit tests: slam"        bash -c "cd $REPO/slam && PYTHONPATH=src python3 -m unittest discover -s test -p 'test_cloud_utils.py'"
run "unit tests: navigation"  bash -c "cd $REPO/navigation/test && python3 -m unittest test_navigation_node"
run "synthetic map pipeline with the REAL pcd_to_map" python3 "$REPO/tools/ab_synthetic_pipeline_check.py"

echo; [ $FAILED -eq 0 ] && echo "PREFLIGHT: ALL PASS - safe to try the live steps" || echo "PREFLIGHT: FAILED - fix the [FAIL] lines first"
exit $FAILED
