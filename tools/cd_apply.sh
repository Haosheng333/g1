#!/usr/bin/env bash
# Run ONCE from anywhere after unzipping CD_reviewed.zip over the repo root:
#     bash tools/cd_apply.sh
# Removes the files section C/D replaces, makes scripts executable, applies the two docs patches and links
# the packages into the catkin workspace.  Safe to re-run.  Does not commit.
set -u
cd "$(dirname "$0")/.."

STALE="
navigation/launch/nav_bringup.launch
navigation/launch/g1_tf.launch
navigation/launch/offline_mapping.launch
navigation/launch/online_navigation.launch
navigation/scripts/fake_g1_sensors.py
navigation/scripts/map_manager_node.py
navigation/config/costmap_common_params.yaml
navigation/config/global_costmap_params.yaml
navigation/config/local_costmap_params.yaml
navigation/config/dwa_local_planner_params.yaml
navigation/config/nav.rviz
navigation/config/local_planner.yaml
navigation/test/test_local_planner.py
"
for f in $STALE; do
  if [ -e "$f" ]; then git rm -q -f "$f" 2>/dev/null || rm -f "$f"; echo "removed  $f"; fi
done
if [ -d navigation/maps ]; then git rm -r -q -f navigation/maps 2>/dev/null || rm -rf navigation/maps; echo "removed  navigation/maps/"; fi

chmod +x navigation/scripts/*.py slam/scripts/*.py tools/*.py tools/*.sh 2>/dev/null
echo "chmod +x done"

for p in docs/patches/docs-claude-md.patch docs/patches/docs-g1ws-readme.patch; do
  if git apply --check "$p" 2>/dev/null; then git apply "$p" && echo "applied  $p"
  elif git apply --check --reverse "$p" 2>/dev/null; then echo "already applied  $p"
  else echo "WARNING: $p does not apply cleanly - send me the error"; fi
done

mkdir -p g1_ws/src
( cd g1_ws/src && ln -sfn ../../navigation g1_navigation && ln -sfn ../../system_bringup system_bringup )
echo "linked   g1_ws/src/g1_navigation -> ../../navigation, g1_ws/src/system_bringup -> ../../system_bringup"
ls -l g1_ws/src | grep -E "g1_navigation|system_bringup|g1_slam"
