# Section F - mapping and the map manager (review items 22-26), on the real robot

Pinned by `navigation/test/test_section_f.py`, `slam/test/test_map_store.py` and `slam/test/test_slam_manager_save.py`.
RTAB-Map was replaced by FAST-LIO2 in A/B (the document allows it), so "rgbd_sync -> icp_odometry -> rtabmap" reads as
**Mid-360 -> FAST-LIO2 -> slam_manager -> pcd_to_map -> grid_composer**.

| # | Issue | Now | Enforced by |
|---|---|---|---|
| 22 | no tiered grid in the mapping pipeline | `pcd_to_map` writes the grid when the map is saved; `grid_composer` turns it into the tiered `/map` (0 / 33 / 66 / 100) in localization. No RTAB-Map anywhere; only one publisher of `/map` | `TestPipeline22` |
| 23 | saving "succeeds" even when it did not; hard-coded manifest | `save_map` builds the folder in a hidden staging dir, verifies it, then renames it into place and re-points `current`. Any failure removes the staging dir, keeps the last good `current` and returns `success: false` with the reason. The manifest holds the real point count, floor height, resolution and voxel size | `TestSaving23`, `test_map_store.py`, `test_slam_manager_save.py` |
| 24 | manager does not own the lifecycle | mapping and localization are separate sessions (FAST-LIO2 restarts, and `camera_init` has one parent, so they cannot run together). The manager checks the saved map when localization starts and keeps `/slam/manager/state` False with the reason if it is unusable | `TestLifecycle24` |
| 25 | `semantic.yaml` is a placeholder, no sofa | `tools/survey_semantic.py` records positions MEASURED on the saved map; `tools/check_saved_map.py --require table,sofa` proves each has a standable, reachable approach pose. The real coordinates come from the real map and cannot be written here | `TestSemantic25` |
| 26 | grid not produced by the pipeline | no hand-made real map in the repo; the only committed map is the labelled synthetic simulator room. Real maps are generated, git-ignored, and shared as a folder (below) | `TestNoHandMadeMaps26` |

Alignment with G: a mapping session starts no navigation, gate or locomotion node (tested), so nothing can walk the
robot while it is being mapped. The operator walks the robot with the hand-held remote.

## Mapping on the real robot (once per site)

Robot on the floor in walking mode, the remote in hand with the e-stop. FAST-LIO2 has **no loop closure**, so drift
accumulates: keep a session under about 10 minutes, walk slowly (under 0.3 m/s), overlap your path, and finish near the
start.

1. `roslaunch g1_slam mapping.launch map_name:=lab rviz:=true`. `/joint_states` is not needed to map.
2. Walk the whole area with the remote, including both doorways and the space around the table and the sofa.
3. `rosservice call /slam/manager/save_current_map "{}"`. The reply says `saved + verified` or why not.
4. `python3 tools/check_saved_map.py lab`. Every line should PASS: complete files, one connected region, a standable
   start cell. A split map means drift or a doorway that is too narrow, so map again.
5. Start the localization stack, then survey the objects:

   ```
   roslaunch system_bringup navigation.launch map_name:=lab joint_mode:=dds network_interface:=enp2s0
   # rviz: Fixed Frame = map, add the /map display, use "Publish Point"
   python3 tools/survey_semantic.py add table
   python3 tools/survey_semantic.py add sofa
   python3 tools/check_saved_map.py lab --require table,sofa
   ```

   The orchestrator's default target is `table`; the second point of the route is the `sofa`.
6. Share the map. Only `manifest.yaml` and `semantic.yaml` are tracked by git (the cloud and grid are large and
   git-ignored): commit those two, and copy the folder to the robot PC and teammates with
   `tar czf lab.tgz -C slam/maps lab` and extract it into `slam/maps/` on the other machine.

Two ways to save exist. Use `slam_manager`'s `save_current_map` for anything navigation will use. The older
`rosrun g1_slam save_map.py` copies FAST-LIO2's raw `scans.pcd` into a flat file and is only for the offline
`pcd_to_map` workflow in CLAUDE.md. `map_server.launch` also publishes a plain `/map`; use it only to look at a map,
never together with the navigation stack.

## Still unverified (needs the robot)

* A real mapping session: FAST-LIO2 drift, the floor-height estimate on your floor, and the real `pcd_to_map` run
  (`tools/ab_synthetic_pipeline_check.py` exercises the binary on a synthetic room, offline).
* The surveyed `table` and `sofa` positions themselves.
* That `g1_tf_publisher` stays quiet without `/joint_states` during mapping, as `slam_bringup.launch` documents.
