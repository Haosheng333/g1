# g1_navigation

Navigation module of the grasp-and-deliver pipeline (framework doc section 4.2 / 4.3): **one node, three
layers**, plus a localization gate, semantic goals, a motion gate and a simulator.

```
/map (tiers, ~1 Hz, g1_slam/grid_composer)     /obstacle_cloud (live layer, ~10 Hz)
              \                                      /
               v                                    v
        (1) Costmap       distance transforms + tier penalties + soft decay + 5-circle footprint   costmap.py
               |
        (2) Global planner  A* (tier penalties) -> line-of-sight pruning -> Bezier smoothing         global_planner.py
               |                                                        -> /navigation/global_path
        (3) Local planner   holonomic DWA: NAVIGATION -> APPROACH -> TERMINAL, footprint checks      local_planner.py
               |
        /navigation/cmd_vel --(cmd_vel_gate, only when motion enabled)--> /locomotion/cmd_vel
```

## Frames

`map` (z-UP, the saved map) -> `camera_init` (relocalizer, `g1_slam`) -> `pelvis` (`g1_tf_publisher`, **needs the real
`/joint_states`**). FAST-LIO2's `camera_init` is z-DOWN, so navigation never works in it. `base_link` is an optional identity
alias of `pelvis` (`with_base_link:=true` on `navigation.launch`; off by default because `slam_health_check.py` FAILs if it sees `base_link`).

## Interfaces

| Interface | Type | Notes |
| --- | --- | --- |
| `/move_base_simple/goal` | `PoseStamped` in | rviz "2D Nav Goal"; yaw is honoured |
| `/navigation/navigate` | `NavigateToObject` srv | `class_name` from `semantic.yaml`, `approach_distance`; spiral search for a standable pose, faces the object |
| `/navigation/status` | `String` out | `PLANNING / FOLLOWING / RECOVERING / SUCCESS / FAILURE`, re-published every tick, terminal state persists until the next goal |
| `/navigation/cancel` | `Empty` in | abort -> `FAILURE`, robot stops |
| `/navigation/global_path`, `/local_path`, `/costmap` | rviz | |
| `/navigation/cmd_vel` -> `/locomotion/cmd_vel` | `Twist` | `cmd_vel_gate`: disabled by default, clamps, watchdog |
| `/navigation/enable_motion` | `SetBool` srv | the only way motion starts |
| `/slam/manager/state` | `Bool` in | localized flag; goals are **suspended, never rejected** while False (3 s still, +-0.52 rad sweep at 0.4 rad/s, 60 s timeout) |

Arrival: spec **< 0.10 m and < 0.1 rad** (the controller aims at 0.08 / 0.08 to keep a margin).

## Run

```bash
roslaunch g1_navigation sim.launch                                  # simulator, no robot
python3 tools/nav_acceptance_test.py --dynamic                      # automated acceptance run

roslaunch system_bringup navigation.launch map_name:=lab joint_mode:=relay joint_input_topic:=<topic>   # robot
```

## Parameters

`config/navigation.yaml` (loaded by `launch/navigation.launch`). Launch args win over the file, the file wins over code
defaults. Check live values with `rosparam get /navigation_node`.

## Tests

```bash
cd navigation/test && python3 -m unittest test_navigation_node test_costmap_planner_dwa test_sim_room -v
```

No ROS needed (rospy / tf are stubbed). They include closed-loop acceptance scenarios around furniture. The ROS-level
checks are `tools/ab_preflight.sh`, `tools/cd_preflight.sh` (headless simulation) and `tools/ab_live_check.py`.

## Known limits

* The live layer comes from the lidar scan only; sensors on the robot's own arms are masked by `body_radius` (0.30 m).
* Python A* on a 0.05 m grid is bounded (3 s budget, weighted heuristic); very large maps may need a coarser grid.
* Velocity limits in `navigation.yaml` are conservative defaults: tune with the robot on a stand first.
