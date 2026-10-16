# Section G - stack and structure conflicts, and what it takes to run on the real robot

Review items 27-29 plus "not defined in the document". Pinned by `navigation/test/test_section_g.py` (CI) and
`navigation/test/test_locomotion_bridge.py`.

| # | Issue | Now | Where it is enforced |
|---|---|---|---|
| 27 | a second nav stack (AMCL + move_base + DWAPlannerROS) next to the one navigation node | removed; one `navigation_node` | `Item27OneStack` |
| 28 | several publishers for `odom -> base_link` | no `odom` frame; chain is `map -> camera_init -> pelvis`; `base_link` is an optional identity child of `pelvis`, published only in `system_bringup/launch/navigation.launch` behind `with_base_link` (default on since docs/FRAME_CONTRACT.md) | `Item28OneTfPublisher` |
| 29 | acceptance test not testable (constant scan, no obstacles) | `sim_world.py` + a furnished room; `tools/nav_acceptance_test.py` checks 0.10 m / 0.1 rad and ground-truth collisions | `Item29Testable`, `cd_preflight.sh` part 2 |
| - | `g1_joint_state_bridge` published all-zero `/joint_states` while logging "Live" | relay / dds modes only; zeros need `allow_fake_zeros:=true` and are in `run_fake()` alone | `NotDefinedInDocument` |
| - | package.xml / CMakeLists / srv / rviz / map | present | `NotDefinedInDocument` |

## New for the real robot: `locomotion_bridge.py`

`cmd_vel_gate` ended at `/locomotion/cmd_vel`, but nothing in the repo turned that topic into walking (only
`sim_world` consumed it). `locomotion_bridge.py` does: `/locomotion/cmd_vel` -> `unitree_sdk2py` `LocoClient.Move`.

Safety layers, any one stops the robot: navigation only publishes with an active goal; `cmd_vel_gate` is disabled
until `/navigation/enable_motion`; the bridge is **dry-run by default**, clamps again, limits acceleration, stops on a
0.4 s stale command or a non-finite value, and latches a fault after 3 failed SDK calls; the robot drops `Move()` after
about 1 s; the hand-held remote always wins. The bridge never changes the robot's mode and sends nothing while idle, so
it does not fight the remote.

Walking needs ALL of: robot in walking mode (remote), `locomotion:=true locomotion_dry_run:=false`, and
`rosservice call /navigation/enable_motion "data: true"`.

## Ownership (alignment with the other modules)

| Topic / frame | Only publisher | Others |
|---|---|---|
| `/joint_states` | `g1_joint_state_bridge` (dds or relay) | arm_control does not publish it; its `rt/lowstate` DDS channel is read-only shared |
| `map -> camera_init`, `camera_init -> pelvis` | g1_slam (`relocalizer`, `g1_tf_publisher`) | nobody else broadcasts TF |
| `/map`, `/slam/*`, `/obstacle_cloud` | g1_slam / navigation | |
| `/navigation/*` | navigation | the orchestrator only calls the service and reads the status |
| `/locomotion/cmd_vel` | `cmd_vel_gate` | `locomotion_bridge` subscribes |

`Ownership` tests fail in CI if a teammate's package starts publishing any of these.

Orchestrator contract (unchanged, pinned): service `/navigation/navigate` (`string class_name`, `float32
approach_distance` -> `bool success`, `string message`; returns once the goal is ACCEPTED), topic `/navigation/status`
(`String`: PLANNING | FOLLOWING | RECOVERING | SUCCESS | FAILURE), `/navigation/enable_motion` (`std_srvs/SetBool`).
`orchestrator/scripts/orchestrator.py` and `handover/scripts/auto_release_monitor.py` are empty files in `main` today,
so nothing calls this yet; whoever fills them in should use `navigation/scripts/nav_client.py` (`StatusWatcher`).
`arm_planning` uses `torso_link` as its base; navigation works in `map`/`pelvis`. The grasp side must take the object
pose from `map` and transform it itself.

DDS: `arm_control`, `g1_joint_state_bridge` (dds) and `locomotion_bridge` are three separate processes, each calling
`ChannelFactoryInitialize(0, <iface>)`. Use the same interface name everywhere (for example `enp2s0`).

## Real-robot bring-up order

Robot hung on its stand / feet free for steps 3-6 unless stated. Someone holds the remote with the e-stop.

1. `python3 tools/verify_loco_api.py enp2s0` - robot does not move; checks the SDK API the bridge uses.
2. `python3 tools/verify_joint_bridge.py` - dds motor-index table, waist index 12.
3. FAST-LIO2 up, then `python3 tools/check_tf_timing.py` - TF latency and rates.
4. `roslaunch system_bringup navigation.launch map_name:=<map> joint_mode:=dds network_interface:=enp2s0 locomotion:=true`
   (dry run). Send a goal; `/locomotion/bridge_state` says `dry_run`; the log shows the commands, nothing moves.
5. `python3 tools/measure_deadband.py` - robot WILL walk. Then set `dwa.*_limits` and the terminal speed from the result.
6. Dry-run again with the real map: `python3 tools/nav_acceptance_test.py` against the live stack.
7. Live: `locomotion_dry_run:=false`, enable motion, one 1 m goal in rviz, e-stop in hand. Then the acceptance test at
   the low limits.

## Still unverified (cannot be done without the robot)

* The exact `unitree_sdk2py` module path and `LocoClient` method names were written from the SDK's examples, not from
  your installed copy. Step 1 checks it; if it fails, only `DdsLocoClient` in `locomotion_bridge.py` needs changing.
* Walking dead-band and gain, FAST-LIO2 TF timing, the dds motor-index table, real mapping and localization.
