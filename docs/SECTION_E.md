# Section E (items 17-21): local planner, footprint, arrival

Implemented inside the C/D navigation stack, so E and the rest share ONE controller, ONE costmap and ONE
parameter file (`navigation/config/navigation.yaml`, block `dwa:`).  The standalone `HolonomicDWA` /
`local_planner.yaml` of the first E delivery are gone: `cd_apply.sh` removes them.

| # | Requirement | Code | Test (`navigation/test/test_section_e.py`) |
|---|---|---|---|
| 17 | Holonomic DWA, 3 phases NAVIGATION -> APPROACH -> TERMINAL, vx/vy/wz | `scripts/local_planner.py` `DwaController` (phase switch at `approach_dist` 0.8 m / `terminal_dist` 0.30 m, hysteresis 1.3x / 1.5x) | `Item17Holonomic3Phase` (phase order, per-phase limits, strafing with vy, translate-then-rotate, acceleration limits) |
| 18 | Footprint collision checks | `scripts/costmap.py`: 5 circles r = 0.10 m, centres y in [-0.2, 0.2] (0.60 m body); bilinear distance + half-cell surface offset; used for A* standable test AND every DWA trajectory sample | `Item18Footprint`; `test_robustness.py` checks zero ground-truth collisions in random rooms with noise and dead-band |
| 19 | Arrival < 0.10 m AND < 0.1 rad | `local_planner.SPEC_XY_TOL/SPEC_YAW_TOL`; `navigation_node._follow` reports SUCCESS after the pose stays inside the spec for `spec_settle_time` (2 s) when the controller cannot get tighter | `Item19Arrival`; `test_navigation_node.test_spec_tolerance_is_accepted_when_the_robot_cannot_get_tighter` |
| 20 | Tolerances | `dwa.xy_tol` / `dwa.yaw_tol` 0.08 (tighter than the spec), `spec_settle_time` 2.0, all read from `navigation.yaml` | `Item20Tolerances` (tighter than spec; yaml values reach the controller) |
| 21 | Speed limits | `dwa.nav_limits` 0.30/0.15/0.60, `approach_limits` 0.15/0.10/0.40, `terminal_limits` 0.06/0.06/0.30; `cmd_vel_gate` clamps at 0.5/0.3/1.0 | `Item21Limits` (phase order, inside the gate) |

Closed-loop evidence (40 random furnished rooms, lag 0.25 s): ideal 40/40 within spec; 1 cm localisation noise 40/40;
0.03 dead-band 40/40; 2 cm noise + gain 0.8 38/40; 0 collisions, 0 stuck in every case.

Still to measure on the G1: walking dead-band and gain (`tools/measure_deadband.py`).  If the dead-band is
above `terminal_limits` (0.06 m/s) the last 0.3 m cannot be completed - raise `terminal_limits`.
