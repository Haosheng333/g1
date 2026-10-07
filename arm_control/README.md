# arm_control

Module 3 (execution): the 250 Hz arm control loop over `unitree_sdk2py`
(`rt/lowstate` / `rt/arm_sdk`), now packaged as a catkin package and exposed
as a ROS service.

## What changed here vs. the original `DIP.py`

- `src/arm_control/arm_controller.py` is the original control logic
  (`JointIndex`, `ArmController`, `Control_Loop_250Hz`, `Run_Arm_Controller`,
  `Set_Target`, `Wait_Until_Arrived`, `Execute_Waypoint_Path`), **unchanged**
  except removing the `if __name__ == "__main__":` smoke-test block.
- `scripts/arm_control_node.py` is new: starts the same background 250 Hz
  process (shared memory + pipe, same as the original `__main__` block) when
  the ROS node starts, and exposes `/arm/move_arm_joints`
  (`srv/MoveArmJoints.srv`) so other packages can drive a waypoint path
  without touching multiprocessing/DDS directly.
- `package.xml`, `CMakeLists.txt`, `setup.py` are new, so this builds as a
  normal catkin package (`rosrun`/`roslaunch` can find it) instead of being
  run as a standalone script.

No gains, joint indices, velocity limits, or timing were changed.

## Service

`/arm/move_arm_joints` (`MoveArmJoints.srv`):
```
string    arm                   # "left" or "right"
float64[] path_flat             # N x 5 joint waypoints (rad), flattened
float64   timeout_per_waypoint  # seconds; <= 0 uses the node's default (5s)
---
bool      success
string    message
```

## Build

```bash
cd ~/catkin_ws/src
pip install -r arm_control/requirements.txt
cd ~/catkin_ws
catkin build arm_control
source devel/setup.bash
rosrun arm_control arm_control_node.py
```

**Untested** -- built on a machine with no ROS, no `unitree_sdk2py`, and no
real robot. Please verify the node starts cleanly and `/arm/move_arm_joints`
drives the real arm before relying on it.
