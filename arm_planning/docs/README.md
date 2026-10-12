# Module 2: G1 Arm Planning and ROS 2 Execution

Module 2 provides inverse kinematics, collision-aware joint-space planning, grasp-sequence generation, and optional ROS 2 trajectory execution for the five-joint left and right arms of the Unitree G1 23-DOF configuration.

The planning software has been validated offline for both arms. Physical robot deployment still requires live ROS controller integration, robot-TF validation, simulation testing, gripper integration, emergency-stop testing, and controlled low-speed validation.

## 1. What Module 2 Does

Module 2 converts a requested Cartesian target or detected object position into a verified, collision-aware arm trajectory.

```text
Cartesian target or object point
-> input and workspace validation
-> hybrid neural inverse kinematics
-> forward-kinematics verification
-> collision-aware numerical IK fallback, if required
-> static collision-mask lookup
-> nearest-free grid-goal selection
-> edge-checked A*
-> exact-goal bridge
-> dense five-joint collision validation
-> structured plan result
-> optional ROS 2 trajectory execution
```

Primary interfaces:

```python
plan_to_pose(arm, current_joints, target_pose)
plan_grasp_from_point(arm, current_joints, object_xyz)
```

`plan_to_pose()` plans to `[x, y, z, rx, rz]`. `plan_grasp_from_point()` returns `to_pregrasp`, `approach`, `lift`, and `retract` segments.

## 2. Current Completion Status

| Component | Status |
|---|---|
| Forward kinematics | Complete |
| Differentiable PyTorch FK | Complete |
| Hybrid learned IK | Complete |
| Runtime FK verification | Complete |
| Left and right collision masks | Complete |
| Edge-checked collision-aware A* | Complete |
| Five-joint path validation | Complete |
| `plan_to_pose()` | Complete |
| `plan_grasp_from_point()` | Complete in software |
| ROS 2 planning and execution wrapper | Implemented |
| Simulator and physical robot validation | Pending |

The final hybrid models use 1,000,000 training samples per arm. Held-out mean position error was 4.37 mm for the left arm and 4.17 mm for the right arm. Runtime FK verification remains mandatory.

## 3. Python Environment Setup

### Windows

```cmd
cd C:\Users\jayde\Documents\arm_planning
python -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -r requirements-dev.txt
python -m pytest -v
```

### Linux for ROS 2

```bash
export PYTHONPATH=/absolute/path/to/arm_planning/scripts:$PYTHONPATH
```

## 4. Required Dependencies

`requirements.txt`:

```text
numpy
PyYAML
torch
scipy
trimesh
python-fcl
```

`requirements-dev.txt`:

```text
-r requirements.txt
pytest
```

ROS dependencies are declared in the ROS packages and include `rclpy`, `module2_interfaces`, `sensor_msgs`, `trajectory_msgs`, `control_msgs`, and `std_srvs`.

## 5. Point-to-Pose Example

```python
from module2_planner import Module2Planner

planner = Module2Planner()
result = planner.plan_to_pose(
    arm="left",
    current_joints=[0.0, 0.0, 0.0, 0.0, 0.0],
    target_pose=[0.30, 0.15, 0.00, 0.0, 0.0],
)

if not result.success:
    raise RuntimeError(result.message)  # Execute nothing.

joint_path = result.joint_path
```

Command-line example:

```cmd
python scripts\module2_planner.py --arm left --current-joints 0 0 0 0 0 --target-pose 0.30 0.15 0.00 0.0 0.0 --output-json left_plan_result.json
```

## 6. Grasp-Planning Example

```python
from module2_grasp_planner import Module2GraspPlanner

grasp_planner = Module2GraspPlanner()
result = grasp_planner.plan_grasp_from_point(
    arm="right",
    current_joints=[0.0, 0.0, 0.0, 0.0, 0.0],
    object_xyz=[0.30, -0.15, 0.00],
)

if not result.success:
    raise RuntimeError(result.message)  # Execute no segment.

for name in ["to_pregrasp", "approach", "lift", "retract"]:
    path = result.segments[name]["joint_path"]
```

Required sequence:

```text
Execute to_pregrasp
-> open gripper
-> execute approach
-> close gripper
-> execute lift
-> execute retract
```

## 7. ROS 2 Build and Launch

Workspace layout:

```text
ros2_ws/src/module2_interfaces
ros2_ws/src/module2_ros
```

Build:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select module2_interfaces module2_ros
source install/setup.bash
export PYTHONPATH=/absolute/path/to/arm_planning/scripts:$PYTHONPATH
```

Edit `module2_ros/config/module2.yaml`, especially `project_root`, joint-state topic, controller action names, and gripper services. Keep `allow_execution: false` initially.

Launch:

```bash
ros2 launch module2_ros module2.launch.py
```

Planning-only request:

```bash
ros2 service call /module2/plan_to_pose module2_interfaces/srv/PlanToPose \
"{arm: left, current_joints: [0,0,0,0,0], target_pose: [0.30,0.15,0.00,0.0,0.0], execute: false}"
```

Grasp request:

```bash
ros2 service call /module2/plan_grasp_from_point module2_interfaces/srv/PlanGraspFromPoint \
"{arm: right, current_joints: [0,0,0,0,0], object_xyz: [0.30,-0.15,0.00], execute: false}"
```

Stop active execution:

```bash
ros2 service call /module2/stop module2_interfaces/srv/StopExecution "{}"
```

## 8. Repository Structure

```text
arm_planning/
├── README.md
├── requirements.txt
├── requirements-dev.txt
├── .gitignore
├── .gitattributes
├── docs/
│   └── Module_2_Arm_Planning_and_ROS2_Integration_Documentation.docx
├── config/
│   └── arm_kinematics.yaml
├── scripts/
│   ├── module2_planner.py
│   ├── module2_grasp_planner.py
│   ├── guarded_ik_solver.py
│   ├── collision_aware_ik.py
│   ├── collision_aware_planner.py
│   ├── edge_checked_astar.py
│   ├── joint_space_astar.py
│   ├── collision_mask.py
│   ├── g1_collision_probe.py
│   ├── g1_forward_kinematics.py
│   ├── grasp_geometry.py
│   └── torch_g1_fk.py
├── models_hybrid_1m/
│   ├── g1_left_hybrid_best.pt
│   └── g1_right_hybrid_best.pt
├── collision_masks/
├── meshes/
├── urdf/
├── tests/
└── ros2_ws/src/
    ├── module2_interfaces/
    └── module2_ros/
```

### Important file clarification

`scripts/torch_g1_fk.py` defines `TorchG1FK`, the differentiable PyTorch forward-kinematics module used by `scripts/train_ik_hybrid.py`. The production runtime planner primarily uses `scripts/g1_forward_kinematics.py` for deterministic FK verification, but `torch_g1_fk.py` should remain in the repository for training reproducibility and maintenance.

## 9. Safety Limitations

The planner checks configured joint limits, trained-workspace bounds, runtime FK error, selected arm/body collisions, configured self-collision pairs, dense A* edges, and final five-joint waypoints.

The current collision model does not yet include the table, grasped object, other workspace objects, live obstacles, people, cables, attachments, or the opposite arm moving simultaneously.

The ROS wrapper assumes a compatible `control_msgs/action/FollowJointTrajectory` controller. It does not publish Unitree low-level motor commands directly.

Keep:

```yaml
allow_execution: false
```

until controller names, joint ordering, robot or simulator TF, stop behavior, gripper services, simulation playback, and reduced-speed physical tests have been validated.

## 10. Detailed Documentation

See:

[Module 2 Arm Planning and ROS 2 Integration Documentation](docs/Module_2_Arm_Planning_and_ROS2_Integration_Documentation.docx)

## Handoff Rule

```python
if not result.success:
    handle_failure(result.message)
    return  # Do not command any arm movement.
```
