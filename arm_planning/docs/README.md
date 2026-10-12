# Module 2: G1 Arm Planning and ROS 2 Execution

Module 2 provides inverse kinematics, collision-aware joint-space planning, grasp-sequence generation, and optional ROS 2 trajectory execution for the five-joint left and right arms of the Unitree G1 23-DOF configuration.

The planning software has been validated offline for both arms. Physical robot deployment still requires live ROS controller integration, robot-TF validation, simulation testing, gripper integration, emergency-stop testing, and controlled low-speed validation.

---

## 1. What Module 2 Does

Module 2 converts a requested Cartesian target or detected object position into a verified, collision-aware arm trajectory.

The complete planning flow is:

```text
Cartesian target or object point
→ input and workspace validation
→ hybrid neural inverse kinematics
→ forward-kinematics verification
→ collision-aware numerical IK fallback, if required
→ static collision-mask lookup
→ nearest-free grid-goal selection
→ edge-checked A*
→ exact-goal bridge
→ dense five-joint collision validation
→ structured plan result
→ optional ROS 2 trajectory execution
```

Module 2 provides two primary Python interfaces:

```python
plan_to_pose(
    arm,
    current_joints,
    target_pose,
)
```

and:

```python
plan_grasp_from_point(
    arm,
    current_joints,
    object_xyz,
)
```

### `plan_to_pose()`

Plans an arm trajectory from the current five-joint configuration to a Cartesian target:

```text
[x, y, z, rx, rz]
```

### `plan_grasp_from_point()`

Plans a complete grasp sequence from an object position:

```text
[x, y, z]
```

It returns four separate motion segments:

```text
to_pregrasp
approach
lift
retract
```

This allows the orchestrator to insert gripper commands between the arm-motion segments.

---

## 2. Current Completion Status

| Component | Status |
|---|---|
| Forward kinematics | Complete |
| Vectorized and differentiable FK | Complete |
| Hybrid learned IK | Complete |
| Runtime FK verification | Complete |
| Left static collision mask | Complete |
| Right static collision mask | Complete |
| Collision-aware A* | Complete |
| Dense A* edge checking | Complete |
| Five-joint path validation | Complete |
| Nearest-free goal bridging | Complete |
| Collision-aware numerical IK fallback | Complete |
| `plan_to_pose()` | Complete |
| `plan_grasp_from_point()` | Complete |
| Left-arm grasp sequence | Software validation passed |
| Right-arm grasp sequence | Software validation passed |
| ROS 2 service wrapper | Implemented |
| ROS 2 trajectory execution wrapper | Implemented |
| Real-robot
