# arm_planning

Module 2: collision-aware arm planning for the Unitree G1 23-DOF
configuration. The planning algorithms live in `scripts/` and are documented in
`docs/README.md` and
`docs/Module_2_Arm_Planning_and_ROS2_Integration_Documentation.docx` (referred
to below as "doc <section>").

This directory is now a standard ROS 1 (catkin) package that wraps those
algorithms in two planning services. **The specification's wrapper is ROS 2
(`module2_interfaces` / `module2_ros`, doc section 9); this is the ROS 1
Noetic equivalent for this repository**, which is ROS 1 throughout.

## Interface (for other modules)

Two services. Both are **planning only** (doc 2.1, 12.1) — nothing in this
package commands the robot. Execution is `arm_control`'s
`/arm/move_arm_joints`, called separately by the orchestrator after its own
enablement gate.

| Service | Type |
|---|---|
| `/arm/plan_to_pose` | `arm_planning/PlanToPose` |
| `/arm/plan_grasp_from_point` | `arm_planning/PlanGraspFromPoint` |

Plus a readiness pair for preflight, mirroring `g1_slam`'s
`/slam/manager/{state,status}`:

| Topic | Type | Meaning |
|---|---|---|
| `/arm/planner/ready` | `std_msgs/Bool` (latched) | true = models, masks, meshes and deps all loaded |
| `/arm/planner/status` | `std_msgs/String` (latched) | `READY: ...` or `NOT_READY: <blockers>` |

### Joint order

Every `current_joints` input and every returned path is, per doc 2.3:

```
shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll
```

This is the same order `arm_control` writes to the SDK
(`arm_controller.py`, `JointIndex.*ShoulderPitch/Roll/Yaw/Elbow/WristRoll`), and
`test/test_plan_conversion.py` pins both ends so a transposition cannot pass
review silently.

### Reconstructing paths

Responses carry flat row-major paths plus explicit counts (doc 11.4):

```python
path = np.asarray(resp.flattened_joint_path).reshape(resp.waypoint_count, 5)

offset, segments = 0, {}
for name, count in zip(resp.segment_names, resp.segment_waypoint_counts):
    length = count * 5
    segments[name] = np.asarray(
        resp.flattened_segment_paths[offset:offset + length]
    ).reshape(count, 5)
    offset += length
```

`segment_names` is always `to_pregrasp, approach, lift, retract` in that order.
The gripper steps belong **between** segments and are the orchestrator's
(doc 6.1): `to_pregrasp` → open → `approach` → close → `lift` → `retract`.

### Frames — read before calling

`target_pose` and `object_xyz` are **`torso_link`** coordinates (doc 2.3,
13.4). This node performs **no frame conversion**, deliberately:

* `perception` reports object positions in `base_link`, which it documents as
  `pelvis` at zero waist yaw (`m1_perception.py`).
* `slam` refuses to publish `pelvis -> base_link` at all
  (`slam/config/g1_tf.yaml`: `base_link_alias: ""`, marked UNRESOLVED pending
  the navigation team).
* The two vendored URDFs disagree on where `torso_link` sits by **10 mm**
  (`slam/urdf/g1_23dof_mode_10.urdf` has `waist_yaw_joint` at z=0.044,
  `arm_planning/urdf/g1_23dof.urdf` at z=0.054), which is the same order as
  this planner's own 0.01 m position tolerance.

So a perception point cannot yet be converted into a planner target without
guessing. Callers must supply `torso_link` coordinates themselves, and the
orchestrator keeps the full grasp route `NOT_READY` until that contract is
resolved.

## Run

```bash
roslaunch arm_planning arm_planning.launch
rostopic echo -n1 /arm/planner/status
```

Models (2 × 3.2 MB) and masks (2 × 16 MB) load once at startup. Paths are
resolved with `rospkg` against the package directory, never the current
directory — the upstream planner defaults are cwd-relative and name a
`models_hybrid_1m/` directory that does not exist here; both that name and the
actual `models/` are accepted and the resolved one is reported.

If an asset or python dependency is missing the node still starts, publishes
`ready=false` with the reason, and refuses every request with
`NOT_READY: <blockers>` — so the orchestrator's preflight can name the blocker
instead of timing out. `exit_on_init_failure:=true` makes startup fatal.

### Runtime dependencies

`numpy`, `PyYAML` and `scipy` come from rosdep (`package.xml`). `torch`,
`trimesh` and `python-fcl` have no rosdep keys; see `requirements.txt`. All are
imported lazily.

## Known blocker: collision meshes

`meshes/` is an incomplete download. `g1_collision_probe.py` loads
`left_wrist_roll_rubber_hand.STL` and `right_wrist_roll_rubber_hand.STL`; both
are **absent**. `*_hand_palm_link.STL` is present but is a *different*
geometry, so substituting it would silently change the collision model and is
not done here.

Until they are fetched from the URL in `meshes/README.md`, the node reports:

```
NOT_READY: missing Module 2 assets under <pkg>: collision meshes
left_wrist_roll_rubber_hand.STL, right_wrist_roll_rubber_hand.STL
(download from the URL in meshes/README.md)
```

Which mesh is correct, and recording its provenance, is the arm_planning
owner's decision.

## Tests

Offline, no ROS/torch/numpy needed — they cover the glue this package adds
(validation, flattening, joint order, asset resolution, response conversion),
not the planner algorithms, which have their own suite in `tests/`:

```bash
cd arm_planning && python3 -m unittest discover -s test -v
```
