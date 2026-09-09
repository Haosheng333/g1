# Vendored robot description — provenance

`g1_23dof_mode_10.urdf` in this directory is an **exact, unmodified copy** of
Unitree's official robot description. It is vendored rather than depended on
because `g1_description` is a plain directory inside the `unitree_ros`
repository — it has **no `package.xml`**, so it is not a ROS package and
`$(find g1_description)` can never resolve.

## Upstream

| Field | Value |
| --- | --- |
| Repository | <https://github.com/unitreerobotics/unitree_ros.git> |
| Branch | `master` |
| Commit | `7d6075f7f58588b189b940130e3edab3c839b2df` |
| Commit date | 2026-08-28 |
| Commit subject | `add R1_Robotic_Arm` |
| Upstream path | `robots/g1_description/g1_23dof_mode_10.urdf` |
| Retrieved | 2026-09-09 |
| Local reference checkout | `~/g1_ros1_ws/reference/unitree_ros` (outside this repo) |

The file was verified unmodified against upstream `HEAD` at the time of copying.

## Integrity (SHA-256)

```
9333e89c51614c92b416c2c22a28f1614284d02321548e10479cce73552bcf43  g1_23dof_mode_10.urdf   26702 bytes
84aac59fd3246e3ddc49d1387644e8fcf43b0def4f0b9fe687f372e90446df2d  LICENSE                  1558 bytes
```

Verify with:

```bash
sha256sum $(rospack find g1_slam)/urdf/g1_23dof_mode_10.urdf \
          $(rospack find g1_slam)/urdf/LICENSE
```

`slam_health_check.py --component tf` checks both digests against the values
above and FAILs on any mismatch, so an accidental local edit to the vendored
copy cannot pass unnoticed.

## Scope: kinematics and TF only — NOT visualization

The URDF references 26 mesh files as **relative** paths
(`filename="meshes/<link>.STL"`). Those 167 upstream mesh files are **not**
vendored here, so none of the mesh references resolve.

- **Works:** the link/joint graph. `robot_state_publisher` needs only that, so
  every TF this package depends on is correct — including
  `pelvis -> torso_link` (`waist_yaw_joint`, revolute) and
  `torso_link -> mid360_link` (`mid360_joint`, fixed, `rpy` roll = pi because the
  Mid-360 is mounted upside down).
- **Does not work:** any mesh geometry. An RViz `RobotModel` display will render
  nothing and log unresolved-mesh errors. **No robot mesh visualization support
  is claimed or implied.** If visualization is ever needed, vendor the
  `meshes/` directory too, or point RViz at the reference checkout.

The joint origins this package relies on are mirrored as constants in
`src/g1_slam/tf_chain.py`, and `tf_chain.verify_against_urdf()` cross-checks
them against this file so the two cannot silently drift apart.

## License

Upstream is BSD 3-Clause. The unmodified upstream `LICENSE` is vendored
alongside the URDF in this directory and applies to `g1_23dof_mode_10.urdf`:

> Copyright (c) 2016-2022 HangZhou YuShu TECHNOLOGY CO.,LTD. ("Unitree Robotics")

## Updating

1. Fetch the reference checkout and note the new commit hash.
2. Re-copy the URDF (and `LICENSE` if it changed).
3. Update the commit hash and both SHA-256 digests above, and the matching
   constants in `scripts/slam_health_check.py`.
4. Run `rosrun g1_slam slam_health_check.py --component tf` — it re-verifies the
   digests and that `tf_chain.py`'s joint constants still match the URDF.
