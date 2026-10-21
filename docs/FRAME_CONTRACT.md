# Frame contract: navigation, perception and the arm planner

Settles the item the orchestrator lists as `FRAME_CONTRACT` ("unresolved ... pending the navigation team").
Pinned by `navigation/test/test_frame_contract.py`.

## Decision: `base_link` is an identity alias of `pelvis`

| Frame | Published by | Meaning |
|---|---|---|
| `map` | relocalizer (static `map -> world` for rviz) | z-up saved-map frame |
| `camera_init` | relocalizer parent `map`; FAST-LIO2 origin | z-down odometry origin |
| `pelvis` | `g1_tf_publisher` (`camera_init -> pelvis`, needs real `/joint_states`) | URDF root; what navigation drives |
| `base_link` | `system_bringup` static transform, `with_base_link` (**on by default**) | identical to `pelvis` |
| `torso_link` and the rest of the URDF | `robot_state_publisher` | `pelvis -> torso_link` through the revolute `waist_yaw_joint` |

Why identity is exact: the pelvis does not rotate with the waist (the waist is its child), so "the pelvis at zero
waist yaw" is `pelvis` itself, for every waist angle. g1_slam never publishes `base_link` (`base_link_alias` must stay
empty), so the frame has exactly one publisher.

**Update (2026-10):** `perception/` no longer reports in `base_link`. It reports in `torso_link`, the frame the camera is
rigidly mounted on, so its points hold at every waist angle and the `torso_link <- base_link` lookup below is not needed
for perception output (it is still the right conversion for a point stored in the map). What remains open for
perception is the extrinsics value itself, see `perception/README.md` "Camera extrinsics".

## What the grasp side does with it

A point `p` in `base_link` is a point in `pelvis`. The arm planner takes `torso_link`. TF gives
`torso_link <- base_link` including the CURRENT waist yaw, because `robot_state_publisher` uses the real
`/joint_states` that `g1_joint_state_bridge` supplies (dds mode reads it from `rt/lowstate`). So the conversion is one
TF lookup (`lookup_transform("torso_link", "base_link", Time(0))`), never a hand-written offset. For an object in the map:
`map -> pelvis` (navigation's localization) then the same lookup. This needs the localization stack to be running.

## One question left, and it is about the URDFs, not about navigation

`python3 tools/check_urdf_agreement.py` reports, from the two vendored files:

| URDF | `pelvis -> torso_link` at zero waist yaw |
|---|---|
| `slam/urdf/g1_23dof_mode_10.urdf` (feeds TF, the Mid-360 pose) | (-0.0040, 0, **0.0440**) m |
| `arm_planning/urdf/g1_23dof.urdf` (feeds the arm planner) | (-0.0040, 0, **0.0540**) m |

A 10 mm systematic difference, the size of the arm planner's own 0.01 m tolerance. `arm_planning/config/arm_kinematics.yaml`
is consistent with its own URDF (`torso_height_from_floor - pelvis_height_from_floor = 0.054`). Neither can be chosen by
reading files. **Measure it on the standing robot:** the vertical distance from the pelvis joint origin to the torso/waist
joint origin (or the heights of both above the floor with a level and a ruler, difference of the two). Correct the wrong
URDF and its config. Until then a grasp pose derived through TF is off by up to 10 mm in z.

## For the orchestrator team

Navigation's side of `FRAME_CONTRACT` is closed: `pelvis -> base_link` is published (identity). What remains blocked is
(a) the URDF measurement above and (b) implementing the perception -> arm_planning conversion with the TF lookup.
`orchestrator/src/orchestrator/capabilities.py` still says slam refuses `pelvis -> base_link` and that navigation has not
decided; update that text and its `unresolved` flag once (a) and (b) are done.
