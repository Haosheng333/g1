# Grasp-and-deliver integration — handoff

**Last updated:** 2026-10-13 (session 2) · **Branch:** `feature/grasp-deliver-integration`

A resume point, not a design doc. Per-package detail lives in
`arm_planning/README.md`, `handover/README.md`, `orchestrator/README.md` and
`g1_ws/README.md`. Specification: `arm_planning/docs/Module_2_Arm_Planning_and_ROS2_Integration_Documentation.docx`
(cited as "doc §N").

---

## 0. Status at a glance

| | State |
|---|---|
| Session 1 work (3 new packages + workspace wiring) | **4 commits, pushed** to `origin/feature/grasp-deliver-integration` @ `2e6d145` |
| Session 2 Part A (environment + import fixes) | **UNCOMMITTED** in the working tree — 11 modified, 4 new files |
| Session 2 Part B (URDF/planner audit) | **read-only**, findings in §5 below; no files changed |
| Delivery flow (disarm/release/retract/home) | **sequencing IMPLEMENTED** and exercised offline; three steps NOT_READY on missing poses — `docs/DELIVERY_FLOW.md` |
| Session token handshake | **IMPLEMENTED**: `handover/ArmMonitor.srv` carries side + token; one generator, monitor echoes it; 16 real monitor↔orchestrator tests |
| PR | **not opened** |
| Tests | **342 passing** |
| Real robot motion | **none, ever** |

**Nothing in session 2 is committed.** If you want it kept, commit it before
switching branches.

### ⚠ Completion status — read before trusting any "done" below

An earlier version of these notes and of `orchestrator/README.md` described the
release path as if the owning task consumed the validated handover event. **It
does not. End-to-end delivery is incomplete.**

| Category | What qualifies |
|---|---|
| **implemented and exercised** | detector state machine, release-request *validation*, non-blocking lock, dry-run traversal, preflight, unconditional cleanup, detection-only mode |
| **implemented but hardware-blocked** | `navigate_report` real run, arm planning, tactile detection end-to-end, arm/hand execution, `grasp` real run (frame contract) |
| **missing or unreachable** | the **handover**, **stow** and **home** poses (none exist anywhere in the workspace); **release completion** — `hand_control`'s `release` preset leaves `require_settle` absent, so a `success` does not establish that the fingers opened; the perception→planner TF conversion; and physical verification of object transfer, which nothing in this workspace can observe. All four are **preflight** capabilities now, so `deliver` is refused before it navigates rather than after it opens the hand. (The earlier "0 callers" finding is **resolved**: every `HandoverState` method has a production caller.) |

Full classification, the proposed route, the unresolved inputs and the
integration-test plan: **`docs/DELIVERY_FLOW.md`**.

**Frame contract update (PR #10):** navigation has settled `base_link` as an
identity alias of `pelvis` at **every** waist angle — the pelvis is the waist's
*parent*, so it never rotates with the waist — published once by
`system_bringup`, on by default. Their half is closed, and `base_link` carries
**no** zero-waist limitation. What remains:

* **(a)** the 10 mm `pelvis -> torso_link` disagreement (their
  `tools/check_urdf_agreement.py` reproduces this branch's finding exactly).
  This is a **convention question first**: the net `pelvis -> shoulder` and
  `pelvis -> d435` are identical in both files, so the two may simply place the
  `torso_link` *label* differently. A measurement settles it only if that origin
  is a physical feature; otherwise the resolution is to declare each consumer's
  convention and convert explicitly. Either way the TF `torso_link` must be
  shown to match the planner's convention before a target is routed through it.
* **(b)** implementing the perception→planner conversion as a TF lookup, in
  `arm_planning` (the frame's owner).
* **(c)** a **separate, independent** requirement: perception's camera
  extrinsics are hard-coded for a torso-mounted D435 that *does* rotate with the
  waist, so a perception point is only valid near waist yaw ≈ 0 regardless of
  how `base_link` is defined. **The zero-waist limitation belongs to those
  extrinsics, never to `base_link`.** Resolving (a) and (b) does not resolve it.

`grasp` and `deliver` stay NOT_READY until (a) and (b) are done.

---

## 1. Git state — read this first

| | |
|---|---|
| Feature branch | `feature/grasp-deliver-integration`, pushed, PR **#9** open against `main` |
| `origin/main` | `3f936bc` (PR #10, `fix/nav-base-link`) — **already merged into this branch**, 0 commits behind |
| Local `main` | `c5c22c4`, a **local-only** merge; left alone deliberately (the PR is what should land) |
| Working tree | clean after the session-3→5 commits below |

Commit list: `git log --oneline origin/main..feature/grasp-deliver-integration`.
The branch carries the four session-1 feature commits, the session-2 environment
and import fixes, two `origin/main` integration merges, and the session-3→5
sequencing / handshake / contract commits described in §10.

### The merge with `origin/main` is DONE

`origin/main` moved three times while this branch ran (PR #6, #7, #10). The one
conflicting file was `.github/workflows/offline-tests.yml`, adjacent-line and
additive on both sides; it was resolved by keeping **their** longer navigation
`run:` line **and** the new arm_planning / handover / orchestrator steps. Their
`system_bringup/launch/navigation.launch` change only *added* args
(`locomotion`, `locomotion_dry_run`, `with_base_link`), so the include in
`full_system.launch` still passes only valid args. Re-verified: `git log
feature/grasp-deliver-integration..origin/main` is **empty**.

### ⚠ Local `main` still needs cleaning up — by a human, after the PR merges

Local `main` carries a local-only merge `c5c22c4` made before `origin/main`
moved. **The PR should be what lands, not this local merge.** After PR #9
merges:

```bash
git checkout main
git fetch origin
git reset --hard origin/main      # drops the local merge c5c22c4
```

Nothing is lost — every change is in the pushed feature branch.

### The PR

```
https://github.com/Haosheng333/g1/pull/9
```

---

## 2. Session 1 — what was built (committed, pushed)

Three empty/absent packages became working catkin packages; six packages joined
the build.

### `arm_planning` — ROS1 wrapper around the existing Module 2 planner

| Interface | Type |
|---|---|
| `/arm/plan_to_pose` | `arm_planning/PlanToPose` |
| `/arm/plan_grasp_from_point` | `arm_planning/PlanGraspFromPoint` |
| `/arm/planner/ready` | `std_msgs/Bool` (latched) |
| `/arm/planner/status` | `std_msgs/String` (latched) |

* **Planning only** — no service client, action client or command publisher; a
  test asserts it against the comment/docstring-stripped source.
* Paths resolved with `rospkg`, **never the CWD**. Both the documented
  `models_hybrid_1m/` and this repo's actual `models/` are accepted, and the
  resolved one is reported.
* `torch`/`scipy`/`trimesh`/`fcl` imported lazily → a missing dep leaves the
  node up publishing `ready=false` with reasons.
* Flat row-major paths + explicit counts (doc §11.4); any path dropped when
  `success=false` (doc §12.1).
* Joint order pinned against **both** `arm_kinematics.yaml` and `arm_control`'s
  `JointIndex` ordering — verified identical.

### `handover` — auto-release monitor

| Interface | Type |
|---|---|
| `/auto_release_monitor/arm` | `std_srvs/Trigger` |
| `/auto_release_monitor/disarm` | `std_srvs/Trigger` |
| `/auto_release_monitor/state` | `std_msgs/String` (latched) |
| `/auto_release_monitor/ready` | `std_msgs/Bool` (latched) |

ROS-free `IDLE → BASELINING → MONITORING → TRIGGERED` with time injected.
**Detection only**; its sole client is `/orchestrator/run_auto_release`.
Missing/stale/non-finite data → `WAIT` and cancels any detection in progress; a
timeout is never reported as a release; `TRIGGERED` is terminal.

### `orchestrator` — declarative sequencing

| Interface | Type |
|---|---|
| `/orchestrator/run_navigate_report` | `orchestrator/RunRoute` |
| `/orchestrator/run_grasp` | `orchestrator/RunRoute` |
| `/orchestrator/run_deliver` | `orchestrator/RunRoute` |
| `/orchestrator/run_auto_release` | `orchestrator/RunRoute` — **not a route**: it validates and records the monitor's request for the task that already owns the execution lock, takes no lock and issues no motion (see §10.3) |

**Dry-run by default and safe by construction**: with `enable_motion:=false` the
sequencer runs on a backend that constructs **no robot client at all** yet still
traverses every step. Nothing in a request can force motion on. Preflight runs
before any step and to completion (every blocker, not the first); one blocker ⇒
zero steps, zero motion. `cancel_navigation` + `disable_motion` in a `finally`
on every exit path. Non-blocking lock. Contains **no robotics** — enforced by test.

### Workspace + bringup

Relative `g1_ws/src` symlinks for the six packages; the three 0-byte
`system_bringup` launch files filled in with **every hardware component
defaulting to disabled**; `navigation/test/test_launch_files.py` extended from 3
packages to all 9 (11 → 19 launch files).

---

## 3. Session 2 Part A — environment + import fixes (UNCOMMITTED)

### Changed files

**New (4, plus this doc)**

| File | Purpose |
|---|---|
| `tools/bootstrap_workspace.sh` | configurable per-machine dependency discovery; `--check` default |
| `g1_ws/local.conf.example` | documents the two override keys (`local.conf` is untracked) |
| `perception/test/test_lazy_import.py` | 9 tests |
| `arm_control/test/test_lazy_import.py` | 11 tests |

**Modified (11)**

| File | Change |
|---|---|
| `perception/src/perception/__init__.py` | eager → lazy (PEP 562 `__getattr__`) |
| `arm_control/src/arm_control/__init__.py` | eager → lazy |
| `perception/package.xml` | + `python3-numpy`, `python3-opencv`, `realsense2_camera` |
| `arm_control/package.xml` | + `python3-numpy` |
| `arm_planning/package.xml` | `rospkg` → `python3-rospkg` (invalid key, mine) |
| `slam/package.xml` | `rospkg` → `python3-rospkg` (invalid key, pre-existing) |
| `handover/package.xml` | + `test_depend python3-yaml` (tests only) |
| `g1_ws/.gitignore` | ignore `local.conf`, `src/local_*`, `CATKIN_IGNORE` |
| `g1_ws/README.md` | "Two layouts, one repo" + "System dependencies" |
| `docs/ROS_FIRST_RUN.md` | new step 0: bootstrap + rosdep |
| `.github/workflows/offline-tests.yml` | +2 suites (**this is the conflicting file, §1**) |

### A.1 Livox discovery — FIXED, tracked symlink untouched

This machine already had `livox_ros_driver2` **and** `fast_lio` in the *outer*
workspace (`g1_ros1_ws/src/`), outside the repo. The failure was that
`g1_ws/src/livox_ros_driver2` is a **tracked** symlink to the other machine's
path, dangling here.

`tools/bootstrap_workspace.sh` discovers both from, in priority order:
`$G1_LIVOX_DIR`/`$G1_FAST_LIO_DIR` → `g1_ws/local.conf` (untracked) →
`rospack find` → repo-relative candidates. No personal path is hardcoded;
`local.conf` is parsed with `sed`, never sourced. `--check` is read-only;
`--apply` writes only untracked files.

The tracked symlink is **left alone**. Where it dangles, `--apply` adds a
**relative** untracked overlay `src/local_<pkg>` beside it — relative because an
absolute link resolves on only one side of the container bind mount.

**Exactly one package per name, verified both layouts:**

| Source space | Names | Duplicated |
|---|---|---|
| outer `g1_ros1_ws/src` | 12 | none |
| nested `g1_ws/src` | 11 | none |

⚠ **Caused by session 1 and now mitigated:** commit `2e6d145` added six
`g1_ws/src` symlinks, which raised the duplicate count *without*
`g1_ws/CATKIN_IGNORE` from 3 to **9**. That marker was already present on this
machine, but a teammate on this layout without it would now hit 9
duplicate-package errors. The bootstrap checks for it and creates it under
`--apply`.

### A.2 Reproducible dependencies — declared, NOT installed

`rospkg` is **not a valid rosdep key**; `rosdep install` hard-errored in two
packages. After the fix, `rosdep check --from-paths src --ignore-src` runs clean
and reports the complete gap:

```
apt  ros-noetic-rviz
apt  ros-noetic-cv-bridge
apt  python3-opencv
apt  ros-noetic-realsense2-camera
```

`realsense2_camera` was genuinely undeclared — `perception/launch/perception.launch`
does `$(find realsense2_camera)/…` but never declared it, so that launch file
could not resolve.

**Nothing was installed.** Awaiting approval:

```bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
```

`g1_navigation` was deliberately **not** touched: its `unitree_sdk2py` import is
inside `run_dds()` in a try/except — optional by design, already documented.

### A.3 Service imports — FIXED and verified end-to-end

Both `__init__.py` files resolve their public names through a module-level
`__getattr__` and cache the result. Public API unchanged (same `__all__`,
`dir()`, `from … import *`); only *when* a missing optional dep surfaces moves
from `import <pkg>` to first use of the algorithm. `arm_controller.py` and
`m1_perception.py` are untouched — no control logic, gain, joint index or timing
changed.

Verified in the container against the real generated messages:

```
perception.srv  OK -> ['success','x','y','z','frame_id','message']
arm_control.srv OK -> ['success','message']
heavy modules pulled in: NONE
```

### A.4 Validation

| Suite | Tests | Result |
|---|---|---|
| arm_planning | 60 | OK |
| handover | 78 | OK |
| orchestrator | 95 | OK |
| perception lazy-import | 9 | OK (1 skip: no ultralytics) |
| arm_control lazy-import | 11 | OK (1 skip: no SDK) |
| navigation | 63 | OK |
| slam | 26 | OK |
| **total** | **342** | **all pass** |

`git diff --check` clean · byte-compile clean · `bash -n` clean · 47
launch/package XML + 9 YAML parse · `catkin_make` **exit 0, 0 errors** · all
four `srv` modules import clean post-build.

**Launch resolution now 8/8** (was 6/8): `system_bringup full_system` (14 nodes)
and `system_bringup sensors` both fixed. Defaults verified safe — with no args
`perception_node`, `hand_*`, `arm_control_node`, `auto_release_monitor` are all
**absent**. NOT_READY re-confirmed: motion-enabled `run_grasp` →
`success:False`, `steps_completed:[]`, `motion_issued:False`, 10 blockers.

> Launch resolution proves roslaunch can **find** the packages. It is **not**
> evidence that the LiDAR, camera, hand or robot will start. The bootstrap
> prints this itself.

---

## 4. Session 2 Part B — URDF / planner audit (read-only, nothing changed)

### The finding that reframes the 10 mm problem

**The planner never loads a URDF.** Its geometry comes entirely from
`config/arm_kinematics.yaml`, plus `models/*.pt`, `collision_masks/*.npy` and
`meshes/*.STL`. `arm_planning/urdf/g1_23dof.urdf` is a **reference artifact, not
a runtime input**. So the question is not "do the two URDFs agree" but "which
convention is `arm_kinematics.yaml` built in".

### Compatibility table

| # | Item | Finding | Verdict |
|---|---|---|---|
| 1 | Provenance | md5-identical to upstream: `slam/…mode_10.urdf` = `g1_23dof_mode_10.urdf`; `arm_planning/…g1_23dof.urdf` = `g1_23dof.urdf` (md5 `b65b639f2ad0b64d5060ef5cb79e4dd3`). Same commit, **two variants**, neither modified. slam has `SOURCE.md`+`LICENSE`; arm_planning's `README.md` is 1 byte. | **Verified** |
| 2 | Files actually loaded | `arm_kinematics.yaml`, 2 `.pt`, 2 `.npy`, 11 `.STL`. **No URDF.** | **Verified** |
| 3 | Frame / units / convention | `torso_link`; `[x,y,z,rx,rz]`; m/rad; "URDF fixed-axis XYZ Euler, rx=roll rz=yaw, pitch omitted"; order `shoulder_pitch, roll, yaw, elbow, wrist_roll`; EE = `left_grasp_frame`, a **YAML-only** frame (`grasp_xyz=[0.215,0.006,0.025]`) absent from both URDFs. Limits + axes match **both** URDFs exactly, all 5 joints. | **Verified** |
| 4 | Checkpoint metadata | Keys only `arm`, `model_state`, `x_mean`, `x_std`. Embedded `joint_midpoint`/`joint_half_range` match YAML limits to float32 (**5.8e-08 / 1.6e-07**). **No** key names a URDF, commit, frame, units or config version. | **UNVERIFIED** (geometry/pose-frame provenance) |
| 4b | Mask metadata | `arm`, `shape 83×55×76×46`, `joint_order` (4 joints), limits, `resolution_deg 4.0`, `wrist_roll_rad 0.0`, penetration allowance, completion state. Pins joint-space convention; names **no** URDF, mesh set or frame. | joint space **verified**; geometry **UNVERIFIED** |
| 5 | FK in a common pelvis frame | `pelvis→EE` **bit-identical**, `dxyz = 0.000e+00`, `dR = 0.000e+00`, across zero / bent arm / **waist +0.40** / **waist −0.90** / waist+bent. `pelvis→torso_link` differs by a **constant 1.000e-02 m**, waist-independent. `pelvis→shoulder_pitch` sums identical: `[-0.0000072, 0.10022, 0.29178]`. | **Shifted frame origin, NOT different geometry** |
| 6 | Can 10 mm be handled by conversion? | Not inferred from the torso origin alone: (a) FK above; (b) arm joints 2–5 byte-identical in both URDFs; (c) YAML shoulder_pitch z = **0.23778** ⇒ derived from `g1_23dof`; (d) `g1_collision_probe.py` hardcodes body meshes at `(0.0039635, 0, −0.054)` = the pelvis origin **in the g1_23dof convention**; (e) in *both* variants `head_joint` origin = inverse of `waist_yaw_joint`, so body links sit at the pelvis origin either way. Whole stack self-consistent; body-relative-to-arm geometry variant-independent. | **Compatible with a verified adapter. No retraining, no mask regeneration.** |
| 7 | Missing collision meshes | Both 23-DOF URDFs name the 5th link `left/right_wrist_roll_rubber_hand` — exactly the missing `*_wrist_roll_rubber_hand.STL`. `hand_palm_link` appears **only** in `g1_29dof_*_with_hand*`. The probe's expectation is **correct for this robot**; the present `*_hand_palm_link.STL` is from a **different robot**. | **Expected mesh confirmed; substitution would be wrong** |

### Classification

| Option | Class |
|---|---|
| Keep YAML + models + masks as they are | **compatible without changes** |
| pelvis ⇄ planner-torso conversion using arm_planning's own `[-0.0039635, 0, 0.054]` + `Rz(waist_yaw)` | **compatible with a verified adapter** — must **never** take `pelvis→torso_link` from TF, which is mode_10's (10 mm lower) |
| Fetch the two `*_wrist_roll_rubber_hand.STL` from upstream `7d6075f7` | **compatible without changes** |
| Retrain IK / regenerate masks for mode_10 | **not required** — no evidence supports it |
| Bind the checkpoint's pose frame to a URDF | **insufficient evidence** — needs one torch run (§6) |
| Substitute `hand_palm_link` for the missing hand mesh | **rejected** — different robot variant |

---

## 5. Blockers — current status

### 5.1 Frame contract — CHARACTERISED, still needs owner sign-off
Was "unresolved, possibly needs retraining". Part B shows it is a **pure frame
offset**: `pelvis→EE` FK is bit-identical between variants, the 10 mm lives only
in the `torso_link` label, and the planner stack is internally consistent on the
`g1_23dof` convention. An explicit adapter suffices.
**Still blocks a real `grasp` run** until the owners confirm §6 Q1/Q6 and the
adapter is reviewed. `orchestrator`'s `FRAME_CONTRACT` capability stays
`unresolved`; only its wording was corrected (see §10.5). `grasp` and `deliver`
remain NOT_READY, and they are refused at **preflight**.

### 5.2 Missing collision meshes — EVIDENCE COMPLETE, awaiting approval
`left/right_wrist_roll_rubber_hand.STL` absent; confirmed to be the correct link
name for this robot on **both** 23-DOF variants. Both files already exist at
`~/g1_ros1_ws/reference/unitree_ros/robots/g1_description/meshes/`. Not copied:
vendoring another team's binaries (with `SOURCE.md` + `LICENSE` per the
`slam/urdf/SOURCE.md` convention) is the owner's call. **Cheapest blocker to
clear — it alone flips `/arm/planner/ready` to true.**
`arm_planning/test/test_asset_resolution.py` fails loudly once the meshes
appear, so it cannot silently drift.

### 5.3 Hand tactile schema + threshold — unchanged
No tactile topic exists anywhere; `hand_control` reads the Revo2 over serial and
publishes no sensor topic. Adapter ships `enabled: false`; `release_delta` and
`direction` have no defaults. A test asserts the shipped config **cannot be
armed**.

---

## 6. Exact questions still needed from owners

**arm_planning**
1. Is the robot the **23-DOF** variant, and does its `torso_link` follow
   `g1_23dof` (z=0.054) or `mode_10` (z=0.044)? *Everything in the planner
   assumes `g1_23dof`.*
2. Approve fetching the two `*_wrist_roll_rubber_hand.STL` from upstream
   `7d6075f7` with `SOURCE.md` + `LICENSE`.
3. Fill `arm_planning/urdf/README.md` with the provenance identified in §4.
4. **The one measurement that closes Q4:** with torch installed, run
   `GuardedIKSolver.solve()` on ~20 poses spanning the training workspace and
   report mean/P95 `position_error_m`. ~4 mm (matching the doc's 4.37/4.17 mm)
   confirms the checkpoint's pose frame matches `arm_kinematics.yaml`; a
   systematic ~10 mm floor indicates a frame mismatch. The existing
   `max_position_error_m = 0.01` guard already rejects the bad case at runtime,
   so this is confirmation, not a safety gap.
5. Was `grasp_xyz = [0.215, 0.006, 0.025]` measured on the real hand or derived?
   It exists in no URDF.
6. Is `torso_link`'s origin a **physical feature** that can be measured, or a
   modelling datum? That decides whether §0(a) is a measurement or a convention
   declaration.
7. **The handover, stow and home poses.** A search for
   `home|rest|stow|neutral|park|tuck` across the workspace returns **zero**
   hits, and the grasp retraction is not reusable: it starts at the *lift* pose
   and inherits y/z from `object_xyz`, so replaying it after a handover would
   command a jump back above the table. Three `deliver` steps and one `grasp`
   step need validated joint targets. Nothing here will invent them.

**navigation** — *settled, PR #10*
`base_link` is an identity alias of `pelvis` at every waist angle, published
once by `system_bringup` (`with_base_link`, on by default); `g1_tf.yaml`'s
`base_link_alias: ""` keeps the frame to a single publisher and is not a
refusal. Nothing outstanding here.

**perception**
8. Must the camera pose come from TF, or is the waist held at ≈0 whenever
   perception is used? The extrinsics are hard-coded for a torso-mounted D435
   that rotates with the waist. This is **independent** of the frame contract.

**hand_control**
9. Tactile topic, message type, field path, and **units** — raw device readings
   alone do not establish physical units; this needs vendor documentation or a
   calibration. Then someone measures `release_delta` and `direction`.
10. **Release completion.** `/hand_{side}/release` returns only `success` and
   `message`. `revo_hand_link.wait_until_settled` *does* poll fresh
   `read_positions()` against the target within `settle_tolerance` until
   `settle_timeout_s`, but the `release` preset leaves `require_settle`
   **absent (defaults false)**, so `_execute` does not fail on a settling
   timeout and returns success with *"release commanded, did not settle"*.
   Either set `require_settle` on that preset or expose settling in the
   response (a structured flag, or the feedback positions). Until then the
   orchestrator refuses to retract after a release — there is no override.

**integration / you**
11. Approve `rosdep install` for the 4 apt packages; decide whether
   `python3-scipy` and `ultralytics` are baked into the `g1-ros1` image.

---

## 7. Proposed next patch (NOT implemented)

1. **`arm_planning/src/arm_planning/frames.py`** — pure, ROS-free
   `pelvis_point_to_planner_torso(xyz, waist_yaw)` using arm_planning's **own**
   `[-0.0039635, 0, 0.054]`, with a docstring forbidding composition with TF's
   `torso_link`. Tests pinning the constant, transform direction, round-trip
   identity, and one asserting it differs from the mode_10 constant by exactly
   10 mm so a future "cleanup" cannot unify them. **In `arm_planning/`, because
   the conversion belongs to the frame's owner — not in `orchestrator/`.**
2. `arm_planning/urdf/README.md` + `SOURCE.md` with the §4 provenance.
3. Fetch the two STLs with provenance, after owner approval.
4. **Only then** give `orchestrator`'s `FRAME_CONTRACT` a `resolved_by` note and
   have `to_planner_frame` call the arm_planning helper. Until step 1 is
   reviewed by both owners, `run_grasp` stays **NOT_READY**.

Land Part A first — it is independent and already green. Hold Part B's patch
until Q1, Q2 and Q4 are answered.

---

## 8. Environment notes

### `python3-scipy` still not reproducibly provisioned
Installed into the **running** `g1-ros1` container only
(`docker exec g1-ros1 apt-get install -y python3-scipy`, 1.3.3), **not** baked
into the image, and no repo file provisions it. Without it, 6 navigation suites
and 1 slam test error out. CI installs it via pip, so CI is unaffected. `pip3`
is absent in that container — use `apt-get install python3-<pkg>`.

Also absent: `torch`, `trimesh`, `python-fcl` (arm_planning), `ultralytics`,
`cv_bridge` (perception), `unitree_sdk2py` (arm_control). By design those report
`NOT_READY` rather than crashing. `perception/test/test_m1_perception.py` errors
on missing `ultralytics` — environmental, verified identical with the session-2
change stashed.

### Eager imports — FIXED in session 2
Was a blocker; see §3 A.3. `perception.srv` and `arm_control.srv` now import
with no heavy deps.

### Two machines, different layouts
Machine A (`haosheng`, `/home/haosheng/g1_ros1_ws/src/g1`, container `g1-ros1`,
builds the **parent** source space) vs machine B (`g1-robot`,
`~/Workspace/DIP`, container `g1_intellect`, builds the nested `g1_ws`). Machine
A needs the untracked `g1_ws/CATKIN_IGNORE`. Never "fix" machine B's paths in
tracked files — run `tools/bootstrap_workspace.sh` instead.

---

## 9. Commands to re-run everything

```bash
# Per-machine wiring (read-only first)
tools/bootstrap_workspace.sh
tools/bootstrap_workspace.sh --apply

# System deps: report, then install (needs approval)
cd /home/haosheng/g1_ros1_ws
rosdep check   --from-paths src --ignore-src --rosdistro noetic
rosdep install --from-paths src --ignore-src --rosdistro noetic -y

# New offline tests (no ROS, no torch, no numpy)
cd arm_planning      && python3 -m unittest discover -s test -v
cd handover          && python3 -m unittest discover -s test -v
cd orchestrator/test && python3 -m unittest discover -s . -v
cd perception        && python3 -m unittest discover -s test -p test_lazy_import.py -v
cd arm_control       && python3 -m unittest discover -s test -v

# Regressions (need scipy — see §8)
docker exec g1-ros1 bash -c 'cd /root/catkin_ws/src/g1/navigation/test && \
  python3 -m unittest test_navigation_node test_costmap_planner_dwa test_sim_room \
  test_robustness test_launch_files test_slam_nav_contract test_joint_bridge_table test_section_e'
docker exec g1-ros1 bash -c 'cd /root/catkin_ws/src/g1/slam && \
  PYTHONPATH=src python3 -m unittest discover -s test -p "test_*.py"'

# Build (whitelist: g1_slam/g1_navigation need livox + fast_lio, wired by the bootstrap)
docker exec g1-ros1 bash -lc 'source /opt/ros/noetic/setup.bash && \
  cd /root/catkin_ws/src/g1/g1_ws && catkin_make -j4 \
  -DCATKIN_WHITELIST_PACKAGES="arm_planning;handover;orchestrator;perception;arm_control;hand_control"'

# Launch resolution (all 8 pass)
docker exec g1-ros1 bash -lc 'source /opt/ros/noetic/setup.bash && \
  source /root/catkin_ws/devel/setup.bash && \
  roslaunch --nodes system_bringup full_system.launch'

# Rehearse a route with the robot powered off
roslaunch orchestrator orchestrator.launch
rosservice call /orchestrator/run_navigate_report "{target: table}"
```

Real motion needs **two** deliberate steps, both off by default:
`enable_motion:=true` on the orchestrator **and**
`rosservice call /navigation/enable_motion "data: true"`.

---

## 10. Sessions 3–5 — the delivery flow, the handshake, the contract gates

Everything in §§2–9 above predates these. Three things were built, and one
earlier claim was retracted.

### 10.1 The retracted claim

Session 1's orchestrator shipped `HandoverState` with
`begin / mark_awaiting_pull / consume_pending / mark_released / mark_releasing /
abandon` and a passing unit suite — and **zero production callers**. The active
route never consumed a handover event. Interface presence and green unit tests
are not evidence of an end-to-end path; the rule applied since is to grep for
callers before claiming a step works.

### 10.2 The production `deliver` route (14 steps)

```
require_held_object -> await_localization -> enable_motion -> navigate
  -> plan_handover_pose -> execute_handover_pose -> settle_handover
  -> arm_monitor -> await_pull -> disarm_monitor
  -> release_hand -> confirm_released -> settle_recipient -> execute_retract_home
```

`grasp` gained `record_held_object` + `execute_stow`. The old `AUTO_RELEASE`
route was **deleted** — its steps were in reverse order. `Backend.await_pull()`
lives in the *base* class so the real and fake backends share one polling loop.
The route's `finally` block always runs `cancel_navigation()`,
`disable_motion()` and `disarm_monitor_cleanup()`.

Two real defects the new tests caught, both now fixed: a failed `release_hand`
left the state `HELD` because fail-fast skips `confirm_released` (now
`UNCERTAIN` at the failure site), and a resume at `release_hand` could open the
hand with no consumed event (now guarded by `release_permitted()`).

### 10.3 One authoritative session token, and the `ARMING` window

`handover/srv/ArmMonitor.srv` replaces the fieldless `Trigger`:

```
string side           # must match the monitor's own ~side
string session_token  # opaque, non-empty; echoed verbatim in the release request
---
bool   success
string message
string session_token
```

The **orchestrator** generates the token (`new_session_token()` is the one
generator); the monitor only stores and echoes it. The orchestrator enters
`ARMING` *before* arming, so a pull that lands between arming and
`AWAITING_PULL` is accepted and queued rather than acted on early or dropped —
`mark_awaiting_pull` deliberately does not clear `_pending`. Every arming
failure path (call raised, monitor refused, token echo mismatch) calls
`abandon()`, which clears both the queued event and the token.

### 10.4 Three distinct release states — do not collapse them

`RELEASE_COMMANDED` (the service returned) ≠ `RELEASE_SETTLED` (fingers
confirmed open) ≠ object transferred. `mark_release_settled(evidence)` raises
`ValueError` without checkable evidence from the hand result — an orchestrator
config flag is **not** evidence, and an earlier `release_preset_requires_settle`
flag plus an `allow_retract_after_command_ack` override were both removed.
`retraction_allowed()` refuses `RELEASE_COMMANDED` with **no override**:
retracting on a command acknowledgement would move the arm away from a person
who might still be holding the object. `transfer_verified` is hard-coded
`false` at all three `RunRoute` response sites; no code path can set it true,
because nothing in this workspace observes the object leaving the hand.

### 10.5 Known gaps are preflight blockers, not runtime surprises

`handover_pose`, `stow_pose`, `home_pose` and `release_completion` are
`unresolved` capabilities, so with every service and flag present:

```
deliver          4 preflight blockers: frame_contract, handover_pose,
                                      release_completion, home_pose
grasp            2 preflight blockers: frame_contract, stow_pose
navigate_report  0
```

No runtime state can clear an `unresolved` capability, so `deliver` stops before
it navigates — not after it has opened the hand. The per-step checks stay, so a
service dying mid-route still fails fast.

### 10.6 What the tests do and do not prove

| | |
|---|---|
| **A — fake backends** | `test_delivery_flow.py` runs the *production* runner with `FakeBackend(seq.Backend)`, an injected clock and injected pull events. Proves sequencing, call order, timeout/cancel, duplicate/stale handling and state preservation on failure. |
| **B — in-process service bus** | `test_monitor_integration.py` instantiates the real `AutoReleaseMonitorNode` and the real `OrchestratorNode` and wires them through a `BUS` dict. **This is logic validation. It is not ROS transport validation and not hardware validation.** |
| **C — real adapters** | never run. No arm, hand, tactile or camera code has executed. |
| **D — missing** | the three poses, release completion, the frame conversion, transfer verification. |

Offline totals: **608 run / 606 passed / 2 skipped / 0 failed**;
`perception/test/test_m1_perception.py` is **blocked** (no `ultralytics`) and is
not counted as passing.
