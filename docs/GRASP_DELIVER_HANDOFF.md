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
| 3 | Frame / units / convention | `torso_link`; `[x,y,z,rx,rz]`; m/rad; "URDF fixed-axis XYZ Euler, rx=roll rz=yaw, pitch omitted"; order `shoulder_pitch, roll, yaw, elbow, wrist_roll`; EE = `left_grasp_frame`, a **YAML-only** frame absent from both URDFs (grep count 0 in each). **`grasp_xyz` corrected 2026-10-14 to `[0.215, 0.06, 0.025]`** — the old `0.006` was short by 10×. Its reference frame is now established from code, not assumed: `forward_kinematics()` applies it after the 5th joint, so it is expressed in `<side>_wrist_roll_rubber_hand`. Limits + axes match **both** URDFs exactly, all 5 joints. | **Verified; offset corrected** |
| 4 | Checkpoint metadata | **CORRECTED 2026-10-14 — the earlier "no key names a frame or config version" was wrong.** Top-level keys are `arm`, `config`, `epoch`, `model_state`, `optimizer_state`, `weights`, `x_mean`, `x_std`, `best_validation_position_mean_m`. **`config` embeds the whole `arm_kinematics.yaml` as trained**, including `grasp_xyz`, read with no torch by `arm_planning/src/arm_planning/checkpoint_meta.py`. Both shipped checkpoints embed `grasp_xyz = [0.215, ±0.006, 0.025]`, i.e. the **pre-correction** tool point, and record validation errors of 4.378 mm (left) / 4.180 mm (right). | **Pose-frame provenance now VERIFIED — and the checkpoints are STALE: 54.0 mm, 12–13× their own validation error** |
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

### 5.3 Hand force schema — narrowed, not cleared

**Retired 2026-10-14:** the hand model is confirmed (BrainCo Revo2 on a Unitree
G1 23-DOF), the SDK and runtime are available, and the team has already used
this hardware configuration for IK. "Identify the robot/hand model for SDK
installation" is **closed** and the former Python 3.8-vs-3.9 concern is moot.

**Still open, on its own evidence:** the force *schema* — which SDK call returns
force, its field names and types, the channel→finger mapping, the **unit**, and
the frame rate. A number from the device does not state its own unit, so the
requested 3 N threshold is uninterpretable until that is documented or
calibrated.

The detection rule is now the **consecutive-frame, any-channel** rule: per
channel, this frame minus the previous one; any channel past 3 N produces one
release request; otherwise the reference advances. The moving average, adaptive
baseline, `drift_gain`, `debounce_sec` and `min_channels` are gone. The adapter
still ships `enabled: false`, and a test asserts the shipped config **cannot be
armed** — now because no channels are declared, not because a threshold is
missing.

**Also still open and not implied by the hand model being settled:**
collision-model compatibility (the URDFs model the stock
`*_wrist_roll_rubber_hand`, not a Revo2) and orchestrator-to-SDK integration
(nothing has run against the real SDK).

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
5. ~~Was `grasp_xyz = [0.215, 0.006, 0.025]` measured on the real hand or
   derived?~~ **ANSWERED 2026-10-14: the intended offset is
   `[0.215, 0.06, 0.025]`** — the second component is 0.06, not 0.006. Applied
   to `left`, and mirrored to `right` as `-0.06` under this file's own
   y-negation convention (verified against both URDFs for all four lateral
   origins, and `tests/test_forward_kinematics.py:11` asserts the mirror holds).
   **Two narrow ambiguities remain:** which physical hand it was measured on
   (immaterial under the mirror convention *unless* the mount is asymmetric),
   and whether it was measured in the `*_wrist_roll_rubber_hand` frame the code
   applies it in rather than from some other datum. Consequence for the
   checkpoints: see row 4 — they are stale by 54.0 mm and must be retrained.
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

---

## 11. Session 6 — confirmed hardware facts, tool-point correction, new detection rule

Scope of this session: **unambiguous, hardware-independent changes only.** The
robot was unavailable throughout. No hardware was commanded, nothing was
committed, merged or pushed.

### 11.1 Facts now confirmed — stop treating these as open

| Fact | Effect |
|---|---|
| Robot is a **Unitree G1 23-DOF**; installed hand is a **BrainCo Revo2** | "identify the robot/hand model for SDK installation" is **retired** |
| **SDK and runtime are available**; the team has already used this hardware configuration for IK | SDK availability and the Python 3.8-vs-3.9 question are **retired** |
| Intended hand offset is **`(0.215, 0.06, 0.025)`** | `arm_kinematics.yaml` corrected; see 11.2 |
| Handover force feedback will be read **directly through the SDK** | no separate ROS tactile-publisher implementation is requested now |
| Detection rule is **consecutive-frame, any channel, 3 N** | detector rewritten; see 11.3 |

**Explicitly NOT implied by the above**, and still unsupported by evidence:

* **collision-model compatibility** — both URDFs model the stock Unitree
  `*_wrist_roll_rubber_hand`; no Revo2 geometry exists anywhere in the
  workspace, and `g1_collision_probe.py` loads
  `<side>_wrist_roll_rubber_hand.STL`, which is still absent from
  `arm_planning/meshes/`;
* **orchestrator-to-SDK integration** — no code in this workspace has ever run
  against the real SDK;
* the **unit** of any force reading;
* the **sign** a human pull produces on a force channel.

### 11.2 Tool point corrected — and the checkpoints are now stale

`arm_kinematics.yaml` `grasp_xyz`: left `[0.215, 0.006, 0.025]` →
**`[0.215, 0.06, 0.025]`**, right mirrored to `[0.215, -0.06, 0.025]`.

**Reference frame, established from code rather than assumed.**
`g1_forward_kinematics.forward_kinematics()` composes the five joint
origins/rotations and applies `grasp_xyz` **last**, so it is expressed in the
frame after the 5th joint — `<side>_wrist_roll_rubber_hand`. It is *not* in
`torso_link`, and `<side>_grasp_frame` exists in **neither** URDF (grep count 0
in both). `torch_g1_fk.TorchG1FK` applies the identical value as `tool_xyz`, so
the two FK paths agree by construction.

**Why the right hand was mirrored rather than left alone.** This file mirrors
every lateral quantity, and the URDFs agree: shoulder_pitch `±0.10022`,
shoulder_roll `±0.038`, shoulder_yaw `±0.00624`, wrist_roll `±0.00188791`. An
asymmetric `grasp_xyz` would also have broken
`tests/test_forward_kinematics.py:11`, which asserts
`left.y ≈ -right.y` on `pose_5d`. Re-run with the corrected values: the mirror
holds (left `y = +0.208635`, right `y = -0.208625`).

| Consumer | Effect of the change |
|---|---|
| **FK** (`g1_forward_kinematics.py:64`, `torch_g1_fk.py:35-36`) | tool point moves 54.0 mm in y. Both paths read the YAML, so both follow. |
| **IK checkpoints** (`models/g1_*_hybrid_best.pt`) | **STALE.** Each embeds the `arm_kinematics.yaml` it was trained with, including `grasp_xyz = [0.215, ±0.006, 0.025]`. The 54.0 mm redefinition is **12–13×** their own recorded validation error (4.378 mm left, 4.180 mm right). **Retraining is required**; this is evidence from the files, not an assumption. |
| **Collision masks** (`collision_masks/*.npy`) | **Unaffected, with evidence.** `g1_collision_probe.link_transforms()` composes only `origins_xyz`/`origins_rpy`/`axes` for the five links ending at `hand`, and never reads `grasp_xyz`; the manifests cover 4 joints with `wrist_roll_rad 0.0`. **No regeneration needed.** |
| **Tool-point / checkpoint definitions** | now cross-checked automatically — see 11.4 |
| **Downstream grasp geometry** | `grasp_geometry.py` works from `grasp_lateral_offset` and `arm_sign`, not `grasp_xyz`, so it is untouched. |

**Remaining ambiguity, recorded rather than resolved:** which physical hand the
measurement was taken on (immaterial under the mirror convention *unless* the
mount is asymmetric), and whether it was taken in the
`*_wrist_roll_rubber_hand` frame the code applies it in, rather than from a palm
face, finger pad or CAD datum. Both are written into the config header.

### 11.3 Detection rule replaced

Requested rule, now implemented in `handover/src/handover/tactile_detector.py`:
compare consecutive frames per channel; if **any** channel's delta passes its
threshold emit **one** release request; otherwise make that frame the new
reference and continue. **Any-channel, never a sum across fingers.**

* threshold **3.0** per channel (`DEFAULT_RELEASE_THRESHOLD`) — the *requested
  setting*, in the SDK's own unconfirmed unit, **not** a measured figure;
* removed: the 10-frame moving average, the adaptive baseline, `drift_gain`,
  `stable_tolerance`, `debounce_sec`, `min_channels`, and the `candidate|` event
  stage. Advancing the reference on every non-qualifying frame *is* the drift
  tracking;
* kept: fault handling (a delta is never measured across a dropout or a stale
  gap), post-arm suppression, one-shot triggering, channel masking, timeouts;
* **sign convention preserved, not reinvented**: `tactile_frame.difference()`
  returns a signed `new - reference`, and `release_monitor.DIRECTIONS`
  (`rise` / `drop` / `either`) selects the comparison. This project never
  defined *which* applies to a pull, so `either` is the default — the only
  choice that cannot miss a real pull by guessing the sign wrong. **Narrow
  ambiguity, flagged in code and config: set `direction` explicitly once the SDK
  channel semantics are known.**

**Inherent property, recorded so it cannot surprise anyone:** a pull slower than
the threshold *per frame* is tracked rather than detected — at 30 Hz and 3 N,
below roughly 90 N/s. `test_a_slow_ramp_never_triggers` pins it.

**Task ownership unchanged.** Reading force through the SDK creates **no second
release path**. The session-token handshake, held-object and task-state
validation, the `ARMING` window, cancellation, duplicate-request handling and
the three release states all still live in `orchestrator/handover_state.py`, and
`handover` still only calls `/orchestrator/run_auto_release`. Automatic hardware
release stays **disabled** (`release_integration: false`), and a command
acknowledgement is still not treated as confirmed finger opening or verified
transfer.

### 11.4 New guard: the config and the checkpoints can no longer drift silently

`arm_planning/src/arm_planning/checkpoint_meta.py` reads a checkpoint's embedded
config **without torch** (a `.pt` is a zip; only the tensor rebuilds need torch,
so they are stubbed) and compares its tool point against the live YAML.
`arm_planning/test/test_checkpoint_consistency.py` exercises it and records the
current stale state. Nothing had compared the two before, which is exactly why a
10× error in `grasp_xyz` could sit in the config unnoticed.

### 11.5 Test results for this session

| Suite | Result |
|---|---|
| `handover` | **155 passed** (detector suite rewritten for the new rule: 54 tests) |
| `orchestrator` | **199 passed** (incl. the 16 real monitor↔orchestrator tests, unchanged in intent) |
| `arm_planning/test` | **71 passed**, including 11 new checkpoint-consistency tests |
| `arm_planning/tests` | **not run anywhere** — pytest-style and no `pytest` on host or in the container (pre-existing). The one assertion the offset change could break, the FK mirror, was executed directly with numpy in the container and **passes**. |
| `slam` | passes in the container; 3 host errors are `numpy` absent on the host (pre-existing) |

### 11.6 Deferred until the robot is available — NOT passed, NOT attempted

* live TF checks and the `camera_init → pelvis` chain;
* any real sensor reading, SLAM validation or map check;
* **force-data inspection**: the SDK call, field names, channel mapping, unit,
  rate and pull sign;
* IK **retraining** against the corrected tool point, and the ~20-pose
  `GuardedIKSolver` error measurement that would confirm it;
* collision-model compatibility for the Revo2;
* orchestrator-to-SDK integration;
* every physical motion test.

"TF cannot be tested now" does **not** retire the coordinate-conversion
requirement. Offline frame analysis continued this session and is what produced
11.2.

---

## 12. Session 7 — the pipeline was actually tested across module boundaries

Full evidence: **`docs/DATA_PIPELINE_VALIDATION.md`**. Summary only here.

Tested at `7bf01cb` with the previous session's work still uncommitted and
preserved. No hardware commanded; nothing committed, merged or pushed.

### 12.1 The headline

The pipeline is **broken at boundary 3** (perception → planner frame
conversion). Boundaries 1, 2 and 7 work over real ROS. Boundaries 3, 4, 5, 6
and 8 have **no production implementation**: `to_planner_frame`, `plan_grasp`,
`execute_segment`, `plan_handover_pose`, `execute_stow` and
`execute_retract_home` are hard-coded `StepResult(False, …)` stubs, and the
routes containing them can never execute because five capabilities are
`unresolved`.

**First failing boundary: preflight**, before step 1 — `run_grasp` refuses on 2
blockers, `run_deliver` on 4, the new `run_deliver_task` on 5, each with zero
steps and zero hardware calls.

### 12.2 There was no full-task entry point — one was added

`grasp` and `deliver` were separate services and `deliver` begins with
`require_held_object`, so the framework task needed two calls. Added
**`/orchestrator/run_deliver_task`** (route `deliver_task`): the grasp steps
then the delivery steps, **28 steps, one sequencer, one execution lock**, never
calling another route service. Delivery-phase step names are disambiguated so
`start_step` resume is unambiguous.

Contract gap recorded, not invented around: `RunRoute.srv` carries one `target`
but the task needs two destinations.

### 12.3 A prerequisite nobody had done

`orchestrator`, `handover`, `g1_navigation`, `arm_planning` and `arm_control`
had **no generated Python service modules** in the workspace — only
`perception` did, so nothing could have talked to anything. All six now build;
the old `move_base_msgs` blocker is gone (`g1_navigation` dropped that
dependency).

### 12.4 Two defects found and fixed

* **Route output never crossed the service boundary.** `run_route` discarded the
  final step's detail, so `navigate_report` — whose whole job is to report a
  pose — returned `"completed (5 steps)"` with the detection lost. Fixed in
  `sequencer.py`; the payload now arrives.
* **A production module could not be imported on Python 3.8.**
  `joint_space_astar.py` assigned `State = tuple[int, int, int, int]` at
  runtime, which raises on the interpreter ROS Noetic runs, taking
  `edge_checked_astar` and `collision_aware_planner` with it. Fixed to
  `typing.Tuple`; guarded by `arm_planning/test/test_python38_compat.py`.

### 12.5 One defect recorded, deliberately not fixed

perception's `_default_R_base_opt()` is **not orthonormal** (`det = 1.0003760`,
`max|RᵗR − I| = 3.76e-04`), so `optical_to_base` applies ~188 µm/m of
scale-and-shear — ~0.19 mm at 1 m, immaterial beside perception's own 1.46 cm
error. Re-orthonormalising a calibration matrix is perception's call.

### 12.6 Results

**667 passed, 0 failed** — 29 cross-process ROS integration tests plus 638
offline (handover 155, orchestrator 225, navigation 119, arm_planning 74,
slam 47, perception 8, arm_control 10). **16 modules blocked, 2 skipped**:
`ultralytics`, `torch`, `trimesh`, `python-fcl` and `pytest` are all absent.

### 12.7 Verdicts

| Scope | Verdict |
|---|---|
| Full-task sequencing with simulated hardware | **PARTIAL** — 28 steps traverse in order, but only in dry run; 6 steps have no implementation |
| Cross-process ROS transport and contracts | **PASS** |
| Real algorithm / data processing | **PARTIAL** — perception geometry and arm FK real and correct; YOLO, IK, collision and the nav planner could not run |
| Physical robot execution | **NOT TESTED** — robot unavailable |

### 12.8 Still deferred to robot access

Everything in §11.6, unchanged, plus: force units/fields/rate/sign, camera
calibration and physical geometry correctness (distinct from the
frame-conversion *code*, which is now tested), and any motion.

The harness is retained and re-runs unchanged once the frame contract and the
three poses land: `orchestrator/test/integration/run_integration.sh`.

---

## 13. Session 8 — adapters wired to the real downstream interfaces

Full detail: **`docs/DATA_PIPELINE_VALIDATION.md` §10**, which supersedes §§1–9
of that report wherever they disagree. Tested at `7bf01cb` with all prior
uncommitted work preserved. No hardware commanded; nothing committed, merged or
pushed.

### 13.1 Correcting an earlier claim

Session 7 reported six adapters as having "no production implementation at
all". That was true of the *orchestrator*, but misleading about the system:
**every downstream service it needed already existed as a production node.**
`/arm/plan_to_pose` and `/arm/plan_grasp_from_point` are implemented in
`arm_planning/scripts/arm_planning_node.py` with their own readiness gating, and
`/arm/move_arm_joints` in `arm_control/scripts/arm_control_node.py`. **The gap
was wiring, not implementation.** No adapter was category B.

### 13.2 What changed

| Adapter | Before | Now |
|---|---|---|
| `plan_grasp` | stub | **real** `/arm/plan_grasp_from_point`; stores all four segments |
| `execute_segment` | stub | **real** `/arm/move_arm_joints`; sends the planner's own path |
| `plan_handover_pose` | stub | **real** `/arm/plan_to_pose`; blocked while `~poses/handover` is unset |
| `execute_stow` | stub | **real** plan + execute; blocked while `~poses/stow` is unset |
| `execute_retract_home` | stub | **real** plan + execute; blocked on the pose *and* on release completion |
| `to_planner_frame` | stub | **still a stub** — `frame_contract` unresolved |

Also added: `current_joints` capture from `/joint_states` (five arm joints, by
the planner's own names), `_check_joint_names` refusing a permuted joint order,
path-integrity checks before anything is sent, and `~poses` — which **ships
empty**.

### 13.3 What was deliberately not done

* **No preflight check removed or weakened.** `frame_contract`,
  `handover_pose`, `stow_pose`, `home_pose` and `release_completion` are still
  `unresolved`, so `grasp`, `deliver` and `deliver_task` are still refused
  before step 1.
* **No pose invented.** `~poses: {}`. Writing a number there would not clear the
  capability either; `arm_planning` owns validating reachability, workspace
  membership and collision-freedom.
* **No guessed posture.** Missing arm joints → NOT_READY naming them.
* **No dummy success.** Every new path validates length, finiteness, joint order
  and arm side, and propagates the downstream message verbatim.

### 13.4 Results

**683 passed, 0 failed.** 43 cross-process ROS integration tests (was 29; +14
exercising the new adapters over real ROS against arm-service doubles) and 640
offline (handover 155, orchestrator 227, navigation 119, arm_planning 74,
slam 47, perception 8, arm_control 10). 16 modules blocked, 2 skipped.

Two defects fixed: `test_no_kinematics_or_tf_maths` forbade the substring
`plan_to_pose(` and began false-positiving on a legitimate service call — the
guard now forbids *defining* a planner and positively asserts the three services
are wired; and a harness race where a call record could be snapshotted before it
propagated.

### 13.5 Verdicts

| Scope | Verdict |
|---|---|
| Full-task sequencing, simulated hardware | **PARTIAL (improved)** — 28 steps, correct order, one lock; five adapters now real; a real run still refused at preflight |
| Cross-process ROS transport and contracts | **PASS** — 43 tests, all six service types real and generated |
| Real planner / perception computation | **NOT VERIFIED** — the arm services here are **doubles**; `torch`/`trimesh`/`python-fcl`/`ultralytics` absent |
| Physical robot execution | **NOT TESTED** — robot unavailable |

**A test-double success is not a working production adapter.** The claim is
exactly this: the orchestrator now calls the real interfaces with
correctly-shaped requests and handles their responses and failures correctly.

### 13.6 PR-ready summary

> **Wire the orchestrator's implementable adapters to the real arm services**
>
> Five of six hard-coded failing adapters are replaced with real calls to
> existing production services. The sixth (`to_planner_frame`) stays NOT_READY
> because the frame contract is unresolved.
>
> * `plan_grasp` → `/arm/plan_grasp_from_point`, keeping all four segments
> * `execute_segment` → `/arm/move_arm_joints`, sending the planner's own path
> * `plan_handover_pose`, `execute_stow`, `execute_retract_home` →
>   `/arm/plan_to_pose` + `/arm/move_arm_joints`, blocked while the poses are unset
> * new `/orchestrator/run_deliver_task`: the whole framework task as 28 steps
>   under one execution lock, never calling another route service
> * joint order, units, arm side, planner output and failure messages are
>   preserved and checked across every boundary
> * `~poses` ships empty and **no preflight check was removed**: the five
>   unresolved capabilities still refuse the route before step 1
>
> Adds an isolated cross-process ROS integration harness (own master on :11399,
> hardware-facing doubles with real generated types, no SDK imported) —
> **43 tests**. Fixes a route-output wiring defect, a Python 3.8 import defect in
> `arm_planning`, an over-broad safety guard and a harness race. Records, without
> fixing, a non-orthonormal perception extrinsics matrix (~188 µm/m).
>
> **683 tests pass, 0 fail.** Physical execution remains untested: the robot is
> unavailable, and the arm services in these tests are doubles.

### 13.7 Still blocked, in dependency order

1. **`frame_contract`** — `base_link → torso_link`, plus the 10 mm `torso_link`
   disagreement. Unblocks `plan_grasp` and the four grasp segments. **First.**
2. **Three validated poses** from `arm_planning`. The adapters are ready.
3. **`release_completion`** — `hand_control` must expose settling.
4. **`/joint_states` must carry the five arm joints** (only the waist is confirmed).
5. **Deployment deps** — `torch`, `trimesh`, `python-fcl`, `ultralytics`.

---

## 14. Session 9 — boundary 3 implemented; the 10 mm resolved

Full detail: **`docs/DATA_PIPELINE_VALIDATION.md` §11**, which supersedes §10 and
§§1–9 of that report. Tested at `7bf01cb` with all prior uncommitted work
preserved. No hardware commanded; nothing committed, merged or pushed.

### 14.1 The frame conversion is now production code

New service **`/arm/convert_observation`** (`arm_planning/ConvertObservation`),
maths in `arm_planning/src/arm_planning/frame_conversion.py` (pure, no ROS), TF
client in `arm_planning_node.py`. The orchestrator's `to_planner_frame` passes
the point, its frame and its timestamp through that interface and **performs no
TF mathematics** — verified by a source scan (`tf2_ros`, `lookupTransform`,
`quaternion`, `math.cos`, `math.sin`, `CONVENTION_OFFSET`: 0 occurrences).

### 14.2 The 10 mm was a convention difference, and is resolved as one

| chain | mode_10 (live TF) | g1_23dof (planner) |
|---|---|---|
| `pelvis → torso_link` | 0.044 | 0.054 |
| `torso_link → shoulder_pitch` | 0.24778 | 0.23778 |
| `pelvis → shoulder_pitch` | **0.29178** | **0.29178** |

The sums are identical, so neither URDF is wrong — they place the same datum
10 mm apart. Resolution: a declared convention translation of **−0.010 m in z**
applied after the TF lookup, **exact at every waist angle** because the waist
rotates about z and both joint origins have `rpy = 0 0 0`. **No TF frame was
redefined and neither URDF was modified.**

### 14.3 Frame readiness did NOT clear

`frame_contract` was **split**: `planner_frame_conversion` (resolved, a service)
and `perception_extrinsics` (**still unresolved** — wrong-robot provenance,
waist-coupled camera, and a non-orthonormal rotation matrix). A correct
base→planner transform cannot repair a point computed with the wrong camera
extrinsics, and the code says so.

```
grasp  before: frame_contract, stow_pose
       after:  perception_extrinsics, stow_pose
```

### 14.4 The perception timestamp limitation

`GetObjectPosition.srv` has **no stamp field**. Addressed explicitly rather than
hidden: `detect_object` records a **receipt time**, `to_planner_frame` sends it
with `stamp_is_observation_time=false`, and the conversion refuses when the waist
moved across that window. The guard is **stateless** (two lookups from the same
request's TF history). The interface change still needed is a `time stamp` field
on that response, which is perception's contract to change.

### 14.5 Two audit findings reconciled

* **`RunRoute` now carries `destination`.** A distinct `navigate_to_destination`
  action drives the delivery leg, so it cannot reuse the pickup waypoint;
  `deliver_task` is refused up front with an empty destination.
* **Spec 2.4 ordering defect fixed.** `execute_stow` used to plan *and* execute
  at step 14 — after `record_held_object`, i.e. **after the hand had closed**.
  Now `plan_stow` runs at step 7, before any grasp motion, planning from
  `plan_grasp`'s own `final_joints`; `execute_stow` only replays it.

### 14.6 Results

**729 passed, 0 failed** — 60 cross-process ROS integration tests (was 43) and
669 offline (handover 155, orchestrator 227, arm_planning 103, navigation 119,
slam 47, perception 8, arm_control 10). 16 modules blocked, 2 skipped.

Four defects of mine found by the tests and fixed: a `NameError` from computing
`destination` in `run()` but using it in `_execute()`; `waist_yaw_rad` reporting
the inverse transform's yaw; a stateful, order-dependent waist guard; and a dead
`execute_stow` backend action. One method note worth keeping: **"stop publishing
TF" does not empty a tf2 buffer** (it prunes on insert, not on read), so
staleness must be tested with an out-of-range stamp.

### 14.7 Verdicts

| Scope | Verdict |
|---|---|
| Sequencing with test doubles | **PARTIAL (improved)** — 29 steps, 12 ordering requirements + spec 2.4; zero stubs left; real run still refused at preflight |
| Actual ROS transport | **PASS** — 60 tests, four real processes incl. the real arm_planning node |
| Real planner / perception computation | **PARTIAL** — the frame conversion is real and correct; the planner and executor are **doubles**; YOLO still cannot run |
| Physical robot execution | **NOT TESTED** — robot unavailable |

### 14.8 Blockers

**Removed:** `frame_contract` as a monolithic blocker.

**Remaining:** 1. `perception_extrinsics` (perception) — **first blocker**;
2. a `stamp` field on `GetObjectPosition.srv`; 3. the three validated poses
(`handover`, `stow`, `home`) — adapters are ready, `~poses` ships empty;
4. `release_completion`; 5. the five arm joints on `/joint_states`;
6. `torch`, `trimesh`, `python-fcl`, `ultralytics`.

### 14.9 PR-ready summary

> **Implement the perception→planner frame conversion; resolve the 10 mm**
>
> New `/arm/convert_observation` in arm_planning converts an observed point from
> `base_link` into the planner's `torso_link`, resolving the two URDFs' 10 mm
> `torso_link` disagreement as the convention difference it is: the
> `pelvis → shoulder_pitch` sums are identical in both files, so a declared
> −0.010 m z translation is applied after the TF lookup, exact at every waist
> angle. No TF frame redefined, no URDF modified.
>
> * conversion maths is pure and offline-testable (29 tests with independent
>   expectations at zero and nonzero waist, both transform directions, the 10 mm,
>   and stale/missing transforms)
> * the orchestrator passes the observation through and does **no** TF maths
> * `frame_contract` split: the conversion resolves, `perception_extrinsics`
>   stays **unresolved** (wrong-robot provenance, waist-coupled camera,
>   non-orthonormal matrix), so grasp is still refused
> * perception's missing response timestamp is handled with an explicit
>   receipt-time upper bound plus a stateless waist-motion guard, and the needed
>   `.srv` change is recorded
> * `RunRoute` gains `destination`; the delivery leg can no longer reuse the
>   pickup waypoint
> * **spec 2.4 ordering defect fixed**: the stow recovery path is now planned
>   before any grasp motion instead of after the hand closes
> * confirmed hand offset `(0.215, 0.06, 0.025)` untouched; checkpoints still
>   recorded as stale (54 mm) and collision masks still unproven against it —
>   frame conversion is **not** evidence about either
>
> **729 tests pass, 0 fail.** Physical execution remains untested; the planner
> and executor in these tests are doubles.

---

## 15. Session 10 — standing-pose waypoint recorder

Teleoperate the robot to a spot, press `A`, and navigation can return it to that
exact position and heading. Added to `slam/`; detail in `slam/README.md` §7b.

### 15.1 Why a new store rather than semantic.yaml

`semantic.yaml` holds **object centres**, and `/navigation/navigate` stands
`approach_distance` away from them. A recorded standing pose is the opposite: it
is already the spot the robot stood on, so it must go through navigation's
**exact-pose** interface, `/move_base_simple/goal`, with **no further offset**.
Offsetting a standing pose would park the robot a metre from the place that was
deliberately chosen. Two meanings, two files, two goal paths.

| | file | goal path | offset |
|---|---|---|---|
| object centre | `semantic.yaml` | `/navigation/navigate` | yes |
| standing pose | `standing_waypoints.yaml` | `/move_base_simple/goal` | **none** |

### 15.2 Files

| File | |
|---|---|
| `slam/src/g1_slam/standing_store.py` | new — pure store (no rospy): atomic writes, map identity, unique naming |
| `slam/scripts/record_waypoints.py` | new — `A` save, `L` list, `Q` quit |
| `slam/scripts/goto_waypoint.py` | new — exact-pose goal, `_list:=true`, `_dry_run:=true` |
| `slam/test/test_standing_store.py` | new — 32 tests |
| `slam/test/test_waypoint_scripts.py` | new — 29 tests |
| `slam/CMakeLists.txt`, `slam/README.md` | scripts installed; §7b added |

### 15.3 Guarantees

* **`A` only records.** The recorder contains zero occurrences of
  `rospy.Publisher`, `move_base_simple`, `enable_motion`, `Twist`, `cmd_vel` or
  `NavigateToObject` in its code — asserted by a test that strips the docstring.
* **No root keyboard access.** `termios` / `tty.setcbreak` on the terminal's own
  stdin; no `keyboard` module, no `/dev/input`, no `sudo`. Exits cleanly if
  stdin is not a TTY.
* **Never silent odometry.** A press is refused unless `/slam/manager/state` is
  True, `map → pelvis` exists and is fresher than `_max_tf_age`, and the
  transform's own frames are the configured ones — a `camera_init` parent is
  rejected explicitly.
* **Map identity.** Records carry the map's `map_name` + `created_at` from its
  `manifest.yaml`. A different *or re-made* map is refused, on record and again
  before a goal is sent.
* **Atomic writes** (temp + fsync + rename), always rewriting from what is on
  disk, so existing and hand-edited records survive.
* **Motion gates untouched.** `goto_waypoint.py` never calls
  `/navigation/enable_motion`; it reports `/navigation/motion_enabled` so a
  closed gate explains itself.
* **No new localisation algorithm.** The optional `.pcd` snapshot is a human
  reference; nothing reads it back. The relocalizer still owns localisation.

### 15.4 Restart validity

`map` is defined by the **saved map**, not the session: FAST-LIO2 re-creates
`camera_init` every power-on, and the relocalizer publishes `map → camera_init`
so `map` means the same thing again. Records therefore store map-frame
`x, y, yaw` plus the map identity; after a restart, once localisation to that
same saved map succeeds, the coordinates denote the same physical spot.

### 15.5 Tests

**61 new, all passing.** `slam` went 47 → **108**. Covered: recording, reload,
hand-edited records surviving an append, invalid/stale/odometry-frame/zero-stamp
TF, map mismatch (including a map re-made under the same name), goal mapping
with exact equality (proving no offset), atomicity under a failed `os.replace`,
and a drift guard that reads navigation's own `nav_client.py` so the mirrored
`ACTIVE`/`TERMINAL` sets cannot diverge (g1_slam must not import g1_navigation —
that would be a dependency cycle).

Verified end to end on an isolated master: `_list:=true`, `_dry_run:=true`, and
a refused map mismatch.

### 15.6 Known gap

**Standing poses are not yet consumable by `run_deliver_task`.** Its `target` /
`destination` resolve through `semantic.yaml` via `lookup_object` and then apply
`default_approach_distance`, so passing a standing-pose name fails at the
lookup — and copying the pose into `semantic.yaml` would reintroduce the 0.6 m
offset. Driving to a standing pose is `goto_waypoint.py` for now.

Closing it needs the orchestrator to distinguish the two kinds of target. The
open decision is **how**: an explicit `target_kind` field on `RunRoute`
(preferred — a naming convention that silently switches goal interface is a trap)
or a prefix convention.
