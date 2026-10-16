# Grasp-and-deliver: status classification and proposed delivery flow

**Status: the sequencing in section 3 is now IMPLEMENTED and exercised offline.**
Real execution of grasp and delivery remains NOT_READY: three steps are blocked
on poses that do not exist, and the frame contract is unfinished.

| Category | What |
|---|---|
| **A. exercised offline with fake backends** — the sequencing logic is real and run, but no real adapter was involved | the 14-step `deliver` order; `require_held_object`; `record_held_object`; the single-token handshake; `await_pull` consuming the event **while the route holds the execution lock**; `disarm_monitor` before `release_hand`; the command-ack / settled / transfer distinction; `release_permitted` refusing a resume into release; `retraction_allowed` refusing on a bare ack and while `UNCERTAIN`; `mark_done`; unconditional cleanup incl. disarming the monitor; token staleness. **34 tests** in `test_delivery_flow.py` drive the real `run_route()` with a fake backend and injected clock. |
| **B. exercised offline through an IN-PROCESS service bus** — both production nodes' Python objects, real detector, real handlers, but **rospy is a stub**: no ROS master, no serialisation, no network, no `.srv` marshalling, no concurrency, no timeouts, no hardware | a synthetic tactile pull driving the **real** detector to a verified trigger, the **real** monitor echoing the orchestrator's token into the **real** validator; the handshake window, arm failure, echo mismatch, cancellation, re-arming, old tokens, duplicates, disarm, timeout, tokenless, wrong side, detection-only. **24 tests** in `test_monitor_integration.py`. **This is logic validation, not ROS transport validation and not hardware validation.** A real deployment additionally has to survive message serialisation, service-call latency and failure, node restarts and clock behaviour, none of which these tests touch. |
| **C. real adapters that have never run against their service** | `navigate`, `enable_motion`, `execute_segment` (arm_control), `release_hand` (hand_control), `arm_monitor`/`disarm_monitor` over ROS, `detect_object` (perception). The client code exists and the launch files resolve; **no call has ever reached the real server**, because none of those nodes runs on the bench. Treat as unverified. |
| **D. still missing entirely** | the **handover**, **stow** and **home** poses — three steps fail with an explicit `NOT_READY`; the perception→planner TF conversion; and physical verification of object transfer, which nothing in this workspace can observe. |

Categories A and B are code this branch runs. Category C is code this branch
*wrote* but has never executed end to end. Do not read a green test suite as
evidence for category C.

---

## 1. Status of every delivery step

Three categories, as distinct as the evidence allows. "Implemented and
exercised" means the production code path was run and observed, not that an
interface exists or a unit test passes.

### 1.1 Implemented and exercised

| Capability | How it was exercised |
|---|---|
| Multi-channel tactile detector state machine | 157 offline tests, synthetic frames, time injected |
| Release-request **validation** logic | 31 offline tests; live rejection observed against a bare roscore |
| Non-blocking execution lock (no deadlock, no stall) | test holds the lock and validates concurrently; 12-thread race accepts exactly one |
| Dry-run traversal of all four routes | live: 5/5 `navigate_report` steps, 12/12 `grasp` steps, `motion_issued=False` |
| Preflight blocker collection | live: 6 blockers for `navigate_report`, 10 for `grasp`, zero steps executed |
| Unconditional cleanup (`cancel_navigation`, `disable_motion`) | tests cover success, step failure, raised exception, blocked start |
| `arm_planning` planning services (as services) | built, imported, advertised; refuse with `NOT_READY` |
| Detection-only mode constructs no release client | test drives a synthetic pull to a confirmed trigger and asserts zero clients built |

### 1.2 Implemented but hardware-blocked

| Capability | Blocker |
|---|---|
| `navigate_report` route (real run) | localization, map, `/joint_states`, navigation, perception all absent on the bench |
| `arm_planning` plan generation | `torch`/`trimesh`/`fcl` absent; two `*_wrist_roll_rubber_hand.STL` meshes missing |
| Force pull detection end-to-end | **updated 2026-10-14:** the hand (BrainCo Revo2) and the SDK are confirmed, and the Python 3.8-vs-≥3.9 conflict is retired. What remains is the force **schema** — the SDK call, field names, channel→finger mapping, **unit** and rate. The threshold is no longer a blocker: 3 N per channel is the requested setting. |
| `arm_control` execution | `unitree_sdk2py` absent; never invoked by any reachable route |
| Hand open/close | SDK available, but no code here has run against it — orchestrator-to-SDK integration is unverified |
| `grasp` route (real run) | the `pelvis`/`base_link`/`torso_link` frame contract is unresolved |

### 1.3 Missing or unreachable

**The active delivery route does not consume handover events. End-to-end
delivery is incomplete.**

| Step (spec C / E) | Status | Evidence |
|---|---|---|
| C1 Confirm object held + hand side | **missing** | `HandoverState.begin()` has 0 callers |
| C2 Arm reaches handover pose, hand closed | **missing** | no arm-motion step in `deliver`; it does not even require `arm_planning`/`arm_control`; **no handover pose is defined anywhere** |
| C3 Wait ~1 s for settling | **missing** | no step, no constant |
| C4 Arm the monitor | **dry-run only** | step exists; real run blocked at preflight |
| C5 Baseline and watch | **blocked** | detector done; adapter disabled |
| C6 One request per verified pull | **implemented, blocked** | one-shot guard in the monitor; no tactile data |
| C7 Validate before release | **implemented but unreachable** | always rejects, because C1 never runs |
| E1 Disarm the monitor | **missing from the delivery path** | exists only in the orphaned `auto_release` route, and *after* release |
| E2 Call `/hand_{side}/release` | **missing from the delivery path** | orphaned route only |
| E3 Confirm release succeeded, then update state | **missing** | `_hand_call` checks `response.success`, but `mark_released()` has 0 callers |
| E4 ~1.5 s for the recipient to secure | **missing** | no constant, no step |
| E5 Validated retract / home | **missing** | no retract or home step in `deliver`; no home pose exists |

Also unreachable: `consume_pending()` (0 callers), the whole `auto_release`
route (no service binds it), and `final_step="await_pull"` names a step that
exists in no route.

---

## 1b. Force units are not established by reading the device

A field list of the shape `TouchFingerItem` — `normal_force1/2/3`,
`tangential_force1/2/3`, `tangential_direction1/2/3`, `self_proximity1/2`,
`mutual_proximity`, `status`, all `builtins.int` — would settle the *schema*.
**It has not been read off this robot's SDK and must be confirmed there**; it is
recorded here as the expected shape only. Either way it does **not** settle the
*units*.

Reading numbers off the device does not establish physical units either. A
stream of integers tells you the range and the noise, not whether a count is
newtons, millinewtons, a raw ADC value, or a vendor-scaled index — nor whether
the mapping is linear, nor where zero is. `tangential_direction` is an `int`
with no documented range, so even treating it as an angle is an assumption.

Units require **vendor documentation or a calibration against a known
reference** (a mass, a force gauge). Until then the detector treats these as
`force_like` counts and all thresholds are *counts*, valid only for the
specific hand they were recorded on. That is why `release_threshold` and
`stable_tolerance` have no defaults.

## 2. Unresolved inputs — checked, not assumed

Searched `arm_planning/config`, `arm_planning/scripts`, `arm_control`,
`hand_control/config`, `orchestrator`, `handover` for
`home|rest|stow|neutral|park|tuck`: **zero hits.** The complete set of
pose-defining configuration is `grasp_geometry` (7 keys, all grasp-relative)
plus `training_workspace`.

| Input | State | Why it cannot be invented |
|---|---|---|
| **Home pose** | **absent** | The only candidate is `arm_control`'s `left_target`/`right_target = [0,0,0,0,0]`, which is a shared-memory *initialisation seed*, not a validated home. Moving there also needs a collision-checked plan, and `Execute_Waypoint_Path` does velocity limiting only, no collision checking. |
| **Handover pose** | **absent** | Must be reachable, in the trained workspace, comfortable for a human, and collision-free. All four are physical judgements. |
| **Stow / carry pose** | **absent** | Needed to carry an object while walking without the arm swinging into the body or the environment. |
| **Grasp retract path — NOT reusable for handover** | checked | `retract = lift.copy()`, `x = retract_target_x (0.15)`, `rz = atan2(y, x)` — so it inherits y and z from the **lift** pose, which derives from `object_xyz`. And `_cartesian_segment(..., start_pose=poses['lift'], ...)` means it **starts at the lift pose**, just above where the object sat on the table. After a handover the arm is at the handover pose, so replaying it would command a jump to the lift pose. It is an object-relative grasp segment, not a homing move. |
| `nominal_environment_geometry` | `provisional: true` | Flagged provisional in the config itself; table heights feed the training workspace. |
| Frame contract | **navigation's half CLOSED** | PR #10 / `docs/FRAME_CONTRACT.md`: `base_link` is an identity alias of `pelvis` at **every** waist angle — the pelvis is the waist's *parent*, so it never rotates with the waist — published once by `system_bringup` with `with_base_link` now **on by default**. `base_link` carries **no** zero-waist limitation; that limitation belongs entirely to perception's hard-coded camera extrinsics, see (c) below. Two items remain, both ours or perception's. |
| Collision meshes | missing | two `*_wrist_roll_rubber_hand.STL` |
| Tactile schema + thresholds | unconfirmed | see `handover/README.md` |

### What the frame contract leaves open

PR #10 closes navigation's side and names the two remaining items itself:

**(a) The 10 mm `torso_link` difference — a frame-definition question, not
necessarily a wrong file.** `tools/check_urdf_agreement.py` reports:

```
slam          pelvis -> torso_link at zero waist yaw = (-0.0040, 0.0000, 0.0440)
arm_planning  pelvis -> torso_link at zero waist yaw = (-0.0040, 0.0000, 0.0540)
[DIFFER] dz=0.0100 m (tolerance 0.0010 m)
```

An earlier version of this document said "correct the wrong URDF". That
overstated it. The evidence does not establish that either file is physically
wrong:

* the **net** `pelvis -> left_shoulder_pitch` is **identical** in both
  (`0.29178 m`), and so is `pelvis -> d435`. Both files therefore describe the
  same physical arm and the same camera mount;
* they differ only in where the intermediate **label** `torso_link` is placed,
  and each is internally consistent with its own consumers —
  `arm_kinematics.yaml` matches its URDF (`torso_height - pelvis_height = 0.054`)
  and the collision probe hard-codes the matching `-0.054`;
* so this is most likely **two different modelling conventions for the same
  robot**, not a manufacturing or transcription error.

That matters for the fix. A tape measure settles it **only if** `torso_link`'s
origin corresponds to an identifiable physical feature (a joint axis, a machined
face). If it is an arbitrary modelling datum, measuring cannot say which z is
"right", and the correct resolution is instead to **declare which convention
each consumer uses and convert explicitly**, or to align the two files on one
stated convention.

Either way the practical consequence is unchanged: a planner target derived
through TF (whose `torso_link` comes from the slam URDF) is offset by up to
10 mm in z from what `arm_kinematics.yaml` expects, which is the size of the
planner's own tolerance. **Question for arm_planning: is `torso_link`'s origin a
physical feature that can be measured, or a modelling datum?** That decides
whether this is a measurement or a convention declaration.

**(b) Implementing that conversion.** Still ours, and it belongs in
`arm_planning` (the frame's owner), not in `orchestrator`.

**It must not be wired up before (a) is settled.** The TF tree's `torso_link`
comes from the slam URDF (z = 0.044); the planner's geometry, its collision
probe and its trained checkpoints all use the other convention (z = 0.054).
Feeding a TF-derived target straight into the planner therefore introduces a
systematic 10 mm z offset — at the planner's own tolerance — in a direction
nothing in the pipeline would flag. **The conversion is only safe once the TF
`torso_link` and the planner's `torso_link` are known to be the same
convention**, either by aligning the two files or by applying a declared,
tested offset between them. Until then this stays a preflight blocker.

**(c) A separate, independent requirement — perception's moving camera.**
This is *not* satisfied by resolving (a) or (b): even with both URDFs aligned
and the conversion implemented, the input to that conversion would still be
wrong whenever the waist is turned.
`base_link ≡ pelvis` is exact for the *frame*. But `perception` computes the
object position with a **hard-coded** optical→base transform (`Extrinsics()` in
`m1_perception.py`, "not TF"), and the D435 is mounted on `torso_link`
(`d435_joint`'s parent), which **does** rotate with the waist. So at nonzero
waist yaw the camera has physically moved while the hard-coded transform assumes
it has not: the frame label stays correct and the reported point is wrong.
Either perception must take its camera pose from TF, or the waist must be held
at ≈0 whenever perception is used. **Question for the perception owner.** Until
answered, a grasp planned from a perception point while the waist is turned is
not trustworthy.

### Runtime wording corrected

`orchestrator/src/orchestrator/capabilities.py`'s `FRAME_CONTRACT` text used to
say something false — that "slam refuses to publish `pelvis -> base_link`" and
that navigation "has not decided". `docs/FRAME_CONTRACT.md` asked the
orchestrator team to correct that text once (a) and (b) were done. (a) and (b)
are **not** done, so the capability **stays unresolved** and `grasp` stays
NOT_READY; only the wording changed. It now states that `base_link` is an
identity alias of `pelvis` at every waist angle, that the 10 mm may be two
different frame *definitions* rather than a wrong URDF, and that perception's
extrinsics are a separate requirement that owns the zero-waist limitation.
That edit is held for review rather than applied silently.

**Questions for the arm_planning / arm_control owners** (each blocks a step
above, and none can be answered from the repo):

1. Is there a validated **home** pose? If it is all-zero joints, say so
   explicitly, and confirm whether reaching it must be planned and
   collision-checked or may be a direct interpolated move.
2. What is the **handover** pose, in `torso_link`, and was it validated as
   reachable and collision-free?
3. Is there a **stow/carry** pose for walking with a held object, or should the
   arm hold the lift/retract pose while navigating?
4. Should post-handover retraction be a *planned* path (`plan_to_pose` to the
   home pose) or a *replayed* fixed trajectory? The former needs the frame
   contract and meshes; the latter needs a recorded, reviewed trajectory.

---

## 3. Proposed complete task flow

Two routes, one shared task state, one lock owner at a time.

```
run_grasp                                        run_deliver
─────────                                        ───────────
 1 await_localization                             1 await_localization
 2 enable_motion                                  2 require_held_object      <- reads task state
 3 navigate(table)                                3 enable_motion
 4 detect_object                                  4 navigate(destination)
 5 to_planner_frame        [BLOCKED]              5 plan_handover_pose       [BLOCKED]
 6 plan_grasp (all 4 segments, before motion)     6 execute_handover_pose
 7 execute_to_pregrasp                            7 settle(handover_settle_sec)
 8 open_gripper                                   8 arm_monitor
 9 execute_approach                               9 await_pull               <- consume_pending()
10 close_gripper                                 10 disarm_monitor
11 execute_lift                                  11 release_hand
12 record_held_object     -> task state          12 confirm_released         <- checks service success
13 execute_stow                                  13 clear_held_object        -> task state
   DONE (object held)                            14 settle(recipient_settle_sec)
                                                 15 execute_retract_home     [BLOCKED]
                                                    DONE
```

Step 6 keeps the existing rule that the whole grasp is planned before anything
moves. Step 12 is the only writer of "object held"; step 13 the only clearer.

### 3.1 Responsibilities

| | `run_grasp` | `run_deliver` |
|---|---|---|
| Ends with | an object **held**, arm stowed | an object **released**, arm home |
| Writes task state | `begin(side, label)` on success at step 12 | `mark_awaiting_pull`, `mark_releasing`, `mark_released` |
| Requires a held object | no — refuses if one **is** already held | **yes** — refuses if none is held |
| Arms the monitor | never | step 8 only, after settling |
| Opens the hand | `open_gripper` before grasping (empty hand) | `release_hand` only via a consumed event |
| Navigation | to the table | to the destination |

Neither route releases on a timer, and neither arms the detector while the arm
is moving: `arm_monitor` comes strictly after `execute_handover_pose` and
`settle`.

### 3.2 Shared task state and lock ownership

`HandoverState` becomes the single task record, extended with the held-object
fields it already implies. One rule: **the execution lock is held for the whole
of a route, and `HandoverState` is guarded by its own mutex.** The service
callback touches only the latter.

* `run_grasp` and `run_deliver` each take the execution lock for their whole
  duration, so they cannot interleave.
* `await_pull` (step 9) **blocks inside the route, while holding the lock**,
  polling `consume_pending()` until an event arrives or the timeout expires.
  This is the point of the design and the part that does not exist today.
* `/orchestrator/run_auto_release` continues to only validate and record. It
  never takes the execution lock and never moves anything.
* The orphaned `auto_release` route is **deleted**; its steps move into
  `run_deliver` in the specified order (disarm before release).

### 3.3 Timeout and cancel

| Event | Behaviour |
|---|---|
| `await_pull` timeout | **no release.** Route fails with `TIMEOUT`, `abandon()` leaves the object **held**, monitor disarmed in cleanup. Never reported as a handover. |
| Cancel during `await_pull` | same as timeout: disarm, no release, object stays held |
| Cancel during `navigate` | existing `cancel_navigation` + `disable_motion` cleanup; object stays held |
| `release_hand` returns failure | **object state becomes `UNCERTAIN`, not released.** Retraction is **not** executed, because the object may still be in the hand or falling. Route fails and asks for human inspection. |
| `confirm_released` cannot confirm | same as above |
| Any exception | `finally` cleanup already disarms navigation and the motion gate; extend it to disarm the monitor |

**An accepted release request is not a completed release.** Acceptance only
means the event was validated and recorded; only `confirm_released` observing a
successful `/hand_{side}/release` response may clear the held object.

### 3.4 Resume rules

`start_step` resume must never invent state.

* Resuming any `run_deliver` step at or after `require_held_object` **re-runs
  the state check**: if `HandoverState` does not record a held object, the
  route refuses. Resume cannot fabricate one.
* Resuming at or after `await_pull` requires phase `AWAITING_PULL`; otherwise
  it refuses rather than waiting for an event that can never be validated.
* Resuming at or after `release_hand` is **refused outright** unless a
  validated event was already consumed in the current session, so a resume
  cannot open the hand without a pull.
* Resuming into `execute_retract_home` while the object state is `UNCERTAIN`
  is refused.
* Resume never skips `settle` steps, because they exist to protect the
  detector and the recipient, not to pad the sequence.

---

## 4. Files the implementation would touch

| File | Change |
|---|---|
| `orchestrator/src/orchestrator/routes.py` | rewrite `DELIVER`; extend `GRASP` with `record_held_object`, `execute_stow`; delete `AUTO_RELEASE` |
| `orchestrator/src/orchestrator/handover_state.py` | add held-object record + `UNCERTAIN` state; add `require_held_object()` and resume guards |
| `orchestrator/src/orchestrator/sequencer.py` | add the backend actions for the new steps; teach `run_route` the blocking `await_pull` wait |
| `orchestrator/src/orchestrator/capabilities.py` | `deliver` must now require `arm_planning`, `arm_control`, `hand`, and the frame contract |
| `orchestrator/scripts/orchestrator_node.py` | implement the new `RosBackend` actions; wire task-state writes; extend `finally` to disarm the monitor |
| `orchestrator/config/orchestrator.yaml` | `handover_settle_sec`, `recipient_settle_sec`, `await_pull_timeout_sec` |
| `orchestrator/test/test_delivery_flow.py` | **new** — the integration tests in section 5 |
| `orchestrator/README.md`, this file | status updates |

Not touched: arm planning algorithms, URDFs, calibration, motor limits, hand
presets, navigation, perception.

Note `deliver` gaining `arm_planning` + the frame contract means **it inherits
the frame blocker**: once wired, `run_deliver` is NOT_READY for a real run on
the same grounds as `run_grasp`, until the 10 mm measurement, the meshes and
the conversion are resolved. That is correct, not a regression — it reflects
that moving the arm to a handover pose needs a planner.

---

## 5. Proposed integration tests

These run the **production route** through `run_route()` with fake backends and
an injected pull event — not the detector, and not a hand. They are integration
tests of the sequencing, still offline, still no hardware.

| Test | Asserts |
|---|---|
| `test_complete_sequence_call_order` | the fake backend records exactly `require_held_object, enable_motion, navigate, plan_handover_pose, execute_handover_pose, settle, arm_monitor, await_pull, disarm_monitor, release_hand, confirm_released, clear_held_object, settle, execute_retract_home` — in that order, with disarm strictly before release |
| `test_await_pull_timeout_produces_no_release` | no `release_hand` call; route fails `TIMEOUT`; object still held; monitor disarmed |
| `test_cancel_during_await_pull_produces_no_release` | same, via cancel rather than timeout |
| `test_release_failure_preserves_held_state_and_blocks_retraction` | `release_hand` returns failure ⇒ no `execute_retract_home`, no `clear_held_object`, state `UNCERTAIN`, route fails |
| `test_duplicate_request_causes_no_extra_motion` | two accepted-then-replayed events ⇒ exactly one `release_hand` |
| `test_stale_request_is_ignored` | an event from a previous session ⇒ no `release_hand` |
| `test_event_is_consumed_while_the_task_owns_the_lock` | during `await_pull`, `lock.busy` is true and the holder is the deliver task; the injected request still validates |
| `test_resume_cannot_invent_a_held_object` | `start_step=await_pull` with no held object ⇒ refused, zero steps |
| `test_resume_into_release_without_a_consumed_event_is_refused` | `start_step=release_hand` ⇒ refused, no `release_hand` call |
| `test_accepted_is_not_completed` | acceptance alone leaves the object held until `confirm_released` succeeds |
| `test_dry_run_traverses_the_whole_new_route_without_motion` | all 14 steps, `motion_issued=False`, no client constructed |

Fake backends record call order and return scripted failures, exactly as
`test_sequencer.py` already does for `RecordingBackend`.

---

## 6. Order of work

1. This document, reviewed. **← current step**
2. Answers to the four owner questions in section 2.
3. Implement sections 3–4 with the section 5 tests, `deliver` NOT_READY for
   real runs (it inherits the frame blocker).
4. Clear the cheap blockers: the two meshes, then the frame contract.
5. Only then supervised hardware testing, with the tactile acceptance targets
   in `handover/README.md` as the gate.
