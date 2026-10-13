# Grasp-and-deliver: status classification and proposed delivery flow

**Status: PROPOSAL. Nothing in section 3 onward is implemented.** Real execution
of grasp and delivery remains NOT_READY.

This document exists because an earlier description of the release path
overstated it. Section 1 classifies every step by what is actually true;
sections 3–6 propose the missing design without implementing it.

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
| Tactile pull detection end-to-end | no tactile topic exists; SDK unverified, Python 3.8 vs ≥3.9 conflict; thresholds uncalibrated |
| `arm_control` execution | `unitree_sdk2py` absent; never invoked by any reachable route |
| Hand open/close | `bc_stark_sdk` absent |
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
| Frame contract | unresolved | see `orchestrator/README.md` |
| Collision meshes | missing | two `*_wrist_roll_rubber_hand.STL` |
| Tactile schema + thresholds | unconfirmed | see `handover/README.md` |

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
the same grounds as `run_grasp`, until the frame contract and meshes are
resolved. That is correct, not a regression — it reflects that moving the arm
to a handover pose needs a planner.

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
