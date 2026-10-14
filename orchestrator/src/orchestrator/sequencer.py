"""Step traversal. Pure control flow over an injected backend.

The sequencer decides ORDER, RETRIES, FAIL-FAST and CLEANUP. It never performs
a robot action itself: every step is dispatched to a backend method named by
the step's `action`. Two backends exist:

  * `DryRunBackend` -- records calls, performs nothing, constructs no clients.
  * the node's RosBackend -- real service proxies, built only when a route is
    about to execute for real and preflight has passed.

That split is what makes a dry run safe by construction rather than by a flag
checked in a dozen places: in a dry run no robot client object is ever created.

Cleanup is unconditional. `cancel_navigation` and `disable_motion` run in a
finally block on every exit path -- success, failure, exception, unknown step --
so a route can never leave the motion gate open or navigation chasing a goal.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from orchestrator import routes as routes_mod


@dataclass
class StepResult:
    ok: bool
    detail: str = ""
    #: True when this call actually sent a command to hardware.
    motion_issued: bool = False


@dataclass
class RouteContext:
    """Carried through a route; steps read and write it instead of globals."""

    route: str
    target: str = ""
    side: str = "right"
    dry_run: bool = True
    #: Filled by detect_object; consumed (in principle) by to_planner_frame.
    detection: dict = field(default_factory=dict)
    #: Filled by plan_grasp: segment name -> flat path + count.
    plan: dict = field(default_factory=dict)
    notes: list = field(default_factory=list)
    #: The task record (orchestrator.handover_state.HandoverState). The route
    #: reads and writes it; the release service callback only writes pending
    #: events into it. None in routes that do not touch handover state.
    handover: object = None
    #: Session token issued by mark_awaiting_pull, quoted by a release request.
    session_token: str = ""
    #: Label/side of the object a grasp recorded, for a later delivery.
    held_label: str = ""
    #: Set by confirm_released: what the service result actually established.
    release_outcome: str = ""


@dataclass
class RouteResult:
    success: bool
    message: str
    steps_completed: list = field(default_factory=list)
    steps_skipped: list = field(default_factory=list)
    blockers: list = field(default_factory=list)
    final_step: str = ""
    motion_issued: bool = False
    dry_run: bool = True


@dataclass
class RetryPolicy:
    """Attempts per action name. Anything unlisted gets exactly one attempt."""

    attempts: dict = field(default_factory=dict)
    default_attempts: int = 1

    def for_action(self, action: str) -> int:
        return max(1, int(self.attempts.get(action, self.default_attempts)))


class Backend(object):
    """Every action a route can dispatch. Subclasses implement them.

    `now()` and `sleep()` are the injected clock. `await_pull` is implemented
    HERE rather than per backend, so the production polling loop is the same
    code the integration tests drive with a fake clock.
    """

    #: Seconds to wait for a validated pull before giving up.
    await_pull_timeout_sec = 120.0
    #: Poll interval while waiting.
    await_pull_poll_sec = 0.05

    # --- injected clock ---------------------------------------------------------
    def now(self) -> float:
        raise NotImplementedError

    def sleep(self, seconds: float) -> None:
        raise NotImplementedError

    def aborted(self) -> bool:
        """True when the route has been cancelled and should stop waiting."""
        return False

    # --- shared: consume the validated event while we own the lock -------------
    def await_pull(self, ctx):
        """Block until the task record yields a validated pull, or time out.

        This runs INSIDE the route, which holds the execution lock, so the
        release service callback never needs it. A timeout or a cancel returns
        failure and performs NO release: the caller's cleanup disarms the
        monitor and the object stays held.
        """
        if ctx.handover is None:
            return StepResult(False, "await_pull has no task record to consume from")
        started = self.now()
        deadline = started + float(self.await_pull_timeout_sec)
        while True:
            if self.aborted():
                return StepResult(
                    False, "CANCELLED while awaiting a pull after %.1fs; no release issued"
                    % (self.now() - started))
            event = ctx.handover.consume_pending(self.now())
            if event is not None:
                return StepResult(
                    True,
                    "consumed validated pull event %d (session %d, token %r) after %.1fs"
                    % (event.sequence, event.session, event.event_token,
                       self.now() - started))
            if self.now() >= deadline:
                return StepResult(
                    False, "TIMEOUT: no validated pull within %.1fs; no release issued"
                    % (float(self.await_pull_timeout_sec),))
            self.sleep(float(self.await_pull_poll_sec))

    #: Actions that only ever inspect state; never retried differently.
    def require_held_object(self, ctx):
        raise NotImplementedError

    def record_held_object(self, ctx):
        raise NotImplementedError

    def plan_handover_pose(self, ctx):
        raise NotImplementedError

    def settle_handover(self, ctx):
        raise NotImplementedError

    def settle_recipient(self, ctx):
        raise NotImplementedError

    def arm_monitor(self, ctx):
        raise NotImplementedError

    def disarm_monitor(self, ctx):
        raise NotImplementedError

    def confirm_released(self, ctx):
        raise NotImplementedError

    def execute_retract_home(self, ctx):
        raise NotImplementedError

    def execute_stow(self, ctx):
        raise NotImplementedError

    def await_localization(self, ctx):
        raise NotImplementedError

    def enable_motion(self, ctx):
        raise NotImplementedError

    def navigate(self, ctx):
        raise NotImplementedError

    def detect_object(self, ctx):
        raise NotImplementedError

    def report(self, ctx):
        raise NotImplementedError

    def to_planner_frame(self, ctx):
        raise NotImplementedError

    def plan_grasp(self, ctx):
        raise NotImplementedError

    def execute_segment(self, ctx, step):
        raise NotImplementedError

    def open_gripper(self, ctx):
        raise NotImplementedError

    def close_gripper(self, ctx):
        raise NotImplementedError

    def release_hand(self, ctx):
        raise NotImplementedError

    # --- cleanup, always called -----------------------------------------------------
    def cancel_navigation(self):
        raise NotImplementedError

    def disable_motion(self):
        raise NotImplementedError

    def disarm_monitor_cleanup(self):
        """Always called: a route must never exit leaving the detector armed."""
        raise NotImplementedError


class DryRunBackend(Backend):
    """Traverses every step, performs nothing, creates no client of any kind.

    Each action records itself and succeeds, so a dry run exercises the real
    route order, the real resume logic and the real cleanup path.
    """

    def __init__(self):
        self.calls = []
        self.cleanup = []
        self._t = 0.0

    # A dry run needs no real clock and must never block.
    def now(self):
        self._t += 0.01
        return self._t

    def sleep(self, seconds):
        self._t += float(seconds)

    def await_pull(self, ctx):
        # Overridden: a dry run must traverse this step, not wait for a human.
        return self._record("await_pull", "dry run: no pull awaited, no event consumed")

    def _record(self, action, detail=""):
        self.calls.append(action)
        return StepResult(True, detail or "dry run: %s not performed" % action, motion_issued=False)

    def await_localization(self, ctx):
        return self._record("await_localization")

    def enable_motion(self, ctx):
        return self._record("enable_motion", "dry run: motion gate NOT opened")

    def navigate(self, ctx):
        return self._record("navigate", "dry run: would navigate to %r" % ctx.target)

    def detect_object(self, ctx):
        ctx.detection = {"dry_run": True, "label": ctx.target, "frame_id": "<unresolved>"}
        return self._record("detect_object")

    def report(self, ctx):
        return self._record("report", "dry run: detection %s" % (ctx.detection,))

    def to_planner_frame(self, ctx):
        return self._record("to_planner_frame")

    def plan_grasp(self, ctx):
        ctx.plan = {name: {"waypoint_count": 0} for name in
                    ("to_pregrasp", "approach", "lift", "retract")}
        return self._record("plan_grasp", "dry run: no plan requested")

    def execute_segment(self, ctx, step):
        return self._record(step.name, "dry run: %s NOT executed" % step.name)

    def open_gripper(self, ctx):
        return self._record("open_gripper")

    def close_gripper(self, ctx):
        return self._record("close_gripper")

    def release_hand(self, ctx):
        return self._record("release_hand")

    def require_held_object(self, ctx):
        return self._record("require_held_object", "dry run: held-object check skipped")

    def record_held_object(self, ctx):
        return self._record("record_held_object", "dry run: no state written")

    def plan_handover_pose(self, ctx):
        return self._record("plan_handover_pose", "dry run: no plan requested")

    def settle_handover(self, ctx):
        return self._record("settle_handover", "dry run: not waited")

    def settle_recipient(self, ctx):
        return self._record("settle_recipient", "dry run: not waited")

    def arm_monitor(self, ctx):
        return self._record("arm_monitor", "dry run: monitor NOT armed")

    def disarm_monitor(self, ctx):
        return self._record("disarm_monitor")

    def confirm_released(self, ctx):
        return self._record("confirm_released", "dry run: nothing to confirm")

    def execute_retract_home(self, ctx):
        return self._record("execute_retract_home", "dry run: arm NOT moved")

    def execute_stow(self, ctx):
        return self._record("execute_stow", "dry run: arm NOT moved")

    def cancel_navigation(self):
        self.cleanup.append("cancel_navigation")

    def disable_motion(self):
        self.cleanup.append("disable_motion")

    def disarm_monitor_cleanup(self):
        self.cleanup.append("disarm_monitor_cleanup")


def _dispatch(backend: Backend, step, ctx) -> StepResult:
    handler = getattr(backend, step.action, None)
    if handler is None:
        return StepResult(False, "backend has no action %r" % (step.action,))
    # execute_segment needs to know WHICH segment; every other action does not.
    if step.action == "execute_segment":
        return handler(ctx, step)
    return handler(ctx)


def run_route(route, backend: Backend, ctx: RouteContext, start_step: str = "",
              retry: RetryPolicy = None, blockers=None) -> RouteResult:
    """Walk `route` through `backend`, fail-fast, always cleaning up.

    `blockers` is the preflight blocker list. A non-empty list means the route
    is reported NOT_READY and NOT A SINGLE STEP runs.
    """
    retry = retry or RetryPolicy()
    blockers = list(blockers or [])

    try:
        skipped_steps, to_run = routes_mod.plan_steps(route, start_step)
    except KeyError as exc:
        return RouteResult(
            success=False, message=str(exc), blockers=blockers, dry_run=ctx.dry_run
        )

    skipped = [s.name for s in skipped_steps]

    if blockers:
        return RouteResult(
            success=False,
            message="NOT_READY: %d blocker(s); no step executed" % len(blockers),
            steps_skipped=skipped,
            blockers=blockers,
            dry_run=ctx.dry_run,
        )

    completed = []
    motion_issued = False
    final_step = ""
    try:
        for step in to_run:
            final_step = step.name
            attempts = retry.for_action(step.action)
            result = None
            for attempt in range(1, attempts + 1):
                try:
                    result = _dispatch(backend, step, ctx)
                except Exception as exc:  # noqa: BLE001 - a raising step is a failed step
                    result = StepResult(False, "%s raised %s: %s" % (
                        step.name, type(exc).__name__, exc))
                if result.motion_issued:
                    motion_issued = True
                if result.ok:
                    break
                if attempt < attempts:
                    ctx.notes.append(
                        "%s attempt %d/%d failed: %s" % (step.name, attempt, attempts, result.detail)
                    )
            if not result.ok:
                # Fail fast: no later step runs, including cleanup-looking ones.
                return RouteResult(
                    success=False,
                    message="step %r failed: %s" % (step.name, result.detail),
                    steps_completed=completed,
                    steps_skipped=skipped,
                    final_step=final_step,
                    motion_issued=motion_issued,
                    dry_run=ctx.dry_run,
                )
            completed.append(step.name)
        return RouteResult(
            success=True,
            message="route %r completed (%d steps)" % (route.name, len(completed)),
            steps_completed=completed,
            steps_skipped=skipped,
            final_step=final_step,
            motion_issued=motion_issued,
            dry_run=ctx.dry_run,
        )
    finally:
        # Unconditional: success, failure, exception or early return. A route
        # must never exit leaving navigation chasing a goal, the motion gate
        # open, or the tactile detector armed.
        backend.cancel_navigation()
        backend.disable_motion()
        backend.disarm_monitor_cleanup()
