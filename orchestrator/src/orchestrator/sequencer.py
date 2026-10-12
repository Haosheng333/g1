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
    """Every action a route can dispatch. Subclasses implement them."""

    #: Actions that only ever inspect state; never retried differently.
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

    def arm_handover(self, ctx):
        raise NotImplementedError

    def disarm_handover(self, ctx):
        raise NotImplementedError

    # --- cleanup, always called -----------------------------------------------------
    def cancel_navigation(self):
        raise NotImplementedError

    def disable_motion(self):
        raise NotImplementedError


class DryRunBackend(Backend):
    """Traverses every step, performs nothing, creates no client of any kind.

    Each action records itself and succeeds, so a dry run exercises the real
    route order, the real resume logic and the real cleanup path.
    """

    def __init__(self):
        self.calls = []
        self.cleanup = []

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

    def arm_handover(self, ctx):
        return self._record("arm_handover")

    def disarm_handover(self, ctx):
        return self._record("disarm_handover")

    def cancel_navigation(self):
        self.cleanup.append("cancel_navigation")

    def disable_motion(self):
        self.cleanup.append("disable_motion")


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
        # Unconditional: success, failure, exception or early return.
        backend.cancel_navigation()
        backend.disable_motion()
