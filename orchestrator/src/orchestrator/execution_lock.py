"""Non-blocking single-route lock.

A second route request must be REFUSED immediately, not queued and not blocked
on. Two routes interleaving would mean two owners of the arm and the motion
gate at once, and a blocking lock would hold a service call open until a
minutes-long navigation finished.
"""
from __future__ import annotations

import threading


class Busy(Exception):
    """Raised when a route is already running."""

    def __init__(self, holder: str):
        super(Busy, self).__init__("another route is already running: %s" % holder)
        self.holder = holder


class ExecutionLock(object):
    def __init__(self):
        self._lock = threading.Lock()
        self._holder = None

    @property
    def holder(self):
        return self._holder

    @property
    def busy(self) -> bool:
        return self._holder is not None

    def acquire(self, holder: str) -> bool:
        """Try to take the lock. Returns False at once if it is held."""
        if not self._lock.acquire(False):
            return False
        self._holder = holder
        return True

    def release(self) -> None:
        if self._holder is None:
            return
        self._holder = None
        self._lock.release()

    class _Held(object):
        def __init__(self, parent):
            self._parent = parent

        def __enter__(self):
            return self._parent

        def __exit__(self, *exc):
            self._parent.release()
            return False

    def hold(self, holder: str):
        """Context manager that raises Busy instead of waiting."""
        if not self.acquire(holder):
            raise Busy(self._holder or "unknown")
        return self._Held(self)
