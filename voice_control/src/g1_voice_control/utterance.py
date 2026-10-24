"""Join final ASR results into one utterance ended by an end word. No ROS.

Vosk emits a final result at every pause, so "go to the desk, pick up a can,
go to the chair, and pass me, over" arrives as several results. The node
pushes each one here; the buffer hands back the whole utterance once a result
carries a trailing end word ("over"), and discards a buffer that waited too
long, so a half-sentence from minutes ago can never be glued onto a new one.
"""

from __future__ import annotations

from typing import Callable, Optional, Tuple


class UtteranceBuffer:
    def __init__(self, parser, clock: Callable[[], float], timeout_sec: float) -> None:
        if float(timeout_sec) <= 0.0:
            raise ValueError("timeout_sec must be positive")
        self._parser = parser
        self._clock = clock
        self._timeout = float(timeout_sec)
        self._parts: list = []
        self._started_at: Optional[float] = None

    @property
    def text(self) -> str:
        return " ".join(self._parts)

    @property
    def pending(self) -> bool:
        return bool(self._parts)

    def push(self, text: str) -> Tuple[Optional[str], Optional[str]]:
        """-> (complete_utterance or None, dropped_stale_utterance or None)."""
        dropped = self.expire()
        if not self._parts:
            self._started_at = self._clock()
        self._parts.append(text)
        if self._parser.has_end_word(text):
            full = self.text
            self.clear()
            return full, dropped
        return None, dropped

    def expire(self) -> Optional[str]:
        """Drop and return the buffer if it has waited longer than the timeout."""
        if self._parts and self._clock() - self._started_at > self._timeout:
            stale = self.text
            self.clear()
            return stale
        return None

    def clear(self) -> None:
        self._parts = []
        self._started_at = None
