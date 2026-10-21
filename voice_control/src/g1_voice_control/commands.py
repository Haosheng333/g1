"""Deterministic command matching. No ROS, no Vosk, no I/O.

Matches a FINAL ASR transcript against a small allowlist of complete phrases.
Deliberately conservative: an utterance must equal one allowlisted phrase
after limited normalisation, not merely contain it, so "不要开始抓取" never
matches "开始抓取". An utterance matching more than one command, or carrying
a negation marker, is rejected as unrelated rather than guessed.
"""

from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass

# Negation markers checked against the whole normalised utterance. Keeping
# this list short and explicit (not a general sentiment classifier) is the
# point: a false negative here just means "ask the person to repeat it",
# which is safe; a false positive would be a command silently swallowed.
NEGATION_MARKERS = (
    "不要", "不用", "别", "先别", "暂时不", "不是",
    "don't", "do not", "dont", "no ", "not ", "never ", "cancel that",
)


def normalise(text: str) -> str:
    """Lowercase, strip, collapse internal whitespace, normalise width.

    This is the ENTIRE normalisation step -- deliberately limited and
    documented, per the spec, rather than stemming/fuzzy-matching text that
    could blur "开始抓取" into something it did not say.
    """
    text = unicodedata.normalize("NFKC", text)  # full-width -> half-width, etc.
    text = text.strip().lower()
    text = re.sub(r"[，,。.!！？?\s]+", " ", text).strip()
    # Vosk joins recognised vocabulary tokens with spaces for every language.
    # Remove only whitespace whose two neighbours are CJK characters, so
    # common Mandarin output such as "开始 抓取" matches "开始抓取" while
    # English word boundaries remain significant.
    text = re.sub(r"(?<=[\u3400-\u9fff])\s+(?=[\u3400-\u9fff])", "", text)
    return text


def _contains_negation(normalised: str) -> bool:
    return any(marker in normalised for marker in NEGATION_MARKERS)


@dataclass(frozen=True)
class CommandSpec:
    """One allowlisted command: an id plus the exact phrases that trigger it."""

    command_id: str
    phrases: tuple  # exact phrases, pre-normalisation; matched after normalise()


@dataclass(frozen=True)
class MatchResult:
    matched: bool
    command_id: str = ""
    reason: str = ""  # set when matched=False: "no_match" | "negated" | "ambiguous"


class CommandMatcher:
    def __init__(self, specs: list):
        self._by_normalised_phrase = {}
        for spec in specs:
            for phrase in spec.phrases:
                key = normalise(phrase)
                if key in self._by_normalised_phrase and self._by_normalised_phrase[key] != spec.command_id:
                    raise ValueError(
                        f"phrase {phrase!r} is claimed by both "
                        f"{self._by_normalised_phrase[key]!r} and {spec.command_id!r}"
                    )
                self._by_normalised_phrase[key] = spec.command_id

    def match(self, transcript: str) -> MatchResult:
        normalised = normalise(transcript)
        if not normalised:
            return MatchResult(matched=False, reason="no_match")

        if _contains_negation(normalised):
            return MatchResult(matched=False, reason="negated")

        # Exact match against the whole normalised utterance only -- no
        # substring containment, so trailing/leading extra words also miss
        # rather than being guessed at.
        command_id = self._by_normalised_phrase.get(normalised)
        if command_id is not None:
            return MatchResult(matched=True, command_id=command_id)

        return MatchResult(matched=False, reason="no_match")


def default_specs() -> list:
    """Built-in defaults used by isolated matcher tests and small utilities.

    The ROS node loads its deployment phrases from config/commands.yaml.
    """
    return [
        CommandSpec("grasp", ("开始抓取", "start grasping")),
        CommandSpec("deliver", ("开始递送", "start delivery")),
        CommandSpec("status", ("查看状态", "status")),
        CommandSpec("cancel", ("取消任务", "cancel task")),
        CommandSpec("confirm", ("确认执行", "confirm")),
    ]
