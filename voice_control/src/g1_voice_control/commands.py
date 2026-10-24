"""Deterministic command matching. No ROS, no Vosk, no I/O.

Two recognisers, tried in this order on a FINAL ASR transcript:

1. Exact phrases (`CommandMatcher`): an utterance must equal one allowlisted
   phrase after limited normalisation, not merely contain it, so "不要开始抓取"
   never matches "开始抓取". Status, cancel, confirm and the legacy single-route
   starts live here.
2. The slot grammar (`TaskParser`, config/lexicon.yaml): the spoken
   fetch-and-deliver task, "go to the desk, pick up a can, go to the chair, and
   pass me, over". The parser tokenises longest-phrase-first, skips unknown
   words, splits into segments, fills one frame per segment and lowers the
   frames to navigate / grasp / handover steps. Exactly ONE shape is accepted
   -- navigate, grasp, navigate, handover -- and becomes the `fetch_deliver`
   command with its waypoints and label as slots. Any other shape is reported
   as understood-but-unsupported, never guessed into a different task.

An utterance carrying a negation marker is rejected before either recogniser.
"""

from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass, field
from typing import Optional

# Negation markers checked against the whole normalised utterance. Keeping
# this list short and explicit (not a general sentiment classifier) is the
# point: a false negative here just means "ask the person to repeat it",
# which is safe; a false positive would be a command silently swallowed.
NEGATION_MARKERS = (
    "不要", "不用", "别", "先别", "暂时不", "不是",
    "don't", "do not", "dont", "no ", "not ", "never ", "cancel that",
)

#: The one task the slot grammar may dispatch.
FETCH_DELIVER = "fetch_deliver"


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
    text = re.sub(r"(?<=[㐀-鿿])\s+(?=[㐀-鿿])", "", text)
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
    reason: str = ""  # set when matched=False: "no_match" | "negated" | "ambiguous" | "unsupported_plan: ..."
    #: fetch_deliver only: {"target": <pickup waypoint>, "destination": <delivery
    #: waypoint>, "label": <perception label>}. Empty for the exact-phrase commands.
    slots: dict = field(default_factory=dict)
    #: fetch_deliver only: what will be done, in words, for the operator log.
    summary: str = ""


# --------------------------------------------------------------------------- slot grammar
@dataclass
class Frame:
    """One clause of the utterance, as the slots it filled."""

    verb: Optional[str] = None       # verb class: navigate | grasp | fetch | give
    obj: Optional[str] = None        # perception label
    obj_spoken: str = ""             # the word that was said, for the summary
    place: Optional[str] = None      # standing waypoint name
    recipient: bool = False

    def empty(self) -> bool:
        return self.verb is None and self.obj is None and self.place is None and not self.recipient


@dataclass(frozen=True)
class ParseResult:
    kind: str                         # "none" | "fetch_deliver" | "unsupported"
    steps: tuple = ()                 # the lowered steps, for diagnostics
    slots: dict = field(default_factory=dict)
    summary: str = ""
    reason: str = ""


class TaskParser:
    """Slot-grammar parser for the spoken task. Pure python, configured by a dict."""

    SUPPORTED_SHAPE = ("navigate", "grasp", "navigate", "handover")

    def __init__(self, lexicon: dict) -> None:
        lex = lexicon or {}
        self.objects = {str(k).lower(): str(v) for k, v in (lex.get("objects") or {}).items()}
        self.places = {str(k).lower(): str(v) for k, v in (lex.get("places") or {}).items()}
        self.obj_prior = {str(k): str(v) for k, v in (lex.get("object_location_prior") or {}).items()}
        self.user_loc = lex.get("user_location_prior")
        self.user_loc = str(self.user_loc) if self.user_loc else None
        self.recipients = {str(w).lower() for w in lex.get("recipients") or ()}
        self.connectors = {str(w).lower() for w in lex.get("connectors") or ()}
        self.fillers = {str(w).lower() for w in lex.get("fillers") or ()}
        self.wake_words = sorted((str(w).lower() for w in lex.get("wake_words") or ()),
                                 key=len, reverse=True)
        self.end_words = [str(w).lower() for w in lex.get("end_words") or ()]
        self.extra_words = [str(w).lower() for w in lex.get("grammar_extra_words") or ()]

        self.verbs = {}
        for cls, words in (lex.get("verb_classes") or {}).items():
            for w in words or ():
                self.verbs[str(w).lower()] = str(cls)
        if not self.verbs or not self.places:
            raise ValueError("lexicon needs at least verb_classes and places")

        # Unified vocabulary for the tokeniser: phrase -> (category, value).
        self.vocab = {}
        for ph, cls in self.verbs.items():
            self.vocab[ph] = ("verb", cls)
        for ph, lab in self.objects.items():
            self.vocab[ph] = ("object", lab)
        for ph, tgt in self.places.items():
            self.vocab[ph] = ("place", tgt)
        for ph in self.recipients:
            self.vocab[ph] = ("recipient", True)
        for ph in self.connectors:
            self.vocab[ph] = ("connector", None)
        for ph in self.fillers:          # last: a filler always wins its spelling
            self.vocab[ph] = ("filler", None)
        self.maxlen = max((len(ph.split()) for ph in self.vocab), default=1)

        # target -> the first alias listed for it, for the spoken summary
        self.place_names = {}
        for alias, tgt in self.places.items():
            self.place_names.setdefault(tgt, alias)

    # --- vocabulary for the recogniser -------------------------------------------
    def vocabulary(self) -> list:
        """Every word the grammar knows, for a constrained Vosk grammar."""
        words = set()
        for phrase in list(self.vocab) + self.wake_words + self.end_words + self.extra_words:
            for w in phrase.split():
                words.add(w)
        return sorted(words)

    # --- normalise / tokenise / segment -----------------------------------------
    def strip_end_word(self, text: str) -> tuple:
        """(text_without_end_word, had_end_word). Only a TRAILING end word counts."""
        t = normalise(text)
        for w in self.end_words:
            m = re.search(r"(^|\s)%s\s*$" % re.escape(w), t)
            if m:
                return t[:m.start()].strip(), True
        return t, False

    def has_end_word(self, text: str) -> bool:
        return self.strip_end_word(text)[1]

    def _normalize(self, text: str) -> str:
        t = unicodedata.normalize("NFKC", text or "").lower()
        t = re.sub(r"['’]s\b", "", t)
        t = re.sub(r"['’]", " ", t)
        t = re.sub(r"[,;，；]", " | ", t)             # comma -> segment marker
        t = re.sub(r"[!?。.：:！？]", " ", t)
        for w in self.wake_words:
            t = re.sub(r"\b%s\b" % re.escape(w), " ", t)
        for w in self.end_words:
            t = re.sub(r"\b%s\s*$" % re.escape(w), " ", t)
            t = re.sub(r"\b%s\s*\|\s*$" % re.escape(w), " ", t)
        return re.sub(r"\s+", " ", t).strip()

    def _tokenize(self, norm: str) -> list:
        words = norm.split()
        tokens, i = [], 0
        while i < len(words):
            if words[i] == "|":
                tokens.append(("sep", None, "|"))
                i += 1
                continue
            hit = None
            for length in range(min(self.maxlen, len(words) - i), 0, -1):
                phrase = " ".join(words[i:i + length])
                if phrase in self.vocab:
                    hit = (self.vocab[phrase], length, phrase)
                    break
            if hit is None:
                i += 1                     # unknown word -> skipped
                continue
            (cat, val), length, phrase = hit
            if cat != "filler":
                tokens.append((cat, val, phrase))
            i += length
        return tokens

    @staticmethod
    def _segment(tokens: list) -> list:
        """Split on connectors, comma markers and a second verb in one segment."""
        segments, cur = [], []
        for tok in tokens:
            cat = tok[0]
            if cat in ("connector", "sep"):
                if cur:
                    segments.append(cur)
                cur = []
                continue
            if cat == "verb" and any(t[0] == "verb" for t in cur):
                segments.append(cur)
                cur = [tok]
            else:
                cur.append(tok)
        if cur:
            segments.append(cur)
        return segments

    @staticmethod
    def _frame(segment: list) -> Frame:
        f = Frame()
        for cat, val, spoken in segment:
            if cat == "verb" and f.verb is None:
                f.verb = val
            elif cat == "object" and f.obj is None:
                f.obj, f.obj_spoken = val, spoken
            elif cat == "place" and f.place is None:
                f.place = val
            elif cat == "recipient":
                f.recipient = True
        return f

    # --- lowering ---------------------------------------------------------------
    def _lower(self, f: Frame, nxt: Optional[Frame]) -> list:
        nav, grasp, hand = (lambda t: ("navigate", t)), (lambda l, s: ("grasp", l, s)), ("handover",)
        vc, obj, place, recip = f.verb, f.obj, f.place, f.recipient
        user = self.user_loc
        if vc == "grasp" and place and not obj:            # "take me to the chair"
            return [nav(place)]
        if vc in ("navigate", "grasp", "fetch") and place and obj:
            return [nav(place), grasp(obj, f.obj_spoken)]
        if vc == "fetch" and obj and recip:                 # "bring a can to me"
            return [grasp(obj, f.obj_spoken)] + ([nav(user)] if user else []) + [hand]
        if vc in ("grasp", "fetch") and obj:
            return [grasp(obj, f.obj_spoken)]
        if vc == "fetch" and recip:                         # "bring it to me"
            return ([nav(user)] if user else []) + [hand]
        if vc == "give":
            return [hand]
        if vc == "navigate" and place:
            return [nav(place)]
        if vc == "navigate" and recip and user:
            return [nav(user)]
        if vc == "navigate" and obj:
            tgt = self.obj_prior.get(obj)
            return [nav(tgt)] if tgt else []
        if vc == "navigate" and nxt is not None and nxt.obj and not nxt.place:
            tgt = self.obj_prior.get(nxt.obj)
            return [nav(tgt)] if tgt else []
        return []

    def _complete(self, steps: list) -> list:
        """Fill the gaps the priors can fill, then merge repeated drives."""
        out = []
        for step in steps:
            if step[0] == "grasp" and not (out and out[-1][0] == "navigate"):
                prior = self.obj_prior.get(step[1])
                if prior:
                    out.append(("navigate", prior))
            if step[0] == "handover" and not (out and out[-1][0] == "navigate") and self.user_loc:
                out.append(("navigate", self.user_loc))
            if step[0] == "navigate" and out and out[-1] == step:
                continue
            out.append(step)
        return out

    def _summary(self, steps: list) -> str:
        bits = []
        for step in steps:
            if step[0] == "navigate":
                bits.append("go to the %s (%s)" % (self.place_names.get(step[1], step[1]), step[1]))
            elif step[0] == "grasp":
                bits.append("pick up the %s (%s)" % (step[2] or step[1], step[1]))
            elif step[0] == "handover":
                bits.append("hand it over")
        if not bits:
            return ""
        if len(bits) == 1:
            return "I will %s." % bits[0]
        return "I will " + ", ".join(bits[:-1]) + ", then " + bits[-1] + "."

    # --- entry point ------------------------------------------------------------
    def parse(self, text: str) -> ParseResult:
        norm = self._normalize(text)
        if not norm:
            return ParseResult("none", reason="empty")
        frames = [self._frame(seg) for seg in self._segment(self._tokenize(norm))]
        frames = [f for f in frames if not f.empty()]
        steps = []
        for i, f in enumerate(frames):
            steps += self._lower(f, frames[i + 1] if i + 1 < len(frames) else None)
        if not steps:
            return ParseResult("none", reason="no command words recognised")
        steps = self._complete(steps)
        shape = tuple(s[0] for s in steps)
        summary = self._summary(steps)
        if shape != self.SUPPORTED_SHAPE:
            return ParseResult(
                "unsupported", steps=tuple(steps), summary=summary,
                reason="understood [%s] but only navigate, grasp, navigate, handover is "
                       "supported (the fetch-and-deliver task)" % " -> ".join(shape))
        pickup, grasp, delivery = steps[0][1], steps[1], steps[2][1]
        slots = {"target": pickup, "label": grasp[1], "destination": delivery}
        return ParseResult(FETCH_DELIVER, steps=tuple(steps), slots=slots, summary=summary)


# --------------------------------------------------------------------------- matcher
class CommandMatcher:
    def __init__(self, specs: list, task_parser: Optional[TaskParser] = None):
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
        self._parser = task_parser

    @property
    def parser(self) -> Optional[TaskParser]:
        return self._parser

    def match_exact(self, transcript: str) -> MatchResult:
        """Only the exact-phrase allowlist; the slot grammar is not consulted."""
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

    def match(self, transcript: str) -> MatchResult:
        exact = self.match_exact(transcript)
        if exact.matched or exact.reason == "negated" or self._parser is None:
            return exact
        # A trailing end word ("over") is protocol, not content.
        text, _had_end = self._parser.strip_end_word(transcript)
        parsed = self._parser.parse(text)
        if parsed.kind == FETCH_DELIVER:
            return MatchResult(matched=True, command_id=FETCH_DELIVER,
                               slots=dict(parsed.slots), summary=parsed.summary)
        if parsed.kind == "unsupported":
            return MatchResult(matched=False, reason="unsupported_plan: " + parsed.reason,
                               summary=parsed.summary)
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
