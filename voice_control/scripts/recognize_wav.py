#!/usr/bin/env python3
"""Run Vosk recognition over a WAV file and print each final transcript plus
what the command layer makes of it -- for testing the ASR + matching path
without ROS, a microphone, or live hardware.

Usage:
    python3 recognize_wav.py --model models/vosk-model-small-en-us-0.15 --wav sample.wav
    python3 recognize_wav.py --model ... --wav ... --no-grammar     # free dictation

The final results are also joined into one utterance (the node does the same
until it hears the end word) and parsed as a whole, so a sentence Vosk split
at its commas is still checked as one command.
"""

from __future__ import annotations

import argparse
import os
import sys

# Allow running directly from scripts/ without building the catkin package.
PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from g1_voice_control.asr_adapter import MissingDependencyError, WavFileAdapter  # noqa: E402
from g1_voice_control.configuration import load_commands_config  # noqa: E402


def describe(matcher, text: str) -> str:
    result = matcher.match(text)
    if result.matched:
        slots = (" " + str(result.slots)) if result.slots else ""
        return f"command={result.command_id!r}{slots}"
    extra = f" | understood: {result.summary}" if result.summary else ""
    return f"no match ({result.reason}){extra}"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True, help="Path to a downloaded Vosk model directory")
    parser.add_argument("--wav", required=True, help="16-bit mono PCM WAV file")
    parser.add_argument(
        "--commands-config",
        default=os.path.join(PACKAGE_ROOT, "config", "commands.yaml"),
        help="Command phrase/target YAML (default: package config/commands.yaml)",
    )
    parser.add_argument("--no-grammar", action="store_true",
                        help="decode without the lexicon grammar (free dictation)")
    args = parser.parse_args()

    cfg = load_commands_config(args.commands_config)
    matcher = cfg.matcher()
    grammar = None
    if not args.no_grammar and matcher.parser is not None:
        grammar = matcher.parser.vocabulary()
        for spec in cfg.command_specs:
            for phrase in spec.phrases:
                grammar += phrase.lower().split()

    try:
        adapter = WavFileAdapter(args.model, args.wav, grammar=grammar)
    except MissingDependencyError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    found_any = []

    def on_final(text: str) -> None:
        found_any.append(text)
        print(f"  -> {text!r}  =>  {describe(matcher, text)}")

    adapter.run(on_final=on_final, should_stop=lambda: False)
    adapter.close()

    if not found_any:
        print("(no final transcripts -- check the WAV is 16-bit mono PCM and contains speech)")
        return 1
    if len(found_any) > 1:
        joined = " ".join(found_any)
        print(f"joined utterance {joined!r}  =>  {describe(matcher, joined)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
