#!/usr/bin/env python3
"""Run Vosk recognition over a WAV file and print each final transcript plus
whether it matches a supported command -- for testing the ASR+matching path
without ROS, a microphone, or live hardware.

Usage:
    python3 recognize_wav.py --model models/vosk-model-small-cn-0.22 --wav sample.wav
"""

from __future__ import annotations

import argparse
import sys

# Allow running directly from scripts/ without building the catkin package.
sys.path.insert(0, __file__.rsplit("/scripts/", 1)[0] + "/src")

from g1_voice_control.asr_adapter import MissingDependencyError, WavFileAdapter  # noqa: E402
from g1_voice_control.commands import CommandMatcher, default_specs  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", required=True, help="Path to a downloaded Vosk model directory")
    parser.add_argument("--wav", required=True, help="16-bit mono PCM WAV file")
    args = parser.parse_args()

    try:
        adapter = WavFileAdapter(args.model, args.wav)
    except MissingDependencyError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    matcher = CommandMatcher(default_specs())
    found_any = []

    def on_final(text: str) -> None:
        found_any.append(text)
        result = matcher.match(text)
        if result.matched:
            print(f"  -> {text!r}  =>  command={result.command_id!r}")
        else:
            print(f"  -> {text!r}  =>  no match ({result.reason})")

    adapter.run(on_final=on_final, should_stop=lambda: False)
    adapter.close()

    if not found_any:
        print("(no final transcripts -- check the WAV is 16-bit mono PCM and contains speech)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
