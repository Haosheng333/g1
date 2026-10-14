#!/usr/bin/env python3
"""List available input (microphone) devices, with the index to pass as
~input_device (or --device to recognize_wav.py's live-mic sibling). No ROS,
no Vosk -- only needs sounddevice."""

from __future__ import annotations

import sys


def main() -> int:
    try:
        import sounddevice as sd
    except ImportError as exc:
        print(f"sounddevice is not installed: {exc}\npip install sounddevice", file=sys.stderr)
        return 1

    devices = sd.query_devices()
    print(f"{'idx':>4}  {'in ch':>5}  {'default sr':>10}  name")
    for idx, dev in enumerate(devices):
        if dev["max_input_channels"] > 0:
            print(f"{idx:4d}  {dev['max_input_channels']:5d}  {dev['default_samplerate']:10.0f}  {dev['name']}")

    try:
        default_in = sd.default.device[0]
        print(f"\ndefault input device index: {default_in}")
    except Exception:
        print("\ncould not determine the default input device")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
