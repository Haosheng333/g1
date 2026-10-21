"""Validated command configuration with no ROS, Vosk, or audio imports."""

from __future__ import annotations

import math
from dataclasses import dataclass

from g1_voice_control.commands import CommandMatcher, CommandSpec


@dataclass(frozen=True)
class CommandsConfig:
    confirmation_window_sec: float
    targets: dict
    command_specs: tuple


def load_commands_config(path: str) -> CommandsConfig:
    import yaml

    with open(path, "r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}

    window = float(raw.get("confirmation_window_sec", 8.0))
    if not math.isfinite(window) or window <= 0:
        raise ValueError("confirmation_window_sec must be a finite number greater than zero")

    phrases = raw.get("phrases")
    if not isinstance(phrases, dict):
        raise ValueError("commands config must contain a 'phrases' mapping")

    command_specs = []
    for command_id in ("grasp", "deliver", "status", "cancel", "confirm"):
        values = phrases.get(command_id)
        if not isinstance(values, list) or not values:
            raise ValueError(f"phrases.{command_id} must be a non-empty list")
        if any(not isinstance(value, str) or not value.strip() for value in values):
            raise ValueError(f"phrases.{command_id} entries must be non-empty strings")
        command_specs.append(CommandSpec(command_id, tuple(value.strip() for value in values)))

    targets = {}
    for command_id in ("grasp", "deliver"):
        target_key = f"{command_id}_target"
        valid_key = f"valid_{command_id}_targets"
        target = raw.get(target_key)
        if target is not None:
            if not isinstance(target, str) or not target.strip():
                raise ValueError(f"{target_key} must be null or a non-empty string")
            target = target.strip()

        valid_targets = raw.get(valid_key, [])
        if not isinstance(valid_targets, list) or any(
            not isinstance(value, str) or not value.strip() for value in valid_targets
        ):
            raise ValueError(f"{valid_key} must be a list of non-empty strings")
        valid_targets = {value.strip() for value in valid_targets}
        if target is not None and target not in valid_targets:
            raise ValueError(
                f"{target_key}={target!r} is not allowlisted in {valid_key}; "
                "refusing an unvalidated orchestrator target"
            )
        targets[command_id] = target

    command_specs = tuple(command_specs)
    CommandMatcher(command_specs)
    return CommandsConfig(
        confirmation_window_sec=window,
        targets=targets,
        command_specs=command_specs,
    )
