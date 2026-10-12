#!/usr/bin/env python3
"""Collision-mask interface for A*. Start with EmptyCollisionMask, replace later."""
from __future__ import annotations
from pathlib import Path
import numpy as np


class EmptyCollisionMask:
    """Collision-unaware mode: every valid grid state is free."""
    def __call__(self, state) -> bool:
        return False


class BooleanGridCollisionMask:
    """Memory-mapped boolean grid where True means blocked.

    The array shape must match the four-dimensional joint grid counts.
    """
    def __init__(self, path: str | Path, expected_shape):
        self.mask = np.load(path, mmap_mode="r")
        if tuple(self.mask.shape) != tuple(expected_shape):
            raise ValueError(f"Collision mask shape {self.mask.shape} does not match {tuple(expected_shape)}")
        if self.mask.dtype != np.bool_:
            raise ValueError("Collision mask must have boolean dtype")

    def __call__(self, state) -> bool:
        return bool(self.mask[state])


class CombinedCollisionMask:
    """Blocked when any supplied collision source reports True."""
    def __init__(self, *masks):
        self.masks = masks

    def __call__(self, state) -> bool:
        return any(mask(state) for mask in self.masks)
