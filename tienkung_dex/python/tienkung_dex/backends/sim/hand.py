#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sim hand: equivalent two-finger pinch model (design doc §6, 01 §2.1).

Brainco positions (1=straight .. 1000=bent) map to a normalized opening
0..1 (closed -> open). No ROS topics: pure in-process model, verified
through the unit tests; gz joint coupling stays in the main project's
simulation package.
"""

from __future__ import annotations

import time
from typing import Optional, Sequence

from tienkung_dex.core.base import DexterousHandBase
from tienkung_dex.core.types import HandStatus, TouchReading


class SimDexterousHand(DexterousHandBase):
    """Two-finger equivalent hand; contact state derived from closing."""

    def __init__(self, node, side: str, logger=None, touch_contact_value=800):
        super().__init__(node, side, vendor='brainco-sim')
        self._log = logger
        self._positions = (1,) * 6
        self._touch_contact = touch_contact_value
        self._touch_cbs = []
        self._last_touch = None
        self._last_status_monotonic = time.monotonic()

    def on_start(self) -> None:
        if self._log is not None:
            self._log.info(f'{self.name} (sim): two-finger pinch model')

    def on_stop(self) -> None:
        pass

    @property
    def is_active(self) -> bool:
        return True

    @property
    def opening(self) -> float:
        """Normalized opening 0 (closed) .. 1 (open), from index+middle+ring+pinky average."""
        fingers = self._positions[2:6]
        avg = sum(fingers) / max(len(fingers), 1)
        return (1000.0 - avg) / 999.0

    @property
    def in_contact(self) -> bool:
        return self._positions[0] >= self._touch_contact and self.opening < 0.3

    def _notify_touch(self) -> None:
        reading = TouchReading(values=((1,) * 5,) if self.in_contact else ())
        self._last_touch = reading
        for cb in tuple(self._touch_cbs):
            cb(reading)

    def set_positions(self, positions: Sequence[int]) -> None:
        self._positions = tuple(int(p) for p in positions[:6])
        self._last_status_monotonic = time.monotonic()
        self._notify_touch()

    def set_gesture(self, gesture: str) -> bool:
        presets = {
            'ok': (450, 800, 450, 1, 1, 1),
            'rock': (1000, 700, 1000, 1000, 1000, 1000),
            'scissors': (1000, 500, 1, 1, 1000, 1000),
            'paper': (1, 500, 1, 1, 1, 1),
        }
        if gesture not in presets:
            return False
        self.set_positions(presets[gesture])
        return True

    def set_force(self, forces: Sequence[int]) -> None:
        # Two-finger model has no force channel; interpreted as a nudge
        # toward closing (documented deviation, design doc §6).
        if forces and int(forces[0]) > 0:
            positions = list(self._positions)
            positions[0] = min(1000, positions[0] + int(forces[0]))
            self.set_positions(positions)

    def set_speed(self, speeds: Sequence[int]) -> None:
        pass  # instantaneous model

    def get_status(self) -> Optional[HandStatus]:
        return HandStatus(positions=self._positions)

    def on_touch(self, cb) -> None:
        self._touch_cbs.append(cb)
