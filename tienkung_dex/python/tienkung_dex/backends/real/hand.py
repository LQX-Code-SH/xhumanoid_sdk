#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real dexterous hand (brainco, 6 motors) + optional touch (design §4.3).

SetMotorMulti fields (verified against the gesture demo):
    mode, positions[6], speeds[6], currents[6], pwms[6], durations[6]
Position scale: 1 = fully straight .. 1000 = fully bent.
TouchStatus: data[] items with normal_force1 (0.01 N), tangential_force1,
tangential_direction1 (65535 = N/A), self_proximity1, status.
"""

from __future__ import annotations

from typing import Callable, Optional, Sequence

from tienkung_dex.core.base import DexterousHandBase
from tienkung_dex.core.types import HandStatus, TouchReading

from . import _msgs

# Gesture preset table copied from the gesture demo (motor order:
# thumb flex, thumb rotate, index, middle, ring, pinky).
GESTURE_POSITIONS = {
    'ok': (450, 800, 450, 1, 1, 1),
    'rock': (1000, 700, 1000, 1000, 1000, 1000),
    'scissors': (1000, 500, 1, 1, 1000, 1000),
    'paper': (1, 500, 1, 1, 1, 1),
}

MOTOR_COUNT = 6
POS_MIN, POS_MAX = 1, 1000


class RealDexterousHand(DexterousHandBase):
    """Brainco 6-motor hand via set_motor_multi / motor_status."""

    def __init__(self, node, side: str, topics: dict, logger,
                 control_mode: int = 1):
        super().__init__(node, side, vendor='brainco')
        self._topics = topics
        self._log = logger
        self._control_mode = control_mode
        self._pub = None
        self._sub = None
        self._msg_cls = None
        self._status_cls = None
        self._status = None
        self._touch_cbs: list[Callable[[TouchReading], None]] = []

    def on_start(self) -> None:
        self._msg_cls, self._status_cls, _touch_cls = (None, None, None)
        module = None
        try:
            import importlib
            module = importlib.import_module('brainco_hand_msgs.msg')
        except Exception as exc:
            raise RuntimeError(
                f'{self.name}: brainco_hand_msgs not importable ({exc})')
        self._msg_cls = getattr(module, 'SetMotorMulti')
        self._status_cls = getattr(module, 'MotorStatus')
        self._touch_cls = getattr(module, 'TouchStatus', None)

        self._pub = self._node.create_publisher(
            self._msg_cls, self._topics['cmd'], 10)
        self._sub = self._node.create_subscription(
            self._status_cls, self._topics['status'],
            self._on_status, 10)
        if self._touch_cls is not None:
            self._touch_sub = self._node.create_subscription(
                self._touch_cls, self._topics['touch'],
                self._on_touch, 10)
        if self._log is not None:
            self._log.info(
                f'{self.name} (brainco): pub {self._topics["cmd"]}, '
                f'sub {self._topics["status"]} (+touch)')

    def on_stop(self) -> None:
        self._pub = None
        self._sub = None
        self._status = None

    @property
    def is_active(self) -> bool:
        return self._status is not None

    def _on_status(self, msg) -> None:
        positions = tuple(int(getattr(msg, 'positions', ())[i]) if i < len(
            getattr(msg, 'positions', ())) else 0 for i in range(MOTOR_COUNT))
        self._status = HandStatus(positions=positions, raw=msg)

    def _on_touch(self, msg) -> None:
        items = []
        for item in getattr(msg, 'data', ()) or ():
            items.append((
                int(getattr(item, 'normal_force1', 0)),
                int(getattr(item, 'tangential_force1', 0)),
                int(getattr(item, 'tangential_direction1', 65535)),
                int(getattr(item, 'self_proximity1', 0)),
                int(getattr(item, 'status', 0)),
            ))
        reading = TouchReading(values=tuple(items))
        for cb in tuple(self._touch_cbs):
            cb(reading)

    # -- control ----------------------------------------------------------
    def _publish(self, positions, speeds=None, currents=None) -> None:
        if self._pub is None:
            raise RuntimeError(f'{self.name} not started')
        msg = self._msg_cls()
        msg.mode = self._control_mode
        positions = tuple(positions)[:MOTOR_COUNT]
        speeds = tuple(speeds or ())[:MOTOR_COUNT] or (0,) * MOTOR_COUNT
        currents = tuple(currents or ())[:MOTOR_COUNT] or (0,) * MOTOR_COUNT
        for i in range(MOTOR_COUNT):
            msg.positions[i] = int(positions[i]) if i < len(positions) else POS_MIN
            msg.speeds[i] = int(speeds[i]) if i < len(speeds) else 0
            msg.currents[i] = int(currents[i]) if i < len(currents) else 0
            msg.pwms[i] = 0
            msg.durations[i] = 0
        self._pub.publish(msg)

    def set_positions(self, positions: Sequence[int]) -> None:
        clipped = tuple(max(POS_MIN, min(POS_MAX, int(p))) for p in positions)
        self._publish(clipped)

    def set_gesture(self, gesture: str) -> bool:
        preset = GESTURE_POSITIONS.get(gesture)
        if preset is None:
            if self._log is not None:
                self._log.warn(
                    f'{self.name}: unknown gesture {gesture!r} '
                    f'(known: {sorted(GESTURE_POSITIONS)})')
            return False
        self._publish(preset)
        return True

    def set_force(self, forces: Sequence[int]) -> None:
        # Brainco demo controls position only; force setpoints are not
        # demonstrated - accept and ignore with a warning (inspire-style
        # interface kept for symmetry).
        if self._log is not None:
            self._log.warn(f'{self.name}: set_force not supported by the '
                           'brainco demo interface (ignored)')

    def set_speed(self, speeds: Sequence[int]) -> None:
        self._publish((POS_MIN,) * MOTOR_COUNT, speeds=speeds)

    def get_status(self) -> Optional[HandStatus]:
        return self._status

    def on_touch(self, cb: Callable[[TouchReading], None]) -> None:
        self._touch_cbs.append(cb)
