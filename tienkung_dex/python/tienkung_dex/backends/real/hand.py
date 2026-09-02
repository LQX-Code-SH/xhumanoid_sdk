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


INSPIRE_JOINT_COUNT = 13


class RealInspireHand(DexterousHandBase):
    """Inspire 13-joint hand (vendor demos 07/15): angle/force/speed_set
    commands, angle_actual/force_actual/touch_data feedback and the
    SetClearError service. joint_values are broadcast to all 13 joints
    like the demo does."""

    def __init__(self, node, side: str, topics: dict, logger):
        super().__init__(node, side, vendor='inspire')
        self._topics = topics
        self._log = logger
        self._pubs = {}          # 'angle' | 'force' | 'speed' -> publisher
        self._msg_cls = {}       # same keys -> message classes
        self._status = None
        self._clear_cli = None
        self._clear_srv_cls = None
        self._touch_cbs: list[Callable[[TouchReading], None]] = []

    def on_start(self) -> None:
        classes, err = _msgs.inspire_hand_msgs()
        (angle_cls, force_cls, speed_cls,
         angle_act_cls, _force_act_cls, touch_cls) = classes
        if angle_cls is None:
            raise RuntimeError(f'{self.name}: {err}')

        self._msg_cls = {'angle': angle_cls, 'force': force_cls,
                         'speed': speed_cls}
        self._pubs = {
            'angle': self._node.create_publisher(
                angle_cls, self._topics['angle_cmd'], 10),
            'force': self._node.create_publisher(
                force_cls, self._topics['force_cmd'], 10),
            'speed': self._node.create_publisher(
                speed_cls, self._topics['speed_cmd'], 10),
        }
        self._sub_angle = self._node.create_subscription(
            angle_act_cls, self._topics['angle_actual'],
            self._on_angle_actual, 10)
        if touch_cls is not None:
            self._sub_touch = self._node.create_subscription(
                touch_cls, self._topics['touch'], self._on_touch_msg, 10)

        self._clear_srv_cls, err = _msgs.clear_error_service()
        if self._clear_srv_cls is not None:
            self._clear_cli = self._node.create_client(
                self._clear_srv_cls, self._topics['clear_error'])
        elif self._log is not None:
            self._log.warn(f'{self.name}: {err}; clear_error unavailable')

        if self._log is not None:
            self._log.info(
                f'{self.name} (inspire): pub {self._topics["angle_cmd"]} '
                f'+ force/speed, sub {self._topics["angle_actual"]} '
                f'+ touch')

    def on_stop(self) -> None:
        self._pubs = {}
        self._status = None

    @property
    def is_active(self) -> bool:
        return self._status is not None

    def _on_angle_actual(self, msg) -> None:
        values = (getattr(msg, 'joint_values', None)
                  or getattr(msg, 'angles', None)
                  or getattr(msg, 'angle', None) or ())
        positions = tuple(int(v) for v in values)
        self._status = HandStatus(positions=positions, raw=msg)

    def _on_touch_msg(self, msg) -> None:
        # TouchData layout is not demo-documented (demo 15 only counts
        # frames); pass through per-item values when present.
        items = []
        for item in getattr(msg, 'data', ()) or ():
            if hasattr(item, 'value'):
                items.append((int(item.value),))
            else:
                items.append(())
        for cb in tuple(self._touch_cbs):
            cb(TouchReading(values=tuple(items)))

    # -- control ----------------------------------------------------------
    def _joint_values(self, values: Sequence[int], default: int = 0) -> list:
        padded = list(values) + [default] * (INSPIRE_JOINT_COUNT - len(values))
        return [int(v) for v in padded[:INSPIRE_JOINT_COUNT]]

    def _publish(self, kind: str, values: Sequence[int]) -> None:
        if kind not in self._pubs:
            raise RuntimeError(f'{self.name} not started')
        msg = self._msg_cls[kind]()
        msg.hand_id = 1 if self.side == 'left' else 2
        msg.joint_values = self._joint_values(values)
        self._pubs[kind].publish(msg)

    def set_positions(self, positions: Sequence[int]) -> None:
        clipped = tuple(max(0, min(1000, int(p))) for p in positions)
        self._publish('angle', clipped)

    def set_force(self, forces: Sequence[int]) -> None:
        self._publish('force', tuple(int(f) for f in forces))

    def set_speed(self, speeds: Sequence[int]) -> None:
        self._publish('speed', tuple(int(s) for s in speeds))

    def clear_error(self) -> bool:
        if self._clear_cli is None:
            return False
        if not self._clear_cli.wait_for_service(timeout_sec=2.0):
            if self._log is not None:
                self._log.error(f'{self.name}: clear_error service '
                                'not reachable')
            return False
        self._clear_cli.call_async(self._clear_srv_cls.Request())
        return True

    def get_status(self) -> Optional[HandStatus]:
        return self._status

    def on_touch(self, cb: Callable[[TouchReading], None]) -> None:
        self._touch_cbs.append(cb)
