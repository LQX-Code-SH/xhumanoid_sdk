#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real power monitoring : battery + board + key status.

PowerBatteryStatus fields (demo-confirmed): master_battery_voltage / V,
master_battery_current / A, master_battery_power / W. The board status is
only counted for liveness; key_status feeds is_estop/is_power_on with the
same std_msgs/Bool unwrap as the safety monitor.
"""

from __future__ import annotations

import time
from typing import Optional

from tienkung_dex.core.base import PowerSystemBase
from tienkung_dex.core.types import PowerReading

from . import _msgs
from .safety import _bool_field


def parse_battery(msg) -> tuple:
    """Pure extraction of the master-battery triple (V, A, W)."""
    return (
        float(getattr(msg, 'master_battery_voltage', 0.0)),
        float(getattr(msg, 'master_battery_current', 0.0)),
        float(getattr(msg, 'master_battery_power', 0.0)),
    )


class RealPowerSystem(PowerSystemBase):
    """Subscribes /power/battery|board/status and /power/board/key_status."""

    def __init__(self, node, topics: dict, logger=None,
                 stale_timeout: float = 5.0):
        super().__init__(node)
        self._topics = topics
        self._log = logger
        self._stale_timeout = stale_timeout
        self._sub_battery = None
        self._sub_board = None
        self._sub_key = None
        self._voltage = 0.0
        self._current = 0.0
        self._power_w = 0.0
        self._is_estop = False
        self._is_power_on = False
        self._last_seen = None

    def on_start(self) -> None:
        classes, err = _msgs.power_msgs()
        battery_cls, board_cls, key_cls = classes
        if battery_cls is None:
            if self._log is not None:
                self._log.error(f'power: {err}; stream inactive')
            return
        self._sub_battery = self._node.create_subscription(
            battery_cls, self._topics['battery'], self._on_battery, 10)
        self._sub_board = self._node.create_subscription(
            board_cls, self._topics['board'], self._on_board, 10)
        self._sub_key = self._node.create_subscription(
            key_cls, self._topics['key_status'], self._on_key, 10)
        if self._log is not None:
            self._log.info(f'power: sub {self._topics["battery"]} '
                           f'+ {self._topics["board"]} '
                           f'+ {self._topics["key_status"]}')

    def on_stop(self) -> None:
        self._sub_battery = None
        self._sub_board = None
        self._sub_key = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < self._stale_timeout)

    def _on_battery(self, msg) -> None:
        self._voltage, self._current, self._power_w = parse_battery(msg)
        self._last_seen = time.monotonic()
        self._emit(self.latest())

    def _on_board(self, msg) -> None:
        # Board status carries no demo-documented payload worth extracting;
        # its liveness is implied by the battery stream.
        pass

    def _on_key(self, msg) -> None:
        self._is_estop = _bool_field(msg, 'is_estop')
        self._is_power_on = _bool_field(msg, 'is_power_on')
        # latest() is None until the first battery message: emitting None
        # would violate the on_update(PowerReading) contract.
        reading = self.latest()
        if reading is not None:
            self._emit(reading)

    def latest(self) -> Optional[PowerReading]:
        if self._last_seen is None:
            return None
        return PowerReading(
            voltage=self._voltage, current=self._current,
            power_w=self._power_w, is_estop=self._is_estop,
            is_power_on=self._is_power_on)
