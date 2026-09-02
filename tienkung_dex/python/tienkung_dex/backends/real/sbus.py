#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real RC SBUS receiver (vendor demo 09): Joy axes + SbusData buttons.

The joystick path (/sbus_data, sensor_msgs/Joy) is standard and always
available; the button event path (/sbus_data/event, bodyctrl_msgs/SbusData)
degrades gracefully - buttons stay empty when the package is missing.
"""

from __future__ import annotations

import time
from typing import Optional

from tienkung_dex.core.base import SbusStreamBase
from tienkung_dex.core.types import SbusReading

from . import _msgs


class RealSbusStream(SbusStreamBase):
    """Merges the Joy axes and SbusData buttons into one reading."""

    def __init__(self, node, topics: dict, logger=None,
                 stale_timeout: float = 1.0):
        super().__init__(node)
        self._topics = topics
        self._log = logger
        self._stale_timeout = stale_timeout
        self._sub_joy = None
        self._sub_event = None
        self._axes = ()
        self._buttons = ()
        self._last_seen = None

    def on_start(self) -> None:
        from sensor_msgs.msg import Joy
        self._sub_joy = self._node.create_subscription(
            Joy, self._topics['joy'], self._on_joy, 10)
        msg_cls, err = _msgs.sbus_event_msg()
        if msg_cls is not None:
            self._sub_event = self._node.create_subscription(
                msg_cls, self._topics['event'], self._on_event, 10)
        elif self._log is not None:
            self._log.warn(f'sbus: {err}; button events unavailable')
        if self._log is not None:
            self._log.info(f'sbus: sub {self._topics["joy"]}'
                           f'{"" if self._sub_event else " (joy only)"}')

    def on_stop(self) -> None:
        self._sub_joy = None
        self._sub_event = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < self._stale_timeout)

    def _on_joy(self, msg) -> None:
        self._axes = tuple(float(a) for a in (msg.axes or ()))
        self._last_seen = time.monotonic()
        self._emit(self.latest())

    def _on_event(self, msg) -> None:
        self._buttons = tuple(int(getattr(msg, field, 0)) for field in
                              ('button_a', 'button_b', 'button_c',
                               'button_d', 'button_e', 'button_f'))
        self._emit(self.latest())

    def latest(self) -> Optional[SbusReading]:
        if self._last_seen is None:
            return None
        return SbusReading(axes=self._axes, buttons=self._buttons)
