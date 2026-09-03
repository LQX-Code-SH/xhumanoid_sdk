#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real RC SBUS receiver : Joy axes + SbusData buttons/events.

Data layout (vendor): /sbus_data carries a sensor_msgs/Joy with 12 axes in
[-1, 1] (3 sticks; per-stick axis mapping pending live capture) and an
unused, always-empty buttons array; the actual key levels/events live in
/sbus_data/event (bodyctrl_msgs/SbusData).
The joy path is standard and always available; the event path degrades
gracefully - button/event fields stay zero when the package is missing.
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
        self._event_new = 0
        self._event_old = 0
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
        reading = self.latest()
        if reading is not None:
            self._emit(reading)

    def _on_event(self, msg) -> None:
        # SbusData declares button_a..button_h (8 keys, A-H), levels
        # -1 released / 0 middle / 1,2 ends; all 8 are forwarded.
        self._buttons = tuple(int(getattr(msg, field, 0)) for field in
                              ('button_a', 'button_b', 'button_c',
                               'button_d', 'button_e', 'button_f',
                               'button_g', 'button_h'))
        # key_event_new = state after the change, key_event_old = state
        # before it (KEY_* codes of SbusData.msg, see SbusReading.key_name()).
        self._event_new = int(getattr(msg, 'key_event_new', 0))
        self._event_old = int(getattr(msg, 'key_event_old', 0))
        # A button event proves the receiver is alive even before the first
        # Joy message (and keeps is_active fresh on event-only streams).
        self._last_seen = time.monotonic()
        reading = self.latest()
        if reading is not None:
            self._emit(reading)

    def latest(self) -> Optional[SbusReading]:
        if self._last_seen is None:
            return None
        return SbusReading(axes=self._axes, buttons=self._buttons,
                           event_new=self._event_new,
                           event_old=self._event_old)
