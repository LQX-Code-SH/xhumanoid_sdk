#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real e-stop monitor over /power/board/key_status (design doc §4.6).

Fields is_estop / is_remote_estop are confirmed by HWI §7.1; the message
class name is not pinned by any demo and is probed by _msgs.key_status_msg().
"""

from __future__ import annotations

import time

from typing import Optional

from tienkung_dex.core.base import SafetyMonitorBase
from tienkung_dex.core.errors import EstopActiveError

from . import _msgs


def _bool_field(msg, name: str, default: bool = False) -> bool:
    """Read a vendor bool field that may be plain bool or std_msgs/Bool.

    PowerBoardKeyStatus declares is_estop etc. as std_msgs/Bool: the
    attribute is a wrapper OBJECT whose truthiness is always True - the
    real value lives in .data. Never bool() the object itself.
    """
    value = getattr(msg, name, None)
    if value is None:
        return default
    data = getattr(value, 'data', value)
    return bool(data)


class RealSafetyMonitor(SafetyMonitorBase):
    """Subscribes key_status; edge-triggered on_estop callbacks.

    is_active follows the same staleness semantics as every other data
    stream (measured /power/board/key_status rate on the real host:
    ~12.6 Hz, so the default 0.5 s window has >6 message periods of
    margin): False before the first message and once the stream goes
    silent, so health() flags a dead e-stop source instead of reporting
    green forever.
    """

    def __init__(self, node, topic: str, logger, stale_timeout: float = 0.5):
        super().__init__(node)
        self._topic = topic
        self._log = logger
        self._stale_timeout = stale_timeout
        self._sub = None
        self._estopped = False
        self._remote = False
        self._last_seen = None

    def on_start(self) -> None:
        msg_cls, err = _msgs.key_status_msg()
        if msg_cls is None:
            if self._log is not None:
                self._log.error(
                    f'safety: {err}; e-stop monitoring INACTIVE - '
                    'command interception disabled (L1 degraded)')
            return
        self._sub = self._node.create_subscription(
            msg_cls, self._topic, self._on_key_status, 10)
        if self._log is not None:
            self._log.info(f'safety: sub {self._topic} ({msg_cls.__name__})')

    def on_stop(self) -> None:
        self._sub = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < self._stale_timeout)

    def _on_key_status(self, msg) -> None:
        estop = _bool_field(msg, 'is_estop')
        remote = _bool_field(msg, 'is_remote_estop')
        combined = estop or remote
        previous = self._estopped or self._remote
        self._estopped = estop
        self._remote = remote
        self._last_seen = time.monotonic()
        if combined != previous:
            self._emit(combined)

    @property
    def is_estopped(self) -> bool:
        return self._estopped or self._remote

    @property
    def is_remote_estopped(self) -> bool:
        return self._remote

    def guard(self) -> None:
        """E-stop pre-check installed on every JointGroup command path."""
        if self._sub is not None and self.is_estopped:
            raise EstopActiveError(
                'robot e-stop active: joint commands rejected (L1)')
