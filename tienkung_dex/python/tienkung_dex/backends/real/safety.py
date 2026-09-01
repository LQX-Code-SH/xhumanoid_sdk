#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real e-stop monitor over /power/board/key_status (design doc §4.6).

Fields is_estop / is_remote_estop are confirmed by HWI §7.1; the message
class name is not pinned by any demo and is probed by _msgs.key_status_msg().
"""

from __future__ import annotations

from typing import Optional

from tienkung_dex.core.base import SafetyMonitorBase
from tienkung_dex.core.errors import EstopActiveError

from . import _msgs


class RealSafetyMonitor(SafetyMonitorBase):
    """Subscribes key_status; edge-triggered on_estop callbacks."""

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
        return self._sub is not None

    def _on_key_status(self, msg) -> None:
        estop = bool(getattr(msg, 'is_estop', False))
        remote = bool(getattr(msg, 'is_remote_estop', False))
        combined = estop or remote
        previous = self._estopped or self._remote
        import time
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


class NullSafetyMonitor(SafetyMonitorBase):
    """Degraded fallback: no key_status message available, interception off.

    Keeps the L1 semantics visible: is_estopped stays False but is_active
    reports False, so the health() summary flags the degradation.
    """

    def __init__(self, node, logger, reason: str):
        super().__init__(node)
        self._log = logger
        self._reason = reason

    def on_start(self) -> None:
        if self._log is not None:
            self._log.error(f'safety: {self._reason}')

    def on_stop(self) -> None:
        pass

    @property
    def is_active(self) -> bool:
        return False

    @property
    def is_estopped(self) -> bool:
        return False

    def guard(self) -> None:
        pass
