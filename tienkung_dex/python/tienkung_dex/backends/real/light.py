#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real light-strip control over /xsys/light/ctrl .

LightCtrl fields (demo-confirmed): cmd (preset id), data (payload),
caller_id, caller_msg. Presets live in core.topics.LIGHT_CMDS.
"""

from __future__ import annotations

from typing import Sequence

from tienkung_dex.core import topics as t
from tienkung_dex.core.base import LightControlBase

from . import _msgs


class RealLightControl(LightControlBase):
    """Pure publisher; degrades to inactive when bodyctrl_msgs is absent."""

    def __init__(self, node, topic: str, logger=None):
        super().__init__(node)
        self._topic = topic
        self._log = logger
        self._pub = None
        self._msg_cls = None

    def on_start(self) -> None:
        self._msg_cls, err = _msgs.light_msg()
        if self._msg_cls is None:
            if self._log is not None:
                self._log.error(f'light: {err}; control inactive')
            return
        self._pub = self._node.create_publisher(
            self._msg_cls, self._topic, 10)
        if self._log is not None:
            self._log.info(f'light: pub {self._topic}')

    def on_stop(self) -> None:
        self._pub = None

    @property
    def is_active(self) -> bool:
        return self._pub is not None

    def set_cmd(self, cmd: int, data: Sequence[int] = ()) -> None:
        if self._pub is None:
            raise RuntimeError(f'{self.name} not started')
        msg = self._msg_cls()
        msg.cmd = int(cmd)
        msg.data = [int(d) for d in data]
        msg.caller_id = 'tienkung_dex'
        msg.caller_msg = f'light cmd={cmd}'
        self._pub.publish(msg)

    def set_mode(self, mode: str) -> bool:
        cmd = t.LIGHT_CMDS.get(mode)
        if cmd is None:
            if self._log is not None:
                self._log.warn(
                    f'light: unknown mode {mode!r} '
                    f'(known: {sorted(t.LIGHT_CMDS)})')
            return False
        self.set_cmd(cmd)
        return True
