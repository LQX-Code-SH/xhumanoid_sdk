#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sim joint group over gz topics (design doc §6).

State  : sensor_msgs/JointState on /joint_states (gz standard)
Command: sensor_msgs/JointState positions on /tienkung_dex/joint_cmds -
         the main project's ros_gz bridge config maps this into the gz
         joint interface (values configurable via factory params).
"""

from __future__ import annotations

import time
from typing import Optional

from tienkung_dex.core.base import JointGroupBase
from tienkung_dex.core.types import ControlMode, JointCommand, JointReading


class SimJointGroup(JointGroupBase):
    """JointState-backed joint group; all four groups share one stream."""

    def __init__(self, node, group: str, state_topic: str, cmd_topic: str,
                 logger, stale_timeout: float = 0.5):
        super().__init__(node, group)
        self._state_topic = state_topic
        self._cmd_topic = cmd_topic
        self._log = logger
        self._stale_timeout = stale_timeout
        self._sub = None
        self._pub = None
        self._readings: dict[int, JointReading] = {}
        self._last_seen = None

    def on_start(self) -> None:
        from sensor_msgs.msg import JointState
        self._sub = self._node.create_subscription(
            JointState, self._state_topic, self._on_joint_state, 10)
        self._pub = self._node.create_publisher(
            JointState, self._cmd_topic, 10)
        if self._log is not None:
            self._log.info(
                f'{self.name} (sim): sub {self._state_topic}, '
                f'pub {self._cmd_topic}')

    def on_stop(self) -> None:
        self._sub = None
        self._pub = None
        self._readings = {}

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < self._stale_timeout)

    @property
    def last_update_age(self) -> Optional[float]:
        if self._last_seen is None:
            return None
        return time.monotonic() - self._last_seen

    def _on_joint_state(self, msg) -> None:
        readings = {}
        # Joint names follow the gz model; SDK joint IDs are namespaced
        # integers in the real robot - the sim maps name '<group>_<id>' to
        # int id and falls back to the array index.
        for i, name in enumerate(msg.name):
            try:
                prefix, _, suffix = name.rpartition('_')
                jid = int(suffix) if prefix == self.group else None
            except ValueError:
                jid = None
            if jid is None and name.isdigit():
                jid = int(name)
            if jid is None:
                jid = i   # fallback: positional id (documented deviation)
            pos = float(msg.position[i]) if i < len(msg.position) else 0.0
            vel = float(msg.velocity[i]) if i < len(msg.velocity) else 0.0
            readings[jid] = JointReading(
                joint_id=jid, pos=pos, vel=vel, tor=0.0,
                stamp=msg.header.stamp)
        self._readings = readings
        self._last_seen = time.monotonic()
        self._emit(dict(readings))

    def get_state(self, joint_id: int) -> Optional[JointReading]:
        return self._readings.get(joint_id)

    def get_states(self) -> dict[int, JointReading]:
        return dict(self._readings)

    def publish_command(self, cmds, mode: ControlMode) -> None:
        if self._pub is None:
            raise RuntimeError(f'{self.name} not started')
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self.group
        for cmd in cmds:
            msg.name.append(f'{self.group}_{cmd.joint_id}')
            msg.position.append(float(cmd.pos))
            msg.velocity.append(float(cmd.spd))
            msg.effort.append(float(cmd.tor))
        self._pub.publish(msg)
