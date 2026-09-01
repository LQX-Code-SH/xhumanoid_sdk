#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real joint group + shared /robot_state cache (design doc §4.2, §5).

RobotStateCache subscribes /robot_state once per backend and dispatches
parsed per-group snapshots to every RealJointGroup, the xsens IMU stream
and any other state consumer - one subscription, many observers.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from tienkung_dex.core.base import JointGroupBase
from tienkung_dex.core.types import ControlMode, JointCommand, JointReading

from . import _msgs

ROBOT_STATE_GROUP_FIELDS = ('arm', 'head', 'waist', 'leg')


def _getattr_or(msg, name: str, default):
    """Tolerant field access: vendor .msg layouts drift between SDK
    versions, so missing fields degrade to defaults instead of raising."""
    try:
        return getattr(msg, name)
    except Exception:
        return default


def parse_robot_state(msg) -> dict:
    """Pure conversion RobotState -> {group: {joint_id: JointReading}}.

    arm.status[] is demo-confirmed; the other groups follow the same
    convention (open question, design doc §11.1). status[].name holds the
    joint ID.
    """
    groups = {}
    for group in ROBOT_STATE_GROUP_FIELDS:
        container = _getattr_or(msg, group, None)
        statuses = _getattr_or(container, 'status', None) if container is not None else None
        readings = {}
        if statuses is not None:
            for item in statuses:
                jid = int(_getattr_or(item, 'name', 0))
                reading = JointReading(
                    joint_id=jid,
                    pos=float(_getattr_or(item, 'pos', 0.0)),
                    vel=float(_getattr_or(item, 'vel', 0.0)),
                    tor=float(_getattr_or(item, 'tor', 0.0)),
                    stamp=None,
                )
                readings[jid] = reading
        groups[group] = readings
    return groups


class RobotStateCache:
    """Single /robot_state subscription shared by all state consumers."""

    def __init__(self, node, topic: str, logger=None, qos_depth: int = 10):
        self._node = node
        self._topic = topic
        self._log = logger
        self._sub = None
        self._groups: dict[str, dict[int, JointReading]] = {}
        self._last_stamp = None
        self._last_monotonic = None
        self._last_msg = None           # raw RobotState for the xsens IMU path
        self._lock = threading.Lock()
        self._callbacks: list[Callable[[dict], None]] = []

    def start(self) -> bool:
        msg_cls, err = _msgs.robot_state_msg()
        if msg_cls is None:
            if self._log is not None:
                self._log.error(f'RobotStateCache: {err}')
            return False
        try:
            self._sub = self._node.create_subscription(
                msg_cls, self._topic, self._on_robot_state, qos_depth)
        except Exception as exc:
            if self._log is not None:
                self._log.error(f'RobotStateCache: subscribe failed: {exc}')
            return False
        return True

    def _on_robot_state(self, msg) -> None:
        groups = parse_robot_state(msg)
        with self._lock:
            self._groups = groups
            self._last_msg = msg
            self._last_monotonic = time.monotonic()
        for cb in tuple(self._callbacks):
            cb(groups)

    def raw_msg(self):
        """Latest raw RobotState message (xsens IMU consumers)."""
        with self._lock:
            return self._last_msg

    def subscribe(self, cb: Callable[[dict], None]) -> None:
        self._callbacks.append(cb)

    def snapshot(self) -> dict[str, dict[int, JointReading]]:
        with self._lock:
            return self._groups

    def last_update_age(self) -> Optional[float]:
        with self._lock:
            if self._last_monotonic is None:
                return None
            return time.monotonic() - self._last_monotonic


class RealJointGroup(JointGroupBase):
    """Publishes /{group}/cmd, reads state from the shared RobotStateCache."""

    def __init__(self, node, group: str, state_cache: RobotStateCache,
                 joints_table, logger, cmd_topic: str,
                 stale_timeout: float = 0.5):
        super().__init__(node, group)
        self._state_cache = state_cache
        self._joints_table = joints_table
        self._log = logger
        self._cmd_topic = cmd_topic
        self._stale_timeout = stale_timeout
        self._pub = None
        self._msg_cls = None
        self._msg_cls_fallback_warned = False

    # -- lifecycle --------------------------------------------------------
    def on_start(self) -> None:
        self._msg_cls, err = _msgs.joint_cmd_msg(self.group)
        if self._msg_cls is None:
            raise RuntimeError(f'{self.name}: {err}')
        if self.group != 'arm' and err == '' and not hasattr(
                self._msg_cls, 'ctrl'):
            # Fallback ArmCtrl already happened inside joint_cmd_msg.
            pass
        if self._msg_cls.__name__ == 'ArmCtrl' and self.group != 'arm':
            if self._log is not None and not self._msg_cls_fallback_warned:
                self._log.warn(
                    f'{self.name}: {self.group.capitalize()}Ctrl not found; '
                    'falling back to ArmCtrl (identical field set per HWI §7.1)')
                self._msg_cls_fallback_warned = True
        self._pub = self._node.create_publisher(
            self._msg_cls, self._cmd_topic, 10)
        self._state_cache.subscribe(self._on_state_update)
        if self._log is not None:
            self._log.info(
                f'{self.name}: pub {self._cmd_topic} ({self._msg_cls.__name__}), '
                f'sub {self._state_cache._topic} (shared)')

    def on_stop(self) -> None:
        self._pub = None

    @property
    def is_active(self) -> bool:
        age = self.last_update_age
        return age is not None and age < self._stale_timeout

    # -- state ------------------------------------------------------------
    def _on_state_update(self, groups: dict) -> None:
        self._emit(groups.get(self.group, {}))

    def get_state(self, joint_id: int) -> Optional[JointReading]:
        return self._state_cache.snapshot().get(self.group, {}).get(joint_id)

    def get_states(self) -> dict[int, JointReading]:
        return dict(self._state_cache.snapshot().get(self.group, {}))

    @property
    def last_update_age(self) -> Optional[float]:
        return self._state_cache.last_update_age()

    # -- command ----------------------------------------------------------
    def _check_joint_ids(self, cmds) -> None:
        if self._joints_table is None or self._joints_table.is_empty:
            return
        for cmd in cmds:
            if not self._joints_table.known(self.group, cmd.joint_id):
                from tienkung_dex.core.errors import JointIdError
                message = (f'{self.group}: unknown joint_id {cmd.joint_id} '
                           f'(joints table: {self._joints_table._source})')
                if self._joints_table.strict:
                    raise JointIdError(message)
                if self._log is not None:
                    self._log.warn(message)

    def publish_command(self, cmds, mode: ControlMode) -> None:
        if self._pub is None:
            raise RuntimeError(f'{self.name} not started')
        self._check_joint_ids(cmds)

        msg = self._msg_cls()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self.group
        msg.mode = int(mode)
        msg.label = 0

        ctrl_cls = type(msg.ctrl[0]) if len(getattr(msg, 'ctrl', ()) or []) else None
        for cmd in cmds:
            ctrl = ctrl_cls() if ctrl_cls is not None else _new_ctrl(msg)
            ctrl.name = cmd.joint_id
            ctrl.pos = float(cmd.pos)
            ctrl.spd = float(cmd.spd)
            ctrl.cur = float(cmd.cur)
            ctrl.kp = float(cmd.kp)
            ctrl.kd = float(cmd.kd)
            ctrl.tor = float(cmd.tor)
            msg.ctrl.append(ctrl)
        self._pub.publish(msg)


def _new_ctrl(msg):
    """Create a MotorCtrl instance matching the vendor ArmCtrl family.

    The demo constructs msg.ctrl.append(MotorCtrl()) - the element class is
    imported alongside the command message. Prefer the element prototype;
    fall back to the ros2_bridge_msgs MotorCtrl class.
    """
    ctrl_field = getattr(msg, 'ctrl', None)
    if ctrl_field is not None and len(ctrl_field) > 0:
        return type(ctrl_field[0])()
    _, err = _msgs._resolve_msg('ros2_bridge_msgs', 'MotorCtrl')
    if err:
        raise RuntimeError(err)
    from importlib import import_module
    return getattr(import_module('ros2_bridge_msgs.msg'), 'MotorCtrl')()
