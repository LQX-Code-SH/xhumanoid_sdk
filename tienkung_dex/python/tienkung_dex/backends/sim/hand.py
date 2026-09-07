#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sim dexterous hand over the main project's 6-DOF hand bridge (§6).

The gz model and URDF now carry full 6-DOF hands (per side: thumb flex /
thumb rot + index/middle/ring/pinky), driven by the simulation package's
hand_bridge: the SDK's 6-code commands on
/tienkung_dex/{left,right}_hand/joint_cmds are expanded into the gz
JointTrajectoryController (12 gz joints, proximal + distal followers);
gz state returns the same way as per-hand 6 codes on
/tienkung_dex/{left,right}_hand/state.

Brainco semantics are preserved end to end (SDK brainco scale, no rad on
the wire): motor order thumb_flex thumb_rot index middle ring pinky,
code 1 = fully straight .. 1000 = fully bent.  No touch channel exists on
the sim bridge, so on_touch() stays silent (registry kept for symmetry).
"""

from __future__ import annotations

import time
from typing import Callable, Optional, Sequence

from tienkung_dex.core.base import DexterousHandBase
from tienkung_dex.core.presets import GESTURE_POSITIONS
from tienkung_dex.core.types import HandStatus, TouchReading

MOTOR_COUNT = 6
POS_MIN, POS_MAX = 1, 1000
MOTORS = ('thumb_flex', 'thumb_rot', 'index', 'middle', 'ring', 'pinky')


class SimDexterousHand(DexterousHandBase):
    """6-DOF brainco-style hand over the gz hand-bridge topics."""

    def __init__(self, node, side: str, cmd_topic: str, state_topic: str,
                 logger=None, stale_timeout: float = 0.5):
        super().__init__(node, side, vendor='brainco-sim')
        self._cmd_topic = cmd_topic
        self._state_topic = state_topic
        self._log = logger
        self._stale_timeout = stale_timeout
        self._pub = None
        self._sub = None
        self._status: Optional[HandStatus] = None
        self._last_seen: Optional[float] = None
        self._touch_cbs: list[Callable[[TouchReading], None]] = []

    def on_start(self) -> None:
        from sensor_msgs.msg import JointState
        self._pub = self._node.create_publisher(
            JointState, self._cmd_topic, 10)
        self._sub = self._node.create_subscription(
            JointState, self._state_topic, self._on_state, 10)
        if self._log is not None:
            self._log.info(
                f'{self.name} (sim, 6-DOF): pub {self._cmd_topic}, '
                f'sub {self._state_topic}')

    def on_stop(self) -> None:
        self._pub = None
        self._sub = None
        self._status = None
        self._last_seen = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < self._stale_timeout)

    def status_age(self) -> Optional[float]:
        """Seconds since the latest state frame (None = never seen)."""
        if self._last_seen is None:
            return None
        return time.monotonic() - self._last_seen

    def _on_state(self, msg) -> None:
        vals = list(msg.position)
        positions = tuple(int(round(v)) for v in vals[:MOTOR_COUNT])
        positions = positions + (POS_MIN,) * (MOTOR_COUNT - len(positions))
        self._status = HandStatus(positions=positions, raw=msg)
        self._last_seen = time.monotonic()

    # -- control ----------------------------------------------------------
    def _wait_pub_matched(self, timeout: float = 3.0) -> None:
        """Publish 前等至少一个订阅者（hand_bridge 常驻但 Fast-DDS
        discovery 滞后，首帧常在匹配完成前发出而静默丢弃；与 real 后端
        同款策略，此处内联以保持 sim 后端不 import real 的隔离）。"""
        if self._pub is None:
            return
        import rclpy
        executor = getattr(self._node, 'executor', None)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                if self._pub.get_subscription_count() > 0:
                    return
            except Exception:
                pass
            try:
                if executor is not None:
                    executor.spin_once(timeout_sec=0.05)
                else:
                    rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception:
                time.sleep(0.05)
        if self._log is not None:
            self._log.warn(
                f'{self.name}: {timeout:.0f}s 内未发现匹配订阅者'
                '（hand_bridge 未运行？），仍尝试发送')

    def _publish(self, positions: Sequence[int], *,
                 wait_match: bool = True) -> None:
        """wait_match=False skips the discovery wait (non-blocking path for
        the hand controller closed loop, same rationale as real backend)."""
        if self._pub is None:
            raise RuntimeError(f'{self.name} not started')
        if wait_match:
            self._wait_pub_matched()
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self.side
        msg.name = list(MOTORS)
        msg.position = [float(p) for p in positions]
        self._pub.publish(msg)

    def publish_positions(self, positions: Sequence[int], *,
                          wait_match: bool = True) -> None:
        """Send 6 clipped codes over the hand-bridge JointState cmd topic."""
        clipped = tuple(max(POS_MIN, min(POS_MAX, int(p)))
                        for p in positions[:MOTOR_COUNT])
        clipped = clipped + (POS_MIN,) * (MOTOR_COUNT - len(clipped))
        self._publish(clipped, wait_match=wait_match)

    def set_positions(self, positions: Sequence[int]) -> None:
        self.publish_positions(positions, wait_match=True)

    def set_gesture(self, gesture: str) -> bool:
        preset = GESTURE_POSITIONS.get(gesture)
        if preset is None:
            if self._log is not None:
                self._log.warn(
                    f'{self.name}: unknown gesture {gesture!r} '
                    f'(known: {sorted(GESTURE_POSITIONS)})')
            return False
        # gz 无真机的关节干涉/堵转，无需 real 后端的两段式序列，直发即可。
        self.set_positions(preset)
        return True

    def set_force(self, forces: Sequence[int]) -> None:
        # Brainco 接口仅位置控制；force 无通道，告警忽略（与 real 一致）。
        if self._log is not None:
            self._log.warn(f'{self.name}: set_force not supported by the '
                           'brainco sim interface (ignored)')

    def set_speed(self, speeds: Sequence[int]) -> None:
        if self._log is not None:
            self._log.warn(f'{self.name}: set_speed not supported by the '
                           'brainco sim interface (ignored)')

    def get_status(self) -> Optional[HandStatus]:
        return self._status

    def on_touch(self, cb: Callable[[TouchReading], None]) -> None:
        # 仿真手桥无触觉通道：保留注册点，静默不发。
        self._touch_cbs.append(cb)
