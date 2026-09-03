#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real vector-velocity walking on /hric/robot/cmd_vel (HRIC 运控).

Leg joints belong to the locomotion policy (run_patrol / RL full-body
control), so walking is requested as a robot-body-frame velocity stream:
geometry_msgs/TwistStamped with linear.x/y = vx/vy and angular.z = wz,
published continuously at ~20 Hz (reference: 具身天工DEX-矢量行走接口.md).

geometry_msgs is a standard ROS 2 package present on the robot host; it is
imported lazily inside on_start() so importing this module stays safe on
machines without ROS (mirrors the vendor-message lazy accessors). When it
is missing the subsystem stays inactive and set_velocity() raises.
"""

from __future__ import annotations

from tienkung_dex.core import topics as t
from tienkung_dex.core.base import VectorWalkBase
from tienkung_dex.core.errors import EstopActiveError
from tienkung_dex.core.types import VelocityCommand


class RealVectorWalk(VectorWalkBase):
    """Streams the latest setpoint on the walking cmd_vel topic.

    set_velocity()/stop() emit one frame immediately (low latency) and the
    backend timer keeps re-publishing the setpoint on the configured cadence
    (default 20 Hz) so the locomotion keeps walking until zeroed. The timer
    is driven by the node's executor - callers must keep the node spinning
    (the demo base does this for you).
    """

    def __init__(self, node, cmd_topic: str = None,
                 frame_id: str = None, rate_hz: float = None,
                 logger=None):
        super().__init__(node, 'walk', logger=logger)
        self._topic = cmd_topic or t.WALK_CMD_TOPIC
        self._frame_id = frame_id or t.WALK_CMD_FRAME_ID
        self._rate_hz = (t.WALK_DEFAULT_RATE_HZ if rate_hz is None
                         else float(rate_hz))
        self._pub = None
        self._timer = None
        self._msg_cls = None

    def on_start(self) -> None:
        try:
            from geometry_msgs.msg import TwistStamped  # noqa: PLC0415
            self._msg_cls = TwistStamped
        except Exception as exc:  # pragma: no cover - robot host has it
            if self._log is not None:
                self._log.error(f'walk: geometry_msgs unavailable ({exc}); '
                                'cmd_vel control inactive')
            return
        self._pub = self._node.create_publisher(
            self._msg_cls, self._topic, 10)
        if self._rate_hz > 0:
            self._timer = self._node.create_timer(
                1.0 / self._rate_hz, self._pump)
        if self._log is not None:
            self._log.info(f'walk: pub {self._topic} @ {self._rate_hz} Hz '
                           f'frame={self._frame_id}')

    def on_stop(self) -> None:
        self._timer = None
        self._pub = None

    @property
    def is_active(self) -> bool:
        return self._pub is not None

    # -- VectorWalkBase ----------------------------------------------------
    def publish_command(self, cmd: VelocityCommand) -> None:
        if self._pub is None:
            raise RuntimeError(f'{self.name}: not started / geometry_msgs '
                               'unavailable - cmd_vel publish failed')
        self._emit(cmd)

    def _pump(self) -> None:
        if self._pub is None:
            return
        if self._guard is not None:
            try:
                self._guard()
            except EstopActiveError:
                # E-stop while streaming: force a standing setpoint instead
                # of holding the last (walking) velocity.
                self._emit(VelocityCommand())
                return
        self._emit(self._velocity)

    def _emit(self, cmd: VelocityCommand) -> None:
        msg = self._msg_cls()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.twist.linear.x = float(cmd.vx)
        msg.twist.linear.y = float(cmd.vy)
        msg.twist.angular.z = float(cmd.wz)
        self._pub.publish(msg)
        self._note_publish()
