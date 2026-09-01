#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Real sensor streams: IMU (dual source), lidar, GPS, force (§4.7, §5).

Unit notes (design doc §11.4): livox angles are derived from the standard
quaternion (rad); xsens RobotState.imu fields are passed through as
published (SDK demos treat them as degrees) - the facade does not convert.
"""

from __future__ import annotations

import math
import time
from typing import Optional

from tienkung_dex.core.base import (ForceStreamBase, GpsStreamBase,
                                    ImuStreamBase, LidarStreamBase)
from tienkung_dex.core.types import GpsFixReading, ImuReading, WrenchReading

from . import _msgs


def _sensor_qos():
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)


def quaternion_to_euler(w, x, y, z):
    """Standard ZYX euler extraction from a sensor_msgs quaternion (rad)."""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = (math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0
             else math.asin(sinp))
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ImuStream(ImuStreamBase):
    """source='livox' -> sensor_msgs/Imu; 'xsens' -> RobotState.imu.

    'sim' subscribes the gz standard /imu topic (also sensor_msgs/Imu).
    """

    def __init__(self, node, source: str, topic: str, state_cache=None,
                 logger=None, stale_timeout: float = 0.5):
        super().__init__(node, source)
        self._topic = topic
        self._state_cache = state_cache
        self._log = logger
        self._stale_timeout = stale_timeout
        self._sub = None
        self._latest = None
        self._last_seen = None

    def on_start(self) -> None:
        if self.source == 'xsens':
            if self._state_cache is None:
                raise RuntimeError('imu(xsens): state cache required')
            self._state_cache.subscribe(self._on_state_update)
        else:
            from sensor_msgs.msg import Imu
            self._sub = self._node.create_subscription(
                Imu, self._topic, self._on_imu, _sensor_qos())
        if self._log is not None:
            self._log.info(f'imu ({self.source}): {self._topic}')

    def on_stop(self) -> None:
        self._sub = None
        self._latest = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < self._stale_timeout)

    def _store(self, reading: ImuReading) -> None:
        self._latest = reading
        self._last_seen = time.monotonic()
        self._emit(reading)

    def _on_imu(self, msg) -> None:
        roll, pitch, yaw = quaternion_to_euler(
            msg.orientation.w, msg.orientation.x,
            msg.orientation.y, msg.orientation.z)
        self._store(ImuReading(
            roll=roll, pitch=pitch, yaw=yaw,
            wx=msg.angular_velocity.x, wy=msg.angular_velocity.y,
            wz=msg.angular_velocity.z,
            ax=msg.linear_acceleration.x, ay=msg.linear_acceleration.y,
            az=msg.linear_acceleration.z,
            stamp=msg.header.stamp, source=self.source))

    def _on_state_update(self, groups: dict) -> None:
        # RobotState.imu: pass through as published (units unverified).
        msg = self._state_cache.raw_msg()
        if msg is None:
            return
        imu = getattr(msg, 'imu', None)
        if imu is None:
            return
        self._store(ImuReading(
            roll=float(imu.roll), pitch=float(imu.pitch),
            yaw=float(imu.yaw), wx=float(imu.wx), wy=float(imu.wy),
            wz=float(imu.wz), ax=float(imu.ax), ay=float(imu.ay),
            az=float(imu.az), source='xsens'))

    def latest(self) -> Optional[ImuReading]:
        return self._latest


class LidarStream(LidarStreamBase):
    """Livox point cloud, SensorData QoS."""

    def __init__(self, node, topic: str, logger=None,
                 stale_timeout: float = 1.0):
        super().__init__(node, 'lidar')
        self._topic = topic
        self._log = logger
        self._stale_timeout = stale_timeout
        self._sub = None
        self._latest = None
        self._last_seen = None

    def on_start(self) -> None:
        from sensor_msgs.msg import PointCloud2
        self._sub = self._node.create_subscription(
            PointCloud2, self._topic, self._on_cloud_msg, _sensor_qos())
        if self._log is not None:
            self._log.info(f'lidar: {self._topic}')

    def on_stop(self) -> None:
        self._sub = None
        self._latest = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < self._stale_timeout)

    def _on_cloud_msg(self, msg) -> None:
        self._latest = msg
        self._last_seen = time.monotonic()
        self._emit(msg)

    def latest(self):
        return self._latest


class GpsStream(GpsStreamBase):
    """navigation_msgs/GpsFix -> GpsFixReading (optional hardware)."""

    def __init__(self, node, topic: str, logger=None,
                 stale_timeout: float = 5.0):
        super().__init__(node, 'gps')
        self._topic = topic
        self._log = logger
        self._stale_timeout = stale_timeout
        self._sub = None
        self._latest = None
        self._last_seen = None

    def on_start(self) -> None:
        msg_cls, err = _msgs.gps_msg()
        if msg_cls is None:
            if self._log is not None:
                self._log.error(f'gps: {err}; stream inactive')
            return
        self._sub = self._node.create_subscription(
            msg_cls, self._topic, self._on_fix_msg, 10)
        if self._log is not None:
            self._log.info(f'gps: {self._topic}')

    def on_stop(self) -> None:
        self._sub = None
        self._latest = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < self._stale_timeout)

    def _on_fix_msg(self, msg) -> None:
        reading = GpsFixReading(
            latitude=float(getattr(msg, 'latitude', 0.0)),
            longitude=float(getattr(msg, 'longitude', 0.0)),
            altitude=float(getattr(msg, 'altitude', 0.0)),
            status=int(getattr(msg, 'status', 0)),
            num_sats=int(getattr(msg, 'num_sats', 0)),
            hdop=float(getattr(msg, 'hdop', 0.0)),
            speed=float(getattr(msg, 'speed', 0.0)),
            heading=float(getattr(msg, 'heading', 0.0)),
        )
        self._latest = reading
        self._last_seen = time.monotonic()
        self._emit(reading)

    def latest(self) -> Optional[GpsFixReading]:
        return self._latest


class ForceStream(ForceStreamBase):
    """Six-axis force (HWI §7, unverified): silent-inactive until data.

    Accepts the vendor ForceState message if present, otherwise
    geometry_msgs/WrenchStamped. Topic must be set explicitly
    (FORCE_TOPIC is empty by default).
    """

    def __init__(self, node, topic: str, logger=None,
                 stale_timeout: float = 0.5):
        super().__init__(node, 'force')
        self._topic = topic
        self._log = logger
        self._stale_timeout = stale_timeout
        self._sub = None
        self._latest = None
        self._last_seen = None

    def on_start(self) -> None:
        if not self._topic:
            if self._log is not None:
                self._log.warn('force: no topic configured; inactive '
                               '(HWI §7 unverified hardware)')
            return
        msg_cls, err = _msgs.force_msg()
        if msg_cls is None:
            if self._log is not None:
                self._log.error(f'force: {err}')
            return
        self._sub = self._node.create_subscription(
            msg_cls, self._topic, self._on_force_msg, 10)

    def on_stop(self) -> None:
        self._sub = None
        self._latest = None

    @property
    def is_active(self) -> bool:
        return (self._last_seen is not None
                and time.monotonic() - self._last_seen < self._stale_timeout)

    def _on_force_msg(self, msg) -> None:
        wrench = getattr(msg, 'wrench', msg)
        reading = WrenchReading(
            fx=float(getattr(wrench.force, 'x', 0.0)),
            fy=float(getattr(wrench.force, 'y', 0.0)),
            fz=float(getattr(wrench.force, 'z', 0.0)),
            tx=float(getattr(wrench.torque, 'x', 0.0)),
            ty=float(getattr(wrench.torque, 'y', 0.0)),
            tz=float(getattr(wrench.torque, 'z', 0.0)),
            stamp=getattr(msg.header, 'stamp', None),
            frame_id=getattr(msg.header, 'frame_id', ''),
        )
        self._latest = reading
        self._last_seen = time.monotonic()
        self._emit(reading)

    def latest(self) -> Optional[WrenchReading]:
        return self._latest
