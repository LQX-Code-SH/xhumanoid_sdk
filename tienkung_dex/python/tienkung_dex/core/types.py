#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Value objects crossing the facade boundary (design doc §4.1).

All are frozen dataclasses: immutable snapshots safe to share between the
executor callback thread and caller threads without copying.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple

import numpy as np

try:
    from builtin_interfaces.msg import Time as RosTime  # type: ignore
except Exception:  # pragma: no cover - off-robot environments
    RosTime = None  # type: ignore


class ControlMode(IntEnum):
    """Joint control modes, aligned with the confirmed SDK modes 0-5
    (HWI §7.1). Mode 3 (ZERO_CALIB) is dangerous and locked by default.
    """

    POSITION = 0       # pos / spd / cur
    IMPEDANCE = 1      # kp / kd / pos (force-position hybrid)
    VELOCITY = 2
    ZERO_CALIB = 3     # zero calibration - requires unlock_calibration_mode()
    DISTANCE = 4
    CURRENT = 5


@dataclass(frozen=True)
class JointCommand:
    """A single-joint command (unit conventions follow the SDK demo)."""

    joint_id: int          # SDK joint number; full mapping: design appendix A
    pos: float = 0.0       # rad
    spd: float = 0.0       # speed limit
    cur: float = 0.0       # current limit (position mode)
    kp: float = 0.0        # stiffness (impedance mode)
    kd: float = 0.0        # damping (impedance mode)
    tor: float = 0.0       # torque feedforward


@dataclass(frozen=True)
class JointReading:
    """Observed joint state from /robot_state (design doc §4.1)."""

    joint_id: int
    pos: float                 # rad
    vel: float = 0.0
    tor: float = 0.0
    stamp: Optional[object] = None   # builtin_interfaces Time when available


@dataclass(frozen=True)
class CameraFrame:
    """One paired color+depth frame (design doc §4.1).

    color: BGR uint8 (H, W, 3); depth: uint16 (H, W) in millimetres,
    0 = invalid. Either plane may be None (e.g. panoramic cameras have no
    depth; a depth drop-out still yields a color-only frame).
    """

    color: Optional[np.ndarray]
    depth: Optional[np.ndarray]
    stamp: Optional[object] = None
    frame_id: str = ''          # matches the topic namespace, e.g. ob_camera_head


@dataclass(frozen=True)
class ImuReading:
    """Unified IMU sample (livox sensor_msgs/Imu or xsens RobotState.imu).

    Unit note (design doc §11.4): xsens angles are passed through as the
    robot publishes them (both SDK demos treat them as degrees); livox is
    derived from a standard quaternion (rad). wx/wy/wz and ax/ay/az units
    follow the source message and are treated as unverified.
    """

    roll: float
    pitch: float
    yaw: float
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    stamp: Optional[object] = None
    source: str = ''            # 'livox' | 'xsens' | 'sim'


@dataclass(frozen=True)
class HandStatus:
    """Dexterous hand motor positions (vendor-specific scale preserved)."""

    positions: Tuple[int, ...] = ()
    stamp: Optional[object] = None
    raw: object = None          # opaque vendor message for power users


@dataclass(frozen=True)
class TouchReading:
    """One touch-sensor sample from an optional brainco hand (design §4.3).

    Values are raw vendor integers (force in 0.01 N units, 65535 = N/A);
    interpretation mirrors the touch_display demo.
    """

    values: Tuple[Tuple[int, ...], ...] = ()
    stamp: Optional[object] = None


@dataclass(frozen=True)
class AudioChunk:
    """One chunk of the /lyre/audio_stream (design doc §4.5)."""

    data: bytes = b''
    sample_rate: int = 0
    channels: int = 0
    bits_per_sample: int = 0
    stamp: Optional[object] = None

    @property
    def duration_seconds(self) -> float:
        """Length of this chunk in seconds (0 when format is unknown)."""
        if self.sample_rate <= 0 or self.bits_per_sample <= 0:
            return 0.0
        bytes_per_sample = max(self.bits_per_sample // 8, 1)
        samples = len(self.data) / max(bytes_per_sample * self.channels, 1)
        return samples / self.sample_rate


@dataclass(frozen=True)
class GpsFixReading:
    """One GPS fix (design doc §4.7, optional hardware)."""

    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    status: int = 0
    num_sats: int = 0
    hdop: float = 0.0
    speed: float = 0.0
    heading: float = 0.0
    stamp: Optional[object] = None

    @property
    def is_valid(self) -> bool:
        return self.status > 0


@dataclass(frozen=True)
class WrenchReading:
    """Six-axis force/torque sample (design doc §4.7, optional hardware)."""

    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    stamp: Optional[object] = None
    frame_id: str = ''
