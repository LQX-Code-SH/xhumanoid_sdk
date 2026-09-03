#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Value objects crossing the facade boundary (design doc §4.1).

All are frozen dataclasses: immutable snapshots safe to share between the
executor callback thread and caller threads without copying.
"""

from __future__ import annotations

import math
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
class VelocityCommand:
    """Robot-body-frame velocity-vector setpoint (矢量行走, HRIC cmd_vel).

    vx: forward speed (m/s), vy: lateral speed (m/s), wz: turning speed
    (rad/s). Per the vector-walk interface doc, ||(vx, vy, wz)|| < 0.05
    makes the locomotion policy hold standing - backends treat any such
    request as a full-zero stop setpoint.
    """

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    @property
    def norm(self) -> float:
        """L2 norm of the velocity vector (m/s-equivalent units)."""
        return math.sqrt(self.vx * self.vx + self.vy * self.vy
                         + self.wz * self.wz)


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


@dataclass(frozen=True)
class PowerReading:
    """Power snapshot from /power/battery|board/status .

    Battery fields are the master-battery values the demo prints
    (voltage/current/power); key fields come from /power/board/key_status
    and share the safety monitor's std_msgs/Bool unwrap semantics.
    """

    voltage: float = 0.0        # master battery voltage (V)
    current: float = 0.0        # master battery current (A)
    power_w: float = 0.0        # master battery power (W)
    is_estop: bool = False      # e-stop button
    is_power_on: bool = False   # power supply state
    stamp: Optional[object] = None


# 遥控器按键事件键码 —— 与 bodyctrl_msgs/SbusData.key_event_* 常量一致
# (SbusData.msg: KEY_NONE=0 .. KEY_H_RIGHT=20)。事件语义（vendor 注释）：
#   A-D  两态按键   UP=松开 / DOWN=按下
#   E-F  三档拨动   UP=上拨 / MID=中间 / DOWN=下拨
#   G-H  左右拨杆   LEFT=左拨 / MID=中间 / RIGHT=右拨
# key_event_new = 变化后的事件；key_event_old = 变化前的事件。
# button_a..h 电平位编码（vendor 注释）：-1=松开/复位, 0=中间位置,
# 1=按下/拨到一端, 2=拨到另一端（两态键 A-D 一般仅出现 -1/1）。
SBUS_KEY_NAME = {
    0: 'NONE',
    1: 'A_UP', 2: 'A_DOWN',
    3: 'B_UP', 4: 'B_DOWN',
    5: 'C_UP', 6: 'C_DOWN',
    7: 'D_UP', 8: 'D_DOWN',
    9: 'E_UP', 10: 'E_MID', 11: 'E_DOWN',
    12: 'F_UP', 13: 'F_MID', 14: 'F_DOWN',
    15: 'G_LEFT', 16: 'G_MID', 17: 'G_RIGHT',
    18: 'H_LEFT', 19: 'H_MID', 20: 'H_RIGHT',
}


@dataclass(frozen=True)
class SbusReading:
    """RC transmitter snapshot : joy axes + button events.

    axes follows sensor_msgs/Joy (/sbus_data) - the vendor publishes 12
    raw axes in [-1, 1] covering 3 sticks (axis-index mapping pending a live
    capture) and leaves Joy.buttons empty; buttons holds the raw
    button_a..button_h fields of bodyctrl_msgs/SbusData (/sbus_data/event)
    (8 keys, A-H), each encoded as -1 released / 0 middle / 1 or 2 an end
    position, and stays empty when the vendor message package is
    unavailable. event_new/event_old are the SbusData key_event codes after
    and before the change (see SBUS_KEY_NAME / key_name()).
    """

    axes: Tuple[float, ...] = ()
    buttons: Tuple[int, ...] = ()   # (A..H) -1松/0中/1端/2端
    event_new: int = 0              # 变化后事件键码 (key_event_new)
    event_old: int = 0              # 变化前事件键码 (key_event_old)
    stamp: Optional[object] = None

    @staticmethod
    def key_name(code: int) -> str:
        """Readable name of a SbusData key code (e.g. 17 -> 'G_RIGHT')."""
        return SBUS_KEY_NAME.get(int(code), f'UNKNOWN({int(code)})')
