#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""core: value objects, subsystem abstract bases and topic constants.

Constraint (design doc §3): this package depends only on standard ROS 2
messages (std_msgs / sensor_msgs / geometry_msgs / builtin_interfaces) and
never imports rclpy or vendor message packages at module level.
"""

from .base import (
    AudioSystemBase,
    CameraStreamBase,
    DexterousHandBase,
    ForceStreamBase,
    GpsStreamBase,
    ImuStreamBase,
    JointGroupBase,
    LidarStreamBase,
    MultiCameraGroupBase,
    SafetyMonitorBase,
    SubsystemBase,
)
from .errors import (
    BackendUnavailableError,
    EstopActiveError,
    JointIdError,
    RobotError,
    UnsafeModeError,
)
from .types import (
    AudioChunk,
    CameraFrame,
    ControlMode,
    GpsFixReading,
    HandStatus,
    ImuReading,
    JointCommand,
    JointReading,
    TouchReading,
    WrenchReading,
)

__all__ = [
    'SubsystemBase',
    'JointGroupBase',
    'DexterousHandBase',
    'CameraStreamBase',
    'MultiCameraGroupBase',
    'AudioSystemBase',
    'SafetyMonitorBase',
    'ImuStreamBase',
    'LidarStreamBase',
    'GpsStreamBase',
    'ForceStreamBase',
    'ControlMode',
    'JointCommand',
    'JointReading',
    'CameraFrame',
    'ImuReading',
    'HandStatus',
    'TouchReading',
    'AudioChunk',
    'GpsFixReading',
    'WrenchReading',
    'RobotError',
    'BackendUnavailableError',
    'EstopActiveError',
    'UnsafeModeError',
    'JointIdError',
]
