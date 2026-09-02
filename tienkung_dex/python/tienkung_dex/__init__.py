#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tienkung_dex: TienkungDex robot facade library.

Design document: docs/详细设计/09-TienkungDex机器人类设计.md (main repo).

Top-level API:
    from tienkung_dex import TienkungDex, create_robot
    robot = create_robot(node, backend='real')
    robot.start()
    ...
    robot.shutdown()

Importing this package never imports rclpy or vendor message packages
(ros2_bridge_msgs / lyre_msgs / ...); both happen lazily inside the real
backend, so the package remains importable off-robot (sim/mock backends,
unit tests).
"""

from .core.errors import (
    BackendUnavailableError,
    EstopActiveError,
    JointIdError,
    RobotError,
    UnsafeModeError,
)
from .core.types import (
    AudioChunk,
    CameraFrame,
    ControlMode,
    GpsFixReading,
    HandStatus,
    ImuReading,
    JointCommand,
    JointReading,
    PowerReading,
    SbusReading,
    TouchReading,
    WrenchReading,
)
from .robot import TienkungDex, create_robot

__version__ = '0.1.0'

__all__ = [
    'TienkungDex',
    'create_robot',
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
    'PowerReading',
    'SbusReading',
    'RobotError',
    'BackendUnavailableError',
    'EstopActiveError',
    'UnsafeModeError',
    'JointIdError',
]
