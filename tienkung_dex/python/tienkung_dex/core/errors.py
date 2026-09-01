#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Exception hierarchy of tienkung_dex (design doc §7.2)."""


class RobotError(Exception):
    """Base class for all tienkung_dex errors."""


class BackendUnavailableError(RobotError):
    """Requested backend cannot be used (missing vendor messages, or an
    unsupported backend/vendor combination such as inspire + sim)."""


class EstopActiveError(RobotError):
    """A control command was rejected because the robot e-stop is active.

    L1 redundancy layer: raised by JointGroupBase.command() *before* any
    message is published (design doc §4.6 / §7.2).
    """


class UnsafeModeError(RobotError):
    """A mode flagged as dangerous (ZERO_CALIB = 3) was requested without
    calling unlock_calibration_mode() first (design doc §4.2)."""


class JointIdError(RobotError):
    """A joint ID is unknown to the loaded joints table (appendix A).

    Raised in strict mode only; non-strict mode logs a warning instead.
    """
