#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lazy accessors for vendor message classes (design doc §3 constraint 2).

Every accessor imports the vendor package inside the function. Returned
errors carry installation guidance for the robot host environment.
"""

from __future__ import annotations

import importlib
from typing import Optional, Tuple


def _resolve_msg(package: str, *names: str) -> Tuple[Optional[type], str]:
    try:
        module = importlib.import_module(f'{package}.msg')
    except Exception as exc:
        return None, (f'{package} not importable ({exc}); expected on the '
                      'robot host (/opt/humanoid/install or ~/xos)')
    for name in names:
        if hasattr(module, name):
            return getattr(module, name), ''
    return None, f'none of {names} found in {package}.msg'


def robot_state_msg():
    return _resolve_msg('ros2_bridge_msgs', 'RobotState')


def joint_cmd_msg(group: str):
    """ArmCtrl confirmed by demo; head/waist/leg classes follow the SDK
    naming convention and fall back to ArmCtrl (identical field set per
    HWI §7.1) when absent - a warning is logged by the caller."""
    if group == 'arm':
        return _resolve_msg('ros2_bridge_msgs', 'ArmCtrl')
    msg, err = _resolve_msg(
        'ros2_bridge_msgs', f'{group.capitalize()}Ctrl', 'ArmCtrl')
    return msg, err


def key_status_msg():
    """E-stop message class name is not pinned by the demos (HWI §7.1
    documents the fields is_estop / is_remote_estop only). The real robot
    publishes bodyctrl_msgs/msg/PowerBoardKeyStatus on /power/board/key_status
    (verified on the Tienkung 3.0 host); probe that first, then the
    ros2_bridge_msgs candidates. Returns the first class exposing is_estop."""
    msg, err = _resolve_msg('bodyctrl_msgs', 'PowerBoardKeyStatus')
    if msg is not None and hasattr(msg, 'is_estop'):
        return msg, ''
    candidates = ('KeyStatus', 'PowerKeyStatus', 'BoardKeyStatus',
                  'EstopStatus', 'PowerBoardStatus')
    for name in candidates:
        msg, err = _resolve_msg('ros2_bridge_msgs', name)
        if msg is not None and hasattr(msg, 'is_estop'):
            return msg, ''
    return None, (f'no e-stop message exposing is_estop among '
                  'bodyctrl_msgs.PowerBoardKeyStatus and the '
                  f'ros2_bridge_msgs candidates {candidates}')


def hand_msgs():
    return _resolve_msg(
        'brainco_hand_msgs', 'SetMotorMulti', 'MotorStatus', 'TouchStatus')


def tts_service():
    try:
        module = importlib.import_module('interaction_msgs.srv')
    except Exception as exc:
        return None, f'interaction_msgs not importable ({exc})'
    if hasattr(module, 'TtsService'):
        return getattr(module, 'TtsService'), ''
    return None, 'TtsService not found in interaction_msgs.srv'


def audio_msgs():
    return _resolve_msg('lyre_msgs', 'AudioControl', 'AudioFrame',
                        'LyreVoiceActivity')


def gps_msg():
    return _resolve_msg('navigation_msgs', 'GpsFix')


def force_msg():
    """Six-axis force source is unverified (HWI §7): accept either the
    vendor message or the standard WrenchStamped. Callers treat the
    returned class as opaque and extract via the adapter."""
    msg, err = _resolve_msg('ros2_bridge_msgs', 'ForceState')
    if msg is not None:
        return msg, ''
    try:
        module = importlib.import_module('geometry_msgs.msg')
        return module.WrenchStamped, ''
    except Exception as exc:
        return None, f'force message unavailable ({exc})'


def power_msgs():
    """(PowerBatteryStatus, PowerStatus, PowerBoardKeyStatus) tuple from
    bodyctrl_msgs; (None, ...) + error when missing."""
    battery, e1 = _resolve_msg('bodyctrl_msgs', 'PowerBatteryStatus')
    board, e2 = _resolve_msg('bodyctrl_msgs', 'PowerStatus')
    key, e3 = _resolve_msg('bodyctrl_msgs', 'PowerBoardKeyStatus')
    errors = [e for e in (e1, e2, e3) if e]
    return (battery, board, key), (errors[0] if errors else '')


def light_msg():
    return _resolve_msg('bodyctrl_msgs', 'LightCtrl')


def sbus_event_msg():
    return _resolve_msg('bodyctrl_msgs', 'SbusData')


def serial_service():
    try:
        module = importlib.import_module('xsys_msgs.srv')
    except Exception as exc:
        return None, f'xsys_msgs not importable ({exc})'
    if hasattr(module, 'GetSerialNumber'):
        return getattr(module, 'GetSerialNumber'), ''
    return None, 'GetSerialNumber not found in xsys_msgs.srv'


def inspire_hand_msgs():
    """(SetAngle, SetForce, SetSpeed, GetAngleAct, GetForceAct, TouchData)
    from inspire_hand_msgs; TouchData is optional."""
    names = ('SetAngle', 'SetForce', 'SetSpeed',
             'GetAngleAct', 'GetForceAct', 'TouchData')
    try:
        module = importlib.import_module('inspire_hand_msgs.msg')
    except Exception as exc:
        return (None,) * len(names), \
            f'inspire_hand_msgs not importable ({exc})'
    classes = [getattr(module, n, None) for n in names]
    missing = [n for n, c in zip(names, classes) if c is None]
    if missing:
        return (None,) * len(names), \
            f'missing in inspire_hand_msgs.msg: {missing}'
    return tuple(classes), ''


def clear_error_service():
    """SetClearError service (bodyctrl_msgs)."""
    try:
        module = importlib.import_module('bodyctrl_msgs.srv')
    except Exception as exc:
        return None, f'bodyctrl_msgs not importable ({exc})'
    if hasattr(module, 'SetClearError'):
        return getattr(module, 'SetClearError'), ''
    return None, 'SetClearError not found in bodyctrl_msgs.srv'
