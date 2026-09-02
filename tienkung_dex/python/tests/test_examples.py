#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Examples demo 结构与 mock 运行测试（实施计划 §四）。

数字开头的模块名不能 `from examples.01_... import`（语法非法），
一律 importlib.import_module 加载。
"""

import importlib
import os
import sys

import pytest

_PYTHON_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PYTHON_DIR not in sys.path:
    sys.path.insert(0, _PYTHON_DIR)

from tienkung_dex import TienkungDex

# module -> demo class（与 examples/README.md 的列表一一对应）
MODULES = {
    '01_robot_state': 'RobotStateDemo',
    '02_imu_demo': 'ImuDemo',
    '03_head_control_demo': 'HeadControlDemo',
    '04_arm_control_demo': 'ArmControlDemo',
    '05_waist_control_demo': 'WaistControlDemo',
    '06_leg_control_demo': 'LegControlDemo',
    '07_hand_control_demo': 'HandControlDemo',
    '08_power_status_demo': 'PowerStatusDemo',
    '09_sbus_demo': 'SbusDemo',
    '10_lidar_demo': 'LidarDemo',
    '11_audio_ai_demo': 'AudioAiDemo',
    '12_camera_demo': 'CameraDemo',
    '13_gps_demo': 'GpsDemo',
    '14_light_demo': 'LightDemo',
    '15_hand_state_demo': 'HandStateDemo',
    '16_serial_number_demo': 'SerialNumberDemo',
    '17_safety_demo': 'SafetyDemo',
    '18_audio_record_demo': 'AudioRecordDemo',
    '19_joint_modes_demo': 'JointModesDemo',
}


def _load(module_name: str):
    mod = importlib.import_module(f'examples.{module_name}')
    return getattr(mod, MODULES[module_name])


@pytest.mark.parametrize('module_name', sorted(MODULES),
                         ids=sorted(MODULES))
def test_demo_class_inherits_facade(module_name):
    cls = _load(module_name)
    assert issubclass(cls, TienkungDex)
    mod = importlib.import_module(f'examples.{module_name}')
    assert callable(getattr(mod, 'main'))


@pytest.mark.parametrize('module_name', sorted(MODULES),
                         ids=sorted(MODULES))
def test_demo_runs_green_on_mock(module_name, capsys):
    cls = _load(module_name)
    demo = cls(backend='mock')
    assert demo.run(observe_seconds=0) == 0
    out = capsys.readouterr().out
    assert '结果: 全部通过' in out


def test_joint_modes_demo_refuses_real():
    mod = importlib.import_module('examples.19_joint_modes_demo')
    with pytest.raises(SystemExit) as exc:
        mod.JointModesDemo(backend='real')
    assert exc.value.code == 2
