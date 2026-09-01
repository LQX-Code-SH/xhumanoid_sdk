#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Adapter pure-function tests (design doc §5 conversion contracts).

All conversions are pure functions taking duck-typed message stubs, so
they run without rclpy or any vendor message package.
"""

from types import SimpleNamespace

import numpy as np

from tienkung_dex.backends.real.audio import parse_voice_activity
from tienkung_dex.backends.real.camera import decode_color, decode_depth
from tienkung_dex.backends.real.joint import parse_robot_state
from tienkung_dex.backends.real.sensors import quaternion_to_euler


def _image(encoding, array):
    return SimpleNamespace(height=array.shape[0], width=array.shape[1],
                           encoding=encoding, data=array.tobytes())


def test_depth_16uc1_is_mm():
    array = np.array([[100, 0], [2500, 65535]], dtype=np.uint16)
    out = decode_depth(_image('16UC1', array))
    assert out.dtype == np.uint16
    assert out[0, 0] == 100 and out[1, 1] == 65535


def test_depth_32fc1_metres_to_mm():
    array = np.array([[0.5, 0.0], [1.25, 2.0]], dtype=np.float32)
    out = decode_depth(_image('32FC1', array))
    assert out[0, 0] == 500 and out[1, 0] == 1250


def test_depth_unsupported_encoding_returns_none():
    array = np.zeros((2, 2), dtype=np.uint8)
    assert decode_depth(_image('8UC1', array)) is None


def test_color_rgb8_to_bgr():
    array = np.zeros((1, 2, 3), dtype=np.uint8)
    array[0, 0] = (10, 20, 30)          # R=10, G=20, B=30
    out = decode_color(_image('rgb8', array))
    assert tuple(out[0, 0]) == (30, 20, 10)


def test_color_rgba8_drops_alpha():
    array = np.zeros((1, 1, 4), dtype=np.uint8)
    array[0, 0] = (10, 20, 30, 255)
    out = decode_color(_image('rgba8', array))
    assert out.shape == (1, 1, 3)
    assert tuple(out[0, 0]) == (30, 20, 10)


def test_color_bgr8_passthrough():
    array = np.zeros((1, 1, 3), dtype=np.uint8)
    array[0, 0] = (30, 20, 10)
    out = decode_color(_image('bgr8', array))
    assert tuple(out[0, 0]) == (30, 20, 10)


def test_parse_robot_state_arm_status_by_name():
    msg = SimpleNamespace(
        arm=SimpleNamespace(status=[
            SimpleNamespace(name=21, pos=-1.588, vel=0.1, tor=0.2),
            SimpleNamespace(name=22, pos=0.0, vel=0.0, tor=0.0),
        ]),
        head=SimpleNamespace(status=[]),
        waist=None,
        leg=SimpleNamespace(status=None),
        imu=None,
    )
    groups = parse_robot_state(msg)
    assert 21 in groups['arm'] and 22 in groups['arm']
    reading = groups['arm'][21]
    assert reading.pos == -1.588 and reading.vel == 0.1 and reading.tor == 0.2
    assert groups['head'] == {}
    assert groups['waist'] == {}
    assert groups['leg'] == {}


def test_parse_robot_state_missing_group_fields_tolerated():
    msg = SimpleNamespace()          # no arm/head/waist/leg at all
    groups = parse_robot_state(msg)
    assert set(groups) == {'arm', 'head', 'waist', 'leg'}
    assert all(value == {} for value in groups.values())


def test_quaternion_identity():
    roll, pitch, yaw = quaternion_to_euler(1.0, 0.0, 0.0, 0.0)
    assert (roll, pitch, yaw) == (0.0, 0.0, 0.0)


def test_quaternion_yaw_90():
    # 90 deg yaw about Z: w=cos45, z=sin45
    import math
    s = math.sin(math.pi / 4)
    _, _, yaw = quaternion_to_euler(math.cos(math.pi / 4), 0.0, 0.0, s)
    assert abs(yaw - math.pi / 2) < 1e-9


def test_voice_activity_asr_result():
    payload = ('{"type":"aiui_event","traceId":"t1","content":'
               '{"eventType":1,"result":{"text":{"ws":[{"cw":[{"w":"你"},'
               '{"w":"好"}]}]}}}}')
    event = parse_voice_activity(payload)
    assert event['event_type'] == 1
    assert event['name'] == 'asr_result'
    assert event['text'] == '你好'
    assert event['trace_id'] == 't1'


def test_voice_activity_keyword_wake():
    payload = ('{"type":"aiui_event","traceId":"t2","content":'
               '{"eventType":4,"result":{"ivw":{"angle":45}}}}')
    event = parse_voice_activity(payload)
    assert event['event_type'] == 4 and event['angle'] == 45


def test_voice_activity_non_aiui_and_invalid():
    assert parse_voice_activity('{"type":"other"}')['event_type'] == -1
    assert parse_voice_activity('not json') is None
    assert parse_voice_activity('') is None
