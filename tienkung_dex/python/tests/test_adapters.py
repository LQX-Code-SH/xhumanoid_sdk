#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Adapter pure-function tests (design doc §5 conversion contracts).

All conversions are pure functions taking duck-typed message stubs, so
they run without rclpy or any vendor message package.
"""

from types import SimpleNamespace

import numpy as np

from tienkung_dex.backends.real.audio import parse_voice_activity
from tienkung_dex.backends.real.camera import (RealCameraStream, decode_color,
                                                decode_depth)
from tienkung_dex.backends.real.hand import RealDexterousHand
from tienkung_dex.backends.real.joint import parse_robot_state
from tienkung_dex.backends.real.power import (RealPowerSystem, parse_battery)
from tienkung_dex.backends.real.safety import _bool_field
from tienkung_dex.backends.real.sensors import quaternion_to_euler
from tienkung_dex.backends.real.sbus import RealSbusStream
from tienkung_dex.core import topics as t


def _stamp(sec: float):
    return SimpleNamespace(sec=int(sec), nanosec=int((sec % 1) * 1e9))


def _img_msg(encoding, array, stamp_sec, frame_id='cam'):
    return SimpleNamespace(height=array.shape[0], width=array.shape[1],
                           encoding=encoding, data=array.tobytes(),
                           header=SimpleNamespace(stamp=_stamp(stamp_sec),
                                                  frame_id=frame_id))


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


def test_parse_robot_state_non_numeric_name_skipped():
    # SDK drift guard: a string name must be skipped, not raise inside the
    # subscription callback.
    msg = SimpleNamespace(
        arm=SimpleNamespace(status=[
            SimpleNamespace(name=21, pos=0.1, vel=0.0, tor=0.0),
            SimpleNamespace(name='arm_pitch', pos=0.2, vel=0.0, tor=0.0),
        ]),
        head=None, waist=None, leg=None, imu=None)
    groups = parse_robot_state(msg)
    assert set(groups['arm']) == {21}


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


def test_parse_battery_master_fields():
    msg = SimpleNamespace(master_battery_voltage=48.2,
                          master_battery_current=-1.5,
                          master_battery_power=72.3)
    assert parse_battery(msg) == (48.2, -1.5, 72.3)


def test_parse_battery_missing_fields_default_zero():
    assert parse_battery(SimpleNamespace()) == (0.0, 0.0, 0.0)


def test_bool_field_std_msgs_wrapper_unwraps_data():
    # PowerBoardKeyStatus declares is_estop as std_msgs/Bool: the object is
    # always truthy, the real value lives in .data (regression: facade
    # permanently reported e-stop active).
    msg = SimpleNamespace(is_estop=SimpleNamespace(data=False),
                          is_remote_estop=SimpleNamespace(data=True))
    assert _bool_field(msg, 'is_estop') is False
    assert _bool_field(msg, 'is_remote_estop') is True


def test_bool_field_plain_bool_and_missing_field():
    msg = SimpleNamespace(is_estop=True)
    assert _bool_field(msg, 'is_estop') is True
    assert _bool_field(msg, 'is_remote_estop') is False


# --- regression: safety is_active follows stream staleness -----------------

def test_safety_is_active_staleness():
    # /power/board/key_status measured ~12.6 Hz on the real host; 0.5 s
    # stale_timeout has >6 message periods of margin. is_active must be
    # False before the first message and once the stream goes silent.
    import time as _time

    from tienkung_dex.backends.real.safety import RealSafetyMonitor
    safety = RealSafetyMonitor(None, '/power/board/key_status', None,
                               stale_timeout=0.5)
    assert not safety.is_active                       # no message yet

    safety._on_key_status(SimpleNamespace(is_estop=False,
                                          is_remote_estop=False))
    assert safety.is_active                           # stream flowing

    safety._last_seen = _time.monotonic() - 1.0
    assert not safety.is_active                       # stream silent

    # Edge emission still works after the staleness change.
    events = []
    safety.on_estop(events.append)
    safety._on_key_status(SimpleNamespace(is_estop=True,
                                          is_remote_estop=False))
    assert events == [True] and safety.is_estopped


# --- regression: power/sbus must never emit None into on_update -----------

def _power_system():
    return RealPowerSystem(None, topics=dict(t.POWER_TOPICS))


def test_power_key_before_battery_does_not_emit_none():
    # key_status arriving before any battery message: latest() is None and
    # the reading must be withheld, not delivered as None (contract break).
    power = _power_system()
    updates = []
    power.on_update(updates.append)
    power._on_key(SimpleNamespace(is_estop=True, is_power_on=True))
    assert updates == []
    assert power.latest() is None

    power._on_battery(SimpleNamespace(master_battery_voltage=48.0,
                                      master_battery_current=2.0,
                                      master_battery_power=96.0))
    assert len(updates) == 1 and updates[0].voltage == 48.0

    power._on_key(SimpleNamespace(is_estop=True, is_power_on=True))
    assert updates[-1].is_estop and updates[-1].voltage == 48.0


def test_sbus_event_before_joy_emits_buttons_and_counts_liveness():
    # A button event before the first Joy message must not emit None and
    # must keep is_active fresh (event-only stream).
    sbus = RealSbusStream(None, topics={'joy': '/sbus_data',
                                        'event': '/sbus_data/event'})
    updates = []
    sbus.on_update(updates.append)
    sbus._on_event(SimpleNamespace(button_a=1, button_b=0, button_c=0,
                                   button_d=0, button_e=0, button_f=0))
    assert len(updates) == 1
    assert updates[0].buttons == (1, 0, 0, 0, 0, 0)
    assert updates[0].axes == ()
    assert sbus.is_active

    sbus._on_joy(SimpleNamespace(axes=(0.5, -0.2)))
    assert updates[-1].axes == (0.5, -0.2)
    assert updates[-1].buttons == (1, 0, 0, 0, 0, 0)


# --- regression: stale plane must not block the camera stream -------------

def _camera():
    stream = RealCameraStream(
        None, 'ob_camera_head',
        topics=t.camera_topics('ob_camera_head'), logger=None)
    frames = []
    stream.on_frame(frames.append)
    return stream, frames


def _color_msg(stamp_sec):
    return _img_msg('bgr8', np.full((2, 2, 3), 8, dtype=np.uint8), stamp_sec,
                    frame_id='ob_camera_head')


def _depth_msg(stamp_sec):
    return _img_msg('16UC1', np.full((2, 2), 1000, dtype=np.uint16),
                    stamp_sec, frame_id='ob_camera_head')


def test_camera_pairs_within_window():
    stream, frames = _camera()
    stream._on_image('depth', _depth_msg(100.00))
    stream._on_image('color', _color_msg(100.01))
    assert len(frames) == 2
    assert frames[-1].color is not None and frames[-1].depth is not None


def test_camera_stale_depth_degrades_to_color_only():
    # Depth stops updating: the next color frame must be emitted color-only
    # (CameraFrame contract) and the stream must keep flowing afterwards.
    stream, frames = _camera()
    stream._on_image('depth', _depth_msg(100.00))
    stream._on_image('color', _color_msg(100.01))
    assert frames[-1].depth is not None

    stream._on_image('color', _color_msg(101.00))   # depth now 1 s stale
    assert frames[-1].color is not None
    assert frames[-1].depth is None                 # drop-out, not a block

    stream._on_image('color', _color_msg(101.03))   # keeps flowing
    assert len(frames) == 4
    assert frames[-1].color is not None and frames[-1].depth is None


def test_camera_stale_color_degrades_to_depth_only():
    stream, frames = _camera()
    stream._on_image('color', _color_msg(100.00))
    stream._on_image('depth', _depth_msg(100.01))
    assert frames[-1].color is not None

    stream._on_image('depth', _depth_msg(101.00))   # color now stale
    assert frames[-1].color is None
    assert frames[-1].depth is not None


# --- regression: brainco set_speed must not publish positions -------------

def test_brainco_set_speed_is_ignored_not_published():
    # Old behaviour published POS_MIN positions alongside the speeds, which
    # physically straightened the hand; now it is accepted and ignored like
    # set_force. Not started -> a publish attempt would raise RuntimeError.
    hand = RealDexterousHand(
        None, 'right', topics=dict(t.HAND_TOPICS['right']), logger=None)
    hand.set_speed([100] * 6)          # must not raise ('not started')
    hand.set_force([100] * 6)          # symmetry with set_force
    assert hand.get_status() is None   # nothing was published/changed


# --- regression: factory hand failure must not crash without a logger -----

def test_factory_hand_failure_without_logger(monkeypatch):
    from tienkung_dex.backends.real import factory as factory_mod

    def boom(*_args, **_kwargs):
        raise RuntimeError('vendor package missing')

    monkeypatch.setattr(factory_mod, 'RealDexterousHand', boom)
    factory = factory_mod.RealBackendFactory(
        None, None, None, enable={'hand'})
    subsystems = factory.build()       # logger=None: must not raise
    assert 'hand_left' not in subsystems and 'hand_right' not in subsystems
    assert 'safety' in subsystems


# --- regression: start_recording must not report success on a dead service -

def test_start_recording_unreachable_service_returns_false():
    from tienkung_dex.backends.real.audio import RealAudioSystem
    audio = RealAudioSystem(None, logger=None, tts_service='/tts',
                            audio_control='/ctrl', audio_stream='/stream',
                            voice_activity='/voice')
    audio._ctrl_cls = SimpleNamespace(
        Request=lambda: SimpleNamespace(enable=True))
    audio._ctrl_client = SimpleNamespace(
        wait_for_service=lambda timeout_sec=None: False)
    assert audio.start_recording() is False
    assert audio.stop_recording() == []      # nothing was buffered


# --- sim factory flags enable keys it cannot simulate ---------------------

def test_sim_factory_warns_on_unsupported_enable():
    from tienkung_dex.backends.sim.factory import SimBackendFactory
    warned = []
    logger = SimpleNamespace(warn=warned.append,
                             info=lambda _m: None,
                             error=lambda _m: None)
    factory = SimBackendFactory(None, logger, None,
                                enable={'joint', 'power'})
    assert any('power' in m for m in warned)
    # build stays functional for the supported subset
    subsystems = factory.build()
    assert 'joint_arm' in subsystems and 'power' not in subsystems
