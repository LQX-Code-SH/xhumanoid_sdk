#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Headless facade + mock backend behaviour tests (design doc §6/§7)."""

import time

import numpy as np
import pytest

from tienkung_dex import (BackendUnavailableError, CameraFrame, ControlMode,
                          EstopActiveError, JointCommand, TienkungDex,
                          UnsafeModeError, create_robot)


@pytest.fixture()
def robot():
    instance = create_robot(None, backend='mock')
    instance.start()
    yield instance
    instance.shutdown()


def test_facade_assembles_all_subsystems(robot):
    assert isinstance(robot, TienkungDex)
    for group in ('arm', 'head', 'waist', 'leg'):
        assert getattr(robot, group) is not None
    assert set(robot.cameras) == {
        'ob_camera_head', 'ob_camera_waist',
        'ob_camera_wrist_left', 'ob_camera_wrist_right'}
    assert robot.hand_left is not None and robot.hand_right is not None
    assert robot.audio is not None and robot.safety is not None
    assert robot.imu is not None and robot.lidar is not None
    assert robot.gps is not None and robot.force is not None
    assert robot.panorama is None  # excluded by default (optional hardware)


def test_health_report(robot):
    health = robot.health()
    assert 'joint_arm' in health and 'safety' in health
    assert 'panorama' not in health
    assert 'backend=mock' in robot.report()


def test_position_mode_snap_when_no_speed(robot):
    robot.arm.move_to({21: -1.588})
    reading = robot.arm.get_state(21)
    assert reading is not None
    assert abs(reading.pos - (-1.588)) < 1e-9


def test_impedance_mode_publishes_targets(robot):
    robot.arm.impedance({21: -0.5})
    reading = robot.arm.get_state(21)
    assert reading.pos == -0.5


def test_estop_intercepts_command(robot):
    events = []
    robot.safety.on_estop(events.append)
    robot.safety.set_estop(True)
    assert robot.safety.is_estopped
    with pytest.raises(EstopActiveError):
        robot.arm.move_to({21: 0.0})
    robot.safety.set_estop(False)
    assert events == [True, False]   # edge-triggered only
    robot.arm.move_to({21: 0.0})     # now allowed
    assert robot.arm.get_state(21).pos == 0.0


def test_zero_calib_mode_locked(robot):
    with pytest.raises(UnsafeModeError):
        robot.arm.command([JointCommand(joint_id=21)], ControlMode.ZERO_CALIB)
    robot.arm.unlock_calibration_mode()
    robot.arm.command([JointCommand(joint_id=21)], ControlMode.ZERO_CALIB)


def test_step_integration_and_wait_until(robot):
    robot.arm.set_position(21, 0.0)   # establish a position history first
    robot.arm.command([JointCommand(joint_id=21, pos=1.0, spd=0.5)])
    start = time.monotonic()
    while robot.arm.get_state(21).pos < 1.0 and time.monotonic() - start < 5:
        robot.arm.step(0.02)
    assert abs(robot.arm.get_state(21).pos - 1.0) < 1e-9
    # wait_until on an already-reached target returns immediately.
    assert robot.arm.wait_until(21, 1.0, tol_deg=1.0, timeout=1.0)
    # A target the joint never reaches times out (mock only steps when
    # driven; headless wait_until polls the static snapshot).
    robot.arm.command([JointCommand(joint_id=21, pos=0.0, spd=0.1)])
    assert not robot.arm.wait_until(21, -1.0, tol_deg=0.5, timeout=0.1)


def test_state_observer(robot):
    snapshots = []
    robot.arm.on_state(snapshots.append)
    robot.arm.move_to({21: 0.3})
    assert snapshots and 21 in snapshots[-1]


def test_camera_injection_and_observer(robot):
    camera = robot.cameras['ob_camera_head']
    frames = []
    camera.on_frame(frames.append)
    frame = CameraFrame(
        color=np.zeros((8, 8, 3), dtype=np.uint8),
        depth=np.full((8, 8), 500, dtype=np.uint16),
        frame_id='ob_camera_head')
    camera.publish_frame(frame)
    assert camera.latest() is frame
    assert frames == [frame]
    assert camera.frame_rate is None          # needs >= 2 stamps
    camera.publish_frame(frame)
    assert camera.frame_rate is not None


def test_voice_event_filter(robot):
    received = []
    robot.audio.on_voice_event(received.append, event_types={1})
    robot.audio.inject_voice_event(1, text='你好')
    robot.audio.inject_voice_event(4)          # filtered out
    assert len(received) == 1
    assert received[0]['text'] == '你好'


def test_speak_failure_injection(robot):
    robot.audio.set_speak_result(False)
    assert not robot.audio.speak('hello')
    robot.audio.set_speak_result(True)
    assert robot.audio.speak('hello')


def test_hand_gesture_preset(robot):
    assert robot.hand_right.set_gesture('ok')
    assert not robot.hand_right.set_gesture('nope')
    status = robot.hand_right.get_status()
    assert status.positions[0] == 450


def test_inspire_sim_rejected():
    with pytest.raises(BackendUnavailableError):
        create_robot(None, backend='sim', hand_vendor='inspire')


def test_unknown_backend_rejected():
    with pytest.raises(BackendUnavailableError):
        create_robot(None, backend='cyborg')


def test_real_backend_unavailable_off_robot():
    # ros2_bridge_msgs does not exist here: factory must fail with the
    # documented BackendUnavailableError, not an ImportError traceback.
    with pytest.raises(BackendUnavailableError):
        create_robot(None, backend='real')


def test_enable_subset(robot):
    instance = create_robot(None, backend='mock',
                            enable={'joint', 'safety'})
    instance.start()
    try:
        assert instance.arm is not None
        assert instance.cameras == {}          # camera excluded
        assert instance.audio is None
    finally:
        instance.shutdown()
