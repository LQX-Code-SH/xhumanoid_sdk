#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Headless facade + mock backend behaviour tests (design doc §6/§7)."""

import time

import numpy as np
import pytest

from tienkung_dex import (BackendUnavailableError, CameraFrame, ControlMode,
                          EstopActiveError, JointCommand, PowerReading,
                          SbusReading, TienkungDex, UnsafeModeError,
                          create_robot)


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
    assert robot.power is not None and robot.light is not None
    assert robot.sbus is not None and robot.serial is not None
    assert robot.walk is not None
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


def test_mock_publish_command_refreshes_liveness():
    # The real /robot_state echo follows a command within the staleness
    # window; the mock model must mirror that (regression: is_active used
    # to go stale after a command that never stepped).
    from tienkung_dex.backends.mock import MockJointGroup
    joint = MockJointGroup(None, 'arm')
    joint.start()
    joint._last_seen = time.monotonic() - 10.0      # age out construction
    assert not joint.is_active
    joint.move_to({21: 0.0})
    assert joint.is_active
    joint.shutdown()


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
        assert instance.power is None
        assert instance.light is None
        assert instance.walk is None           # walk excluded
    finally:
        instance.shutdown()


def test_vector_walk_velocity_stream_and_stop(robot):
    walk = robot.walk
    walk.set_velocity(vx=0.3, vy=0.1, wz=0.2)
    for _ in range(50):                        # drive the model 2.5 s
        walk.step(0.05)
    assert walk.velocity.vx == 0.3
    assert walk.pose_x > 0.6                   # 0.3 * 2.5 = 0.75 m
    assert walk.pose_y > 0.2 and walk.pose_yaw > 0.4
    assert walk.publish_count >= 1
    walk.stop()                                # zero setpoint, stream halts
    assert walk.velocity.norm == 0.0
    assert walk.history[-1].norm == 0.0
    for _ in range(10):
        walk.step(0.05)                        # standing: no more movement
    assert abs(walk.pose_x - 0.75) < 0.01


def test_vector_walk_clamp_estop_and_stop_norm(robot):
    walk = robot.walk
    walk.set_velocity(vx=99.0)                 # clamped to vx_max
    assert walk.velocity.vx == walk.limits['vx_max'] == 1.0
    walk.set_velocity(vx=0.02, vy=0.02)        # norm<0.05 -> standing zero
    assert walk.velocity.norm == 0.0
    robot.safety.set_estop(True)
    with pytest.raises(EstopActiveError):
        walk.set_velocity(vx=0.3)
    robot.safety.set_estop(False)
    walk.set_velocity(vx=0.3)                  # allowed again
    assert walk.velocity.vx == 0.3


def test_power_injection_and_observer(robot):
    readings = []
    robot.power.on_update(readings.append)
    robot.power.inject(PowerReading(voltage=48.1, current=2.5,
                                    power_w=120.0))
    latest = robot.power.latest()
    assert latest.voltage == 48.1
    assert latest.power_w == 120.0
    assert readings == [latest]


def test_light_set_mode_and_command_history(robot):
    assert robot.light.set_mode('wakeup')
    assert robot.light.commands == [(301, ())]
    assert not robot.light.set_mode('disco')
    robot.light.set_cmd(7, (1, 2))
    assert robot.light.commands[-1] == (7, (1, 2))


def test_sbus_injection(robot):
    robot.sbus.inject(SbusReading(axes=(0.5, -0.2, 0.0, 0.0),
                                  buttons=(1, 0, 0, 0, 0, 0),
                                  event_new=17, event_old=16))
    latest = robot.sbus.latest()
    assert latest.axes[0] == 0.5
    assert latest.buttons[0] == 1
    assert latest.event_new == 17 and latest.event_old == 16


def test_serial_number(robot):
    assert robot.serial.get_serial_number() == 'MOCK-SN-0000'
    robot.serial.set_serial('SN-1234')
    assert robot.serial.get_serial_number() == 'SN-1234'
