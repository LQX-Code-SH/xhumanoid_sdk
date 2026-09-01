#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Value object tests (design doc §4.1)."""

import numpy as np

from tienkung_dex import (AudioChunk, CameraFrame, ControlMode, GpsFixReading,
                          JointCommand, JointReading, WrenchReading)


def test_control_mode_values_match_sdk():
    assert [m.value for m in ControlMode] == [0, 1, 2, 3, 4, 5]
    assert ControlMode(1) == ControlMode.IMPEDANCE


def test_joint_command_defaults():
    cmd = JointCommand(joint_id=21)
    assert cmd.pos == 0.0 and cmd.kp == 0.0 and cmd.tor == 0.0


def test_joint_command_frozen():
    cmd = JointCommand(joint_id=21, pos=-1.588)
    try:
        cmd.pos = 0.0
        raise AssertionError('dataclass(frozen=True) violated')
    except Exception:
        pass


def test_reading_frozen_snapshot():
    reading = JointReading(joint_id=21, pos=1.0, vel=2.0, tor=3.0)
    assert reading.joint_id == 21 and reading.pos == 1.0


def test_camera_frame_planes():
    color = np.zeros((4, 4, 3), dtype=np.uint8)
    depth = np.zeros((4, 4), dtype=np.uint16)
    frame = CameraFrame(color=color, depth=depth, frame_id='ob_camera_head')
    assert frame.frame_id == 'ob_camera_head'
    assert frame.depth.dtype == np.uint16


def test_audio_chunk_duration():
    chunk = AudioChunk(data=b'\x00' * 16000, sample_rate=8000,
                       channels=1, bits_per_sample=16)
    assert abs(chunk.duration_seconds - 1.0) < 1e-6
    assert AudioChunk().duration_seconds == 0.0


def test_gps_validity():
    assert not GpsFixReading(status=0).is_valid
    assert GpsFixReading(status=2).is_valid


def test_wrench_defaults():
    wrench = WrenchReading(fx=5.0)
    assert wrench.fx == 5.0 and wrench.ty == 0.0
