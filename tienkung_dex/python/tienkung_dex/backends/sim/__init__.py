#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sim backend: Gazebo/ros_gz topic-compatible subsystems (design doc §6).

Camera/IMU topics are identical to the real machine (main-project 06 doc),
so those subscription classes are reused from backends.real. Only the
joint path differs: gz publishes sensor_msgs/JointState and commands are
forwarded through the bridge on a configurable topic (the actual bridge
mapping lives in the main project's simulation config).
"""

from .factory import SimBackendFactory

__all__ = ['SimBackendFactory']
