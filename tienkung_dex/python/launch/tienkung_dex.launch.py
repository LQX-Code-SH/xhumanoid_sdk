#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Launch the TienkungDex demo node (health report of every subsystem)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    backend = LaunchConfiguration('backend')
    backend_arg = DeclareLaunchArgument(
        'backend', default_value='real',
        description="backend: 'real' (SDK topics) | 'sim' (gz bridge) | 'mock'")

    node = Node(
        package='tienkung_dex',
        executable='tienkung_dex_demo',
        name='tienkung_dex_demo',
        output='screen',
        parameters=[{'backend': backend}],
    )
    return LaunchDescription([backend_arg, node])
