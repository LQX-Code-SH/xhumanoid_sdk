#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienkungDex demo node: builds the facade and reports subsystem health.

Real-machine verification aid (design doc §10 T2-T4): after start, every
subsystem's is_active flag must flip True within its staleness window,
which reproduces each SDK demo's observable behaviour through one facade.

Periodic mode (default):
    ros2 run tienkung_dex tienkung_dex_demo

One-shot self-check mode (terminal report; the exit code is
the number of inactive subsystems, so 0 = all healthy - CI friendly):
    ros2 run tienkung_dex tienkung_dex_demo --ros-args \
        -p backend:=real -p once:=True -p once_duration:=8.0
"""

import sys

import rclpy
from rclpy.node import Node

from .robot import create_robot


class DemoNode(Node):
    def __init__(self):
        super().__init__('tienkung_dex_demo')
        self.declare_parameter('backend', 'real')
        self.declare_parameter('hand_vendor', 'brainco')
        self.declare_parameter('report_hz', 1.0)
        self.declare_parameter('once', False)
        self.declare_parameter('once_duration', 8.0)
        self.declare_parameter('panorama', False)
        self.declare_parameter('gps', False)
        self.declare_parameter('force', False)

        backend = self.get_parameter('backend').value
        hand_vendor = self.get_parameter('hand_vendor').value
        report_hz = self.get_parameter('report_hz').value
        once = self.get_parameter('once').value
        once_duration = self.get_parameter('once_duration').value
        panorama = self.get_parameter('panorama').value
        gps = self.get_parameter('gps').value
        force = self.get_parameter('force').value

        enable = {'joint', 'camera', 'hand', 'audio', 'safety',
                  'imu', 'lidar', 'power', 'light', 'sbus', 'serial'}
        if panorama:
            enable.add('panorama')
        if gps:
            enable.add('gps')
        if force:
            enable.add('force')

        self.robot = create_robot(
            self, backend=backend, hand_vendor=hand_vendor,
            enable=enable, logger=self.get_logger())
        self.robot.start()
        self._exit_code = 0

        if once:
            self.create_timer(float(once_duration), self.final_report)
            self.get_logger().info(
                f'自检模式: {float(once_duration):.0f}s 后输出终态报告并退出')
        else:
            self.create_timer(1.0 / max(report_hz, 0.1), self.report)
            self.get_logger().info(self.robot.report())

    def report(self):
        self.get_logger().info(self.robot.report())

    def final_report(self):
        """One-shot terminal report with exit code =
        number of inactive subsystems."""
        health = self.robot.health()
        failures = 0
        print('============================================')
        print('  TienkungDex 健康自检报告')
        print('============================================')
        for name, active in sorted(health.items()):
            mark = '✅' if active else '❌'
            state = '正常' if active else '无数据/未激活'
            print(f'  {mark} {name}: {state}')
            if not active:
                failures += 1
        print('============================================')
        print(f'  总计 {len(health)} 项: '
              f'{len(health) - failures} 正常 / {failures} 异常')
        print('============================================')
        # sys.exit() semantics truncate above 255 - clamp so shell callers
        # can distinguish "some failures" (>=1) from hard errors.
        self._exit_code = min(failures, 255)
        self.robot.shutdown()
        rclpy.shutdown()

    def destroy_node(self):
        self.robot.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DemoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    sys.exit(node._exit_code)


if __name__ == '__main__':
    main()
