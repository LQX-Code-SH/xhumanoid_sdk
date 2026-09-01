#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TienkungDex demo node: builds the facade and reports subsystem health.

Real-machine verification aid (design doc §10 T2-T4): after start, every
subsystem's is_active flag must flip True within its staleness window,
which reproduces each SDK demo's observable behaviour through one facade.

Run:
    ros2 run tienkung_dex tienkung_dex_demo
    ros2 run tienkung_dex tienkung_dex_demo --ros-args \
        -p backend:=real -p report_hz:=1.0
"""

import rclpy
from rclpy.node import Node

from .robot import create_robot


class DemoNode(Node):
    def __init__(self):
        super().__init__('tienkung_dex_demo')
        self.declare_parameter('backend', 'real')
        self.declare_parameter('hand_vendor', 'brainco')
        self.declare_parameter('report_hz', 1.0)
        self.declare_parameter('panorama', False)
        self.declare_parameter('gps', False)

        backend = self.get_parameter('backend').value
        hand_vendor = self.get_parameter('hand_vendor').value
        report_hz = self.get_parameter('report_hz').value
        panorama = self.get_parameter('panorama').value
        gps = self.get_parameter('gps').value

        enable = {'joint', 'camera', 'hand', 'audio', 'safety',
                  'imu', 'lidar', 'force'}
        if panorama:
            enable.add('panorama')
        if gps:
            enable.add('gps')

        self.robot = create_robot(
            self, backend=backend, hand_vendor=hand_vendor,
            enable=enable, logger=self.get_logger())
        self.robot.start()

        self.create_timer(1.0 / max(report_hz, 0.1), self.report)
        self.get_logger().info(self.robot.report())

    def report(self):
        self.get_logger().info(self.robot.report())

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
