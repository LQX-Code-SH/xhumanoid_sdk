#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
因时灵巧手手势控制节点 (Python version)

该节点通过 SetAngle (位置模式) 控制因时灵巧手实现预设手势:
- OK手势: 大拇指和食指捏合,其他手指伸直
- 石头: 所有手指弯曲握拳
- 剪刀: 食指和中指伸直,其他手指弯曲
- 布: 所有手指伸直张开

因时手 SetAngle 消息 (inspire_hand_msgs):
- hand_id: 手编号 (1=左手, 2=右手)
- joint_values[13]: 13 个关节的目标位置

⚠️ 13 个关节的真实布局以 GetAngleAct.joint_names 实测为准,
   代码中的手势位置为示例值,需按实际硬件校准。
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from inspire_hand_msgs.msg import SetAngle, GetAngleAct, TouchData
from inspire_hand_gesture_interfaces.srv import GestureCommand


class HandGestureControl(Node):
    """ROS2 Node for inspire hand gesture control"""

    # 关节数量 (SetAngle.joint_values 固定长度)
    JOINT_COUNT = 13

    # 位置范围: 0 ~ 1000 (具体方向需按实际硬件校准)
    POS_MIN = 0         # 示例: 伸直
    POS_MAX = 1000      # 示例: 弯曲

    # 手编号 (vendor demo 07 约定)
    HAND_ID_LEFT = 1
    HAND_ID_RIGHT = 2

    def __init__(self):
        super().__init__('inspire_hand_gesture_control')

        # Declare parameters
        self.declare_parameter('hand_prefix', 'right_hand')
        self.declare_parameter('hand_id', self.HAND_ID_RIGHT)

        # Get parameters
        self.hand_prefix = self.get_parameter('hand_prefix').get_parameter_value().string_value
        self.hand_id = self.get_parameter('hand_id').get_parameter_value().integer_value

        # Build topic names
        angle_cmd_topic = f'{self.hand_prefix}/angle_set'
        angle_actual_topic = f'{self.hand_prefix}/angle_actual'
        touch_topic = f'{self.hand_prefix}/touch_data'

        # Create angle control publisher
        self.angle_pub = self.create_publisher(
            SetAngle,
            angle_cmd_topic,
            10
        )

        # Create angle actual subscriber
        self.status_sub = self.create_subscription(
            GetAngleAct,
            angle_actual_topic,
            self.status_callback,
            10
        )

        # Create touch data subscriber (触觉为选配反馈)
        self.touch_sub = self.create_subscription(
            TouchData,
            touch_topic,
            self.touch_callback,
            10
        )

        # Callback group for concurrent service callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Create gesture control service
        self.gesture_service = self.create_service(
            GestureCommand,
            'gesture_command',
            self.gesture_command_callback,
            callback_group=self.callback_group
        )

        # Initialize gesture positions
        self.gesture_positions = self._init_gesture_positions()

        # Current status
        self.current_status = None
        self._joint_names_logged = False

        self.get_logger().info('因时灵巧手手势控制节点已启动')
        self.get_logger().info(f'手: {self.hand_prefix} (hand_id={self.hand_id})')
        self.get_logger().info(f'控制话题: {angle_cmd_topic}')
        self.get_logger().info(f'状态话题: {angle_actual_topic}')
        self.get_logger().info(f'触觉话题: {touch_topic}')
        self.get_logger().info('服务: /gesture_command')
        self.get_logger().info('支持的手势: ok, rock(石头), scissors(剪刀), paper(布)')

    def _init_gesture_positions(self):
        """Initialize gesture position mappings (13 个关节, 示例值待校准)

        前 6 个关节沿用强脑手 6 电机的手指映射 (拇指弯曲/拇指旋转/
        食指/中指/无名指/小指), 其余 7 个关节按伸直处理。
        真实布局以 angle_actual 话题的 joint_names 为准。
        """
        return {
            'ok': [
                450,     # 拇指弯曲: 中等弯曲
                800,     # 拇指旋转: 适当旋转角度
                450,     # 食指: 弯曲与拇指捏合
                self.POS_MIN,  # 中指: 伸直
                self.POS_MIN,  # 无名指: 伸直
                self.POS_MIN,  # 小指: 伸直
                *([self.POS_MIN] * (self.JOINT_COUNT - 6)),  # 其余关节: 伸直
            ],
            'rock': [
                800,     # 拇指弯曲: 完全弯曲
                500,     # 拇指旋转: 中间位置
                900,     # 食指: 完全弯曲
                900,     # 中指: 完全弯曲
                900,     # 无名指: 完全弯曲
                900,     # 小指: 完全弯曲
                *([900] * (self.JOINT_COUNT - 6)),  # 其余关节: 弯曲
            ],
            'scissors': [
                800,     # 拇指弯曲: 弯曲
                500,     # 拇指旋转: 中间位置
                self.POS_MIN,  # 食指: 伸直
                self.POS_MIN,  # 中指: 伸直
                900,     # 无名指: 弯曲
                900,     # 小指: 弯曲
                *([self.POS_MIN] * (self.JOINT_COUNT - 6)),  # 其余关节: 伸直
            ],
            'paper': [
                self.POS_MIN,  # 拇指弯曲: 伸直
                200,     # 拇指旋转: 张开角度
                self.POS_MIN,  # 食指: 伸直
                self.POS_MIN,  # 中指: 伸直
                self.POS_MIN,  # 无名指: 伸直
                self.POS_MIN,  # 小指: 伸直
                *([self.POS_MIN] * (self.JOINT_COUNT - 6)),  # 其余关节: 伸直
            ]
        }

    def status_callback(self, msg):
        """Angle actual callback"""
        self.current_status = msg
        # 首次收到状态时打印真实关节布局, 辅助校准手势位置
        if not self._joint_names_logged and getattr(msg, 'joint_names', None):
            self._joint_names_logged = True
            names = list(msg.joint_names)
            self.get_logger().info(f'关节布局 (joint_names[13]): {names}')

    def touch_callback(self, msg):
        """Touch data callback (触觉反馈, 仅缓存)"""
        self.current_touch = msg

    def gesture_command_callback(self, request, response):
        """Gesture command service callback"""
        gesture = request.gesture.lower()

        self.get_logger().info(f'接收到手势命令: {gesture}')

        # Check if gesture exists
        if gesture not in self.gesture_positions:
            response.success = False
            response.message = f"未知手势: '{gesture}'. 支持的手势: ok, rock, scissors, paper"
            self.get_logger().warn(response.message)
            return response

        # Execute gesture
        result = self._execute_gesture(gesture)

        response.success = result
        response.message = f"手势 '{gesture}' 执行成功" if result else f"手势 '{gesture}' 执行失败"

        if result:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(response.message)

        return response

    def _execute_gesture(self, gesture):
        """Execute gesture by publishing SetAngle command"""
        msg = SetAngle()

        # 手编号: 1=左手, 2=右手 (vendor demo 07 约定)
        msg.hand_id = self.hand_id

        # Get gesture positions
        positions = self.gesture_positions[gesture]

        # Set joint positions (13 个关节)
        msg.joint_values = positions

        self.get_logger().info(f'正在执行手势: {gesture}')
        self.get_logger().debug(
            f'位置: 前6关节={positions[:6]}, 其余={positions[6:]}'
        )

        # Publish control command
        self.angle_pub.publish(msg)

        return True


def main(args=None):
    rclpy.init(args=args)
    node = HandGestureControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
