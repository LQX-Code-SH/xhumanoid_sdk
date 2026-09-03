#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Topic/service name constants (design doc §5 mapping table).

Single hardcoding point of the library. Naming rule (design doc §3):
absolute paths with a leading '/' everywhere; adapters normalize the SDK
demos' relative/absolute mix.

All names can be overridden per-robot through create_robot(**topic_overrides)
(e.g. tests, custom namespaces).
"""

# --- joint groups ---------------------------------------------------------
JOINT_CMD_TOPIC = {
    'arm': '/arm/cmd',
    'head': '/head/cmd',
    'waist': '/waist/cmd',
    'leg': '/leg/cmd',
}
JOINT_GROUPS = tuple(JOINT_CMD_TOPIC)

ROBOT_STATE_TOPIC = '/robot_state'

# --- cameras (all QoS: BEST_EFFORT + VOLATILE + KEEP_LAST(10)) ------------
CAMERA_NAMESPACES = ('ob_camera_head', 'ob_camera_waist',
                     'ob_camera_wrist_left', 'ob_camera_wrist_right')


def camera_topics(namespace: str) -> dict:
    return {
        'color': f'/{namespace}/color/image_raw',
        'depth': f'/{namespace}/depth/image_raw',
    }


# Panoramic 6-camera group (optional hardware; SDK demo indices skip 3 and 7)
PANORAMA_INDICES = (0, 1, 2, 4, 5, 6)
PANORAMA_TOPIC_PREFIX = '/camera'          # -> /camera{i}/image_raw
PANORAMA_COMPRESSED_SUFFIX = '/image/compressed'

# --- sensors --------------------------------------------------------------
LIDAR_TOPIC = '/livox/lidar'
IMU_TOPIC_LIVOX = '/livox/imu'
GPS_TOPIC = '/gps/fix'                     # demo declares relative 'gps/fix'
FORCE_TOPIC = ''                           # unverified (HWI §7): set explicitly

# --- audio / interaction --------------------------------------------------
TTS_SERVICE = '/intelligent_interaction/tts/play'
AUDIO_CONTROL_SERVICE = '/lyre/audio_control'
AUDIO_STREAM_TOPIC = '/lyre/audio_stream'
VOICE_ACTIVITY_TOPIC = '/lyre/voice_activity'

# --- dexterous hand -------------------------------------------------------
HAND_TOPICS = {
    'left': {
        'cmd': '/left_hand/set_motor_multi',     # SDK demo uses relative
        'status': '/left_hand/motor_status',     # name; normalized here
        'touch': '/left_hand/touch_status',
    },
    'right': {
        'cmd': '/right_hand/set_motor_multi',
        'status': '/right_hand/motor_status',
        'touch': '/right_hand/touch_status',
    },
}

# --- safety ---------------------------------------------------------------
KEY_STATUS_TOPIC = '/power/board/key_status'

# --- power / light / sbus / serial -------------
POWER_TOPICS = {
    'battery': '/power/battery/status',
    'board': '/power/board/status',
    'key_status': KEY_STATUS_TOPIC,
}

LIGHT_TOPIC = '/xsys/light/ctrl'
# LightCtrl.cmd presets copied from demo 14.
LIGHT_CMDS = {
    'off': 0,
    'battery_normal': 201,
    'battery_low': 202,
    'battery_critical': 203,
    'charging': 210,
    'wakeup': 301,
    'listening': 310,
    'thinking': 311,
    'running': 401,
}

SBUS_TOPIC = '/sbus_data'                   # sensor_msgs/Joy
SBUS_EVENT_TOPIC = '/sbus_data/event'       # bodyctrl_msgs/SbusData

SERIAL_SERVICE = '/xsys/get_serial_number'

# --- inspire (13-joint) hand -------------------------
# Feedback topics are demo-15-confirmed; clear_error service from demo 07.
INSPIRE_HAND_TOPICS = {
    'left': {
        'angle_cmd': '/left_hand/angle_set',
        'force_cmd': '/left_hand/force_set',
        'speed_cmd': '/left_hand/speed_set',
        'angle_actual': '/left_hand/angle_actual',
        'force_actual': '/left_hand/force_actual',
        'touch': '/left_hand/touch_data',
        'clear_error': '/inspire_hand/set_clear_error/left_hand',
    },
    'right': {
        'angle_cmd': '/right_hand/angle_set',
        'force_cmd': '/right_hand/force_set',
        'speed_cmd': '/right_hand/speed_set',
        'angle_actual': '/right_hand/angle_actual',
        'force_actual': '/right_hand/force_actual',
        'touch': '/right_hand/touch_data',
        'clear_error': '/inspire_hand/set_clear_error/right_hand',
    },
}

# --- sim backend ----------------------------------------------------------
SIM_JOINT_STATE_TOPIC = '/joint_states'        # sensor_msgs/JointState (gz)
SIM_JOINT_CMD_TOPIC = '/tienkung_dex/joint_cmds'
SIM_IMU_TOPIC = '/imu'                         # gz standard imu topic

# --- vector walking (HRIC cmd_vel, robot host) ------------------------------
# Reference: 具身天工DEX-矢量行走接口.md (飞书文档, 提取于 2026-09-03).
# 腿部关节由行走运控（run_patrol / RL 全身策略）独占，SDK 不可直控 leg；
# 行走只能经本机体系速度矢量流驱动。话题控制前需遥控器切到半身/全身/站走跑
# 策略并 e 键上拨进入话题控制（此时遥控仅 c 键有效），随后以 ~20 Hz 持续发布
# /hric/robot/cmd_vel（geometry_msgs/TwistStamped, frame 约定 'pelvis'）：
#   linear.x = 前进 vx (m/s)、linear.y = 侧移 vy (m/s)、
#   angular.z = 转向 wz (rad/s)；其余字段保持 0。
# 三速度范数 < WALK_STOP_NORM 时机器人保持站立。
WALK_CMD_TOPIC = '/hric/robot/cmd_vel'
WALK_CMD_STATUS_TOPIC = '/hric/robot/cmd_vel_status'   # 遥控指令转发输出
WALK_STATE_TOPIC = '/hric/robot/rl_state'              # diagnostic_msgs/DiagnosticStatus
WALK_CMD_FRAME_ID = 'pelvis'
# 取值范围表（全身/半身行走列；站走跑列上限更高：vx∈[-0.5,2.2]、wz∈[-1.0,1.0]）。
WALK_LIMITS = {
    'vx_min': -0.8, 'vx_max': 1.0,      # forward (m/s)
    'vy_min': -0.5, 'vy_max': 0.5,      # lateral (m/s)
    'wz_min': -0.8, 'wz_max': 0.8,      # yaw (rad/s)
}
WALK_STOP_NORM = 0.05                   # 范数低于该值 → 站立
WALK_DEFAULT_RATE_HZ = 20.0             # 运控要求的持续指令流频率
