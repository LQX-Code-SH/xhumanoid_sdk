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

# --- sim backend ----------------------------------------------------------
SIM_JOINT_STATE_TOPIC = '/joint_states'        # sensor_msgs/JointState (gz)
SIM_JOINT_CMD_TOPIC = '/tienkung_dex/joint_cmds'
SIM_IMU_TOPIC = '/imu'                         # gz standard imu topic
