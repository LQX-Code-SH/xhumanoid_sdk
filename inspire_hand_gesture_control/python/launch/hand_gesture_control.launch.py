from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    启动因时灵巧手手势控制节点

    使用方法:
        # 默认控制右手
        ros2 launch inspire_hand_gesture_control_py hand_gesture_control.launch.py

        # 控制左手
        ros2 launch inspire_hand_gesture_control_py hand_gesture_control.launch.py hand_prefix:=left_hand hand_id:=1
    """

    # Declare launch arguments
    hand_prefix_arg = DeclareLaunchArgument(
        'hand_prefix',
        default_value='right_hand',
        description='手的前缀: right_hand 或 left_hand'
    )

    hand_id_arg = DeclareLaunchArgument(
        'hand_id',
        default_value='2',
        description='手编号: 1=左手, 2=右手 (vendor demo 07 约定)'
    )

    # Create node
    hand_gesture_control_node = Node(
        package='inspire_hand_gesture_control_py',
        executable='hand_gesture_control_node',
        name='inspire_hand_gesture_control',
        output='screen',
        parameters=[{
            'hand_prefix': LaunchConfiguration('hand_prefix'),
            'hand_id': LaunchConfiguration('hand_id'),
        }],
    )

    return LaunchDescription([
        hand_prefix_arg,
        hand_id_arg,
        hand_gesture_control_node,
    ])
