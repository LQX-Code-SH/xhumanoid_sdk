from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    side_arg = DeclareLaunchArgument(
        'side', default_value='left',
        description='Wrist side: left or right'
    )
    serial_arg = DeclareLaunchArgument(
        'serial', default_value='',
        description='Camera serial number (required, e.g. 233522110575)'
    )
    width_arg = DeclareLaunchArgument(
        'width', default_value='640',
        description='Frame width'
    )
    height_arg = DeclareLaunchArgument(
        'height', default_value='480',
        description='Frame height'
    )
    fps_arg = DeclareLaunchArgument(
        'fps', default_value='15',
        description='Frame rate: 5/15/30 (30 fps dual-stream works on USB3)'
    )
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth', default_value='true',
        description='Publish the depth stream alongside color'
    )

    camera_wrist_driver_node = Node(
        package='camera_wrist_driver_py',
        executable='camera_wrist_driver_node',
        name='camera_wrist_driver_node',
        output='screen',
        parameters=[{
            'side': LaunchConfiguration('side'),
            'serial': LaunchConfiguration('serial'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'enable_depth': LaunchConfiguration('enable_depth'),
        }],
    )

    return LaunchDescription([
        side_arg,
        serial_arg,
        width_arg,
        height_arg,
        fps_arg,
        enable_depth_arg,
        camera_wrist_driver_node,
    ])
