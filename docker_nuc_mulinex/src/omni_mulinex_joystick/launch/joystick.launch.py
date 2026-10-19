import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('omni_mulinex_joystick'),
        'config',
        'joystick_params.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
            ),
            Node(
                package='omni_mulinex_joystick',
                executable='omni_mulinex_joystick_node',
                name='omni_mulinex_joystick',
                parameters=[config],
                output='screen',
            ),
        ]
    )
