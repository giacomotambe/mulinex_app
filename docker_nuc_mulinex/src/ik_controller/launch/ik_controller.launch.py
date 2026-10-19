import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ik_controller'), 'config', 'ik_controller_params.yaml'
    )

    return LaunchDescription(
        [
            Node(
                package='ik_controller',
                executable='ik_controller',
                name='ik_controller',
                parameters=[config],
                output='screen',
            ),
        ]
    )
