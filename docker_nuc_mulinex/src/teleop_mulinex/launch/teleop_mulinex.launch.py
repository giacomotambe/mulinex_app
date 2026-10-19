from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='teleop_mulinex',
                executable='teleop_mulinex_node',
                name='teleop_mulinex',
                output='screen',
                prefix='xterm -e',
                parameters=[
                    {
                        'linear_step': 0.1,
                        'angular_step': 0.1,
                        'body_pos_step': 0.005,
                        'body_ang_step': 0.02,
                        'max_height': 0.05,
                        'max_roll': 0.3,
                        'max_pitch': 0.3,
                        'max_yaw': 0.15,
                        'max_pos_x': 0.05,
                        'max_pos_y': 0.05,
                    }
                ],
            ),
        ]
    )
