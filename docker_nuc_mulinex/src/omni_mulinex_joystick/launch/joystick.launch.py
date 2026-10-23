import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _joystick_node(context, *args, **kwargs):
    config_dir = os.path.join(get_package_share_directory('omni_mulinex_joystick'), 'config')
    params = [os.path.join(config_dir, 'joystick_params.yaml')]
    # robot:=traquad loads the traquad overrides after the common parameters
    robot = LaunchConfiguration('robot').perform(context)
    if robot == 'traquad':
        params.append(os.path.join(config_dir, 'joystick_traquad.yaml'))
    elif robot != 'omniquad':
        raise RuntimeError(f"robot must be 'omniquad' or 'traquad', got '{robot}'")
    return [
        Node(
            package='omni_mulinex_joystick',
            executable='omni_mulinex_joystick_node',
            name='omni_mulinex_joystick',
            parameters=params,
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('robot', default_value='omniquad'),
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
            ),
            OpaqueFunction(function=_joystick_node),
        ]
    )
