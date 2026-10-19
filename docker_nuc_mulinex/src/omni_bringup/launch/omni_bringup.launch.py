import os
import subprocess
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BAG_TOPICS = [
    '/events/write_split',
    '/ik_controller/base_pose',
    '/joy',
    '/joy/set_feedback',
    '/nuc_heartbeat',
    '/omni_controller/direct_wheels_cmd',
    'omni_controller/distributors_state',
    '/omni_controller/debug/joints_command',
    '/omni_controller/joints_state',
    '/omni_controller/joints_reference',
    '/omni_controller/performance',
    '/omni_controller/safety_state',
    '/omni_controller/transition_event',
    '/omni_controller/twist_cmd',
    '/state_broadcaster/joints_state',
    '/state_broadcaster/performance_indexes',
    '/state_broadcaster/transition_event',
]


def _save_git_diff(repo_dir: str, out_file: str) -> None:
    try:
        head = subprocess.check_output(
            ['git', '-C', repo_dir, 'log', '-1', '--oneline'],
            text=True, stderr=subprocess.DEVNULL,
        )
        diff = subprocess.check_output(
            ['git', '-C', repo_dir, 'diff', 'HEAD'],
            text=True, stderr=subprocess.DEVNULL,
        )
        with open(out_file, 'w') as f:
            f.write(f'# HEAD: {head}')
            f.write(diff)
    except (subprocess.CalledProcessError, FileNotFoundError) as e:
        with open(out_file, 'w') as f:
            f.write(f'# git info unavailable: {e}\n')


def _setup_recording(context, *args, **kwargs):
    if LaunchConfiguration('record_bag').perform(context).lower() not in ('true', '1', 'yes'):
        return []

    # Workspace root is 4 levels up from <ws>/install/<pkg>/share/<pkg>
    workspace_dir = os.path.abspath(
        os.path.join(get_package_share_directory('omni_bringup'), '..', '..', '..', '..')
    )
    run_dir = os.path.join(
        workspace_dir, 'bags', datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    )
    os.makedirs(run_dir, exist_ok=True)
    _save_git_diff(workspace_dir, os.path.join(run_dir, 'git_diff.txt'))

    return [ExecuteProcess(
        cmd=['ros2', 'bag', 'record', *BAG_TOPICS, '-o', os.path.join(run_dir, 'bag')],
        output='screen',
    )]


def generate_launch_description():
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',
    )

    nuc_heartbeat = Node(
        package='omni_bringup',
        executable='nuc_heartbeat_node',
        name='nuc_heartbeat',
        parameters=[{'rate': 10.0}],
        output='screen',
    )

    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omni_mulinex_joystick'),
                'launch',
                'joystick.launch.py',
            )
        )
    )

    ik_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ik_controller'),
                'launch',
                'ik_controller.launch.py',
            )
        )
    )

    return LaunchDescription([
        record_bag_arg,
        nuc_heartbeat,
        joystick_launch,
        ik_controller_launch,
        OpaqueFunction(function=_setup_recording),
    ])
