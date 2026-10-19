import os

import launch
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = launch.LaunchDescription()

    # ── Launch arguments ─────────────────────────────────────────────────────
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='omnicar',
        choices=['omnicar', 'omniquad12'],
        description='Robot to simulate: omnicar or omniquad12',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World SDF file name (looked up in omni_gazebo/worlds/)',
    )
    gz_verbosity_arg = DeclareLaunchArgument(
        'gz_verbosity',
        default_value='3',
        description='Gazebo verbosity level (0-4)',
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2',
    )

    robot = LaunchConfiguration('robot')
    world = LaunchConfiguration('world')
    gz_verbosity = LaunchConfiguration('gz_verbosity')
    use_rviz = LaunchConfiguration('use_rviz')

    ld.add_action(robot_arg)
    ld.add_action(world_arg)
    ld.add_action(gz_verbosity_arg)
    ld.add_action(use_rviz_arg)

    # ── Resolve paths ────────────────────────────────────────────────────────
    gazebo_pkg_share = FindPackageShare('omni_gazebo')
    description_pkg_share = FindPackageShare('mulinex_description')

    world_path = PathJoinSubstitution([gazebo_pkg_share, 'worlds', world])

    # Use PythonExpression to resolve robot-dependent values at launch time
    xacro_file = PythonExpression([
        "{'omnicar': 'omnicar.xacro', 'omniquad12': 'mulinex_white.xacro'}['", robot, "']"
    ])
    config_file = PythonExpression([
        "{'omnicar': 'omnicar_sim.yaml', 'omniquad12': 'omniquad12_sim.yaml'}['", robot, "']"
    ])
    extra_xacro_args = PythonExpression([
        "{'omnicar': '', 'omniquad12': ' feet_type:=omni'}['", robot, "']"
    ])
    spawn_z = PythonExpression([
        "{'omnicar': '0.5', 'omniquad12': '0.5'}['", robot, "']"
    ])

    xacro_file_path = PathJoinSubstitution([
        description_pkg_share, 'urdf', xacro_file,
    ])
    conf_file_path = PathJoinSubstitution([
        gazebo_pkg_share, 'config', config_file,
    ])

    # ── Environment for Gazebo plugin discovery ──────────────────────────────
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''),
        ]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''),
        ]),
    }

    # ── Gazebo simulator ─────────────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=['ruby $(which ign) gazebo', world_path, '-r', '-v', gz_verbosity],
        output='screen',
        additional_env=gz_env,
        shell=True,
    )

    # ── Robot state publisher ────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', xacro_file_path,
                ' use_gazebo:=true',
                ' yaml_file:=', conf_file_path,
                extra_xacro_args,
            ]),
            'use_sim_time': True,
        }],
    )

    # ── Spawn entity ─────────────────────────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'mulinex',
            '-topic', 'robot_description',
            '-z', spawn_z,
            '-x', '0.0',
            '-y', '0.0',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ── ROS <-> Gazebo bridge ────────────────────────────────────────────────
    # OdometryPublisher (configured in omnicar_ignition.xacro) emits the
    # world → base_link transform as ignition.msgs.Pose_V on
    # /model/mulinex/tf — bridge that into /tf.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/model/mulinex/pose@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/model/mulinex/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        ],
        remappings=[
            ('/model/mulinex/pose', '/tf'),
            ('/model/mulinex/odometry', '/odom'),
        ],
        output='screen',
    )

    # ── Controller spawners (triggered after entity spawn) ───────────────────
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', conf_file_path],
    )

    omni_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_controller', '--param-file', conf_file_path],
    )

    # ── RViz (optional) ─────────────────────────────────────────────────────
    rviz_config_path = PathJoinSubstitution([
        description_pkg_share, 'rviz', 'config.rviz',
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': True}],
    )

    # ── Assemble launch ─────────────────────────────────────────────────────
    ld.add_action(gazebo)
    ld.add_action(bridge)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(rviz)

    # Spawn controllers after entity has been created
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    ))
    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[omni_controller_spawner],
        )
    ))

    return ld
