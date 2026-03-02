"""Launch UR5e in Gazebo Ignition with ros2_control controllers."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ----- Package paths -----
    pkg_ur5e = get_package_share_directory('enfield_robots_ur5e')

    # ----- Launch arguments -----
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use Gazebo simulation clock',
    )
    gz_world_arg = DeclareLaunchArgument(
        'gz_world', default_value='empty.sdf',
        description='Gazebo Ignition world file',
    )

    # ----- Robot description (xacro → URDF) -----
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_ur5e, 'urdf', 'ur5e_enfield.urdf.xacro'),
        ' sim_ignition:=true',
        ' use_fake_hardware:=false',
    ])
    robot_description = {'robot_description': robot_description_content}

    # ----- Nodes -----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Gazebo Ignition
    gz_sim = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r', '-v', '4',
            LaunchConfiguration('gz_world'),
        ],
        output='screen',
    )

    # Spawn robot entity in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'ur5e',
            '-allow_renaming', 'true',
        ],
    )

    # ros_gz_bridge for clock and joint states
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_ur5e, 'config', 'gz_bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    # ----- Controller spawners (after robot is spawned) -----
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Chain: spawn entity → start joint_state_broadcaster → start trajectory controller
    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_jtc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        gz_world_arg,
        robot_state_publisher,
        gz_sim,
        gz_spawn_entity,
        gz_bridge,
        delay_jsb_after_spawn,
        delay_jtc_after_jsb,
    ])
