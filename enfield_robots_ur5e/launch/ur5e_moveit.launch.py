"""Launch MoveIt2 move_group for UR5e planning and execution."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(package_name: str, file_path: str) -> dict:
    """Load a YAML file from a ROS2 package share directory."""
    full_path = os.path.join(
        get_package_share_directory(package_name), file_path
    )
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg_ur5e = get_package_share_directory('enfield_robots_ur5e')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
    )

    # ----- Robot description -----
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_ur5e, 'urdf', 'ur5e_enfield.urdf.xacro'),
        ' sim_ignition:=true',
        ' use_fake_hardware:=false',
    ])
    robot_description = {'robot_description': robot_description_content}

    # ----- SRDF -----
    srdf_path = os.path.join(pkg_ur5e, 'srdf', 'ur5e.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    # ----- MoveIt2 config files -----
    joint_limits = {
        'robot_description_planning': load_yaml(
            'enfield_robots_ur5e', 'config/moveit/joint_limits.yaml'
        )
    }
    kinematics = load_yaml(
        'enfield_robots_ur5e', 'config/moveit/kinematics.yaml'
    )
    ompl_planning = load_yaml(
        'enfield_robots_ur5e', 'config/moveit/ompl_planning.yaml'
    )
    moveit_controllers = load_yaml(
        'enfield_robots_ur5e', 'config/moveit/moveit_controllers.yaml'
    )

    # ----- move_group node -----
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            joint_limits,
            kinematics,
            ompl_planning,
            moveit_controllers,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'planning_scene_monitor_options': {
                    'robot_description': 'robot_description',
                    'joint_state_topic': '/joint_states',
                },
            },
        ],
    )

    # ----- RViz (optional, for debugging) -----
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_ur5e, 'config', 'moveit.rviz')],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        # Disabled by default — uncomment condition to auto-launch
        # condition=IfCondition(LaunchConfiguration('launch_rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        move_group_node,
        # rviz_node,  # Uncomment for visual debugging
    ])
