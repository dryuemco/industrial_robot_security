# Copyright 2026 Yunus Emre Cogurcu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ENFIELD full simulation bringup: UR5e + MoveIt2 + Watchdog nodes."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
    )
    operating_mode_arg = DeclareLaunchArgument(
        'operating_mode', default_value='collaborative',
        description='Robot operating mode: collaborative | fenced',
    )

    # ----- 1. UR5e Gazebo simulation -----
    ur5e_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('enfield_robots_ur5e'),
                'launch', 'ur5e_sim.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # ----- 2. MoveIt2 (delayed 5s for Gazebo startup) -----
    ur5e_moveit = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('enfield_robots_ur5e'),
                        'launch', 'ur5e_moveit.launch.py',
                    ])
                ),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                }.items(),
            ),
        ],
    )

    # ----- 3. Watchdog nodes (delayed 8s) -----
    watchdog_params = PathJoinSubstitution([
        FindPackageShare('enfield_watchdog'),
        'config', 'safety_params.yaml',
    ])

    velocity_monitor = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='enfield_watchdog',
                executable='velocity_monitor',
                output='screen',
                parameters=[
                    watchdog_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            ),
        ],
    )

    zone_monitor = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='enfield_watchdog',
                executable='zone_monitor',
                output='screen',
                parameters=[
                    watchdog_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            ),
        ],
    )

    safety_aggregator = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='enfield_watchdog',
                executable='safety_aggregator',
                output='screen',
                parameters=[
                    watchdog_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
            ),
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        operating_mode_arg,
        ur5e_sim,
        ur5e_moveit,
        velocity_monitor,
        zone_monitor,
        safety_aggregator,
    ])
