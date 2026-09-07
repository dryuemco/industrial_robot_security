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

"""Launch UR5e in Gazebo Ignition with ros2_control controllers."""

import os
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


# Ignition resolves `<plugin filename="ign_ros2_control-system">` (declared in
# ur5e_enfield.urdf.xacro) against IGN_GAZEBO_SYSTEM_PLUGIN_PATH only. The ROS
# install ships libign_ros2_control-system.so under <prefix>/lib, which is not
# on that path by default, so the plugin fails to load with "couldn't find
# shared library" and no controller_manager is ever created.
#
# Returns None when ign_ros2_control is absent, so that importing this launch
# file (e.g. during ament linting) does not fail on a non-simulation install.
def _ign_system_plugin_path():
    """Build the IGN_GAZEBO_SYSTEM_PLUGIN_PATH value for ign_ros2_control."""
    try:
        plugin_dir = os.path.join(get_package_prefix('ign_ros2_control'), 'lib')
    except PackageNotFoundError:
        return None
    existing = os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')
    if not existing:
        return plugin_dir
    if plugin_dir in existing.split(os.pathsep):
        return existing
    return os.pathsep.join([existing, plugin_dir])


def generate_launch_description():
    """Generate UR5e simulation launch description."""
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
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description=(
            'Run Gazebo as a server-only process (ign gazebo -s), with no GUI. '
            'Required on machines without an OpenGL-capable display (CI, '
            'containers, SSH sessions): the GUI aborts on GL context creation '
            'and takes the Gazebo server down with it, so the robot never '
            'reaches controller_manager.'
        ),
    )

    # ----- Robot description (xacro -> URDF) -----
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(
            pkg_ur5e, 'urdf', 'ur5e_enfield.urdf.xacro'
        ),
        ' sim_ignition:=true',
        ' use_fake_hardware:=false',
    ])
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # ----- Nodes -----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Gazebo Ignition. Two variants rather than a conditional argv entry: an
    # empty string in cmd would be passed to ign as an empty positional arg.
    gz_sim = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-r', '-v', '4',
            LaunchConfiguration('gz_world'),
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('headless')),
    )

    gz_sim_headless = ExecuteProcess(
        cmd=[
            'ign', 'gazebo', '-s', '-r', '-v', '4',
            LaunchConfiguration('gz_world'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('headless')),
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
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ----- Controller spawners (after robot is spawned) -----
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    jtc_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager', '/controller_manager',
        ],
        output='screen',
    )

    # Chain: spawn entity -> start JSB -> start JTC
    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[jsb_spawner],
        )
    )
    delay_jtc_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[jtc_spawner],
        )
    )

    actions = [
        use_sim_time_arg,
        gz_world_arg,
        headless_arg,
    ]

    plugin_path = _ign_system_plugin_path()
    if plugin_path is not None:
        actions.append(
            SetEnvironmentVariable(
                'IGN_GAZEBO_SYSTEM_PLUGIN_PATH', plugin_path
            )
        )

    actions += [
        robot_state_publisher,
        gz_sim,
        gz_sim_headless,
        gz_spawn_entity,
        gz_bridge,
        delay_jsb_after_spawn,
        delay_jtc_after_jsb,
    ]

    return LaunchDescription(actions)
