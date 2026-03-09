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

"""Launch MoveIt2 move_group for UR5e planning and execution."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name: str, file_path: str) -> dict:
    """Load a YAML file from a ROS2 package share directory."""
    full_path = os.path.join(
        get_package_share_directory(package_name), file_path
    )
    with open(full_path, 'r') as f:
        return yaml.safe_load(f)


def generate_launch_description():
    """Generate MoveIt2 launch description."""
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
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # ----- SRDF -----
    srdf_path = os.path.join(pkg_ur5e, 'srdf', 'ur5e.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {
            'robot_description_semantic': ParameterValue(
                f.read(), value_type=str
            )
        }

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

    return LaunchDescription([
        use_sim_time_arg,
        move_group_node,
    ])
