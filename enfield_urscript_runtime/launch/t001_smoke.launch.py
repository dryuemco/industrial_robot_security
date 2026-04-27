# SPDX-License-Identifier: Apache-2.0
"""T001 smoke launch: telemetry recorder warm-up, then URScript publish.

Args:
  task_ir_path: absolute path to T001 IR JSON (required)
  output_csv: telemetry CSV destination (default /tmp/telemetry_T001.csv)
  duration_s: total recording window in seconds (default 30.0)
  publish_delay_s: warm-up before publish (default 2.0)
  save_script_path: optional URScript dump path (default '')
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    task_ir_path = LaunchConfiguration('task_ir_path')
    output_csv = LaunchConfiguration('output_csv')
    duration_s = LaunchConfiguration('duration_s')
    publish_delay_s = LaunchConfiguration('publish_delay_s')
    save_script_path = LaunchConfiguration('save_script_path')

    recorder = Node(
        package='enfield_urscript_runtime',
        executable='telemetry_recorder',
        name='telemetry_recorder',
        output='screen',
        parameters=[{
            'output_csv': output_csv,
            'duration_s': duration_s,
        }],
    )

    publisher = Node(
        package='enfield_urscript_runtime',
        executable='urscript_publisher',
        name='urscript_publisher',
        output='screen',
        parameters=[{
            'task_ir_path': task_ir_path,
            'publish_delay_s': 0.5,
            'save_script_path': save_script_path,
        }],
    )

    # Recorder starts immediately; publisher kicks in after warm-up
    delayed_publisher = TimerAction(
        period=publish_delay_s,
        actions=[publisher],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'task_ir_path',
            description='Absolute path to Task IR JSON file',
        ),
        DeclareLaunchArgument(
            'output_csv',
            default_value='/tmp/telemetry_T001.csv',
            description='Telemetry CSV destination',
        ),
        DeclareLaunchArgument(
            'duration_s',
            default_value='30.0',
            description='Recording window seconds',
        ),
        DeclareLaunchArgument(
            'publish_delay_s',
            default_value='2.0',
            description='Recorder warm-up before URScript publish',
        ),
        DeclareLaunchArgument(
            'save_script_path',
            default_value='',
            description='Optional URScript dump path for traceability',
        ),
        recorder,
        delayed_publisher,
    ])
