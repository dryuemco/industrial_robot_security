# SPDX-License-Identifier: Apache-2.0
"""T001 smoke launch: telemetry recorder warm-up, then URScript injection.

Args:
  task_ir_path: absolute path to T001 IR JSON (required)
  output_csv: telemetry CSV destination (default /tmp/telemetry_T001.csv)
  duration_s: total recording window in seconds (default 30.0)
  publish_delay_s: warm-up before URScript injection (default 2.0)
  save_script_path: optional URScript dump path (default '')
  inject_mode: 'topic' (default) or 'primary_tcp'
  ursim_primary_host: URSim host for primary_tcp mode (default 127.0.0.1)
  ursim_primary_port: URSim Secondary client port (default 30002)
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
    inject_mode = LaunchConfiguration('inject_mode')
    ursim_primary_host = LaunchConfiguration('ursim_primary_host')
    ursim_primary_port = LaunchConfiguration('ursim_primary_port')

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
            'inject_mode': inject_mode,
            'ursim_primary_host': ursim_primary_host,
            'ursim_primary_port': ursim_primary_port,
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
            description='Recorder warm-up before URScript injection',
        ),
        DeclareLaunchArgument(
            'save_script_path',
            default_value='',
            description='Optional URScript dump path for traceability',
        ),
        DeclareLaunchArgument(
            'inject_mode',
            default_value='topic',
            description=(
                "URScript injection transport: 'topic' (via "
                "/urscript_interface/script_command) or 'primary_tcp' "
                "(direct TCP write to URSim Secondary client port)"
            ),
        ),
        DeclareLaunchArgument(
            'ursim_primary_host',
            default_value='127.0.0.1',
            description='URSim host for primary_tcp injection mode',
        ),
        DeclareLaunchArgument(
            'ursim_primary_port',
            default_value='30002',
            description='URSim Secondary client port for primary_tcp mode',
        ),
        recorder,
        delayed_publisher,
    ])
