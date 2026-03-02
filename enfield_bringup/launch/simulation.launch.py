"""ENFIELD simulation launch — spawns UR5e + watchdog monitors."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
# Placeholder — will be populated when UR5e adapter is ready (PR-D)

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([
        # TODO (PR-D): UR5e Gazebo spawn
        # TODO (PR-D): MoveIt2 move_group
        # TODO: velocity_monitor node
        # TODO: zone_monitor node
        # TODO: safety_aggregator node
    ])
