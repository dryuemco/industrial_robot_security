"""Velocity monitor — subscribes to joint_states, checks TCP speed limits."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import rclpy
from rclpy.node import Node


class VelocityMonitorNode(Node):
    """Monitor TCP velocity against ISO 10218 mode limits (A1 detection).

    Subscribes to /joint_states, computes TCP speed via forward kinematics,
    and publishes SafetyViolation when speed exceeds the configured limit.
    """

    def __init__(self) -> None:
        super().__init__('velocity_monitor')
        self.declare_parameter('operating_mode', 'collaborative')
        self.declare_parameter('max_tcp_speed_mm_s', 250.0)
        self.declare_parameter('check_rate_hz', 100.0)
        self.get_logger().info('VelocityMonitorNode initialized (placeholder).')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
