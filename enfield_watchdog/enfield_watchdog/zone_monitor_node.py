"""Zone boundary monitor — checks TCP position against safeguarded space."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import rclpy
from rclpy.node import Node


class ZoneMonitorNode(Node):
    """Monitor TCP position against convex polytope safe zone (A2 detection).

    Subscribes to TCP position (from TF or /tool_pose), evaluates halfspace
    constraints, and publishes SafetyViolation on boundary penetration.
    """

    def __init__(self) -> None:
        super().__init__('zone_monitor')
        self.declare_parameter('check_rate_hz', 50.0)
        self.declare_parameter('interpolation_step_mm', 10.0)
        self.get_logger().info('ZoneMonitorNode initialized (placeholder).')


def main(args=None):
    rclpy.init(args=args)
    node = ZoneMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
