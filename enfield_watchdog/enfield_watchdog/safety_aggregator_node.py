"""Safety aggregator — collects violations, logs, and publishes verdicts."""
# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0

import rclpy
from rclpy.node import Node


class SafetyAggregatorNode(Node):
    """Aggregate SafetyViolation messages from all monitors.

    Subscribes to /enfield/violations, maintains per-scenario violation
    counts, and exports verdict JSON/CSV for evidence artifacts.
    """

    def __init__(self) -> None:
        super().__init__('safety_aggregator')
        self.declare_parameter('log_to_file', True)
        self.declare_parameter('log_path', 'results/verdicts/')
        self.get_logger().info('SafetyAggregatorNode initialized (placeholder).')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyAggregatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
