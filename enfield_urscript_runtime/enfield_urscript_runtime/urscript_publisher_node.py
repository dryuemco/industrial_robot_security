# SPDX-License-Identifier: Apache-2.0
"""URScript publisher node.

Loads a Task IR file, translates to URScript via enfield_translators,
publishes once to /urscript_interface/script_command (std_msgs/String),
optionally saves the script to disk for traceability, then exits.

Simulation-only: target is URSim e-Series via ur_robot_driver.
"""
from __future__ import annotations

from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

from enfield_translators.urscript_translator import IRToURScriptTranslator


class URScriptPublisherNode(Node):
    """Publish a translated URScript program to ur_robot_driver."""

    def __init__(self) -> None:
        super().__init__('urscript_publisher')

        # Parameters
        self.declare_parameter('task_ir_path', '')
        self.declare_parameter(
            'topic', '/urscript_interface/script_command'
        )
        self.declare_parameter('publish_delay_s', 2.0)
        self.declare_parameter('linger_s', 3.0)
        self.declare_parameter('save_script_path', '')

        task_ir_path = (
            self.get_parameter('task_ir_path')
            .get_parameter_value().string_value
        )
        topic = (
            self.get_parameter('topic').get_parameter_value().string_value
        )
        publish_delay_s = (
            self.get_parameter('publish_delay_s')
            .get_parameter_value().double_value
        )
        self._linger_s = (
            self.get_parameter('linger_s')
            .get_parameter_value().double_value
        )
        save_script_path = (
            self.get_parameter('save_script_path')
            .get_parameter_value().string_value
        )

        if not task_ir_path:
            self.get_logger().fatal('task_ir_path parameter is required')
            raise SystemExit(2)
        if not Path(task_ir_path).is_file():
            self.get_logger().fatal(
                f'task_ir_path does not exist: {task_ir_path}'
            )
            raise SystemExit(2)

        # Translate
        translator = IRToURScriptTranslator()
        self._script = translator.translate_file(task_ir_path)
        line_count = self._script.count('\n') + 1
        self.get_logger().info(
            f'Translated {task_ir_path} -> {line_count} lines URScript'
        )

        # Optional disk dump for traceability
        if save_script_path:
            out = Path(save_script_path)
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(self._script)
            self.get_logger().info(f'URScript saved to {out}')

        # Publisher: reliable + transient_local so a late-joining
        # ur_robot_driver still receives the script.
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(String, topic, qos)
        self._topic = topic
        self._published = False

        # Schedule publish after warm-up delay
        self._timer = self.create_timer(publish_delay_s, self._publish_once)

    def _publish_once(self) -> None:
        if self._published:
            return
        msg = String()
        msg.data = self._script
        self._pub.publish(msg)
        self._published = True
        self.get_logger().info(
            f'Published URScript to {self._topic} '
            f'({len(self._script)} bytes)'
        )
        # Linger so transient_local has time to deliver, then shutdown.
        self.create_timer(self._linger_s, self._shutdown)

    def _shutdown(self) -> None:
        self.get_logger().info('Linger elapsed, shutting down')
        rclpy.shutdown()


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    try:
        node = URScriptPublisherNode()
    except SystemExit as exc:
        rclpy.shutdown()
        return int(exc.code or 1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
