# SPDX-License-Identifier: Apache-2.0
"""URScript publisher node.

Loads a Task IR file, translates to URScript via enfield_translators,
and injects it into URSim by one of two transports:

* ``inject_mode='topic'`` (default, backward compatible) — publishes once
  to ``/urscript_interface/script_command`` (std_msgs/String). Requires the
  ``ur_robot_driver`` External Control URCap loop to be active and forwards
  through the ROS2 driver path.

* ``inject_mode='primary_tcp'`` — opens a direct TCP connection to URSim's
  Secondary client interface (default port 30002) and writes the URScript
  with a trailing newline. URControl parses the ``def task_*(): ... end``
  block as a named program and auto-runs it. The translator's trailing
  entry-point call (``task_NAME()`` after ``end``) is stripped before
  injection because URSim's Secondary client treats it as a stray
  statement and emits a parse error followed by a disconnect, which in
  turn breaks the ROS2 driver's paired connection and triggers a
  protective stop. With the trailing call stripped, the named program
  loads cleanly and runs to completion.

Simulation-only: target is URSim e-Series via ur_robot_driver or via the
direct primary/secondary client interface.
"""
from __future__ import annotations

import re
import socket
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String

from enfield_translators.urscript_translator import IRToURScriptTranslator


VALID_INJECT_MODES = ('topic', 'primary_tcp')

# Trailing entry-point call pattern emitted by the IR -> URScript
# translator (e.g. "task_T001()") sitting on its own line, optionally
# preceded by a comment line such as "# Entry point". URSim's Secondary
# client treats such a stray call as a parse error after a def...end
# block; we strip it for primary_tcp injection only.
_ENTRY_POINT_LINE_RE = re.compile(
    r'(?:[ \t]*#[^\n]*\n)?[ \t]*[A-Za-z_][A-Za-z0-9_]*\(\)[ \t]*\n?\s*\Z'
)


class URScriptPublisherNode(Node):
    """Inject a translated URScript program into URSim.

    Two transports are supported, selected by the ``inject_mode`` parameter:
    ROS2 topic publish (default) or direct TCP write to URSim's Secondary
    client interface.
    """

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
        # Phase 3.E (Lane 2 closure) parameters
        self.declare_parameter('inject_mode', 'topic')
        self.declare_parameter('ursim_primary_host', '127.0.0.1')
        self.declare_parameter('ursim_primary_port', 30002)
        self.declare_parameter('tcp_connect_timeout_s', 5.0)

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
        self._inject_mode = (
            self.get_parameter('inject_mode')
            .get_parameter_value().string_value
        )
        self._ursim_host = (
            self.get_parameter('ursim_primary_host')
            .get_parameter_value().string_value
        )
        self._ursim_port = (
            self.get_parameter('ursim_primary_port')
            .get_parameter_value().integer_value
        )
        self._tcp_timeout_s = (
            self.get_parameter('tcp_connect_timeout_s')
            .get_parameter_value().double_value
        )

        if self._inject_mode not in VALID_INJECT_MODES:
            self.get_logger().fatal(
                f"inject_mode must be one of {VALID_INJECT_MODES}, "
                f"got {self._inject_mode!r}"
            )
            raise SystemExit(2)

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

        # Optional disk dump for traceability (full unstripped form)
        if save_script_path:
            out = Path(save_script_path)
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(self._script)
            self.get_logger().info(f'URScript saved to {out}')

        # Publisher (always created so 'topic' mode works without rebuild;
        # primary_tcp mode just won't use it).
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(String, topic, qos)
        self._topic = topic
        self._published = False

        # Schedule injection after warm-up delay
        self._timer = self.create_timer(publish_delay_s, self._publish_once)

    # ------------------------------------------------------------------ #
    # Injection paths
    # ------------------------------------------------------------------ #

    def _publish_once(self) -> None:
        if self._published:
            return
        if self._inject_mode == 'topic':
            self._inject_via_topic()
        elif self._inject_mode == 'primary_tcp':
            self._inject_via_primary_tcp()
        # _inject_mode pre-validated; no else branch reachable.
        self._published = True
        self.create_timer(self._linger_s, self._shutdown)

    def _inject_via_topic(self) -> None:
        msg = String()
        msg.data = self._script
        self._pub.publish(msg)
        self.get_logger().info(
            f'Published URScript to {self._topic} '
            f'({len(self._script)} bytes)'
        )

    def _inject_via_primary_tcp(self) -> None:
        # Strip the trailing entry-point call (e.g. "task_T001()") that
        # the translator emits after the def...end block. URSim's
        # Secondary client auto-runs named programs once their def block
        # is received; the trailing call is therefore a stray statement
        # that triggers TCPReceiver::parseLine and disconnects.
        payload = _ENTRY_POINT_LINE_RE.sub('\n', self._script)
        if not payload.endswith('\n'):
            payload = payload + '\n'

        try:
            with socket.create_connection(
                (self._ursim_host, self._ursim_port),
                timeout=self._tcp_timeout_s,
            ) as sock:
                sock.sendall(payload.encode('utf-8'))
        except OSError as exc:
            self.get_logger().fatal(
                f'TCP injection to {self._ursim_host}:{self._ursim_port} '
                f'failed: {exc}'
            )
            raise SystemExit(3)

        self.get_logger().info(
            f'Wrote URScript to {self._ursim_host}:{self._ursim_port} '
            f'({len(payload)} bytes, primary_tcp mode, '
            f'entry-point-stripped)'
        )

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
