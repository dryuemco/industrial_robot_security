# SPDX-License-Identifier: Apache-2.0
"""Telemetry recorder for URSim live execution.

Subscribes to /joint_states (sensor_msgs/JointState) and
/tcp_pose_broadcaster/pose (geometry_msgs/PoseStamped, RELIABLE +
TRANSIENT_LOCAL QoS), writes interleaved rows to a CSV file with one
row per incoming message.

CSV schema:
    stamp_sec, stamp_nsec, source, frame_id,
    j0_pos..j5_pos, j0_vel..j5_vel,
    tcp_x, tcp_y, tcp_z, tcp_qx, tcp_qy, tcp_qz, tcp_qw

JointState rows fill joint columns; pose columns blank.
PoseStamped rows fill pose columns; joint columns blank.

Recorder shuts down after `duration_s` seconds.
Simulation-only.
"""
from __future__ import annotations

import csv
import threading
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped


CSV_HEADER = [
    'stamp_sec', 'stamp_nsec', 'source', 'frame_id',
    'j0_pos', 'j1_pos', 'j2_pos', 'j3_pos', 'j4_pos', 'j5_pos',
    'j0_vel', 'j1_vel', 'j2_vel', 'j3_vel', 'j4_vel', 'j5_vel',
    'tcp_x', 'tcp_y', 'tcp_z',
    'tcp_qx', 'tcp_qy', 'tcp_qz', 'tcp_qw',
]


class TelemetryRecorderNode(Node):
    """Record /joint_states and /tcp_pose_broadcaster/pose to CSV."""

    def __init__(self) -> None:
        super().__init__('telemetry_recorder')

        self.declare_parameter('output_csv', '/tmp/telemetry.csv')
        self.declare_parameter('joint_topic', '/joint_states')
        self.declare_parameter(
            'tcp_topic', '/tcp_pose_broadcaster/pose'
        )
        self.declare_parameter('duration_s', 30.0)

        output_csv = (
            self.get_parameter('output_csv')
            .get_parameter_value().string_value
        )
        joint_topic = (
            self.get_parameter('joint_topic')
            .get_parameter_value().string_value
        )
        tcp_topic = (
            self.get_parameter('tcp_topic')
            .get_parameter_value().string_value
        )
        duration_s = (
            self.get_parameter('duration_s')
            .get_parameter_value().double_value
        )

        out_path = Path(output_csv)
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Open CSV up-front so partial captures still produce a file
        self._csv_lock = threading.Lock()
        self._csv_file = open(out_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(CSV_HEADER)
        self._csv_file.flush()
        self._row_count = 0
        self._output_path = out_path

        # JointState: standard reliable QoS suffices
        joint_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            JointState, joint_topic, self._on_joint_state, joint_qos
        )

        # tcp_pose_broadcaster publishes RELIABLE + TRANSIENT_LOCAL.
        # Subscription must match or messages will be dropped silently.
        tcp_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            PoseStamped, tcp_topic, self._on_tcp_pose, tcp_qos
        )

        self.get_logger().info(
            f'Recording to {out_path} '
            f'(joint={joint_topic}, tcp={tcp_topic}, '
            f'duration={duration_s:.1f}s)'
        )

        self._duration_s = duration_s
        self._shutdown_timer = self.create_timer(
            duration_s, self._shutdown
        )

    def _on_joint_state(self, msg: JointState) -> None:
        # JointState may report fewer than 6 joints during early
        # bring-up; pad with empty strings.
        positions = list(msg.position) + [''] * (6 - len(msg.position))
        velocities = list(msg.velocity) + [''] * (6 - len(msg.velocity))
        row = [
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            'joint_state', msg.header.frame_id or '',
            *positions[:6], *velocities[:6],
            '', '', '', '', '', '', '',
        ]
        self._write_row(row)

    def _on_tcp_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        o = msg.pose.orientation
        row = [
            msg.header.stamp.sec, msg.header.stamp.nanosec,
            'tcp_pose', msg.header.frame_id or '',
            '', '', '', '', '', '',
            '', '', '', '', '', '',
            p.x, p.y, p.z, o.x, o.y, o.z, o.w,
        ]
        self._write_row(row)

    def _write_row(self, row: list) -> None:
        with self._csv_lock:
            self._csv_writer.writerow(row)
            self._csv_file.flush()
            self._row_count += 1

    def _shutdown(self) -> None:
        self.get_logger().info(
            f'Duration {self._duration_s:.1f}s elapsed - '
            f'wrote {self._row_count} rows to {self._output_path}'
        )
        self.close()
        rclpy.shutdown()

    def close(self) -> None:
        with self._csv_lock:
            if not self._csv_file.closed:
                self._csv_file.close()


def main(argv: list[str] | None = None) -> int:
    rclpy.init(args=argv)
    try:
        node = TelemetryRecorderNode()
    except SystemExit as exc:
        rclpy.shutdown()
        return int(exc.code or 1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.close()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
