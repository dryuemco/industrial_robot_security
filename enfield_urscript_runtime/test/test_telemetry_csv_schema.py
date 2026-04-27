# SPDX-License-Identifier: Apache-2.0
"""Offline test: telemetry recorder CSV schema + capture sanity.

Spawns recorder + two fake publishers in-process via rclpy executors,
publishes 5 JointState + 5 PoseStamped messages, lets recorder write
to a tempfile, then validates the CSV.

Runs without URSim or driver; CI gate.
"""
from __future__ import annotations

import csv
import threading
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from enfield_urscript_runtime.telemetry_recorder_node import (
    CSV_HEADER, TelemetryRecorderNode,
)


N_MSGS = 5


class _FakePublishers(Node):
    def __init__(self) -> None:
        super().__init__('fake_publishers')
        self._js_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )
        tcp_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self._tcp_pub = self.create_publisher(
            PoseStamped, '/tcp_pose_broadcaster/pose', tcp_qos
        )

    def publish_joint(self, i: int) -> None:
        m = JointState()
        m.header.stamp.sec = 1000 + i
        m.header.stamp.nanosec = 0
        m.header.frame_id = ''
        m.position = [0.1 * i] * 6
        m.velocity = [0.01 * i] * 6
        self._js_pub.publish(m)

    def publish_tcp(self, i: int) -> None:
        m = PoseStamped()
        m.header.stamp.sec = 2000 + i
        m.header.stamp.nanosec = 0
        m.header.frame_id = 'base'
        m.pose.position.x = 0.4
        m.pose.position.y = 0.0
        m.pose.position.z = 0.2 + 0.01 * i
        m.pose.orientation.w = 1.0
        self._tcp_pub.publish(m)


@pytest.fixture
def csv_path(tmp_path: Path) -> Path:
    return tmp_path / 'telemetry_test.csv'


def test_recorder_csv_schema_and_capture(csv_path: Path) -> None:
    rclpy.init()
    try:
        # Override default '/tmp/telemetry.csv' parameter via the
        # ROS 2 parameter overrides mechanism BEFORE __init__ opens
        # the CSV. We do this by setting environment-style overrides
        # through a parameter file is heavyweight; instead we
        # construct the node and rely on set_parameters_atomically
        # before subscribers fire (publishers don't exist yet).
        # Simpler: monkey-patch the default by direct argv.
        pass
    finally:
        pass

    # Cleanest path: instantiate with command-line args specifying
    # the parameter override.
    argv = [
        '--ros-args',
        '-p', f'output_csv:={csv_path}',
        '-p', 'duration_s:=60.0',
    ]
    # rclpy.init was already called above; re-initialise won't help.
    # Use init args at the start.
    rclpy.shutdown()
    rclpy.init(args=argv)

    try:
        recorder = TelemetryRecorderNode()
    except Exception:
        rclpy.shutdown()
        raise

    publishers = _FakePublishers()

    exec_ = SingleThreadedExecutor()
    exec_.add_node(recorder)
    exec_.add_node(publishers)

    spin_thread = threading.Thread(target=exec_.spin, daemon=True)
    spin_thread.start()

    try:
        # Allow discovery
        time.sleep(0.5)
        for i in range(N_MSGS):
            publishers.publish_joint(i)
            publishers.publish_tcp(i)
            time.sleep(0.05)
        time.sleep(0.5)  # drain
    finally:
        exec_.shutdown()
        recorder.close()
        try:
            publishers.destroy_node()
        except Exception:
            pass
        try:
            recorder.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    # === Validate CSV ===
    assert csv_path.is_file(), f'CSV not written: {csv_path}'
    with open(csv_path, 'r') as f:
        rows = list(csv.reader(f))
    assert len(rows) >= 1, 'CSV is empty (no header)'
    assert rows[0] == CSV_HEADER, (
        f'Header mismatch:\n  got: {rows[0]}\n  exp: {CSV_HEADER}'
    )
    data_rows = rows[1:]
    assert len(data_rows) >= N_MSGS, (
        f'Expected >= {N_MSGS} data rows, got {len(data_rows)}'
    )
    sources = {r[2] for r in data_rows if len(r) > 2}
    assert 'joint_state' in sources, (
        f'No joint_state rows captured (sources={sources})'
    )
    assert 'tcp_pose' in sources, (
        f'No tcp_pose rows captured (sources={sources})'
    )

    # Numeric parse sanity on at least one tcp row
    tcp_rows = [r for r in data_rows if r[2] == 'tcp_pose']
    assert tcp_rows, 'No tcp_pose rows for numeric check'
    sample = tcp_rows[0]
    # tcp_x at index 16, tcp_qw at index 22
    assert float(sample[16]) == pytest.approx(0.4)
    assert float(sample[22]) == pytest.approx(1.0)
