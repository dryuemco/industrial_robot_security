# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""DM-3: A3 Orientation Anomaly — forbidden direction cone test."""

from __future__ import annotations
import math
from typing import Any
from enfield_watchdog_static.violation import Violation


def _quat_to_tool_z(qx: float, qy: float, qz: float, qw: float) -> list[float]:
    """Convert quaternion to tool Z-axis direction in world frame.

    tool_z = R(q) · [0, 0, 1]^T
    Using rotation matrix from quaternion (column 3).
    """
    # Normalize
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-12:
        return [0.0, 0.0, 1.0]
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    # Third column of rotation matrix = tool Z in world
    zx = 2.0 * (qx*qz + qw*qy)
    zy = 2.0 * (qy*qz - qw*qx)
    zz = 1.0 - 2.0 * (qx*qx + qy*qy)
    return [zx, zy, zz]


def _dot3(a: list[float], b: list[float]) -> float:
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]


def check_a3_orientation(task: dict[str, Any]) -> list[Violation]:
    """Check all waypoint orientations against forbidden direction cones.

    For each cone (axis d_hat, half_angle alpha):
    violation if arccos(tool_z · d_hat) <= alpha.
    """
    violations: list[Violation] = []

    cones = task.get("safety_requirements", {}).get(
        "forbidden_orientation_cones", []
    )
    if not cones:
        return violations

    for cmd in task.get("motion_sequence", []):
        tp = cmd.get("target_pose")
        if not tp or "orientation" not in tp:
            continue

        orient = tp["orientation"]
        qx = orient.get("qx", 0.0)
        qy = orient.get("qy", 0.0)
        qz = orient.get("qz", 0.0)
        qw = orient.get("qw", 1.0)

        tool_z = _quat_to_tool_z(qx, qy, qz, qw)

        for ci, cone in enumerate(cones):
            axis = cone["axis"]
            half_angle_deg = cone["half_angle_deg"]
            label = cone.get("label", f"cone_{ci}")

            dot = _dot3(tool_z, axis)
            dot = max(-1.0, min(1.0, dot))  # clamp for acos safety
            angle_deg = math.degrees(math.acos(dot))

            if angle_deg <= half_angle_deg:
                penetration_deg = half_angle_deg - angle_deg
                violations.append(Violation(
                    attack_type="A3",
                    iso_clause="5.3",
                    detection_mechanism="DM-3",
                    description=(
                        f"Tool Z-axis {tool_z} inside forbidden cone "
                        f"'{label}': angle={angle_deg:.1f}° <= "
                        f"half_angle={half_angle_deg:.1f}°"
                    ),
                    severity=round(penetration_deg, 1),
                    location=f"motion_sequence[{cmd['seq']}].target_pose.orientation",
                    metadata={
                        "tool_z": [round(z, 4) for z in tool_z],
                        "cone_axis": axis,
                        "cone_label": label,
                        "angle_deg": round(angle_deg, 2),
                        "half_angle_deg": half_angle_deg,
                        "penetration_deg": round(penetration_deg, 2),
                    },
                ))

    return violations
