# Copyright 2026 Yunus Emre Cogurcu - Apache-2.0
"""URScript utility functions: unit conversions and quaternion → axis-angle."""

from __future__ import annotations

import math
from typing import Any


def mm_to_m(val: float) -> float:
    """Convert millimetres to metres."""
    return val / 1000.0


def mm_s_to_m_s(val: float) -> float:
    """Convert mm/s to m/s."""
    return val / 1000.0


def mm_s2_to_m_s2(val: float) -> float:
    """Convert mm/s² to m/s²."""
    return val / 1000.0


def quaternion_to_axis_angle(
    qx: float, qy: float, qz: float, qw: float
) -> tuple[float, float, float]:
    """Convert quaternion (qx, qy, qz, qw) to axis-angle (rx, ry, rz).

    URScript uses axis-angle representation where the vector direction
    is the rotation axis and its magnitude is the rotation angle in radians.

    Args:
        qx, qy, qz, qw: Unit quaternion components.

    Returns:
        Tuple (rx, ry, rz) — axis-angle rotation vector.
    """
    # Ensure unit quaternion (normalize)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return (0.0, 0.0, 0.0)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

    # Ensure qw >= 0 for shortest path
    if qw < 0:
        qx, qy, qz, qw = -qx, -qy, -qz, -qw

    # Compute rotation angle
    sin_half = math.sqrt(qx * qx + qy * qy + qz * qz)
    angle = 2.0 * math.atan2(sin_half, qw)

    if sin_half < 1e-10:
        # Near-zero rotation → axis is arbitrary, magnitude ≈ 0
        return (0.0, 0.0, 0.0)

    # Axis (unit vector) × angle
    ax = qx / sin_half * angle
    ay = qy / sin_half * angle
    az = qz / sin_half * angle

    return (ax, ay, az)


def pose_to_urscript(
    position: dict[str, float],
    orientation: dict[str, float] | None = None,
) -> str:
    """Convert IR pose to URScript p[x,y,z,rx,ry,rz] string.

    Args:
        position: {"x": mm, "y": mm, "z": mm}
        orientation: {"qx": ..., "qy": ..., "qz": ..., "qw": ...} or None

    Returns:
        String like ``p[0.400, 0.000, 0.350, 0.000, 3.142, 0.000]``
    """
    x = mm_to_m(position["x"])
    y = mm_to_m(position["y"])
    z = mm_to_m(position["z"])

    if orientation:
        rx, ry, rz = quaternion_to_axis_angle(
            orientation.get("qx", 0.0),
            orientation.get("qy", 0.0),
            orientation.get("qz", 0.0),
            orientation.get("qw", 1.0),
        )
    else:
        rx, ry, rz = 0.0, 0.0, 0.0

    return f"p[{x:.6f}, {y:.6f}, {z:.6f}, {rx:.4f}, {ry:.4f}, {rz:.4f}]"


def joints_to_urscript(values: list[float]) -> str:
    """Convert IR joint values (radians) to URScript joint vector string.

    Args:
        values: List of joint positions in radians.

    Returns:
        String like ``[0.0000, -1.5708, 1.5708, -1.5708, -1.5708, 0.0000]``
    """
    parts = [f"{v:.4f}" for v in values]
    return "[" + ", ".join(parts) + "]"


def format_float(val: float, decimals: int = 4) -> str:
    """Format a float to fixed decimal places."""
    return f"{val:.{decimals}f}"
