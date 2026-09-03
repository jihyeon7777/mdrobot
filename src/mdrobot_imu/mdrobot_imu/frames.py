"""Angle conventions and conversions. No ROS, no I/O — just the maths.

Kept out of the node so the survey tool runs on a machine with no ROS
installed, and so the sign mapping — the part that can drive a controller the
wrong way — is unit-testable without a sensor.
"""

from __future__ import annotations

import math


def wrap_deg(angle: float) -> float:
    """Fold an angle into [-180, 180).

    Half-open at the top: exactly 180 comes back as -180. The discontinuity has
    to sit somewhere, and putting it here is what the plain modulo gives. It
    does not matter for the use this has — a yaw error of +180 and one of -180
    are the same rotation, and a controller acting on either turns the same
    amount — but it does matter that only one of them is ever produced, so a
    threshold test cannot see the same attitude as inside the limit one tick
    and outside it the next.
    """
    return (angle + 180.0) % 360.0 - 180.0


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """ZYX (yaw-pitch-roll) Euler angles in radians to (x, y, z, w)."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def to_ros_attitude(
    roll: float,
    pitch: float,
    yaw: float,
    signs: tuple[float, float, float],
    yaw_offset_deg: float = 0.0,
) -> tuple[float, float, float]:
    """Sensor-frame degrees to REP-103 body-frame degrees.

    ``signs`` is (roll, pitch, yaw), each +1 or -1. Which way round they go is
    a property of how the sensor is bolted to a particular robot and cannot be
    reasoned out: on this one the yaw sign was expected to be -1, on the
    grounds that WITMOTION yaw grows clockwise like a compass, and turning the
    machine left proved it +1. See the node's docstring for the three checks.
    """
    return (
        signs[0] * roll,
        signs[1] * pitch,
        wrap_deg(signs[2] * yaw + yaw_offset_deg),
    )
