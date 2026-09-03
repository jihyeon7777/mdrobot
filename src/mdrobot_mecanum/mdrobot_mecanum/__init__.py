"""Mecanum drive layer for a 4-wheel base on two dual-channel MDROBOT controllers.

The generic `mdrobot` library is deliberately kinematics-free; this package is the
robot layer that sits on top of it. It is rclpy-free on purpose, so it runs from a
bare clone with nothing but pyserial and PyYAML — and so a ROS 2 node can later
import the same code unchanged.

    from mdrobot_mecanum import MecanumGeometry, inverse

    geom = MecanumGeometry(wheel_radius=0.05, track=0.30, wheelbase=0.28)
    fl, fr, rl, rr = inverse(vx=0.2, vy=0.0, wz=0.0, geom=geom)   # rad/s at the wheel

See `docs/mecanum-plan.md` for the build order and the hardware verification steps.
"""

from __future__ import annotations

from .config import (
    CONFIG_VERSION,
    ConfigError,
    Limits,
    MecanumConfig,
    Runtime,
    WheelSpec,
)
from .kinematics import (
    FL,
    FR,
    HANDEDNESS,
    PROVISIONAL_LAYOUT,
    RL,
    ROLLER_LAYOUTS,
    RR,
    WHEEL_ABBREV,
    WHEEL_NAMES,
    MecanumGeometry,
    forward,
    inverse,
    scale_to_limit,
    slip_residual,
)

__all__ = [
    "MecanumConfig",
    "ConfigError",
    "WheelSpec",
    "Limits",
    "Runtime",
    "CONFIG_VERSION",
    "MecanumGeometry",
    "inverse",
    "forward",
    "slip_residual",
    "scale_to_limit",
    "WHEEL_NAMES",
    "WHEEL_ABBREV",
    "ROLLER_LAYOUTS",
    "PROVISIONAL_LAYOUT",
    "HANDEDNESS",
    "FL",
    "FR",
    "RL",
    "RR",
]
