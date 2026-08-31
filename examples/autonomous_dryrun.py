#!/usr/bin/env python3
"""Run the autonomous sequence end to end with simulated inputs. No hardware.

The state machine has been tested on its own, but the wired-up node never has:
the sequence only advances if ~/plate_offset really reaches it, if
~/joint_states really turns into travelled distance, and if the drill request
really comes out on ~/command. This drives the real SupervisorNode through the
whole approach with those topics faked, and prints what it does at each step.

The simulated robot moves: commanded vx is integrated into wheel positions and
fed back as ~/joint_states, so the blind entry finishes because the machine
actually travelled, not because a timer expired. The plate is placed off to one
side and disappears once the machine is close, which is what happens under a
real car.

    python3 examples/autonomous_dryrun.py
    python3 examples/autonomous_dryrun.py --entry-distance 0.5 --plate-x 0.6

Exits non-zero if the sequence does not reach `done`.
"""

from __future__ import annotations

import argparse
import math
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray, String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mdrobot_supervisor"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mdrobot_rc_bridge"))

from mdrobot_supervisor.supervisor_node import SupervisorNode  # noqa: E402

# Idle RC frame: sticks centred, autonomous selected on the mode switch.
NEUTRAL = [1500, 1500, 0, 0, 1, 0, 0, 0, 0, 0]
GEAR_RATIO = 20.0
WHEEL_RADIUS = 0.0625
# Mounting direction per wheel, matching the config. The left motors are
# mirrored, so driving FORWARD turns the four encoders [+, -, +, -] — publishing
# the same sign on all four would make the supervisor's signed mean exactly zero.
WHEEL_SIGNS = (1, -1, 1, -1)


class Simulator(Node):
    """Stands in for the board, the camera and the drive."""

    def __init__(self, args) -> None:
        super().__init__("dryrun_sim")
        self.args = args
        self.distance = 0.0
        # Lateral position of the machine and of the plate, both in the body
        # frame where +y is LEFT. The camera reports "positive = plate is to the
        # RIGHT", so a plate reported at +0.5 sits at y = -0.5. Alignment has to
        # close the gap; if the strafe sign is wrong it opens it, which a fixed
        # offset would never reveal.
        self.lateral = 0.0
        self.plate_lateral = -args.plate_x
        self.worst_error = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.phase = ""
        self.seen: list[str] = []
        self.command: list[int] = [0] * 5
        self.drill_seen = False

        ns = "/mdrobot_supervisor"
        self.pub_rc = self.create_publisher(Int32MultiArray, f"{ns}/rc", 10)
        self.pub_plate = self.create_publisher(Point, f"{ns}/plate_offset", 10)
        self.pub_joints = self.create_publisher(JointState, f"{ns}/joint_states", 10)
        self.create_subscription(
            Float64MultiArray, f"{ns}/cmd_wheel_rpm", self._on_wheels, 10)
        self.create_subscription(Int32MultiArray, f"{ns}/command", self._on_command, 10)
        self.create_subscription(String, f"{ns}/auto_phase", self._on_phase, 10)

        self.started = time.monotonic()
        self.create_timer(0.05, self._tick)

    # ── what the supervisor asks for ────────────────────────────────────────
    def _on_wheels(self, msg: Float64MultiArray) -> None:
        """Turn commanded wheel rpm back into a body velocity, roughly."""
        if len(msg.data) != 4:
            return
        # Undo the mounting signs, then average: for pure vx all four agree.
        forward = sum(v * s for v, s in zip(msg.data, WHEEL_SIGNS)) / 4.0
        self.vx = forward / GEAR_RATIO / 60.0 * 2 * math.pi * WHEEL_RADIUS
        # Strafe: the vy column is (-1, +1, +1, -1) at the wheel, so undoing the
        # mounting signs with that pattern recovers the lateral component.
        vy_col = (-1, 1, 1, -1)
        lateral = sum(v * s * c for v, s, c
                      in zip(msg.data, WHEEL_SIGNS, vy_col)) / 4.0
        self.vy = lateral / GEAR_RATIO / 60.0 * 2 * math.pi * WHEEL_RADIUS

    def _on_command(self, msg: Int32MultiArray) -> None:
        self.command = list(msg.data)
        if len(msg.data) >= 3 and msg.data[2]:
            self.drill_seen = True

    def _on_phase(self, msg: String) -> None:
        if msg.data and msg.data != self.phase:
            self.phase = msg.data
            self.seen.append(msg.data)
            print(f"  [{time.monotonic() - self.started:5.1f}s] phase -> {msg.data:11s}"
                  f"  travelled {self.distance:.3f} m  command {self.command}")

    # ── what the world does back ────────────────────────────────────────────
    def _tick(self) -> None:
        dt = 0.05
        self.distance += self.vx * dt
        self.lateral += self.vy * dt
        # Back to the camera's convention: positive means the plate is to the
        # right of where the machine is now.
        error = -(self.plate_lateral - self.lateral)
        self.worst_error = max(self.worst_error, abs(error))

        rc = Int32MultiArray()
        rc.data = list(NEUTRAL)
        self.pub_rc.publish(rc)

        # The plate is visible until the machine gets close, then it slides out
        # of view under the car — which is the cue to go blind.
        if self.distance < self.args.plate_lost_at:
            p = Point()
            p.x = error          # closes as the machine strafes the right way
            p.y = 0.0
            p.z = 0.2
            self.pub_plate.publish(p)

        motor_rad = self.distance / WHEEL_RADIUS * GEAR_RATIO
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["front_left", "front_right", "rear_left", "rear_right"]
        js.position = [motor_rad * s for s in WHEEL_SIGNS]
        self.pub_joints.publish(js)


def write_params(args) -> str:
    """A parameter file for the supervisor, tuned to run fast."""
    text = f"""
/**:
  ros__parameters:
    rate: 20.0
    rc_timeout: 1.0
    wheel_position_units: rad
    wheel_signs: [1, -1, 1, -1]
    roller_layout: unknown
    gear_ratio: {GEAR_RATIO}
    wheel_radius: {WHEEL_RADIUS}
    max_linear_x: 0.19
    max_linear_y: 0.19
    require_neutral_start: true
    mode_names: ['base', 'mecanum', 'autonomous']
    auto_entry_distance: {args.entry_distance}
    auto_entry_speed: 0.15
    auto_approach_speed: 0.15
    auto_drill_seconds: {args.drill_seconds}
    auto_hole_stage: false
    auto_max_align_seconds: 30.0
    auto_max_entry_seconds: 30.0
"""
    handle = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    handle.write(text)
    handle.close()
    return handle.name


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--entry-distance", type=float, default=0.6)
    ap.add_argument("--plate-x", type=float, default=0.5,
                    help="normalised plate offset; positive = right of centre")
    ap.add_argument("--plate-lost-at", type=float, default=0.4,
                    help="metres travelled at which the plate goes out of view")
    ap.add_argument("--drill-seconds", type=float, default=1.0)
    ap.add_argument("--timeout", type=float, default=40.0)
    args = ap.parse_args()

    params = write_params(args)
    rclpy.init(args=["--ros-args", "--params-file", params])
    supervisor = SupervisorNode()
    sim = Simulator(args)

    print(f"\nplate at x={args.plate_x:+.2f}, out of view after "
          f"{args.plate_lost_at:.2f} m, entry {args.entry_distance:.2f} m\n")

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(supervisor)
    executor.add_node(sim)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if sim.phase in ("done", "abort"):
                for _ in range(10):
                    executor.spin_once(timeout_sec=0.02)
                break
    finally:
        supervisor.destroy_node()
        sim.destroy_node()
        rclpy.shutdown()

    final_error = abs(sim.plate_lateral - sim.lateral)
    started_error = abs(sim.plate_lateral)
    print(f"\nphases: {' -> '.join(sim.seen) or '(none)'}")
    print(f"travelled: {sim.distance:.3f} m     drill fired: {sim.drill_seen}")
    print(f"plate offset: started {started_error:.3f}, ended {final_error:.3f}, "
          f"worst {sim.worst_error:.3f}")
    aligned = final_error < started_error
    if not aligned:
        print("  the offset did not shrink — the strafe is going the WRONG WAY")
    # wait_plate is skipped when the plate is already in view on the first tick.
    required = ["align", "enter", "drill", "done"]
    ordered = [p for p in sim.seen if p in required]
    if ordered == required and sim.drill_seen and aligned:
        print("PASS — the whole approach ran on real topics")
        return 0
    print(f"FAIL — expected {' -> '.join(required)} in order, drill firing, "
          f"and the plate offset shrinking")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
