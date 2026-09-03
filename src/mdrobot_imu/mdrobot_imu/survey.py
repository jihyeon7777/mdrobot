"""Record what the sensor actually does, and say what it means.

Two questions have to be answered with numbers before any of this steers the
machine, and this tool answers both.

**How far does the machine really twist?** Drive it the way the phase under
test does and read the yaw excursion off the summary. Under a couple of degrees
and correcting is not worth building — watch it and abort on it instead. Ten or
fifteen and it is the main event.

**Does the gyro report a slow twist at all?** Standing still, all three gyro
axes read exactly 0.0000 for every sample. At the +/-2000 deg/s range one count
is 0.061 deg/s and a real MEMS gyro dithers by at least a count, so the
automatic zero-bias calibration is clamping small rates to zero. Excellent for
standing still, and potentially fatal here, because this machine twists slowly.
``zero-rate samples`` in the summary is the fraction that clamp swallowed;
watch whether yaw moves while the gyro insists it is zero.

Where the samples come from
---------------------------
Two sources, because the answer depends on what else is running.

``--topic`` (needs ROS, works alongside everything else)
    Subscribes to the node's own ``sensor_msgs/Imu``. **Use this whenever
    bringup is up** — for anything that needs the machine driving or the drill
    turning, which is to say most of the interesting measurements.

default (no ROS at all, needs the port to itself)
    Opens the serial port directly. Nothing else may have it open: two readers
    on one tty each get an arbitrary half of the stream and neither can frame
    it, so both quietly run at half rate with gaps. bringup starts the IMU
    node, so this mode and bringup are mutually exclusive.

Usage::

    ros2 run mdrobot_imu imu_survey --seconds 30 --csv drill.csv --topic
    python3 -m mdrobot_imu.survey --seconds 60 --csv shuffle.csv
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

from .frames import wrap_deg
from .reader import BAUDRATE, ImuReader

# The sensor's factory default output rate, and what the fitted one uses.
EXPECTED_RATE_HZ = 10.0
DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
DEFAULT_TOPIC = "/mdrobot_imu/data"

# t, roll, pitch, yaw (degrees), gx, gy, gz (deg/s)
Row = tuple[float, float, float, float, float, float, float]


class Recorder:
    """Accumulates rows, writes the CSV, and draws the live line."""

    def __init__(self, csv_path: str | None, quiet: bool) -> None:
        self.rows: list[Row] = []
        self.reference: tuple[float, float, float] | None = None
        self.quiet = quiet
        self.handle = open(csv_path, "w") if csv_path else None
        if self.handle:
            self.handle.write("t,roll_deg,pitch_deg,yaw_deg,gx_dps,gy_dps,gz_dps\n")

    def add(self, row: Row) -> None:
        self.rows.append(row)
        if self.reference is None:
            self.reference = (row[1], row[2], row[3])
        if self.handle:
            self.handle.write(
                f"{row[0]:.3f},{row[1]:.4f},{row[2]:.4f},{row[3]:.4f},"
                f"{row[4]:.4f},{row[5]:.4f},{row[6]:.4f}\n"
            )

    def draw(self) -> None:
        if self.quiet or not self.rows or self.reference is None:
            return
        t, roll, pitch, yaw = self.rows[-1][:4]
        sys.stdout.write(
            f"\r t={t:6.1f}s  roll {roll:+8.3f}  pitch {pitch:+8.3f}  "
            f"yaw {yaw:+8.3f}  (dyaw {wrap_deg(yaw - self.reference[2]):+7.3f})   "
        )
        sys.stdout.flush()

    def close(self) -> None:
        if self.handle:
            self.handle.close()
            self.handle = None


def collect_serial(args, rec: Recorder) -> int:
    """Read the port directly. Returns bytes dropped as unframeable."""
    reader = ImuReader(args.port, args.baudrate)
    started = time.monotonic()
    with reader:
        while time.monotonic() - started < args.seconds:
            samples = reader.poll()
            if not samples:
                time.sleep(0.005)
                continue
            for s in samples:
                a, g = s.angle, s.gyro
                rec.add((s.monotonic - started, a.roll, a.pitch, a.yaw,
                         g.x, g.y, g.z))
            rec.draw()
    return reader.dropped


def collect_topic(args, rec: Recorder) -> int:
    """Subscribe to the node's sensor_msgs/Imu instead of opening the port.

    Attitude arrives as a quaternion and rates in rad/s, so both are converted
    back to the degrees the rest of this tool works in. These are the node's
    MAPPED values — signs and mount offset applied — where the serial path
    gives the sensor's raw ones. For the questions here that makes no
    difference: an excursion is an excursion and zero is zero either way.
    """
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    started = time.monotonic()

    class Sink(Node):
        def __init__(self) -> None:
            super().__init__("mdrobot_imu_survey")
            self.create_subscription(Imu, args.topic, self._on_imu, 50)

        def _on_imu(self, msg: Imu) -> None:
            q = msg.orientation
            roll = math.degrees(math.atan2(
                2.0 * (q.w * q.x + q.y * q.z),
                1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
            sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
            pitch = math.degrees(math.asin(sinp))
            yaw = math.degrees(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            w = msg.angular_velocity
            rec.add((time.monotonic() - started, roll, pitch, yaw,
                     math.degrees(w.x), math.degrees(w.y), math.degrees(w.z)))
            rec.draw()

    rclpy.init()
    node = Sink()
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def summarise(rec: Recorder, args, dropped: int) -> int:
    rows, reference = rec.rows, rec.reference
    print()
    if not rows or reference is None:
        if args.topic_mode:
            print(f"nothing arrived on {args.topic}. Is the node running?\n"
                  f"  ros2 launch mdrobot_imu imu.launch.py")
        else:
            print("no samples. Wrong port, wrong baud rate, or the sensor is "
                  "not sending acceleration + angular velocity + angle.")
        return 1

    elapsed = rows[-1][0]
    rate = len(rows) / max(elapsed, 1e-9)
    print(f"{len(rows)} samples over {elapsed:.1f} s -> {rate:.1f} Hz")
    if dropped:
        print(f"{dropped} bytes dropped as unframeable")
    # A rate well under the sensor's output rate is not a slow robot, it is
    # lost bytes -- and by far the commonest cause is a second reader on the
    # same port.
    if rate < 0.75 * args.expect_hz:
        print(
            f"\n  !! {rate:.1f} Hz is well under the {args.expect_hz:.0f} Hz "
            f"the sensor sends at. Samples are being lost, so this run has GAPS "
            f"and a peak excursion read off it is a lower bound, not a "
            f"measurement."
        )
        if not args.topic_mode:
            print(
                f"     Something else has the port open. bringup starts the IMU "
                f"node, so\n"
                f"     check with:  pgrep -af 'imu_node|bringup'\n"
                f"     and either stop it, or re-run this with --topic, which "
                f"reads the\n"
                f"     node's own output and works alongside anything."
            )
        print()

    for name, index, ref in (
        ("roll ", 1, reference[0]),
        ("pitch", 2, reference[1]),
        ("yaw  ", 3, reference[2]),
    ):
        deltas = [wrap_deg(r[index] - ref) for r in rows]
        print(
            f"{name}: start {ref:+8.3f}  end {deltas[-1]:+7.3f} from start  "
            f"excursion {min(deltas):+7.3f} .. {max(deltas):+7.3f}  "
            f"peak |d| {max(abs(d) for d in deltas):6.3f} deg"
        )
    zero = sum(1 for r in rows if r[4] == 0.0 and r[5] == 0.0 and r[6] == 0.0)
    print(
        f"zero-rate samples: {zero}/{len(rows)} ({100.0 * zero / len(rows):.1f}%) "
        f"— all three gyro axes exactly 0.0000. A high figure while the yaw is "
        f"visibly moving means the auto zero-bias calibration is swallowing the "
        f"rotation."
    )
    peak = max(max(abs(r[4]), abs(r[5]), abs(r[6])) for r in rows)
    print(f"peak |gyro| seen: {peak:.4f} deg/s (one count is 0.061)")
    if len(rows) > 2:
        gz = [r[6] for r in rows]
        print(f"gz mean {statistics.mean(gz):+.4f} deg/s, "
              f"sd {statistics.pstdev(gz):.4f}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "--topic", nargs="?", const=DEFAULT_TOPIC, default=None,
        metavar="TOPIC",
        help=f"read the node's sensor_msgs/Imu instead of the port "
             f"(default {DEFAULT_TOPIC}). USE THIS when bringup is running")
    # A by-id path, not a number: the CH340 the sensor uses and the FTDI the
    # motor controllers use share the /dev/ttyUSB numbering space, and which is
    # which depends on plug order.
    parser.add_argument(
        "--port", default=DEFAULT_PORT,
        help="serial port; `ls -l /dev/serial/by-id/` to find yours")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--csv", default=None, help="write every sample here")
    parser.add_argument("--quiet", action="store_true", help="no live line")
    parser.add_argument("--expect-hz", type=float, default=EXPECTED_RATE_HZ,
                        help="the sensor's configured output rate")
    args = parser.parse_args(argv)
    args.topic_mode = args.topic is not None

    rec = Recorder(args.csv, args.quiet)
    if args.topic_mode:
        print(f"reading {args.topic} for {args.seconds:.0f} s ...")
    else:
        print(f"reading {args.port} at {args.baudrate} baud for "
              f"{args.seconds:.0f} s ...")
    dropped = 0
    try:
        dropped = (collect_topic if args.topic_mode else collect_serial)(args, rec)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        rec.close()
    return summarise(rec, args, dropped)


if __name__ == "__main__":
    raise SystemExit(main())
