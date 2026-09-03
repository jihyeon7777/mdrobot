"""Record what the sensor actually does, with no ROS and no robot.

Two questions have to be answered with numbers before any of this steers the
machine, and this tool answers both.

**How far does the robot really twist?** Drive it the way the hole search does
— ``auto_hole_max_speed`` is 0.05 m/s — over the floor it will really work on,
for as long as the phase is allowed to run, and read the yaw excursion off the
summary. Under a couple of degrees and yaw hold is not worth building: watch it
and abort on it instead. Ten or fifteen and it is the main event.

**Does the gyro report a slow twist at all?** On the fitted sensor, standing
still, all three gyro axes read *exactly* 0.0000 for 900 consecutive samples
(2026-09-03). At the +/-2000 deg/s range one count is 0.061 deg/s, and a real
MEMS gyro dithers by at least a count — so the automatic zero-bias calibration
is clamping small rates to zero. That is excellent for standing still and
potentially fatal here: this robot twists *slowly*. If the twist rate sits
inside that clamp, the sensor reports no rotation while the machine quietly
turns, which is precisely the failure it was fitted to catch.

To measure the clamp, run this and turn the sensor by hand as slowly as you
can. ``zero-rate samples`` in the summary is the fraction the clamp swallowed;
watch whether yaw moves while the gyro insists it is zero. If it does, the
sensor's own "Gyro Auto Calibrate" has to come off — at the price of bias
drift, which this same tool will then measure standing still.

Usage::

    python3 -m mdrobot_imu.survey --port /dev/ttyUSB1 --seconds 60 --csv run.csv
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time

from .frames import wrap_deg
from .reader import BAUDRATE, ImuReader


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    # A by-id path, not a number: the CH340 the sensor uses and the FTDI the
    # motor controllers use share the /dev/ttyUSB numbering space, and which
    # is which depends on plug order. Reading is harmless either way, but
    # pointing this at the motor bus produces confusing silence.
    parser.add_argument(
        "--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
        help="serial port; `ls -l /dev/serial/by-id/` to find yours")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--csv", default=None, help="write every sample here")
    parser.add_argument("--quiet", action="store_true", help="no live line")
    args = parser.parse_args(argv)

    rows: list[tuple[float, float, float, float, float, float, float]] = []
    handle = open(args.csv, "w") if args.csv else None
    if handle:
        handle.write("t,roll_deg,pitch_deg,yaw_deg,gx_dps,gy_dps,gz_dps\n")

    print(f"reading {args.port} at {args.baudrate} baud for {args.seconds:.0f} s ...")
    started = time.monotonic()
    reference: tuple[float, float, float] | None = None
    try:
        with ImuReader(args.port, args.baudrate) as reader:
            while time.monotonic() - started < args.seconds:
                samples = reader.poll()
                if not samples:
                    time.sleep(0.005)
                    continue
                for s in samples:
                    t = s.monotonic - started
                    a, g = s.angle, s.gyro
                    if reference is None:
                        reference = (a.roll, a.pitch, a.yaw)
                    rows.append((t, a.roll, a.pitch, a.yaw, g.x, g.y, g.z))
                    if handle:
                        handle.write(
                            f"{t:.3f},{a.roll:.4f},{a.pitch:.4f},{a.yaw:.4f},"
                            f"{g.x:.4f},{g.y:.4f},{g.z:.4f}\n"
                        )
                if not args.quiet:
                    last = rows[-1]
                    assert reference is not None
                    sys.stdout.write(
                        f"\r t={last[0]:6.1f}s  roll {last[1]:+8.3f}  "
                        f"pitch {last[2]:+8.3f}  yaw {last[3]:+8.3f}  "
                        f"(dyaw {wrap_deg(last[3] - reference[2]):+7.3f})   "
                    )
                    sys.stdout.flush()
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if handle:
            handle.close()

    print()
    if not rows:
        print("no samples. Wrong port, wrong baud rate, or the sensor is not "
              "sending acceleration + angular velocity + angle.")
        return 1

    elapsed = rows[-1][0]
    print(f"{len(rows)} samples over {elapsed:.1f} s -> {len(rows) / max(elapsed, 1e-9):.1f} Hz")
    assert reference is not None
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
        print(f"gz mean {statistics.mean(gz):+.4f} deg/s, sd {statistics.pstdev(gz):.4f}")
    return 0



if __name__ == "__main__":
    raise SystemExit(main())
