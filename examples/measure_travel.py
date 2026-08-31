#!/usr/bin/env python3
"""Measure how far the machine really goes per encoder count, and how much it slips.

The autonomous approach drives blind under the car for a set distance, measured
on the wheel counters. Two numbers decide whether that can work:

  counts_per_rev  how many counts one motor revolution produces. The controllers
                  report 4 poles, so 3 x 4 = 12 is expected — but that is the
                  datasheet, and this measures it.
  slip            how much less ground the machine covers than the wheels
                  claim. Mecanum rollers slip by design, and this is the error
                  that decides where a hole gets drilled.

Two passes:

  push (default)  torque off and move the machine by hand. Free-rolling wheels
                  do not slip, so this calibrates counts_per_rev honestly.
                  Nothing is driven. Two ways to say how far:
                    --turns N     mark one tyre and turn it N times. Better: it
                                  needs no tape measure and does not care
                                  whether the configured wheel_radius is right.
                    --wheel W     with --turns, read that one wheel only. Prop
                                  the machine up and spin the wheel by hand:
                                  ten turns is four metres of floor otherwise,
                                  and a wheel in the air cannot slip at all.
                    --distance M  push a measured M metres instead.
  drive           drive forward until the counters say the target distance, then
                  stop. Measure the real distance with a tape: the difference is
                  slip. THE MACHINE MOVES.

    python3 examples/measure_travel.py --turns 10 --wheel front_left
    python3 examples/measure_travel.py --turns 10 --circumference 0.393
    python3 examples/measure_travel.py --distance 1.0
    python3 examples/measure_travel.py --distance 1.0 --drive --rpm 60
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mdrobot"))

from mdrobot import DualMotorDriver  # noqa: E402
from mdrobot.exceptions import MdrobotError  # noqa: E402
from mdrobot.protocol import ModbusClient  # noqa: E402
from mdrobot.transport import SerialTransport, resolve_port  # noqa: E402

# Wheel order and where each one hangs off the bus, from mecanum.yaml.
WHEELS = (
    ("front_left", 1, 1, 1),
    ("front_right", 1, 2, -1),
    ("rear_left", 2, 2, 1),
    ("rear_right", 2, 1, -1),
)
PORT = "/dev/ttyUSB0"


def read_positions(drivers) -> dict[str, int]:
    mon = {slave: drivers[slave].read_monitor() for slave in drivers}
    out = {}
    for name, slave, channel, _sign in WHEELS:
        m = mon[slave]
        out[name] = (m.motor1 if channel == 1 else m.motor2).position
    return out


def signed_mean(start: dict[str, int], end: dict[str, int]) -> tuple[float, dict[str, int]]:
    """Mean forward travel in counts, with each wheel's own delta."""
    deltas = {name: (end[name] - start[name]) * sign
              for name, _s, _c, sign in WHEELS}
    return sum(deltas.values()) / len(deltas), deltas


def spread(deltas: dict[str, int], mean: float) -> None:
    print("\nper wheel (sign-corrected so forward is positive):")
    for name, d in deltas.items():
        off = "" if mean == 0 else f"   {(d - mean) / abs(mean) * 100:+.1f}% vs mean"
        print(f"  {name:12s} {d:+7d} counts{off}")


def report_turns(deltas, mean: float, turns: float, gear_ratio: float,
                 circumference: float | None) -> None:
    """Counts per wheel revolution, straight from a marked tyre."""
    spread(deltas, mean)
    print(f"\n  mean          {mean:+7.1f} counts over {turns:g} wheel turns")
    if mean == 0:
        print("  nothing moved — push further")
        return
    per_wheel_rev = mean / turns
    cpr = per_wheel_rev / gear_ratio
    print(f"  counts / wheel revolution      = {per_wheel_rev:.1f}")
    print(f"\n  => counts_per_rev (motor shaft) = {cpr:.2f}")
    print(f"     nearest whole value            = {round(cpr)}")
    print("     (this one needs no tape measure and does not use wheel_radius)")
    if circumference:
        print(f"\n  with the measured circumference {circumference:.3f} m:")
        print(f"     resolution      = {circumference / per_wheel_rev * 1000:.2f} mm/count")
        print(f"     implied radius  = {circumference / (2 * math.pi):.4f} m")


def report(deltas: dict[str, int], mean: float, distance: float,
           gear_ratio: float, wheel_radius: float) -> None:
    spread(deltas, mean)
    print(f"\n  mean          {mean:+7.1f} counts over {distance:.3f} m")
    if mean == 0:
        print("  nothing moved — check the drive, or push further")
        return
    per_m = mean / distance
    wheel_revs = distance / (2 * math.pi * wheel_radius)
    motor_revs = wheel_revs * gear_ratio
    cpr = mean / motor_revs
    print(f"  counts/metre  {per_m:.1f}")
    print(f"  wheel turns   {wheel_revs:.3f}   motor turns {motor_revs:.1f}")
    print(f"\n  => counts_per_rev (motor shaft) = {cpr:.2f}")
    print(f"     nearest whole value            = {round(cpr)}")
    print(f"     resolution                     = {distance / mean * 1000:.2f} mm/count")
    print(f"     (assumes wheel_radius {wheel_radius} m — use --turns to avoid that)")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--distance", type=float, default=1.0,
                    help="metres to travel (default 1.0)")
    ap.add_argument("--turns", type=float, default=None,
                    help="push pass: revolutions of a marked tyre instead of metres")
    ap.add_argument("--wheel", choices=[w[0] for w in WHEELS], default=None,
                    help="measure this one wheel only — for spinning it by hand with "
                         "the machine up on blocks, instead of pushing it along")
    ap.add_argument("--circumference", type=float, default=None,
                    help="measured tyre circumference in metres; with --turns this "
                         "gives metres/count without trusting wheel_radius")
    ap.add_argument("--wheel-radius", type=float, default=0.0625)
    ap.add_argument("--gear-ratio", type=float, default=20.0)
    ap.add_argument("--drive", action="store_true",
                    help="drive the distance instead of pushing it. MOVES THE MACHINE")
    ap.add_argument("--rpm", type=int, default=60, help="motor rpm for --drive")
    ap.add_argument("--counts-per-rev", type=float, default=12.0,
                    help="assumed value, used only to know when to stop in --drive")
    ap.add_argument("--timeout", type=float, default=30.0,
                    help="give up on --drive after this many seconds")
    args = ap.parse_args()

    if args.wheel and not args.turns:
        ap.error("--wheel needs --turns: a single wheel says nothing about metres")
    if args.wheel and args.drive:
        ap.error("--wheel is for turning a wheel by hand, not for --drive")

    transport = SerialTransport(resolve_port(args.port), args.baud, timeout=0.3)
    drivers = {slave: DualMotorDriver(ModbusClient(transport, slave_id=slave))
               for slave in sorted({w[1] for w in WHEELS})}
    try:
        if args.drive:
            return run_driven(args, drivers)
        return run_pushed(args, drivers)
    finally:
        for d in drivers.values():
            for step in (d.stop, d.torque_off_both, d.disable):
                try:
                    step()
                except MdrobotError:
                    pass
        transport.close()


def run_pushed(args, drivers) -> int:
    print("PUSH pass — the wheels are freed and nothing is driven.")
    for d in drivers.values():
        d.torque_off_both()

    if args.turns and args.wheel:
        print(f"Prop the machine up so the {args.wheel} wheel spins free. Put a mark "
              f"on that tyre and a reference beside it, then turn it by hand exactly "
              f"{args.turns:g} times.")
        print("More turns is better: it divides the error of spotting the mark.")
        print("Only that wheel is read, so the others can stay still.")
    elif args.turns:
        print(f"Put a mark on one tyre and on the floor beside it. Push the machine "
              f"straight until that mark has come back round exactly "
              f"{args.turns:g} times.")
        print("More turns is better: it divides the error of spotting the mark.")
    else:
        print(f"Mark a start line and a line exactly {args.distance:.3f} m ahead.")

    input("Line the mark up and press Enter... ")
    start = read_positions(drivers)
    if args.turns and args.wheel:
        input(f"Now turn the {args.wheel} wheel {args.turns:g} times by hand, "
              f"then press Enter... ")
    elif args.turns:
        input(f"Now push until the mark has come round {args.turns:g} times, "
              f"then press Enter... ")
    else:
        input(f"Now push it straight to the {args.distance:.3f} m mark "
              f"and press Enter... ")
    end = read_positions(drivers)
    mean, deltas = signed_mean(start, end)

    if args.wheel:
        # Only one wheel moved, so averaging over four would divide the answer.
        deltas = {args.wheel: deltas[args.wheel]}
        mean = float(deltas[args.wheel])

    if args.turns:
        report_turns(deltas, mean, args.turns, args.gear_ratio, args.circumference)
    else:
        report(deltas, mean, args.distance, args.gear_ratio, args.wheel_radius)
    print("\nFree-rolling wheels barely slip, so this is the honest counts_per_rev.")
    print("Put it in mecanum.yaml, then run again with --drive to measure slip.")
    return 0


def run_driven(args, drivers) -> int:
    circumference = 2 * math.pi * args.wheel_radius
    target = args.distance / circumference * args.gear_ratio * args.counts_per_rev
    print("DRIVE pass — THE MACHINE WILL MOVE FORWARD.")
    print(f"  {args.distance:.3f} m at counts_per_rev={args.counts_per_rev:g} "
          f"is {target:.0f} counts, at {args.rpm} rpm")
    print("  Clear the floor. Keep the e-stop within reach.")
    if input("  Type 'go' to start: ").strip().lower() != "go":
        print("  cancelled")
        return 1

    for d in drivers.values():
        d.enable()
    start = read_positions(drivers)
    started = time.monotonic()
    mean = 0.0
    try:
        while True:
            # Same rpm on every wheel: straight ahead.
            for slave, d in drivers.items():
                pair = [0, 0]
                for _name, s, channel, sign in WHEELS:
                    if s == slave:
                        pair[channel - 1] = args.rpm * sign
                d.set_velocities(pair[0], pair[1])
            time.sleep(0.1)
            mean, deltas = signed_mean(start, read_positions(drivers))
            print(f"\r  {mean:7.0f} / {target:.0f} counts", end="", flush=True)
            if mean >= target:
                print("\n  target reached")
                break
            if time.monotonic() - started > args.timeout:
                print(f"\n  gave up after {args.timeout:.0f} s")
                break
    finally:
        for d in drivers.values():
            try:
                d.stop()
            except MdrobotError:
                pass

    time.sleep(0.5)  # let it coast to a stop before the final read
    mean, deltas = signed_mean(start, read_positions(drivers))
    report(deltas, mean, args.distance, args.gear_ratio, args.wheel_radius)
    print("\nNow measure the REAL distance travelled with a tape.")
    print("  slip = (commanded - measured) / commanded")
    print("  That number, not the encoder resolution, is what limits the blind entry.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
