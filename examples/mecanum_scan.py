#!/usr/bin/env python3
"""Read-only bus scan for a mecanum base: two dual-channel controllers, one RS485 bus.

READ-ONLY BY DESIGN. This script issues Modbus function 0x03 (read) only — it never
writes a register and never turns a motor. Run it first, every session, before
anything that moves.

It reports, per controller: firmware version, supply voltage, active status bits,
and the four settings that decide whether serial drive will work at all:

  ENC_PPR(156)        0 = hall closed loop. Recent firmware ships in ENCODER mode
                      with no encoder wired, and then the first velocity command
                      lurches ~0.6 s and alarms. Nonzero here with no encoder
                      attached is the single most common first-drive failure.
  USE_LIMIT_SW(17)    CTRL limit-switch gating for motor 1. Usually must be 0 for
  USE_LIMIT_SW2(29)   motor 2. serial-only drive; observed to reset after a power cycle.
  MAX_RPM(221)        the controller's own speed cap — sanity-check your rpm limits
                      against it.

It also checks the USB adapter's latency_timer, which sets the real ceiling on the
control rate (an FTDI at the default 16 roughly doubles every round-trip) and is
lost on every reboot and replug.

Usage:
    python3 examples/mecanum_scan.py                     # ids 1 and 2
    python3 examples/mecanum_scan.py --ids 1 2 3
    python3 examples/mecanum_scan.py --scan              # probe ids 1..10
    python3 examples/mecanum_scan.py --port /dev/ttyUSB1

With --port omitted the MDROBOT_PORT environment variable is used
(export MDROBOT_PORT=/dev/ttyUSB0). See manual/setup/port-setup.md.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow running straight from the repo without installing mdrobot.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mdrobot"))

from mdrobot import (  # noqa: E402
    DualMotorDriver,
    MdrobotError,
    ModbusClient,
    SerialTransport,
    active_bits,
    registers as reg,
    resolve_port,
)
from mdrobot.status import STATUS1_BIT_NAMES  # noqa: E402

# Settings worth seeing before any drive attempt: (label, pid, interpreter).
SETTINGS = [
    ("ENC_PPR      (156)", reg.PID_ENC_PPR, lambda v: "0 = hall closed loop"
        if v == 0 else f"{v} PPR — encoder mode; MUST have an encoder wired"),
    ("USE_LIMIT_SW  (17)", reg.PID_USE_LIMIT_SW, lambda v: "0 = disabled (serial drive ok)"
        if v == 0 else f"{v} = CTRL pins gate motion — usually must be 0"),
    ("USE_LIMIT_SW2 (29)", reg.PID_USE_LIMIT_SW2, lambda v: "0 = disabled (serial drive ok)"
        if v == 0 else f"{v} = CTRL pins gate motion — usually must be 0"),
    ("MAX_RPM      (221)", reg.PID_MAX_RPM, lambda v: f"{v} rpm controller cap"),
    ("HALL_TYPE     (21)", reg.PID_HALL_TYPE, lambda v: f"code {v} (0:4p 1:8p 2:10p 3:12p 4:2p 5:6p)"),
]


def report_latency_timer(port: str) -> None:
    """Print the USB-serial adapter's latency_timer, which caps the achievable rate."""
    path = Path("/sys/bus/usb-serial/devices") / Path(port).name / "latency_timer"
    try:
        value = int(path.read_text().strip())
    except (OSError, ValueError):
        print(f"latency_timer : not exposed for {port} "
              "(fine for CH340-class adapters; FTDI exposes it)")
        return
    if value <= 1:
        print(f"latency_timer : {value} ms — good")
    else:
        print(f"latency_timer : {value} ms — SLOW. Every round-trip roughly doubles.")
        print(f"                fix: echo 1 | sudo tee {path}")
        print("                (lost on reboot and on replug — re-apply each session)")


def scan_one(client: ModbusClient) -> bool:
    """Print everything readable from one controller. Returns True if it answered."""
    driver = DualMotorDriver(client)
    try:
        version = driver.get_version()
    except MdrobotError as exc:
        print(f"  no response ({type(exc).__name__})")
        return False

    print(f"  version   : DL={version}  (~v{version // 10}.{version % 10})")
    for label, getter in (("voltage   ", lambda: f"{driver.get_voltage():.1f} V"),
                          ("status    ", lambda: ", ".join(
                              active_bits(driver.get_status().raw, STATUS1_BIT_NAMES)) or "none")):
        try:
            print(f"  {label}: {getter()}")
        except MdrobotError as exc:
            print(f"  {label}: unreadable ({type(exc).__name__})")

    for label, pid, describe in SETTINGS:
        try:
            value = client.read_register(pid)
        except MdrobotError as exc:
            print(f"  {label}: unreadable ({type(exc).__name__})")
            continue
        print(f"  {label}: {describe(value)}")

    # Both channels in one transaction — also confirms this really is a dual controller.
    try:
        mon = driver.read_monitor()
    except MdrobotError as exc:
        print(f"  monitor           : unreadable ({type(exc).__name__})")
    else:
        print(f"  monitor           : M1 spd={mon.motor1.speed_rpm:>5} rpm pos={mon.motor1.position:>9}"
              f" | M2 spd={mon.motor2.speed_rpm:>5} rpm pos={mon.motor2.position:>9}")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Read-only bus scan for a two-controller mecanum base (never writes, never moves).")
    ap.add_argument("--port", default=None,
                    help="serial port; default $MDROBOT_PORT")
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--timeout", type=float, default=0.3)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2],
                    help="Modbus slave ids to query (default: 1 2)")
    ap.add_argument("--scan", action="store_true",
                    help="probe ids 1..10 instead of --ids")
    args = ap.parse_args()

    try:
        port = resolve_port(args.port)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    ids = list(range(1, 11)) if args.scan else args.ids

    print(f"port          : {port} @ {args.baud} 8N1, timeout {args.timeout}s")
    report_latency_timer(port)
    print(f"querying ids  : {', '.join(str(i) for i in ids)}")
    print("mode          : READ-ONLY (function 0x03 only — no writes, no motion)")

    # One transport shared by every client: the transport owns the Modbus t3.5
    # inter-frame gap for the whole bus, which is what makes several slave ids on
    # one port safe. Never give each controller its own transport.
    transport = SerialTransport(port, args.baud, timeout=args.timeout)
    found = []
    try:
        for slave_id in ids:
            print(f"\n--- slave id {slave_id} ---")
            if scan_one(ModbusClient(transport, slave_id=slave_id)):
                found.append(slave_id)
    finally:
        transport.close()

    print(f"\nresponded: {found or 'nothing'}")
    if len(found) < 2:
        print("A mecanum base needs TWO controllers at DISTINCT slave ids.")
        print("If only one answered, re-ID the other: write PID_ID(133) = (new_id << 8) | 0xAA")
        print("with ONLY that unit on the bus, then power-cycle. See manual/ros2_control.md.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
