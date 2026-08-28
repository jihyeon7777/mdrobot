#!/usr/bin/env python3
"""Live view of the STM32 RC telemetry bridge.

A command-line front end for mdrobot_rc_bridge.rc_reader — handy for checking
the link and the transmitter without starting ROS.

    python3 examples/read_rc_bridge.py                 # live view
    python3 examples/read_rc_bridge.py --map           # per-channel range
    python3 examples/read_rc_bridge.py --raw           # echo every frame
    python3 examples/read_rc_bridge.py --port /dev/ttyACM0 --seconds 10
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# Run straight from a checkout, without sourcing the colcon workspace.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "mdrobot_rc_bridge"))

from mdrobot_rc_bridge.rc_reader import (  # noqa: E402
    BAUDRATE, CHANNEL_NAMES, NUM_CHANNELS, PORT_GLOB,
    RcBridgeError, RcBridgeReader, find_port,
)


# ── CLI ─────────────────────────────────────────────────────────────────────
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default=None,
                    help=f"serial port (default: auto-detect via {PORT_GLOB})")
    ap.add_argument("--baud", type=int, default=BAUDRATE)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop after N seconds (default: run until Ctrl-C)")
    ap.add_argument("--hz", type=float, default=10.0,
                    help="screen refresh rate; the port is still read at full speed")
    ap.add_argument("--raw", action="store_true", help="echo every verified frame")
    ap.add_argument("--map", dest="do_map", action="store_true",
                    help="track per-field min/max and report which fields moved")
    ap.add_argument("--no-reconnect", action="store_true",
                    help="exit on a serial error instead of reopening the port")
    args = ap.parse_args()

    if args.port and not os.path.exists(args.port):
        print(f"error: {args.port} does not exist. "
              f"Omit --port to auto-detect via {PORT_GLOB}.")
        return 1

    reader = RcBridgeReader(port=args.port, baudrate=args.baud,
                            reconnect=not args.no_reconnect)
    try:
        port_desc = args.port or find_port()
    except RcBridgeError as exc:
        print(f"error: {exc}")
        return 1
    print(f"port    : {port_desc}")
    print(f"baud    : {args.baud}  format: {NUM_CHANNELS} channels + checksum")
    if args.do_map:
        print("mapping : move every stick and switch through its full range, "
              "then Ctrl-C (or wait for --seconds)")
    print()
    if not args.raw:
        print("  ".join(f"{n:>10s}" for n in CHANNEL_NAMES))

    lo = [None] * NUM_CHANNELS
    hi = [None] * NUM_CHANNELS
    started = time.monotonic()
    last_draw = 0.0
    last_count = 0
    rate = 0.0

    try:
        for frame in reader.frames():
            if args.do_map:
                for i, v in enumerate(frame.channels):
                    lo[i] = v if lo[i] is None else min(lo[i], v)
                    hi[i] = v if hi[i] is None else max(hi[i], v)
            if args.raw:
                print(f"{frame.timestamp - started:8.2f}  {frame}")

            now = time.monotonic()
            if not args.raw and now - last_draw >= 1.0 / args.hz:
                elapsed = now - last_draw
                if last_draw:
                    rate = (reader.stats.frames - last_count) / elapsed
                last_draw, last_count = now, reader.stats.frames
                print(f"\r{frame}   {rate:5.1f} Hz  "
                      f"err={reader.stats.bad_checksum + reader.stats.unparsable}"
                      f" rc={reader.stats.reconnects}   ", end="", flush=True)

            if args.seconds and now - started >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()

    elapsed = time.monotonic() - started
    print("\n")
    print(f"elapsed : {elapsed:.1f} s")
    print(f"stats   : {reader.stats.summary()}")
    if reader.stats.drop_uptimes:
        print(f"drops   : uptime before each drop = {reader.stats.drop_uptimes} s")
    if reader.stats.frames:
        print(f"rate    : {reader.stats.frames / elapsed:.1f} Hz average")

    if args.do_map:
        print("\nper-channel range:")
        moved = []
        for i in range(NUM_CHANNELS):
            if lo[i] is None:
                continue
            span = hi[i] - lo[i]
            tag = "  <-- MOVED" if span > 5 else ""
            if span > 5:
                moved.append(i)
            print(f"  f{i:<2d} {CHANNEL_NAMES[i]:<11s} min={lo[i]:<7d} "
                  f"max={hi[i]:<7d} span={span:<7d}{tag}")
        if moved:
            names = ", ".join(f"f{i} {CHANNEL_NAMES[i]}" for i in moved)
            print(f"\nresponding channels: {names}")
        else:
            print("\nNo channel moved more than +/-5 — nothing was touched, "
                  "or the RC link is down.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
