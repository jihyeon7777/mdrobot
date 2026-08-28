#!/usr/bin/env python3
"""Read the STM32 RC telemetry bridge over USB serial.

The bridge enumerates as a USB CDC-ACM port:
    /dev/serial/by-id/usb-EV_Safety_Robot_STM32_RC_Telemetry_Bridge_*-if00

Wire format (confirmed against 6044 captured frames):
    115200 8N1, CRLF-terminated ASCII lines at ~50 Hz
    f0,f1,f2,f3,f4,f5,f6,f7,f8,f9,f10
    f0..f9  ten signed integer channels
    f10     checksum == sum(f0..f9)

Idle line: 1500,1500,0,0,-1,0,0,0,0,0,2999
    f0, f1  sit at ~1500 with +/-1 jitter -> RC pulse width in microseconds
    f4      constant -1, most likely a link/failsafe status flag
    others  constant 0

The channel-to-control mapping is NOT established yet: in an 86 s capture no
field moved, so either the sticks were not touched or the RC link was down.
Run this script with --map, move every stick and switch, and the summary at
exit names the fields that actually responded.

Prefer the by-id path over /dev/ttyACM0 — the FTDI motor-controller port shares
the same numbering space and the two can swap on reboot.

Usage:
    python3 examples/read_rc_bridge.py                 # live view
    python3 examples/read_rc_bridge.py --map           # channel-mapping helper
    python3 examples/read_rc_bridge.py --raw           # echo the raw lines
    python3 examples/read_rc_bridge.py --port /dev/ttyACM0 --seconds 10
"""

from __future__ import annotations

import argparse
import glob
import os
import time
from dataclasses import dataclass, field
from typing import Iterator

import serial

# The bridge's USB product string. Anything matching this glob is our board.
PORT_GLOB = "/dev/serial/by-id/*STM32_RC_Telemetry_Bridge*"
BAUDRATE = 115200
NUM_CHANNELS = 10  # f0..f9; f10 is the checksum on top of these


class RcBridgeError(Exception):
    """No bridge port could be found."""


def find_port(glob_pattern: str = PORT_GLOB) -> str:
    """Return the tty path of the bridge, resolved through /dev/serial/by-id."""
    matches = sorted(glob.glob(glob_pattern))
    if not matches:
        raise RcBridgeError(
            f"no serial port matching {glob_pattern!r}. "
            "Check that the board is plugged in (lsusb should list 0483:5740)."
        )
    return os.path.realpath(matches[0])


@dataclass(frozen=True)
class RcFrame:
    """One decoded, checksum-verified line."""

    channels: tuple[int, ...]  # f0..f9
    timestamp: float  # time.monotonic() when the line was completed

    def __str__(self) -> str:
        return " ".join(f"{v:>6d}" for v in self.channels)


@dataclass
class RcStats:
    """Counters for one reader, useful for spotting a flaky link."""

    frames: int = 0
    bad_checksum: int = 0
    unparsable: int = 0
    wrong_length: int = 0
    reconnects: int = 0
    opens: int = 0
    drop_uptimes: list[float] = field(default_factory=list)

    def summary(self) -> str:
        return (
            f"frames={self.frames} bad_checksum={self.bad_checksum} "
            f"unparsable={self.unparsable} wrong_length={self.wrong_length} "
            f"opens={self.opens} reconnects={self.reconnects}"
        )


class RcBridgeReader:
    """Framed, checksum-checked reader that survives the board re-enumerating.

    The bridge has been observed dropping off the USB bus and coming back a few
    seconds later, which invalidates the open file descriptor. With
    ``reconnect=True`` (the default) the reader reopens the port by its by-id
    path instead of raising, so a drop shows up as a gap in frames plus a bump
    in ``stats.reconnects``.
    """

    def __init__(
        self,
        port: str | None = None,
        baudrate: int = BAUDRATE,
        timeout: float = 0.2,
        reconnect: bool = True,
        reconnect_delay: float = 0.3,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.reconnect = reconnect
        self.reconnect_delay = reconnect_delay
        self.stats = RcStats()
        self._serial: serial.Serial | None = None
        self._buf = b""
        self._opened_at = 0.0

    # ── connection ──────────────────────────────────────────────────────────
    def _resolve_port(self) -> str:
        # Re-resolve every time: after a re-enumeration the board can land on a
        # different ttyACM number, but the by-id path follows it.
        return self.port if self.port else find_port()

    def open(self) -> None:
        if self._serial is not None:
            return
        path = self._resolve_port()
        self._serial = serial.Serial(path, self.baudrate, timeout=self.timeout)
        self._opened_at = time.monotonic()
        self._buf = b""
        self.stats.opens += 1
        # Drop whatever accumulated while we were not listening, then discard the
        # first (probably partial) line so framing starts on a boundary.
        self._serial.reset_input_buffer()

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self._buf = b""

    def _handle_drop(self, exc: Exception) -> None:
        uptime = time.monotonic() - self._opened_at
        self.stats.drop_uptimes.append(round(uptime, 1))
        self.stats.reconnects += 1
        self.close()
        if not self.reconnect:
            raise exc
        time.sleep(self.reconnect_delay)

    def __enter__(self) -> "RcBridgeReader":
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # ── framing / decoding ──────────────────────────────────────────────────
    def _decode(self, line: bytes, now: float) -> RcFrame | None:
        text = line.decode("ascii", "replace").strip()
        if not text:
            return None
        try:
            fields = [int(x) for x in text.split(",")]
        except ValueError:
            self.stats.unparsable += 1
            return None
        if len(fields) != NUM_CHANNELS + 1:
            self.stats.wrong_length += 1
            return None
        *channels, checksum = fields
        if sum(channels) != checksum:
            self.stats.bad_checksum += 1
            return None
        self.stats.frames += 1
        return RcFrame(channels=tuple(channels), timestamp=now)

    def frames(self) -> Iterator[RcFrame]:
        """Yield verified frames forever (until the caller stops iterating).

        Malformed and bad-checksum lines are counted in ``stats`` and skipped
        rather than raised, so a single corrupted line never kills the stream.
        """
        while True:
            if self._serial is None:
                try:
                    self.open()
                except (serial.SerialException, OSError, RcBridgeError):
                    if not self.reconnect:
                        raise
                    self.stats.reconnects += 1
                    time.sleep(self.reconnect_delay)
                    continue
            try:
                chunk = self._serial.read(256)
            except (serial.SerialException, OSError) as exc:
                self._handle_drop(exc)
                continue
            if not chunk:
                continue
            self._buf += chunk
            now = time.monotonic()
            while b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                frame = self._decode(line, now)
                if frame is not None:
                    yield frame


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
            print(f"  f{i:<2d} min={lo[i]:<7d} max={hi[i]:<7d} span={span:<7d}{tag}")
        if moved:
            print(f"\nresponding channels: {', '.join('f%d' % i for i in moved)}")
        else:
            print("\nNo channel moved more than +/-5. Either nothing was touched, "
                  "or the RC link is down (f4 == -1 is the suspected link flag).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
