"""Serial reader for the STM32 RC telemetry bridge.

The bridge enumerates as a USB CDC-ACM port:
    /dev/serial/by-id/usb-EV_Safety_Robot_STM32_RC_Telemetry_Bridge_*-if00

Wire format (confirmed against 6044 captured frames):
    115200 8N1, CRLF-terminated ASCII lines at ~50 Hz
    f0,f1,f2,f3,f4,f5,f6,f7,f8,f9,f10
    f0..f9  ten signed integer channels
    f10     checksum == sum(f0..f9)

Channel map (from the transmitter layout):
    f0  steer       left / right
    f1  throttle    forward / back
    f2  lift        up / down
    f3  brake
    f4  mode        3-position switch
    f5  drill       drill motor
    f6  actuator
    f7  solenoid    solenoid valve
    f8  limit_up    upper limit switch
    f9  limit_down  lower limit switch

Idle line: 1500,1500,0,0,-1,0,0,0,0,0,2999 — the two stick axes rest at ~1500
(RC pulse width in microseconds, +/-1 jitter) and everything else sits at its
inactive value.

Prefer the by-id path over /dev/ttyACM0 — the FTDI motor-controller port shares
the same numbering space and the two can swap on reboot.

A plain-Python module: it imports nothing from ROS, so the same reader backs
both the ROS 2 node and examples/read_rc_bridge.py.
"""

from __future__ import annotations

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

# Field order as the bridge emits it. Index == position in the CSV line.
CHANNEL_NAMES = (
    "steer",       # f0  left / right
    "throttle",    # f1  forward / back
    "lift",        # f2  up / down
    "brake",       # f3
    "mode",        # f4  3-position switch
    "drill",       # f5  drill motor
    "actuator",    # f6
    "solenoid",    # f7  solenoid valve
    "limit_up",    # f8  upper limit switch
    "limit_down",  # f9  lower limit switch
)
assert len(CHANNEL_NAMES) == NUM_CHANNELS


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

    channels: tuple[int, ...]  # f0..f9, in CHANNEL_NAMES order
    timestamp: float  # time.monotonic() when the line was completed

    def __getattr__(self, name: str) -> int:
        # Let frame.throttle stand in for frame.channels[1]. Only reached for
        # attributes the dataclass itself does not define.
        try:
            return self.channels[CHANNEL_NAMES.index(name)]
        except ValueError:
            raise AttributeError(name) from None

    def as_dict(self) -> dict[str, int]:
        return dict(zip(CHANNEL_NAMES, self.channels))

    def __str__(self) -> str:
        return "  ".join(f"{v:>10d}" for v in self.channels)


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
