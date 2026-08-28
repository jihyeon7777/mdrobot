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
import threading
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

# Downlink: what the board accepts back from ROS, same CSV + checksum shape.
COMMAND_NAMES = (
    "lift",      # c0  up / down motor
    "brake",     # c1
    "drill",     # c2  drill motor
    "actuator",  # c3
    "solenoid",  # c4  solenoid valve
)
NUM_COMMANDS = len(COMMAND_NAMES)

# What each output accepts, as (min, max). The board is the authority here; a
# value outside its range is a bug in the layer above, so the node clamps rather
# than forwarding it to a drill or a valve.
#   lift      signed speed, confirmed -60..+60
#   brake     confirmed on/off
#   drill     confirmed on/off
#   actuator  UNCONFIRMED. -1/0/+1 is the usual retract/stop/extend shape for a
#             linear actuator, but the firmware was never checked -- verify it
#             before trusting this range.
#   solenoid  confirmed on/off
COMMAND_LIMITS = {
    "lift": (-60, 60),
    "brake": (0, 1),
    "drill": (0, 1),
    "actuator": (-1, 1),
    "solenoid": (0, 1),
}
COMMAND_MIN = tuple(COMMAND_LIMITS[n][0] for n in COMMAND_NAMES)
COMMAND_MAX = tuple(COMMAND_LIMITS[n][1] for n in COMMAND_NAMES)

# Uplink mode channel (f4): the operator's three-position switch.
MODE_BASE = "base"
MODE_MECANUM = "mecanum"
MODE_AUTONOMOUS = "autonomous"
MODE_NAMES = (MODE_BASE, MODE_MECANUM, MODE_AUTONOMOUS)

# The board's firmware emits CRLF; which terminator its *parser* expects was
# never established, so it stays configurable. Try "\n" or "\r" if the board
# ignores commands sent with the default.
DEFAULT_TERMINATOR = "\r\n"


def encode_command(values, terminator: str = DEFAULT_TERMINATOR) -> bytes:
    """Build one downlink line: five values plus their sum as the checksum.

    Mirrors the uplink rule (checksum == sum of the preceding fields), which is
    what the board firmware checks.
    """
    ints = [int(v) for v in values]
    if len(ints) != NUM_COMMANDS:
        raise ValueError(
            f"expected {NUM_COMMANDS} values {COMMAND_NAMES}, got {len(ints)}"
        )
    fields = ints + [sum(ints)]
    return (",".join(str(v) for v in fields) + terminator).encode("ascii")


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
    sent: int = 0
    send_errors: int = 0
    drop_uptimes: list[float] = field(default_factory=list)

    def summary(self) -> str:
        return (
            f"frames={self.frames} bad_checksum={self.bad_checksum} "
            f"unparsable={self.unparsable} wrong_length={self.wrong_length} "
            f"opens={self.opens} reconnects={self.reconnects} "
            f"sent={self.sent} send_errors={self.send_errors}"
        )


class RcBridgeReader:
    """Framed, checksum-checked link to the board, surviving a re-enumeration.

    Bidirectional over the one CDC-ACM port: :meth:`frames` reads the operator's
    channels, :meth:`send` writes commands back. The two normally run on
    different threads; ``_io_lock`` serialises opening, closing and writing so a
    reconnect cannot swap the handle out from under a write.

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
        terminator: str = DEFAULT_TERMINATOR,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.reconnect = reconnect
        self.reconnect_delay = reconnect_delay
        self.stats = RcStats()
        self.terminator = terminator
        self._io_lock = threading.RLock()
        self._serial: serial.Serial | None = None
        self._buf = b""
        self._opened_at = 0.0

    # ── connection ──────────────────────────────────────────────────────────
    def _resolve_port(self) -> str:
        # Re-resolve every time: after a re-enumeration the board can land on a
        # different ttyACM number, but the by-id path follows it.
        return self.port if self.port else find_port()

    def open(self) -> None:
        with self._io_lock:
            if self._serial is not None:
                return
            path = self._resolve_port()
            self._serial = serial.Serial(path, self.baudrate, timeout=self.timeout)
            self._opened_at = time.monotonic()
            self._buf = b""
            self.stats.opens += 1
            # Drop whatever accumulated while we were not listening so framing
            # starts on a line boundary.
            self._serial.reset_input_buffer()

    def close(self) -> None:
        with self._io_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._buf = b""

    # ── downlink ────────────────────────────────────────────────────────────
    def send(self, values) -> bool:
        """Write one command line to the board. True if it went out.

        Never raises on a dead port: a command that cannot be delivered is
        counted in ``stats.send_errors`` and the port is dropped so the read
        loop reopens it. Callers resend on their own schedule, so losing one
        line costs a few milliseconds rather than an exception.
        """
        payload = encode_command(values, self.terminator)
        with self._io_lock:
            if self._serial is None:
                self.stats.send_errors += 1
                return False
            try:
                self._serial.write(payload)
                self.stats.sent += 1
                return True
            except (serial.SerialException, OSError):
                self.stats.send_errors += 1
                self.close()
                return False

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
            # Snapshot the handle under the lock but block on read() outside it:
            # holding the lock across a read would stall send() for a whole
            # timeout, which is longer than the command period. pyserial
            # tolerates one reader and one writer; the race worth guarding is
            # close() swapping the handle, and the snapshot covers that.
            with self._io_lock:
                port = self._serial
            if port is None:
                continue  # a failed send closed it; reopen on the next pass
            try:
                chunk = port.read(256)
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
