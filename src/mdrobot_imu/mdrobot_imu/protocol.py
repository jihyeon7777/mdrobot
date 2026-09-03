"""WITMOTION HWT901B serial protocol — framing and decoding, no I/O.

The sensor free-runs: it pushes fixed 11-byte packets down the line at its
configured output rate and never has to be asked. Every packet is

    0x55  kind  d0 d1 d2 d3 d4 d5 d6 d7  checksum

where the checksum is the low byte of the sum of the preceding ten. One
"sample" is several packets in a row — one per enabled content type — so a
reader has to collect a set before it has a full attitude.

Measured on the fitted sensor (2026-09-03, 9600 baud): kinds 0x51 accel,
0x52 gyro, 0x53 angle, 0x54 magnetic field and 0x56 barometer arrive back to
back, 10 sets per second. 5 packets x 11 bytes x 10 Hz = 550 B/s against the
960 B/s a 9600-baud line carries, so the link is already 57% full at the
default rate. Raising the output rate WITHOUT raising the baud rate does not
give more samples, it gives torn packets — see the README.

Everything here is pure: bytes in, numbers out. That is what makes the
resynchronisation logic testable, and resynchronisation is the part that
actually goes wrong on a noisy line.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

HEADER = 0x55
PACKET_LEN = 11

# Full-scale ranges the decoders divide by. These are fixed by the ranges the
# sensor is configured for; the HWT901B's defaults (and its only documented
# values for this model) are +/-16 g, +/-2000 deg/s and +/-180 deg.
ACCEL_FULL_SCALE_G = 16.0
GYRO_FULL_SCALE_DPS = 2000.0
ANGLE_FULL_SCALE_DEG = 180.0
INT16_FULL_SCALE = 32768.0

# Standard gravity, for turning the sensor's g into REP-103 m/s^2.
G_TO_M_S2 = 9.80665


class Kind(IntEnum):
    """Packet types. Only the ones this driver decodes are listed."""

    TIME = 0x50
    ACCEL = 0x51
    GYRO = 0x52
    ANGLE = 0x53
    MAG = 0x54
    PORT = 0x55
    PRESSURE = 0x56
    QUATERNION = 0x59


@dataclass(frozen=True)
class Accel:
    """Linear acceleration in m/s^2, plus the die temperature in Celsius."""

    x: float
    y: float
    z: float
    temperature: float


@dataclass(frozen=True)
class Gyro:
    """Angular rate in deg/s, plus the die temperature in Celsius.

    Left in degrees here because that is the sensor's own unit and the one
    every WITMOTION document quotes; the ROS node converts to rad/s once, at
    the boundary where REP-103 starts applying.
    """

    x: float
    y: float
    z: float
    temperature: float


@dataclass(frozen=True)
class Angle:
    """Fused attitude in degrees: roll about X, pitch about Y, yaw about Z.

    Sensor-frame and sensor-sign, NOT ROS: converting to REP-103 is the node's
    job, because it needs the mounting to be known and this layer does not know
    it. Range is +/-180 for roll and yaw, +/-90 for pitch.
    """

    roll: float
    pitch: float
    yaw: float
    version: int


@dataclass(frozen=True)
class Mag:
    """Magnetic field in the sensor's raw counts.

    Deliberately not scaled to teslas. The count-to-uT factor depends on the
    magnetometer's loop count setting, and the value is wanted here only as a
    health check — a large or lurching field means the 9-axis yaw is being
    pulled around, which is the whole reason this robot should run 6-axis.
    """

    x: int
    y: int
    z: int
    temperature: float


def checksum(packet: bytes) -> int:
    """The low byte of the sum of the first ten bytes."""
    return sum(packet[:PACKET_LEN - 1]) & 0xFF


def is_valid(packet: bytes) -> bool:
    """Is this 11 bytes a well-formed packet?

    A correct header and a correct checksum. That is a weak check — one byte in
    256 of random noise passes it — which is exactly why the framer below
    resynchronises one byte at a time rather than trusting the first 0x55 it
    finds.
    """
    return (
        len(packet) == PACKET_LEN
        and packet[0] == HEADER
        and checksum(packet) == packet[PACKET_LEN - 1]
    )


def _int16s(packet: bytes) -> tuple[int, int, int, int]:
    return struct.unpack("<hhhh", packet[2:10])


def decode(packet: bytes) -> Accel | Gyro | Angle | Mag | None:
    """Turn one validated packet into a reading, or None for kinds not decoded.

    Raises ValueError on a packet that does not pass :func:`is_valid`, so a
    caller cannot silently act on garbage.
    """
    if not is_valid(packet):
        raise ValueError(f"not a valid HWT901B packet: {packet.hex(' ')}")
    kind = packet[1]
    a, b, c, d = _int16s(packet)
    if kind == Kind.ACCEL:
        scale = ACCEL_FULL_SCALE_G / INT16_FULL_SCALE * G_TO_M_S2
        return Accel(a * scale, b * scale, c * scale, d / 100.0)
    if kind == Kind.GYRO:
        scale = GYRO_FULL_SCALE_DPS / INT16_FULL_SCALE
        return Gyro(a * scale, b * scale, c * scale, d / 100.0)
    if kind == Kind.ANGLE:
        scale = ANGLE_FULL_SCALE_DEG / INT16_FULL_SCALE
        # The fourth word is the firmware version, not a temperature.
        return Angle(a * scale, b * scale, c * scale, d)
    if kind == Kind.MAG:
        return Mag(a, b, c, d / 100.0)
    return None


class PacketFramer:
    """Turns an arbitrarily chopped byte stream into whole, valid packets.

    Feed it whatever a read() returned; it yields the packets it can complete
    and keeps the remainder for next time.

    Resynchronisation is one byte at a time. A framer that skips to the next
    0x55 after a bad checksum will happily skip over a good packet whose data
    happens to contain 0x55 — and 0x55 is a common data byte, not a rare one.
    Stepping one byte costs nothing at these rates and cannot skip a packet.

    ``dropped`` counts bytes discarded as unframeable, which is the number that
    tells you the baud rate is wrong or the line is too full.
    """

    def __init__(self, max_buffer: int = 4096) -> None:
        self._buf = bytearray()
        self._max_buffer = max_buffer
        self.dropped = 0

    def feed(self, data: bytes) -> list[bytes]:
        """Add received bytes, return every whole packet now available."""
        self._buf += data
        packets: list[bytes] = []
        while len(self._buf) >= PACKET_LEN:
            candidate = bytes(self._buf[:PACKET_LEN])
            if is_valid(candidate):
                packets.append(candidate)
                del self._buf[:PACKET_LEN]
            else:
                del self._buf[:1]
                self.dropped += 1
        # A buffer that only ever grows means nothing in it will ever frame.
        # Keep the tail: a real packet may be straddling the cut.
        if len(self._buf) > self._max_buffer:
            over = len(self._buf) - PACKET_LEN
            del self._buf[:over]
            self.dropped += over
        return packets
