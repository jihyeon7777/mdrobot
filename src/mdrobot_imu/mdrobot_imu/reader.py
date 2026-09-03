"""Serial transport for the HWT901B: bytes off the port, samples out.

Split from :mod:`mdrobot_imu.protocol` so the decoding can be tested without a
port, and from the node so it can be used from a plain script — which is how
the attitude survey in the README is run.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

from .protocol import Accel, Angle, Gyro, Mag, PacketFramer, decode

BAUDRATE = 9600  # the sensor's factory default, and what the fitted one uses


@dataclass(frozen=True)
class Sample:
    """One coherent set: the attitude, the rate and the acceleration together.

    ``monotonic`` is stamped when the set completed, not when the first packet
    of it arrived. At 10 Hz the set spans about 5 ms, which is far below
    anything this robot steers on.
    """

    monotonic: float
    angle: Angle
    gyro: Gyro
    accel: Accel
    mag: Mag | None = None


class SampleAssembler:
    """Collects packets until a full set is in hand, then emits it.

    The sensor sends its enabled content types back to back, so the angle
    packet — the last of the three this driver needs — marks the end of a set.
    Emitting on the angle packet keeps the three readings from the same burst
    together instead of pairing an angle with the previous burst's gyro.

    A set is only emitted when all three have been seen at least once, so a
    sensor configured with acceleration or angular velocity switched off never
    produces a half-filled sample; it produces none, and the node's watchdog
    says so.
    """

    def __init__(self) -> None:
        self._accel: Accel | None = None
        self._gyro: Gyro | None = None
        self._mag: Mag | None = None

    def push(self, packet: bytes, now: float | None = None) -> Sample | None:
        reading = decode(packet)
        if isinstance(reading, Accel):
            self._accel = reading
        elif isinstance(reading, Gyro):
            self._gyro = reading
        elif isinstance(reading, Mag):
            self._mag = reading
        elif isinstance(reading, Angle):
            if self._accel is None or self._gyro is None:
                return None
            sample = Sample(
                monotonic=time.monotonic() if now is None else now,
                angle=reading,
                gyro=self._gyro,
                accel=self._accel,
                mag=self._mag,
            )
            return sample
        return None


class ImuReader:
    """Reads the sensor over a serial port.

    Non-blocking by design: :meth:`poll` drains whatever has arrived and
    returns the completed samples, so a ROS timer can call it at its own rate
    without a thread. The port is opened lazily and reopened on failure, so a
    sensor that is unplugged and plugged back in recovers on its own rather
    than needing the node restarted — which matters on a machine that is
    reached over SSH with a drill under a car.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = BAUDRATE,
        timeout: float = 0.0,
    ) -> None:
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self._serial = None
        self._framer = PacketFramer()
        self._assembler = SampleAssembler()

    @property
    def dropped(self) -> int:
        """Bytes discarded as unframeable since start. Should stay near 0."""
        return self._framer.dropped

    def open(self) -> None:
        import serial  # imported here so the module imports without pyserial

        if self._serial is not None:
            return
        self._serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None

    def poll(self, max_bytes: int = 4096) -> list[Sample]:
        """Drain the port and return every sample that completed.

        Returns an empty list when nothing has arrived — including when the
        port is down. Errors are turned into a closed port rather than an
        exception so one bad read cannot take the node with it; the caller sees
        it as silence, which its watchdog already has to handle anyway.
        """
        if self._serial is None:
            self.open()
        assert self._serial is not None
        try:
            waiting = self._serial.in_waiting
            data = self._serial.read(min(waiting, max_bytes)) if waiting else b""
        except Exception:
            self.close()
            return []
        samples = []
        for packet in self._framer.feed(data):
            sample = self._assembler.push(packet)
            if sample is not None:
                samples.append(sample)
        return samples

    def __enter__(self) -> "ImuReader":
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()
