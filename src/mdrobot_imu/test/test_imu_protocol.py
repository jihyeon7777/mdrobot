"""Framing and decoding, against bytes captured off the fitted sensor."""

import pytest

from mdrobot_imu.protocol import (
    G_TO_M_S2,
    Accel,
    Angle,
    Gyro,
    Mag,
    PacketFramer,
    checksum,
    decode,
    is_valid,
)

# Captured from the HWT901B on /dev/ttyUSB1, 9600 baud, 2026-09-03: an
# acceleration packet followed by an angular velocity packet.
REAL_STREAM = bytes.fromhex("5551 0000 3d00 2108 070a 1d 5552 0000 0000 0000 070a b8".replace(" ", ""))



def test_checksum_is_the_low_byte_of_the_sum(build):
    packet = build(0x51, 1, 2, 3, 4)
    assert checksum(packet) == packet[-1]
    assert is_valid(packet)


def test_a_corrupt_checksum_is_rejected(build):
    packet = bytearray(build(0x51, 1, 2, 3, 4))
    packet[-1] ^= 0xFF
    assert not is_valid(bytes(packet))
    with pytest.raises(ValueError):
        decode(bytes(packet))


def test_a_wrong_header_is_rejected(build):
    packet = bytearray(build(0x51, 1, 2, 3, 4))
    packet[0] = 0x56
    assert not is_valid(bytes(packet))


def test_decodes_the_real_acceleration_packet():
    accel = decode(REAL_STREAM[:11])
    assert isinstance(accel, Accel)
    # Sitting still and level: about 1 g on Z, in m/s^2, and 25.67 C.
    assert accel.z == pytest.approx(1.0163 * G_TO_M_S2, rel=1e-3)
    assert accel.x == pytest.approx(0.0, abs=1e-6)
    assert accel.temperature == pytest.approx(25.67)


def test_decodes_the_real_gyro_packet_as_the_clamped_zero_it_was():
    gyro = decode(REAL_STREAM[11:22])
    assert isinstance(gyro, Gyro)
    # Exactly zero on every axis — the sensor's automatic zero-bias calibration
    # clamping small rates. See mdrobot_imu.survey for why that matters here.
    assert (gyro.x, gyro.y, gyro.z) == (0.0, 0.0, 0.0)


def test_angle_uses_180_degrees_full_scale(build):
    angle = decode(build(0x53, 16384, -8192, 32767, 0))
    assert isinstance(angle, Angle)
    assert angle.roll == pytest.approx(90.0)
    assert angle.pitch == pytest.approx(-45.0)
    assert angle.yaw == pytest.approx(180.0, rel=1e-4)


def test_gyro_uses_2000_dps_full_scale(build):
    gyro = decode(build(0x52, 16384, 0, 0, 0))
    assert gyro.x == pytest.approx(1000.0)


def test_magnetic_field_stays_in_raw_counts(build):
    mag = decode(build(0x54, 1399, 1844, -5012, 0))
    assert isinstance(mag, Mag)
    # Not scaled: the count-to-uT factor depends on the loop count setting, and
    # the value is wanted as a health check, not a measurement.
    assert (mag.x, mag.y, mag.z) == (1399, 1844, -5012)


def test_undecoded_kinds_return_none_rather_than_raising(build):
    # 0x56 (barometer) is on the wire but not decoded; a driver that raised on
    # it would die on a sensor with more content types enabled than it needs.
    assert decode(build(0x56, 0, 0, 0, 0)) is None


def test_framer_reassembles_packets_split_across_reads():
    framer = PacketFramer()
    assert framer.feed(REAL_STREAM[:5]) == []
    assert framer.feed(REAL_STREAM[5:15]) == [REAL_STREAM[:11]]
    assert framer.feed(REAL_STREAM[15:]) == [REAL_STREAM[11:22]]
    assert framer.dropped == 0


def test_framer_resynchronises_after_leading_garbage():
    framer = PacketFramer()
    packets = framer.feed(b"\x00\x99\x55\xff" + REAL_STREAM[:11])
    assert packets == [REAL_STREAM[:11]]
    assert framer.dropped == 4


def test_framer_does_not_skip_a_packet_whose_data_contains_a_header_byte(build):
    # 0x55 is an ordinary data byte, so a framer that jumps to the next 0x55
    # after a bad checksum can step straight over a good packet. Stepping one
    # byte at a time cannot.
    good = build(0x53, 0x5555, 0x5555, 0x5555, 0)
    assert 0x55 in good[2:10]
    framer = PacketFramer()
    assert framer.feed(b"\xaa" + good) == [good]


def test_framer_does_not_grow_without_bound_on_pure_noise():
    framer = PacketFramer()
    for _ in range(100):
        framer.feed(b"\x00" * 256)
    assert framer.dropped > 20000
    assert len(framer._buf) <= 4096
