"""Sample assembly: three packet types have to arrive before one sample does."""

from mdrobot_imu.reader import SampleAssembler


def test_a_sample_needs_acceleration_rate_and_angle(build):
    assembler = SampleAssembler()
    # An angle on its own is not a sample: pairing it with nothing, or with a
    # stale reading from before the driver started, would publish a lie.
    assert assembler.push(build(0x53, 0, 0, 0, 0), now=1.0) is None
    assert assembler.push(build(0x51, 0, 0, 2048, 0), now=1.0) is None
    assert assembler.push(build(0x52, 0, 0, 0, 0), now=1.0) is None
    sample = assembler.push(build(0x53, 1024, 0, 0, 0), now=1.5)
    assert sample is not None
    assert sample.monotonic == 1.5
    assert sample.angle.roll == 5.625
    assert sample.mag is None


def test_the_angle_packet_closes_the_set(build):
    # The sensor sends its content types back to back, so emitting on the angle
    # packet keeps the three readings of one burst together rather than pairing
    # an angle with the previous burst's rate.
    assembler = SampleAssembler()
    for packet in (build(0x51, 0, 0, 2048, 0), build(0x52, 0, 0, 0, 0)):
        assert assembler.push(packet, now=0.0) is None
    assert assembler.push(build(0x53, 0, 0, 0, 0), now=0.0) is not None


def test_magnetic_field_is_carried_when_it_is_enabled(build):
    assembler = SampleAssembler()
    for packet in (
        build(0x51, 0, 0, 2048, 0),
        build(0x52, 0, 0, 0, 0),
        build(0x54, 1399, 1844, -5012, 0),
    ):
        assembler.push(packet, now=0.0)
    sample = assembler.push(build(0x53, 0, 0, 0, 0), now=0.0)
    assert sample is not None and sample.mag is not None
    assert sample.mag.z == -5012
