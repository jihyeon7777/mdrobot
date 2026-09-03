"""The sensor-to-REP-103 mapping — the part that can drive a loop backwards."""

import math

import pytest

from mdrobot_imu.frames import quaternion_from_rpy, to_ros_attitude, wrap_deg


@pytest.mark.parametrize(
    "angle, folded",
    [
        (0.0, 0.0),
        (179.9, 179.9),
        (181.0, -179.0),
        (-181.0, 179.0),
        # Half-open at the top: the seam sits at +180, which folds to -180.
        (180.0, -180.0),
        (540.0, -180.0),
        (-180.0, -180.0),
    ],
)
def test_wrap_deg_folds_into_a_half_open_turn(angle, folded):
    assert wrap_deg(angle) == pytest.approx(folded)


def test_wrap_deg_never_returns_the_open_end_of_the_range():
    # Only one of +180 / -180 may ever come out, or a threshold test can see the
    # same attitude as inside the limit one tick and outside it the next.
    for step in range(0, 3600):
        assert -180.0 <= wrap_deg(step / 10.0 - 180.0) < 180.0


def test_identity_rotation_is_the_identity_quaternion():
    assert quaternion_from_rpy(0.0, 0.0, 0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_a_quarter_turn_about_z_is_a_pure_z_quaternion():
    x, y, z, w = quaternion_from_rpy(0.0, 0.0, math.radians(90.0))
    assert (x, y) == pytest.approx((0.0, 0.0))
    assert (z, w) == pytest.approx((math.sqrt(0.5), math.sqrt(0.5)))


def test_the_quaternion_is_always_unit_length():
    for rpy in ((0.3, -0.2, 2.9), (-1.5, 0.9, -0.4), (3.1, 1.2, 0.0)):
        q = quaternion_from_rpy(*rpy)
        assert sum(c * c for c in q) == pytest.approx(1.0)


def test_a_negative_yaw_sign_reverses_the_sense_of_rotation():
    # Which sign a robot needs is a property of its mounting and is settled by
    # turning the machine, not by argument: this one was expected to need -1 on
    # the grounds that WITMOTION yaw grows clockwise, and measured +1.
    _, _, yaw = to_ros_attitude(0.0, 0.0, 30.0, (1.0, 1.0, -1.0))
    assert yaw == pytest.approx(-30.0)
    _, _, yaw = to_ros_attitude(0.0, 0.0, 30.0, (1.0, 1.0, 1.0))
    assert yaw == pytest.approx(30.0)


def test_roll_and_pitch_signs_are_applied_independently():
    roll, pitch, _ = to_ros_attitude(5.0, -7.0, 0.0, (-1.0, 1.0, -1.0))
    assert (roll, pitch) == pytest.approx((-5.0, -7.0))


def test_the_mount_offset_is_added_after_the_sign_and_then_wrapped():
    # A sensor bolted on facing 90 deg off. Order matters: flipping the sign
    # after adding the offset would rotate the mount the wrong way.
    _, _, yaw = to_ros_attitude(0.0, 0.0, 170.0, (1.0, 1.0, -1.0), yaw_offset_deg=-90.0)
    assert yaw == pytest.approx(100.0)


def test_a_mount_offset_that_crosses_the_seam_stays_in_range():
    _, _, yaw = to_ros_attitude(0.0, 0.0, -170.0, (1.0, 1.0, -1.0), yaw_offset_deg=90.0)
    assert -180.0 < yaw <= 180.0
    assert yaw == pytest.approx(-100.0)
