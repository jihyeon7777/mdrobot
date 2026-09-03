"""The sign-sensitive helpers in the odometry node.

The integration itself is exercised on the machine — the TF tree was checked
against the measured geometry — but these two decide which way the model turns,
and a sign here is the kind of thing that looks plausible and is backwards.
"""

import math

import pytest

from mdrobot_supervisor.odometry_node import wrap, yaw_from_quaternion


class Q:
    """Enough of geometry_msgs/Quaternion to test against."""

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x, self.y, self.z, self.w = x, y, z, w


def about_z(degrees: float) -> Q:
    half = math.radians(degrees) / 2.0
    return Q(z=math.sin(half), w=math.cos(half))


@pytest.mark.parametrize("degrees", [0.0, 30.0, 90.0, 179.0, -90.0, -179.0])
def test_yaw_comes_back_out_of_the_quaternion(degrees):
    assert math.degrees(yaw_from_quaternion(about_z(degrees))) == pytest.approx(degrees)


def test_a_positive_yaw_is_counter_clockwise():
    # REP-103: +z up, +yaw anticlockwise seen from above. The model turning the
    # wrong way is the whole failure this guards.
    assert yaw_from_quaternion(about_z(45.0)) > 0.0


def test_roll_and_pitch_do_not_leak_into_the_heading():
    # The machine tips as it drives over things; that must not read as a turn.
    half = math.radians(20.0) / 2.0
    rolled = Q(x=math.sin(half), w=math.cos(half))
    assert yaw_from_quaternion(rolled) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("angle, folded", [
    (0.0, 0.0),
    (math.pi / 2, math.pi / 2),
    (math.pi, -math.pi),
    (3 * math.pi, -math.pi),
    (-3.5 * math.pi, 0.5 * math.pi),
])
def test_wrap_folds_into_a_half_open_turn(angle, folded):
    assert wrap(angle) == pytest.approx(folded)


def test_wrap_makes_the_seam_crossing_a_small_step():
    # Driving through the seam must give a small increment, not a whole turn:
    # the odometry rotates each increment by the midpoint heading, and a 2 pi
    # error there throws the position across the map.
    assert wrap(math.radians(-179.0) - math.radians(179.0)) == pytest.approx(
        math.radians(2.0))
