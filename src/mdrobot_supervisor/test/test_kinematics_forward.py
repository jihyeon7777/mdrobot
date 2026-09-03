"""Forward kinematics: it must undo `inverse` exactly, and average slip away."""

import math

import pytest

from mdrobot_supervisor.kinematics import (
    MecanumGeometry,
    forward,
    inverse,
)

GEOM = MecanumGeometry(wheel_radius=0.0625, track=0.575, wheelbase=0.5)


@pytest.mark.parametrize("twist", [
    (0.0, 0.0, 0.0),
    (0.30, 0.0, 0.0),      # straight
    (0.0, 0.19, 0.0),      # strafe
    (0.0, 0.0, 0.37),      # spin
    (0.12, -0.08, 0.15),   # all three at once
    (-0.20, 0.05, -0.30),  # and backwards
])
def test_forward_undoes_inverse(twist):
    assert forward(inverse(*twist, GEOM), GEOM) == pytest.approx(twist, abs=1e-12)


@pytest.mark.parametrize("layout", ["x", "o", "unknown"])
def test_the_round_trip_holds_for_every_roller_layout(layout):
    # roller_layout flips the vy column only, and it has to flip back.
    geom = MecanumGeometry(0.0625, 0.575, 0.5, roller_layout=layout)
    twist = (0.1, 0.2, 0.3)
    assert forward(inverse(*twist, geom), geom) == pytest.approx(twist, abs=1e-12)


def test_an_o_layout_reads_the_strafe_the_other_way():
    x = MecanumGeometry(0.0625, 0.575, 0.5, roller_layout="x")
    o = MecanumGeometry(0.0625, 0.575, 0.5, roller_layout="o")
    omegas = inverse(0.0, 0.2, 0.0, x)
    assert forward(omegas, x)[1] == pytest.approx(0.2)
    assert forward(omegas, o)[1] == pytest.approx(-0.2)


def test_all_four_wheels_forward_is_pure_translation():
    omega = 1.0
    vx, vy, wz = forward((omega, omega, omega, omega), GEOM)
    assert vx == pytest.approx(omega * GEOM.wheel_radius)
    assert (vy, wz) == pytest.approx((0.0, 0.0))


def test_right_wheels_faster_than_left_is_a_left_turn():
    # WZ_COLUMN is (-1, +1, -1, +1): left wheels negative, right positive, so
    # driving the right pair harder must give +wz, counter-clockwise.
    _, _, wz = forward((1.0, 2.0, 1.0, 2.0), GEOM)
    assert wz > 0.0


def test_disagreement_between_wheels_is_averaged_not_rejected():
    # Four equations, three unknowns: real wheels slip, so the readings
    # disagree and the fit is least-squares. One wheel reading high must move
    # the answer by a quarter of its error, not all of it and not none.
    clean = forward((1.0, 1.0, 1.0, 1.0), GEOM)
    slipped = forward((1.4, 1.0, 1.0, 1.0), GEOM)
    assert slipped[0] - clean[0] == pytest.approx(0.4 * GEOM.wheel_radius / 4.0)


def test_a_pure_slip_pattern_reads_as_no_motion_at_all():
    # Four wheels, three unknowns, so one pattern is invisible: the null space.
    # It is (+1, +1, -1, -1) -- the front pair driving forward against the rear
    # pair driving back -- which is orthogonal to all three columns and moves
    # the body nowhere. The wheels just scrub.
    #
    # The fit correctly reports no motion, but it cannot report the scrubbing
    # either: the residual that would reveal it is discarded. That is one more
    # reason heading comes from the IMU, which measures the machine rather than
    # inferring it from wheels that may be lying.
    assert forward((1.0, 1.0, -1.0, -1.0), GEOM) == pytest.approx((0.0, 0.0, 0.0))


def test_the_strafe_pattern_is_not_the_invisible_one():
    # (+1, -1, -1, +1) looks similar and is the opposite case: it is the vy
    # column itself, so it reads as a pure strafe at full scale.
    vx, vy, wz = forward((1.0, -1.0, -1.0, 1.0), GEOM)
    assert (vx, wz) == pytest.approx((0.0, 0.0))
    assert vy == pytest.approx(-GEOM.wheel_radius)


@pytest.mark.parametrize("bad", [(1.0, 1.0, 1.0), (1.0,) * 5])
def test_the_wrong_number_of_wheels_is_rejected(bad):
    with pytest.raises(ValueError, match="4 wheel speeds"):
        forward(bad, GEOM)


def test_a_non_finite_reading_is_rejected():
    with pytest.raises(ValueError, match="finite"):
        forward((1.0, float("nan"), 1.0, 1.0), GEOM)
