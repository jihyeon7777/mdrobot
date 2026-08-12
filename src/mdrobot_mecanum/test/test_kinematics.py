"""Unit tests for the mecanum kinematics. No hardware, no I/O."""

from __future__ import annotations

import math

import pytest

from mdrobot_mecanum.kinematics import (
    FL,
    FR,
    HANDEDNESS,
    PROVISIONAL_LAYOUT,
    RL,
    RR,
    WHEEL_NAMES,
    MecanumGeometry,
    forward,
    inverse,
    scale_to_limit,
    slip_residual,
)

# A deliberately NON-square base, so a test that passes by accident on lx == ly fails here.
R, TRACK, WHEELBASE = 0.05, 0.30, 0.28
LXY = 0.5 * (WHEELBASE + TRACK)   # 0.29


def geom(layout: str = "x", **kw) -> MecanumGeometry:
    return MecanumGeometry(wheel_radius=R, track=TRACK, wheelbase=WHEELBASE,
                           roller_layout=layout, **kw)


# A spread of twists that mixes all three components, including zero and negatives.
TWISTS = [
    (0.0, 0.0, 0.0), (0.4, 0.0, 0.0), (-0.25, 0.0, 0.0),
    (0.0, 0.3, 0.0), (0.0, -0.3, 0.0),
    (0.0, 0.0, 1.2), (0.0, 0.0, -0.7),
    (0.2, 0.15, 0.5), (-0.3, 0.2, -0.9), (0.11, -0.22, 0.33),
]


# --- pure-axis patterns: the whole point of the module ------------------------------

def test_pure_vx_turns_all_four_wheels_equally():
    omega = inverse(0.4, 0.0, 0.0, geom())
    assert omega == pytest.approx((0.4 / R,) * 4)


def test_pure_vy_layout_x_gives_minus_plus_plus_minus():
    omega = inverse(0.0, 0.3, 0.0, geom("x"))
    assert omega == pytest.approx((-0.3 / R, 0.3 / R, 0.3 / R, -0.3 / R))


def test_pure_vy_layout_o_is_the_mirror_of_layout_x():
    assert inverse(0.0, 0.3, 0.0, geom("o")) == pytest.approx(
        tuple(-v for v in inverse(0.0, 0.3, 0.0, geom("x"))))


def test_pure_wz_gives_minus_plus_minus_plus():
    omega = inverse(0.0, 0.0, 1.2, geom())
    k = LXY * 1.2 / R
    assert omega == pytest.approx((-k, k, -k, k))


def test_roller_layout_does_not_affect_the_vx_or_wz_columns():
    """The layout bit flips vy ONLY — that is what makes it safe to guess and fix later."""
    for vx, _, wz in [(0.4, 0, 0), (0, 0, 1.2), (-0.25, 0, -0.7)]:
        assert inverse(vx, 0.0, wz, geom("x")) == pytest.approx(
            inverse(vx, 0.0, wz, geom("o")))


# --- the (lx + ly) derivation, pinned by test --------------------------------------

def test_wz_lever_arm_is_exactly_half_the_sum_of_track_and_wheelbase():
    """|wz coefficient| == lx + ly == (wheelbase + track) / 2. THE rotation gain."""
    wz = 1.0
    for value in inverse(0.0, 0.0, wz, geom()):
        assert abs(value) * R / wz == pytest.approx((TRACK + WHEELBASE) / 2.0)
    assert geom().lxy == pytest.approx((TRACK + WHEELBASE) / 2.0)


def test_wz_coefficients_match_the_first_principles_formula():
    """Re-derive `d_i * x_i - y_i` from wheel positions and check `inverse` agrees."""
    lx, ly = WHEELBASE / 2.0, TRACK / 2.0
    positions = {FL: (+lx, +ly), FR: (+lx, -ly), RL: (-lx, +ly), RR: (-lx, -ly)}
    wz = 1.0
    omega = inverse(0.0, 0.0, wz, geom())
    for i, name in enumerate(WHEEL_NAMES):
        x_i, y_i = positions[i]
        expected = HANDEDNESS[i] * x_i - y_i
        assert omega[i] * R / wz == pytest.approx(expected), name
        assert abs(expected) == pytest.approx(lx + ly), name


def test_the_other_handedness_would_make_a_square_base_unable_to_rotate():
    """Documents the build error the module docstring warns about.

    Flipping every roller's handedness turns the wz coefficient from (lx + ly) into
    (lx - ly) — zero for a square footprint, i.e. no wheel-speed combination yaws the
    robot. Software cannot fix it; the left and right wheels must be swapped.
    """
    lx, ly = WHEELBASE / 2.0, TRACK / 2.0
    wrong = tuple(-d for d in HANDEDNESS)
    positions = {FL: (+lx, +ly), FR: (+lx, -ly), RL: (-lx, +ly), RR: (-lx, -ly)}
    for i in range(4):
        x_i, y_i = positions[i]
        assert abs(wrong[i] * x_i - y_i) == pytest.approx(abs(lx - ly))

    square = WHEELBASE / 2.0   # pretend track == wheelbase
    for i in range(4):
        x_i, y_i = (square if positions[i][0] > 0 else -square,
                    square if positions[i][1] > 0 else -square)
        assert wrong[i] * x_i - y_i == pytest.approx(0.0)


# --- structural properties ----------------------------------------------------------

@pytest.mark.parametrize("layout", ["x", "o"])
@pytest.mark.parametrize("twist", TWISTS)
def test_forward_inverts_inverse(layout, twist):
    assert forward(inverse(*twist, geom(layout)), geom(layout)) == pytest.approx(twist)


@pytest.mark.parametrize("twist", TWISTS)
def test_inverse_is_linear(twist):
    """Linearity is what makes the proportional clamp direction-preserving."""
    a = 0.37
    scaled = inverse(*(a * c for c in twist), geom())
    assert scaled == pytest.approx(tuple(a * v for v in inverse(*twist, geom())))


@pytest.mark.parametrize("layout", ["x", "o"])
@pytest.mark.parametrize("twist", TWISTS)
def test_inverse_output_has_no_slip(layout, twist):
    """Anything `inverse` produces lies in the reachable subspace: residual is 0."""
    assert slip_residual(inverse(*twist, geom(layout))) == pytest.approx(0.0, abs=1e-12)


def test_slip_residual_is_nonzero_for_an_inconsistent_wheel_set():
    assert slip_residual((1.0, 1.0, 0.0, 0.0)) == pytest.approx(2.0)


def test_slip_residual_ignores_geometry_and_layout():
    """It is a pure null-space test, which is why it works as a runtime diagnostic."""
    omega = (1.0, -0.5, 0.25, 2.0)
    assert slip_residual(omega) == slip_residual(list(omega))


# --- geometry validation ------------------------------------------------------------

@pytest.mark.parametrize("field", ["wheel_radius", "track", "wheelbase", "gear_ratio"])
@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf"), "0.05", True])
def test_geometry_rejects_bad_dimensions(field, bad):
    kw = dict(wheel_radius=R, track=TRACK, wheelbase=WHEELBASE, gear_ratio=1.0)
    kw[field] = bad
    with pytest.raises(ValueError, match=field):
        MecanumGeometry(**kw)


def test_geometry_reports_every_problem_at_once():
    with pytest.raises(ValueError) as excinfo:
        MecanumGeometry(wheel_radius=0.0, track=-1.0, wheelbase=WHEELBASE,
                        roller_layout="diagonal")
    message = str(excinfo.value)
    assert "wheel_radius" in message and "track" in message and "roller_layout" in message


def test_geometry_rejects_an_unknown_layout_name():
    with pytest.raises(ValueError, match="roller_layout"):
        geom("X")   # case matters; only "x", "o", "unknown" are valid


def test_inverse_rejects_an_unknown_layout_name():
    bad = MecanumGeometry(wheel_radius=R, track=TRACK, wheelbase=WHEELBASE)
    object.__setattr__(bad, "roller_layout", "nope")   # bypass __post_init__
    with pytest.raises(ValueError, match="roller_layout"):
        inverse(0.1, 0.0, 0.0, bad)


# --- the provisional layout ---------------------------------------------------------

def test_unknown_layout_computes_as_x_but_is_flagged_provisional():
    unknown = geom(PROVISIONAL_LAYOUT)
    assert unknown.layout_is_provisional
    assert not geom("x").layout_is_provisional and not geom("o").layout_is_provisional
    for twist in TWISTS:
        assert inverse(*twist, unknown) == pytest.approx(inverse(*twist, geom("x")))


def test_mirrored_flips_the_strafe_direction_and_round_trips():
    assert geom("x").mirrored().roller_layout == "o"
    assert geom("o").mirrored().roller_layout == "x"
    assert geom("x").mirrored().mirrored().roller_layout == "x"


def test_mirrored_resolves_a_provisional_layout_to_o():
    """It was computing as "x", so the fix for "strafe went the wrong way" is "o"."""
    mirrored = geom(PROVISIONAL_LAYOUT).mirrored()
    assert mirrored.roller_layout == "o"
    assert not mirrored.layout_is_provisional


def test_with_layout_preserves_every_other_field():
    original = geom("x", gear_ratio=13.5)
    changed = original.with_layout("o")
    assert (changed.wheel_radius, changed.track, changed.wheelbase, changed.gear_ratio) == \
           (original.wheel_radius, original.track, original.wheelbase, original.gear_ratio)


def test_gear_ratio_is_carried_but_never_applied_by_the_kinematics():
    """base.py applies it after the clamp; the kinematics works at the wheel."""
    assert inverse(0.4, 0.1, 0.3, geom(gear_ratio=1.0)) == pytest.approx(
        inverse(0.4, 0.1, 0.3, geom(gear_ratio=27.0)))


# --- proportional clamp -------------------------------------------------------------

def test_scale_to_limit_is_a_no_op_below_the_limit():
    values = [10.0, -20.0, 5.0, 0.0]
    scaled, k = scale_to_limit(values, 20.0)
    assert k == 1.0 and scaled == values


def test_scale_to_limit_brings_the_peak_exactly_to_the_limit():
    scaled, k = scale_to_limit([50.0, -100.0, 25.0, 0.0], 40.0)
    assert max(abs(v) for v in scaled) == pytest.approx(40.0)
    assert k == pytest.approx(0.4)


def test_scale_to_limit_preserves_every_ratio():
    """This is the property that keeps a clamped straight line straight."""
    values = [50.0, -100.0, 25.0, 12.5]
    scaled, k = scale_to_limit(values, 40.0)
    for original, result in zip(values, scaled):
        assert result == pytest.approx(original * k)


def test_clamping_a_twist_is_the_same_as_asking_for_a_slower_twist():
    """The clamp cannot change the path, only the speed along it — because IK is linear."""
    fast = inverse(2.0, 1.0, 3.0, geom())
    scaled, k = scale_to_limit(fast, 10.0)
    slower = inverse(2.0 * k, 1.0 * k, 3.0 * k, geom())
    assert scaled == pytest.approx(slower)
    vx, vy, wz = forward(scaled, geom())
    assert (vx, vy, wz) == pytest.approx((2.0 * k, 1.0 * k, 3.0 * k))


def test_scale_to_limit_handles_an_all_zero_vector():
    scaled, k = scale_to_limit([0.0, 0.0, 0.0, 0.0], 40.0)
    assert k == 1.0 and scaled == [0.0, 0.0, 0.0, 0.0]


def test_scale_to_limit_accepts_any_iterable_and_returns_a_list():
    scaled, _ = scale_to_limit((1.0, 2.0, 3.0, 4.0), 10.0)
    assert scaled == [1.0, 2.0, 3.0, 4.0]


@pytest.mark.parametrize("bad", [0.0, -5.0, float("nan"), float("inf"), None, True])
def test_scale_to_limit_rejects_a_bad_limit(bad):
    with pytest.raises(ValueError, match="limit"):
        scale_to_limit([1.0, 2.0, 3.0, 4.0], bad)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -math.inf])
def test_scale_to_limit_rejects_non_finite_wheel_speeds(bad):
    with pytest.raises(ValueError, match="finite"):
        scale_to_limit([1.0, bad, 3.0, 4.0], 10.0)
