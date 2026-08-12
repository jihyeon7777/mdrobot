"""Tests for the on-blocks dispatch verification. No hardware.

A fake base stands in for the robot: it records what `drive` asked for and hands back
whatever position deltas the test wants, so every diagnosis the checker can make is
exercised without a motor.
"""

from __future__ import annotations

import pytest

from mdrobot_mecanum import verify
from mdrobot_mecanum.base import DriveResult
from mdrobot_mecanum.config import MecanumConfig
from mdrobot_mecanum.kinematics import WHEEL_NAMES, inverse

R, TRACK, WHEELBASE = 0.0625, 0.575, 0.500


def make_config(layout="x", **overrides) -> MecanumConfig:
    data = {
        "version": 1,
        "port": "/dev/null",
        "wheels": {
            "front_left": {"slave_id": 1, "channel": 1, "sign": -1},
            "front_right": {"slave_id": 1, "channel": 2, "sign": 1},
            "rear_left": {"slave_id": 2, "channel": 2, "sign": -1},
            "rear_right": {"slave_id": 2, "channel": 1, "sign": 1},
        },
        "roller_layout": layout,
        "geometry": {"wheel_radius": R, "track": TRACK, "wheelbase": WHEELBASE,
                     "gear_ratio": 20.0},
        "limits": {"max_motor_rpm": 600, "max_linear_x": 0.19, "max_linear_y": 0.19,
                   "max_angular_z": 0.36},
    }
    data.update(overrides)
    return MecanumConfig.from_dict(data)


class FakeBase:
    """Enough of MecanumBase for the checker: it drives, then reports a travel."""

    def __init__(self, config, travel_for):
        self.config = config
        #: callable(twist) -> the four position deltas that motion produced
        self._travel_for = travel_for
        self._position = (0, 0, 0, 0)
        self._twist = (0.0, 0.0, 0.0)
        self.drive_calls = 0
        self.stops = 0

    def read_wheel_positions(self):
        return self._position

    def drive(self, vx, vy, wz):
        self.drive_calls += 1
        self._twist = (vx, vy, wz)
        omega = inverse(vx, vy, wz, self.config.geometry)
        return DriveResult(twist=(vx, vy, wz), wheel_rad_s=omega,
                           wheel_rpm=(0, 0, 0, 0), scale=1.0)

    def stop(self):
        self.stops += 1
        # The travel appears once the motion is over, as it does on real hardware.
        delta = self._travel_for(self._twist)
        self._position = tuple(p + d for p, d in zip(self._position, delta))


def perfect_travel(scale=100.0):
    """Position deltas exactly proportional to what the kinematics asked for."""
    def travel(twist):
        omega = inverse(*twist, GEOM)
        biggest = max(abs(w) for w in omega) or 1.0
        return tuple(round(w / biggest * scale) for w in omega)
    return travel


GEOM = make_config().geometry


def run_one(base, twist, hold_s=0.0):
    return verify.check_motion(base, "test", twist, hold_s, settle_s=0.0)


# --- the happy path -----------------------------------------------------------------

@pytest.mark.parametrize("twist", [(0.1, 0, 0), (0, 0.1, 0), (0, 0, 0.2),
                                   (0.05, 0.05, 0.1)])
def test_a_correctly_wired_robot_passes(twist):
    base = FakeBase(make_config(), perfect_travel())
    check = run_one(base, twist)
    assert check.ok, check.problems


def test_all_three_motions_pass_together():
    base = FakeBase(make_config(), perfect_travel())
    checks = verify.run(base, effort=0.5, hold_s=0.0, settle_s=0.0,
                        pause_s=0.0, printer=lambda *a: None)
    assert len(checks) == 3
    assert all(c.ok for c in checks), [c.problems for c in checks]


def test_summarise_returns_zero_when_everything_passes(capsys):
    base = FakeBase(make_config(), perfect_travel())
    checks = verify.run(base, effort=0.5, hold_s=0.0, settle_s=0.0,
                        pause_s=0.0, printer=lambda *a: None)
    assert verify.summarise(checks, base) == 0


# --- the failures it exists to catch ------------------------------------------------

def test_a_wheel_turning_backwards_is_caught():
    def travel(twist):
        deltas = list(perfect_travel()(twist))
        deltas[1] = -deltas[1]          # front_right wired backwards
        return tuple(deltas)

    check = run_one(FakeBase(make_config(), travel), (0.1, 0, 0))
    assert not check.ok
    assert any("front_right" in p and "wrong way" in p for p in check.problems)


def test_a_dead_channel_is_caught():
    def travel(twist):
        deltas = list(perfect_travel()(twist))
        deltas[2] = 0                   # rear_left never moves
        return tuple(deltas)

    check = run_one(FakeBase(make_config(), travel), (0.1, 0, 0))
    assert any("rear_left" in p and "did not move" in p for p in check.problems)


def test_a_wheel_turning_far_too_slowly_is_caught():
    def travel(twist):
        deltas = list(perfect_travel()(twist))
        deltas[3] = deltas[3] // 4      # rear_right barely turns
        return tuple(deltas)

    check = run_one(FakeBase(make_config(), travel), (0.1, 0, 0))
    assert any("rear_right" in p and "expected about" in p for p in check.problems)


def test_every_wheel_reversed_is_caught_on_forward():
    def travel(twist):
        return tuple(-d for d in perfect_travel()(twist))

    check = run_one(FakeBase(make_config(), travel), (0.1, 0, 0))
    assert len(check.problems) == 4


def test_summarise_returns_nonzero_when_a_check_failed(capsys):
    def travel(twist):
        return tuple(-d for d in perfect_travel()(twist))

    base = FakeBase(make_config(), travel)
    checks = verify.run(base, effort=0.5, hold_s=0.0, settle_s=0.0,
                        pause_s=0.0, printer=lambda *a: None)
    assert verify.summarise(checks, base) == 1


# --- what it deliberately does NOT judge --------------------------------------------

def test_the_roller_layout_cannot_change_the_verdict():
    """Wheels in the air have no rollers doing work: the layout is undecidable here.

    The same physical robot must pass with either layout setting, because the checker
    compares wheels against the kinematics it dispatched, not against the ground.
    """
    for layout in ("x", "o"):
        config = make_config(layout)

        def travel(twist, geom=config.geometry):
            omega = inverse(*twist, geom)
            biggest = max(abs(w) for w in omega) or 1.0
            return tuple(round(w / biggest * 100.0) for w in omega)

        base = FakeBase(config, travel)
        checks = verify.run(base, effort=0.5, hold_s=0.0, settle_s=0.0,
                        pause_s=0.0, printer=lambda *a: None)
        assert all(c.ok for c in checks), (layout, [c.problems for c in checks])


def test_a_provisional_layout_is_reported_as_still_unverified():
    messages = []
    base = FakeBase(make_config("unknown"), perfect_travel())
    checks = verify.run(base, effort=0.5, hold_s=0.0, settle_s=0.0,
                        pause_s=0.0, printer=lambda *a: None)
    assert verify.summarise(checks, base, printer=messages.append) == 0
    assert any("STILL UNVERIFIED" in m for m in messages)


def test_a_settled_layout_is_not_flagged():
    messages = []
    base = FakeBase(make_config("x"), perfect_travel())
    checks = verify.run(base, effort=0.5, hold_s=0.0, settle_s=0.0,
                        pause_s=0.0, printer=lambda *a: None)
    verify.summarise(checks, base, printer=messages.append)
    assert not any("STILL UNVERIFIED" in m for m in messages)


# --- mechanics ----------------------------------------------------------------------

def test_the_command_is_refreshed_throughout_the_hold():
    """A velocity command is not a latch — the controller cuts drive on bus silence."""
    base = FakeBase(make_config(), perfect_travel())
    run_one(base, (0.1, 0, 0), hold_s=0.35)
    assert base.drive_calls >= 3


def test_each_motion_ends_with_a_stop():
    base = FakeBase(make_config(), perfect_travel())
    run_one(base, (0.1, 0, 0))
    assert base.stops == 1


def test_effort_scales_the_commanded_twist():
    config = make_config()
    half = verify.motions(FakeBase(config, perfect_travel()), 0.5)
    full = verify.motions(FakeBase(config, perfect_travel()), 1.0)
    assert half[0][1][0] == pytest.approx(full[0][1][0] / 2)
    assert full[0][1][0] == pytest.approx(config.limits.max_linear_x)


def test_the_three_motions_are_pure_single_axis():
    config = make_config()
    (_, vx), (_, vy), (_, wz) = verify.motions(FakeBase(config, perfect_travel()), 0.5)
    assert vx[1] == vx[2] == 0.0
    assert vy[0] == vy[2] == 0.0
    assert wz[0] == wz[1] == 0.0


def test_wheel_names_are_reported_in_the_fixed_order():
    base = FakeBase(make_config(), perfect_travel())
    check = run_one(base, (0.1, 0, 0))
    assert len(check.commanded) == len(check.measured) == len(WHEEL_NAMES)
