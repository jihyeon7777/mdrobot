"""ALIGN must not strafe on a plate reading it has outgrown.

From the run of 2026-09-05 (runs/run_20260905_193604.jsonl). Detection ran at
1.9 Hz and then stopped at t=20.2 s with offset -0.125. ALIGN kept commanding
vy +0.050 for the next 20 ticks -- two full seconds, the whole plate_timeout --
because plate_fresh stays true for that long by design. At 0.05 m/s that is
10 cm of sideways travel chasing an offset that had already been closed, and
the machine finished beside the plate rather than under it.

The two windows answer different questions and must not be the same number:
plate_timeout asks whether the plate is gone, align_offset_max_age asks
whether this reading may still be steered on.
"""

import pytest

from mdrobot_supervisor.autonomous import (
    AutonomousConfig,
    AutonomousSequence,
    Observation,
    Phase,
)

# The last reading of the run, and the gap that followed it.
LAST_OFFSET = -0.125
DETECTOR_PERIOD = 0.5  # 1.9 Hz measured, 0.50 s median gap
STRAFED_BLIND_SECONDS = 2.0


def config(**overrides) -> AutonomousConfig:
    base = dict(
        yaw_hold=False,
        entry_distance=1.0,
        entry_speed=0.08,
        min_approach_width=0.45,
        lift_up_seconds=1.0,
        lift_down_seconds=1.0,
        plate_timeout=2.0,
        align_offset_max_age=0.6,
    )
    base.update(overrides)
    return AutonomousConfig(**base)


def obs(now, *, age, offset=LAST_OFFSET, width=0.30) -> Observation:
    return Observation(
        now=now, plate_offset_x=offset, plate_age=age, distance=0.0,
        plate_width=width,
    )


def aligning(cfg):
    """A sequence sitting in ALIGN, having just seen the plate."""
    seq = AutonomousSequence(cfg)
    seq.step(obs(0.0, age=0.0))
    assert seq.phase is Phase.ALIGN
    return seq


def test_fresh_offset_is_steered_on():
    seq = aligning(config())
    act = seq.step(obs(1.0, age=0.1))
    assert act.vy > 0.0, "a reading a tenth of a second old is the current one"


def test_offset_one_detector_period_old_is_still_steered_on():
    """The window has to clear a normal gap between sightings, or the strafe
    would switch off and on at the detector's own rate."""
    seq = aligning(config())
    act = seq.step(obs(1.0, age=DETECTOR_PERIOD))
    assert act.vy > 0.0


def test_stale_offset_is_not_steered_on():
    seq = aligning(config())
    act = seq.step(obs(1.0, age=STRAFED_BLIND_SECONDS))
    assert act.vy == 0.0, "this is the 10 cm the machine drove sideways blind"


def test_stale_offset_still_drives_forward():
    """Only the strafe is gated. Closing on the car is right whatever the
    lateral offset is doing, and stopping would strand the sequence short."""
    cfg = config()
    seq = aligning(cfg)
    act = seq.step(obs(1.0, age=STRAFED_BLIND_SECONDS))
    assert act.vx == pytest.approx(cfg.approach_speed)


def test_stale_offset_does_not_end_align():
    """The reading is too old to steer on but not old enough to call the plate
    lost -- that is plate_timeout's decision, and it has not expired."""
    seq = aligning(config())
    seq.step(obs(1.0, age=STRAFED_BLIND_SECONDS))
    assert seq.phase is Phase.ALIGN


def test_stale_window_sits_inside_the_lost_window():
    """If these ever crossed, the strafe gate would be dead code: the plate
    would be declared lost before a reading could go stale."""
    cfg = config()
    assert cfg.align_offset_max_age < cfg.plate_timeout


def test_message_says_why_it_stopped_strafing():
    seq = aligning(config())
    act = seq.step(obs(1.0, age=STRAFED_BLIND_SECONDS))
    assert "old" in act.message and "strafing" in act.message


def test_the_recorded_run_would_not_have_strafed_blind():
    """Replay the tail of run_20260905_193604: one sighting, then silence."""
    cfg = config()
    seq = aligning(cfg)
    sideways = 0.0
    t = 0.0
    for _ in range(20):  # the 20 ticks that followed the last sighting
        t += 0.1
        act = seq.step(obs(t, age=t))
        sideways += abs(act.vy) * 0.1
    assert sideways < 0.04, f"drove {sideways*100:.0f} cm sideways blind"
