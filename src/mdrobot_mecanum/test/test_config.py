"""Unit tests for the mecanum config layer. No hardware, no serial port."""

from __future__ import annotations

import pytest
import yaml

from mdrobot_mecanum.config import (
    COMMAND_WATCHDOG_S,
    CONFIG_VERSION,
    ConfigError,
    MecanumConfig,
    WheelSpec,
)
from mdrobot_mecanum.kinematics import PROVISIONAL_LAYOUT, WHEEL_NAMES


def base_dict(**overrides) -> dict:
    """A minimal valid config as plain data."""
    data = {
        "version": CONFIG_VERSION,
        "port": "/dev/ttyUSB0",
        "wheels": {
            "front_left": {"slave_id": 1, "channel": 1, "sign": 1},
            "front_right": {"slave_id": 1, "channel": 2, "sign": -1},
            "rear_left": {"slave_id": 2, "channel": 1, "sign": 1},
            "rear_right": {"slave_id": 2, "channel": 2, "sign": -1},
        },
        "geometry": {"wheel_radius": 0.05, "track": 0.30, "wheelbase": 0.28},
    }
    data.update(overrides)
    return data


def wheels_with(**overrides) -> dict:
    """The wheel block with per-wheel overrides, e.g. front_left={"channel": 2}."""
    wheels = base_dict()["wheels"]
    for name, changes in overrides.items():
        wheels[name] = {**wheels[name], **changes}
    return wheels


# --- happy path ---------------------------------------------------------------------

def test_minimal_config_loads_and_fills_defaults():
    cfg = MecanumConfig.from_dict(base_dict())
    assert cfg.port == "/dev/ttyUSB0"
    assert cfg.baudrate == 19200
    assert cfg.limits.max_motor_rpm == 100
    assert cfg.runtime.loop_hz == 10.0
    assert cfg.runtime.use_batched_velocity is False
    assert cfg.geometry.roller_layout == PROVISIONAL_LAYOUT
    assert cfg.geometry.gear_ratio == 1.0


def test_wheel_specs_are_in_fixed_wheel_order():
    cfg = MecanumConfig.from_dict(base_dict())
    assert [spec.address for spec in cfg.wheel_specs] == [(1, 1), (1, 2), (2, 1), (2, 2)]
    assert cfg.signs == (1, -1, 1, -1)


def test_slave_ids_and_channels_of():
    cfg = MecanumConfig.from_dict(base_dict())
    assert cfg.slave_ids == (1, 2)
    assert cfg.channels_of(1) == {1: "front_left", 2: "front_right"}
    assert cfg.channels_of(2) == {1: "rear_left", 2: "rear_right"}


def test_port_may_be_null_to_mean_the_environment_variable():
    assert MecanumConfig.from_dict(base_dict(port=None)).port is None


def test_roller_layout_may_be_given_at_top_level_or_nested():
    assert MecanumConfig.from_dict(base_dict(roller_layout="o")).geometry.roller_layout == "o"
    nested = base_dict()
    nested["geometry"]["roller_layout"] = "x"
    assert MecanumConfig.from_dict(nested).geometry.roller_layout == "x"


# --- round trips --------------------------------------------------------------------

def test_to_dict_round_trips():
    cfg = MecanumConfig.from_dict(base_dict(roller_layout="x"))
    assert MecanumConfig.from_dict(cfg.to_dict()) == cfg


def test_render_round_trips_through_yaml():
    """load -> render -> load must be an identity, or hand-editing loses information."""
    cfg = MecanumConfig.from_dict(base_dict(roller_layout="o"))
    assert MecanumConfig.from_dict(yaml.safe_load(cfg.render())) == cfg


def test_render_round_trips_with_every_field_customised():
    cfg = MecanumConfig.from_dict(base_dict(
        port=None, baudrate=57600, timeout=0.5, roller_layout="x",
        geometry={"wheel_radius": 0.0762, "track": 0.33, "wheelbase": 0.29,
                  "gear_ratio": 13.5},
        limits={"max_motor_rpm": 250, "max_linear_x": 0.8, "max_linear_y": 0.6,
                "max_angular_z": 2.5, "accel_linear": 1.0, "decel_linear": 2.0,
                "accel_angular": 3.0, "decel_angular": 6.0},
        runtime={"loop_hz": 15.0, "idle_timeout": 0.0, "max_comm_errors": 5,
                 "use_batched_velocity": True, "controller_ramp_s": None,
                 "use_limit_sw": -1, "auto_enable": False}))
    assert MecanumConfig.from_dict(yaml.safe_load(cfg.render())) == cfg


def test_rendered_config_keeps_its_comments():
    text = MecanumConfig.from_dict(base_dict()).render()
    assert "# Mecanum base configuration" in text
    assert "cannot be decided by eye" in text
    assert "bus budget:" in text


def test_render_flags_a_provisional_layout_but_a_settled_one_is_quiet():
    assert "STILL UNVERIFIED" in MecanumConfig.from_dict(base_dict()).render()
    assert "STILL UNVERIFIED" not in \
        MecanumConfig.from_dict(base_dict(roller_layout="x")).render()


# --- files --------------------------------------------------------------------------

def test_save_and_load_a_file(tmp_path):
    cfg = MecanumConfig.from_dict(base_dict(roller_layout="x"))
    path = cfg.save(tmp_path / "mecanum.yaml")
    assert MecanumConfig.load(path) == cfg


def test_save_refuses_to_overwrite_without_force(tmp_path):
    cfg = MecanumConfig.from_dict(base_dict())
    path = cfg.save(tmp_path / "mecanum.yaml")
    with pytest.raises(ConfigError, match="already exists"):
        cfg.save(path)


def test_save_with_force_keeps_a_backup(tmp_path):
    path = tmp_path / "mecanum.yaml"
    first = MecanumConfig.from_dict(base_dict(roller_layout="x"))
    first.save(path)
    MecanumConfig.from_dict(base_dict(roller_layout="o")).save(path, force=True)

    assert MecanumConfig.load(path).geometry.roller_layout == "o"
    assert MecanumConfig.load(path.with_suffix(".yaml.bak")).geometry.roller_layout == "x"


def test_load_reports_the_path_in_its_errors(tmp_path):
    path = tmp_path / "broken.yaml"
    path.write_text("wheels: 3\n")
    with pytest.raises(ConfigError, match="broken.yaml"):
        MecanumConfig.load(path)


def test_load_rejects_an_empty_or_missing_or_malformed_file(tmp_path):
    empty = tmp_path / "empty.yaml"
    empty.write_text("")
    with pytest.raises(ConfigError, match="empty"):
        MecanumConfig.load(empty)

    with pytest.raises(ConfigError, match="cannot read"):
        MecanumConfig.load(tmp_path / "absent.yaml")

    bad = tmp_path / "bad.yaml"
    bad.write_text("wheels: [unclosed\n")
    with pytest.raises(ConfigError, match="not valid YAML"):
        MecanumConfig.load(bad)


# --- unknown keys -------------------------------------------------------------------

def test_unknown_top_level_key_is_an_error_with_a_hint():
    with pytest.raises(ConfigError) as excinfo:
        MecanumConfig.from_dict(base_dict(baudrat=19200))
    assert "unknown key 'baudrat'" in str(excinfo.value)
    assert "did you mean 'baudrate'" in str(excinfo.value)


def test_unknown_limit_key_is_an_error_because_silence_would_be_dangerous():
    """A typo'd cap must not look like a cap that is in force."""
    with pytest.raises(ConfigError) as excinfo:
        MecanumConfig.from_dict(base_dict(limits={"max_moter_rpm": 50}))
    assert "max_motor_rpm" in str(excinfo.value)


@pytest.mark.parametrize("section,payload", [
    ("geometry", {"wheel_radius": 0.05, "track": 0.3, "wheelbase": 0.3, "radius": 1}),
    ("runtime", {"loop_rate": 10}),
])
def test_unknown_key_in_a_nested_section(section, payload):
    with pytest.raises(ConfigError, match="unknown key"):
        MecanumConfig.from_dict(base_dict(**{section: payload}))


def test_unknown_wheel_name_is_rejected():
    wheels = base_dict()["wheels"]
    wheels["middle_left"] = {"slave_id": 3, "channel": 1}
    with pytest.raises(ConfigError, match="middle_left"):
        MecanumConfig.from_dict(base_dict(wheels=wheels))


def test_unknown_key_inside_one_wheel():
    with pytest.raises(ConfigError, match="unknown key 'reverse'"):
        MecanumConfig.from_dict(base_dict(
            wheels=wheels_with(front_left={"reverse": True})))


# --- missing / malformed ------------------------------------------------------------

def test_missing_wheels_and_geometry_are_errors_since_they_have_no_safe_default():
    data = base_dict()
    del data["wheels"], data["geometry"]
    with pytest.raises(ConfigError) as excinfo:
        MecanumConfig.from_dict(data)
    assert "wheels: missing" in str(excinfo.value)
    assert "geometry: missing" in str(excinfo.value)


def test_a_missing_optional_key_falls_back_to_its_default():
    """Dropping a line while hand-editing must not break the file."""
    cfg = MecanumConfig.from_dict(base_dict(limits={"max_motor_rpm": 60}))
    assert cfg.limits.max_motor_rpm == 60
    assert cfg.limits.max_linear_x == 0.30      # untouched default


def test_missing_one_wheel_is_reported_by_name():
    wheels = base_dict()["wheels"]
    del wheels["rear_right"]
    with pytest.raises(ConfigError, match="missing rear_right"):
        MecanumConfig.from_dict(base_dict(wheels=wheels))


def test_missing_geometry_dimension_is_reported():
    with pytest.raises(ConfigError, match="missing wheelbase"):
        MecanumConfig.from_dict(base_dict(
            geometry={"wheel_radius": 0.05, "track": 0.3}))


@pytest.mark.parametrize("value", ["nope", 3, None, []])
def test_wheels_must_be_a_mapping(value):
    with pytest.raises(ConfigError, match="wheels"):
        MecanumConfig.from_dict(base_dict(wheels=value))


def test_top_level_must_be_a_mapping():
    with pytest.raises(ConfigError, match="must be a mapping"):
        MecanumConfig.from_dict(["not", "a", "mapping"])


def test_wrong_version_is_rejected():
    with pytest.raises(ConfigError, match="version"):
        MecanumConfig.from_dict(base_dict(version=99))


# --- topology: two controllers x two channels ---------------------------------------

def test_duplicate_slave_id_and_channel_is_rejected():
    with pytest.raises(ConfigError) as excinfo:
        MecanumConfig.from_dict(base_dict(
            wheels=wheels_with(rear_left={"slave_id": 1, "channel": 1})))
    assert "own motor output" in str(excinfo.value)


def test_three_slave_ids_is_rejected():
    with pytest.raises(ConfigError, match="exactly 2 distinct slave ids"):
        MecanumConfig.from_dict(base_dict(
            wheels=wheels_with(rear_right={"slave_id": 3, "channel": 2})))


def test_one_slave_id_is_rejected():
    with pytest.raises(ConfigError, match="exactly 2 distinct slave ids"):
        MecanumConfig.from_dict(base_dict(wheels=wheels_with(
            rear_left={"slave_id": 1, "channel": 1},
            rear_right={"slave_id": 1, "channel": 2})))


def test_a_controller_must_drive_both_of_its_channels():
    with pytest.raises(ConfigError, match=r"channels \[1, 2\]"):
        MecanumConfig.from_dict(base_dict(wheels=wheels_with(
            front_left={"slave_id": 1, "channel": 1},
            front_right={"slave_id": 2, "channel": 1},
            rear_left={"slave_id": 1, "channel": 2},
            rear_right={"slave_id": 2, "channel": 1})))


@pytest.mark.parametrize("channel", [0, 3, -1, "1", 1.0, True])
def test_channel_must_be_1_or_2(channel):
    with pytest.raises(ConfigError, match="channel must be 1 or 2"):
        MecanumConfig.from_dict(base_dict(
            wheels=wheels_with(front_left={"channel": channel})))


@pytest.mark.parametrize("slave_id", [0, 248, -1, "1", 1.5, True])
def test_slave_id_must_be_a_valid_modbus_address(slave_id):
    with pytest.raises(ConfigError, match="slave_id"):
        MecanumConfig.from_dict(base_dict(
            wheels=wheels_with(front_left={"slave_id": slave_id})))


@pytest.mark.parametrize("sign", [0, 2, -2, "1", 1.0, True, None])
def test_sign_must_be_plus_or_minus_one(sign):
    with pytest.raises(ConfigError, match="sign must be 1 or -1"):
        MecanumConfig.from_dict(base_dict(
            wheels=wheels_with(front_left={"sign": sign})))


# --- limits and runtime -------------------------------------------------------------

@pytest.mark.parametrize("value", [0, -5, 1.5, "100", True])
def test_max_motor_rpm_must_be_a_positive_integer(value):
    with pytest.raises(ConfigError, match="max_motor_rpm"):
        MecanumConfig.from_dict(base_dict(limits={"max_motor_rpm": value}))


def test_max_motor_rpm_above_the_int16_wire_limit_is_rejected():
    with pytest.raises(ConfigError, match="int16"):
        MecanumConfig.from_dict(base_dict(limits={"max_motor_rpm": 40000}))


def test_decel_below_accel_is_rejected():
    """The robot must stop at least as fast as it starts."""
    with pytest.raises(ConfigError, match="decel_linear"):
        MecanumConfig.from_dict(base_dict(
            limits={"accel_linear": 1.0, "decel_linear": 0.5}))
    with pytest.raises(ConfigError, match="decel_angular"):
        MecanumConfig.from_dict(base_dict(
            limits={"accel_angular": 2.0, "decel_angular": 1.0}))


def test_equal_accel_and_decel_is_allowed():
    MecanumConfig.from_dict(base_dict(
        limits={"accel_linear": 1.0, "decel_linear": 1.0}))


@pytest.mark.parametrize("name", ["max_linear_x", "max_linear_y", "max_angular_z",
                                  "accel_linear", "accel_angular"])
@pytest.mark.parametrize("value", [0, -1.0, float("nan"), float("inf"), "fast"])
def test_speed_limits_must_be_positive_and_finite(name, value):
    with pytest.raises(ConfigError, match=name):
        MecanumConfig.from_dict(base_dict(limits={name: value}))


@pytest.mark.parametrize("value", [0, -1.0, float("nan"), "fast"])
def test_loop_hz_must_be_positive_and_finite(value):
    with pytest.raises(ConfigError, match="loop_hz"):
        MecanumConfig.from_dict(base_dict(runtime={"loop_hz": value}))


@pytest.mark.parametrize("value", [0.5, 1.0, 1.9])
def test_a_loop_slower_than_the_command_watchdog_is_rejected(value):
    """Below ~2 Hz the motors stop BETWEEN ticks — a correctness bug, not a comfort one.

    The controller cuts drive after ~2 s of bus silence (measured on PNT50 DL=19).
    """
    with pytest.raises(ConfigError, match="bus silence"):
        MecanumConfig.from_dict(base_dict(runtime={"loop_hz": value}))


def test_the_shipped_loop_rate_has_ample_watchdog_margin():
    cfg = MecanumConfig.from_dict(base_dict())
    assert 1.0 / cfg.runtime.loop_hz < COMMAND_WATCHDOG_S / 10.0


def test_idle_timeout_zero_disables_it_but_negative_is_rejected():
    assert MecanumConfig.from_dict(
        base_dict(runtime={"idle_timeout": 0.0})).runtime.idle_timeout == 0.0
    with pytest.raises(ConfigError, match="idle_timeout"):
        MecanumConfig.from_dict(base_dict(runtime={"idle_timeout": -1.0}))


@pytest.mark.parametrize("value", [-2, 2, "0", None])
def test_use_limit_sw_must_be_minus_one_zero_or_one(value):
    with pytest.raises(ConfigError, match="use_limit_sw"):
        MecanumConfig.from_dict(base_dict(runtime={"use_limit_sw": value}))


def test_controller_ramp_may_be_null_to_leave_the_controller_alone():
    cfg = MecanumConfig.from_dict(base_dict(runtime={"controller_ramp_s": None}))
    assert cfg.runtime.controller_ramp_s is None
    with pytest.raises(ConfigError, match="controller_ramp_s"):
        MecanumConfig.from_dict(base_dict(runtime={"controller_ramp_s": -0.5}))


@pytest.mark.parametrize("name", ["use_batched_velocity", "auto_enable"])
@pytest.mark.parametrize("value", [1, 0, "true", None])
def test_boolean_runtime_flags_must_really_be_booleans(name, value):
    with pytest.raises(ConfigError, match=name):
        MecanumConfig.from_dict(base_dict(runtime={name: value}))


@pytest.mark.parametrize("value", [0, -1, 1.5, "9600"])
def test_baudrate_and_timeout_are_checked(value):
    with pytest.raises(ConfigError, match="baudrate"):
        MecanumConfig.from_dict(base_dict(baudrate=value))
    with pytest.raises(ConfigError, match="timeout"):
        MecanumConfig.from_dict(base_dict(timeout=-1.0))


def test_port_must_be_a_string_or_null():
    with pytest.raises(ConfigError, match="port"):
        MecanumConfig.from_dict(base_dict(port=42))


# --- error aggregation --------------------------------------------------------------

def test_every_problem_is_reported_at_once():
    """One re-run per fix is a bad way to spend a bench session."""
    with pytest.raises(ConfigError) as excinfo:
        MecanumConfig.from_dict(base_dict(
            baudrate=-1, timeout=0,
            limits={"max_motor_rpm": 0, "max_linear_x": -1.0},
            runtime={"loop_hz": 0, "use_limit_sw": 7}))
    message = str(excinfo.value)
    for expected in ("baudrate", "timeout", "max_motor_rpm", "max_linear_x",
                     "loop_hz", "use_limit_sw"):
        assert expected in message
    assert "problems:" in message


def test_a_single_problem_is_reported_without_the_list_header():
    with pytest.raises(ConfigError) as excinfo:
        MecanumConfig.from_dict(base_dict(port=42))
    assert "problems:" not in str(excinfo.value)


# --- edits used by the floor test ---------------------------------------------------

def test_with_signs_flips_wheels_in_fixed_order():
    cfg = MecanumConfig.from_dict(base_dict())
    flipped = cfg.with_signs([-s for s in cfg.signs])
    assert flipped.signs == (-1, 1, -1, 1)
    assert flipped.wheel_specs[0].address == cfg.wheel_specs[0].address
    flipped.validate()


def test_with_signs_rejects_a_wrong_length():
    with pytest.raises(ValueError, match="4 signs"):
        MecanumConfig.from_dict(base_dict()).with_signs([1, -1])


def test_with_layout_settles_a_provisional_layout():
    cfg = MecanumConfig.from_dict(base_dict())
    assert cfg.geometry.layout_is_provisional
    settled = cfg.with_layout("o")
    assert settled.geometry.roller_layout == "o"
    assert not settled.geometry.layout_is_provisional
    settled.validate()


def test_edits_survive_a_render_round_trip(tmp_path):
    cfg = MecanumConfig.from_dict(base_dict()).with_layout("o").with_signs([-1, 1, -1, 1])
    path = cfg.save(tmp_path / "fixed.yaml")
    assert MecanumConfig.load(path) == cfg


# --- bus budget ---------------------------------------------------------------------

def test_bus_budget_counts_one_write_per_wheel():
    cfg = MecanumConfig.from_dict(base_dict())
    cost, period, duty = cfg.bus_budget()
    assert cost == pytest.approx(4 * 0.012)
    assert period == pytest.approx(0.1)
    assert duty == pytest.approx(0.48)


def test_batched_writes_halve_the_bus_cost():
    cfg = MecanumConfig.from_dict(base_dict(runtime={"use_batched_velocity": True}))
    assert cfg.bus_budget()[0] == pytest.approx(2 * 0.012)


def test_bus_budget_note_warns_on_overrun():
    cfg = MecanumConfig.from_dict(base_dict(runtime={"loop_hz": 30.0}))
    assert "OVERRUN" in cfg.bus_budget_note()


def test_bus_budget_note_warns_when_tight():
    cfg = MecanumConfig.from_dict(base_dict(runtime={"loop_hz": 18.0}))
    note = cfg.bus_budget_note()
    assert "tight" in note and "OVERRUN" not in note


def test_bus_budget_note_is_quiet_at_the_shipped_rate():
    note = MecanumConfig.from_dict(base_dict()).bus_budget_note()
    assert "OVERRUN" not in note and "tight" not in note


# --- WheelSpec ----------------------------------------------------------------------

def test_wheel_spec_address_is_the_slave_id_and_channel_pair():
    assert WheelSpec(slave_id=2, channel=1).address == (2, 1)
    assert WheelSpec(slave_id=2, channel=1).sign == 1


def test_wheel_names_are_the_four_corners_in_a_fixed_order():
    assert WHEEL_NAMES == ("front_left", "front_right", "rear_left", "rear_right")
