"""Mecanum robot configuration: dataclasses plus YAML load / render / validate.

The config file is what the operator lives with for the life of the robot, so it is
**rendered from a commented template** rather than dumped: `yaml.safe_dump` throws
every comment away, and the comments are where the non-obvious parts live (that
`roller_layout` cannot be decided by eye, that `max_motor_rpm` is the only hard cap).
`to_dict()` exists for round-trip tests and for anyone who wants a plain dump.

Two rules shape the validation:

- **A missing key is safe; a mistyped key is not.** Every field except `wheels` and
  `geometry` has a default, so a hand-edited file that drops a line keeps working.
  An *unknown* key is a hard error with a "did you mean" hint, because silently
  ignoring `max_moter_rpm` would leave the operator believing a limit is in force
  when it is not.
- **Report every problem at once.** `validate()` collects the whole list and raises a
  single `ConfigError`, rather than making the operator re-run after each fix.

Topology: this package supports exactly one shape — four wheels on **two distinct
slave ids x channels {1, 2}**. Saying so up front turns a whole class of confusing
runtime failures into one clear message at load time.
"""

from __future__ import annotations

import difflib
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import yaml

from .kinematics import PROVISIONAL_LAYOUT, ROLLER_LAYOUTS, WHEEL_NAMES, MecanumGeometry

#: Schema version of the config file. Bump only on an incompatible change.
CONFIG_VERSION = 1

#: Rough cost of one 0x06 velocity write at 19200 baud with latency_timer=1, in
#: seconds. Measured on an FTDI FT232R (manual/setup/port-setup.md).
WRITE_COST_S = 0.012
#: Same, for one multi-register read (read_monitor / PNT_MONITOR).
READ_COST_S = 0.017

#: The controller cuts motor drive after roughly this much bus silence (measured on
#: PNT50 DL=19; see mdrobot_mecanum.base.COMMAND_WATCHDOG_S). A control loop slower
#: than this does not merely feel laggy — the motors stop between ticks.
COMMAND_WATCHDOG_S = 2.0
#: Slowest loop we accept, as a fraction of the watchdog. A quarter leaves room for a
#: retry and a slow tick without ever reaching the cut-off.
WATCHDOG_SAFETY_FACTOR = 4.0


class ConfigError(Exception):
    """The config file is unusable. The message lists every problem found."""


@dataclass(frozen=True)
class WheelSpec:
    """Where one wheel is wired, and which way its motor turns.

    `sign` is +1 when a positive rpm command drives the robot forward, -1 otherwise.
    It is a property of motor wiring and mounting, entirely independent of the roller
    layout, and is applied after the kinematics and after the clamp.
    """

    slave_id: int
    channel: int
    sign: int = 1

    @property
    def address(self) -> tuple[int, int]:
        return (self.slave_id, self.channel)


@dataclass(frozen=True)
class Limits:
    """Speed and acceleration caps.

    `max_motor_rpm` is the only hard limit — it is what the proportional clamp
    enforces at the wheel, and therefore the single source of truth for safety. The
    `max_linear_*` / `max_angular_z` values are teleop *targets*: they shape what the
    operator can ask for, not what the hardware will accept.
    """

    max_motor_rpm: int = 100
    max_linear_x: float = 0.30
    max_linear_y: float = 0.30
    max_angular_z: float = 1.20
    accel_linear: float = 0.40
    decel_linear: float = 0.80
    accel_angular: float = 1.60
    decel_angular: float = 3.20


@dataclass(frozen=True)
class Runtime:
    """Control-loop and safety behaviour."""

    loop_hz: float = 10.0
    idle_timeout: float = 10.0        #: s; 0 disables. Guards a dead terminal.
    max_comm_errors: int = 3          #: consecutive drive failures before aborting.
    use_batched_velocity: bool = False  #: PID_PNT_VEL_CMD(207) — unverified; see docs.
    controller_ramp_s: float | None = 0.2  #: None leaves the controller ramps as-is.
    use_limit_sw: int = 0             #: -1 leave as-is / 0 disable / 1 enable.
    auto_enable: bool = True


@dataclass(frozen=True)
class MecanumConfig:
    """A complete mecanum base configuration."""

    wheels: dict[str, WheelSpec]
    geometry: MecanumGeometry
    version: int = CONFIG_VERSION
    port: str | None = None           #: None -> $MDROBOT_PORT (mdrobot.resolve_port).
    baudrate: int = 19200
    timeout: float = 0.3
    limits: Limits = field(default_factory=Limits)
    runtime: Runtime = field(default_factory=Runtime)

    # --- derived views ---------------------------------------------------------------

    @property
    def wheel_specs(self) -> tuple[WheelSpec, ...]:
        """The four wheels in fixed WHEEL_NAMES order (FL, FR, RL, RR)."""
        return tuple(self.wheels[name] for name in WHEEL_NAMES)

    @property
    def slave_ids(self) -> tuple[int, ...]:
        """The distinct controller ids, ascending. Always exactly two."""
        return tuple(sorted({spec.slave_id for spec in self.wheels.values()}))

    @property
    def signs(self) -> tuple[int, ...]:
        return tuple(spec.sign for spec in self.wheel_specs)

    def channels_of(self, slave_id: int) -> dict[int, str]:
        """{channel: wheel_name} for one controller."""
        return {spec.channel: name for name, spec in self.wheels.items()
                if spec.slave_id == slave_id}

    def with_signs(self, signs) -> "MecanumConfig":
        """Copy with new per-wheel signs, in WHEEL_NAMES order (the floor-test fix)."""
        signs = list(signs)
        if len(signs) != len(WHEEL_NAMES):
            raise ValueError(f"need {len(WHEEL_NAMES)} signs, got {len(signs)}")
        return replace(self, wheels={
            name: replace(self.wheels[name], sign=int(sign))
            for name, sign in zip(WHEEL_NAMES, signs)})

    def with_layout(self, roller_layout: str) -> "MecanumConfig":
        """Copy with a different roller layout (the other floor-test fix)."""
        return replace(self, geometry=self.geometry.with_layout(roller_layout))

    # --- bus budget ------------------------------------------------------------------

    def bus_budget(self) -> tuple[float, float, float]:
        """(write seconds per tick, tick period, duty fraction) at the configured rate.

        Serial bandwidth, not CPU, is what limits a four-wheel base on one 19200 bus.
        Surfacing the number at load time turns "the robot feels laggy" into a
        readable line before anything moves.
        """
        writes = len(self.slave_ids) if self.runtime.use_batched_velocity else len(WHEEL_NAMES)
        cost = writes * WRITE_COST_S
        period = 1.0 / self.runtime.loop_hz
        return cost, period, cost / period

    def bus_budget_note(self) -> str:
        cost, period, duty = self.bus_budget()
        writes = len(self.slave_ids) if self.runtime.use_batched_velocity else len(WHEEL_NAMES)
        note = (f"bus budget: {writes} writes x {WRITE_COST_S * 1e3:.0f} ms = {cost * 1e3:.0f} ms/tick "
                f"vs {period * 1e3:.0f} ms period ({duty * 100:.0f}%)")
        if duty > 1.0:
            note += "  OVERRUN — lower loop_hz or enable batched writes"
        elif duty > 0.8:
            note += "  tight — little headroom for a retry"
        return note

    # --- construction ----------------------------------------------------------------

    @classmethod
    def from_dict(cls, data: dict) -> "MecanumConfig":
        """Build and validate from a plain dict. Raises ConfigError listing every problem."""
        problems: list[str] = []
        if not isinstance(data, dict):
            raise ConfigError(f"config must be a mapping, got {type(data).__name__}")

        _reject_unknown_keys("top level", data, _TOP_LEVEL_KEYS, problems)

        version = data.get("version", CONFIG_VERSION)
        if version != CONFIG_VERSION:
            problems.append(
                f"version: expected {CONFIG_VERSION}, got {version!r} — this file was "
                f"written by a different version of mdrobot_mecanum")

        wheels = _parse_wheels(data.get("wheels"), problems)
        geometry = _parse_geometry(data.get("geometry"), data.get("roller_layout"), problems)
        limits = _parse_section("limits", data.get("limits"), Limits, problems)
        runtime = _parse_section("runtime", data.get("runtime"), Runtime, problems)

        port = data.get("port")
        if port is not None and not isinstance(port, str):
            problems.append(f"port: must be a string or null, got {port!r}")
            port = None
        baudrate = _positive_int("baudrate", data.get("baudrate", 19200), problems, default=19200)
        timeout = _positive_number("timeout", data.get("timeout", 0.3), problems, default=0.3)

        # Build unconditionally — every parse failure above substituted a safe
        # fallback — so the cross-field checks can run too and the operator sees ONE
        # complete list instead of a new error after each fix.
        cfg = cls(wheels=wheels, geometry=geometry, version=CONFIG_VERSION, port=port,
                  baudrate=baudrate, timeout=timeout, limits=limits, runtime=runtime)
        problems.extend(cfg._validation_problems())
        if problems:
            raise ConfigError(_format_problems(problems))
        return cfg

    @classmethod
    def load(cls, path) -> "MecanumConfig":
        """Read and validate a YAML config file."""
        path = Path(path)
        try:
            text = path.read_text()
        except OSError as exc:
            raise ConfigError(f"cannot read {path}: {exc}") from exc
        try:
            data = yaml.safe_load(text)
        except yaml.YAMLError as exc:
            raise ConfigError(f"{path} is not valid YAML: {exc}") from exc
        if data is None:
            raise ConfigError(f"{path} is empty")
        try:
            return cls.from_dict(data)
        except ConfigError as exc:
            raise ConfigError(f"{path}: {exc}") from None

    # --- validation ------------------------------------------------------------------

    def validate(self) -> None:
        """Re-check every cross-field rule. Raises ConfigError listing every problem."""
        problems = self._validation_problems()
        if problems:
            raise ConfigError(_format_problems(problems))

    def _validation_problems(self) -> list[str]:
        """Every cross-field problem, as a list. `from_dict` merges it with its own."""
        problems: list[str] = []

        missing = [name for name in WHEEL_NAMES if name not in self.wheels]
        extra = [name for name in self.wheels if name not in WHEEL_NAMES]
        if missing:
            problems.append(f"wheels: missing {', '.join(missing)}")
        if extra:
            problems.append(f"wheels: unknown wheel name(s) {', '.join(sorted(extra))} "
                            f"— expected exactly {', '.join(WHEEL_NAMES)}")

        if not missing and not extra:
            problems.extend(_topology_problems(self.wheels))

        for name, value, lo in (("max_motor_rpm", self.limits.max_motor_rpm, 1),
                                ("max_comm_errors", self.runtime.max_comm_errors, 1)):
            if not _is_int(value) or value < lo:
                problems.append(f"limits/runtime: {name} must be an integer >= {lo}, got {value!r}")
        if _is_int(self.limits.max_motor_rpm) and self.limits.max_motor_rpm > 32767:
            problems.append(f"limits: max_motor_rpm {self.limits.max_motor_rpm} exceeds the "
                            f"int16 wire limit of 32767")

        for name in ("max_linear_x", "max_linear_y", "max_angular_z",
                     "accel_linear", "decel_linear", "accel_angular", "decel_angular"):
            _positive_number(f"limits: {name}", getattr(self.limits, name), problems)

        # Decel below accel means the robot takes longer to stop than to start. Never
        # what anyone wants, and the failure is silent until an emergency.
        if _both_numbers(self.limits.decel_linear, self.limits.accel_linear) \
                and self.limits.decel_linear < self.limits.accel_linear:
            problems.append(f"limits: decel_linear ({self.limits.decel_linear}) must be >= "
                            f"accel_linear ({self.limits.accel_linear}) — the robot must "
                            f"stop at least as fast as it starts")
        if _both_numbers(self.limits.decel_angular, self.limits.accel_angular) \
                and self.limits.decel_angular < self.limits.accel_angular:
            problems.append(f"limits: decel_angular ({self.limits.decel_angular}) must be >= "
                            f"accel_angular ({self.limits.accel_angular})")

        if _positive_number("runtime: loop_hz", self.runtime.loop_hz, problems) is not None:
            # Below this the motors stop BETWEEN ticks: the controller cuts drive after
            # ~2 s of bus silence, so a slow loop is a correctness bug, not a comfort one.
            slowest = WATCHDOG_SAFETY_FACTOR / COMMAND_WATCHDOG_S
            if self.runtime.loop_hz < slowest:
                problems.append(
                    f"runtime: loop_hz {self.runtime.loop_hz} is too slow — the "
                    f"controller cuts motor drive after ~{COMMAND_WATCHDOG_S:g}s of bus "
                    f"silence, so the loop must run at {slowest:g} Hz or faster")
        if _is_number(self.runtime.idle_timeout) and self.runtime.idle_timeout < 0:
            problems.append(f"runtime: idle_timeout must be >= 0 (0 disables), "
                            f"got {self.runtime.idle_timeout!r}")
        if self.runtime.use_limit_sw not in (-1, 0, 1):
            problems.append(f"runtime: use_limit_sw must be -1, 0 or 1, "
                            f"got {self.runtime.use_limit_sw!r}")
        if self.runtime.controller_ramp_s is not None:
            if not _is_number(self.runtime.controller_ramp_s) or self.runtime.controller_ramp_s < 0:
                problems.append(f"runtime: controller_ramp_s must be >= 0 or null, "
                                f"got {self.runtime.controller_ramp_s!r}")
        for name in ("use_batched_velocity", "auto_enable"):
            if not isinstance(getattr(self.runtime, name), bool):
                problems.append(f"runtime: {name} must be true or false, "
                                f"got {getattr(self.runtime, name)!r}")

        return problems

    # --- serialisation ---------------------------------------------------------------

    def to_dict(self) -> dict:
        """Plain-data view. `from_dict(cfg.to_dict()) == cfg`."""
        return {
            "version": self.version,
            "port": self.port,
            "baudrate": self.baudrate,
            "timeout": self.timeout,
            "wheels": {name: {"slave_id": spec.slave_id, "channel": spec.channel,
                              "sign": spec.sign}
                       for name, spec in ((n, self.wheels[n]) for n in WHEEL_NAMES)},
            "roller_layout": self.geometry.roller_layout,
            "geometry": {"wheel_radius": self.geometry.wheel_radius,
                         "track": self.geometry.track,
                         "wheelbase": self.geometry.wheelbase,
                         "gear_ratio": self.geometry.gear_ratio},
            "limits": {name: getattr(self.limits, name) for name in _fields(Limits)},
            "runtime": {name: getattr(self.runtime, name) for name in _fields(Runtime)},
        }

    def render(self) -> str:
        """The config as commented YAML — what `save()` writes.

        Rendered from a template instead of dumped so the guidance survives. Anyone
        who wants plain data can use `to_dict()` with `yaml.safe_dump`.
        """
        wheels = "\n".join(
            f"  {name + ':':<13}{{slave_id: {spec.slave_id}, channel: {spec.channel}, "
            f"sign: {spec.sign:>2}}}"
            for name, spec in ((n, self.wheels[n]) for n in WHEEL_NAMES))
        provisional = (
            "# STILL UNVERIFIED: computing as \"x\". Run the floor check to settle it.\n"
            if self.geometry.layout_is_provisional else "")
        return _TEMPLATE.format(
            version=self.version,
            port="null" if self.port is None else self.port,
            baudrate=self.baudrate,
            timeout=_num(self.timeout),
            wheels=wheels,
            roller_layout=self.geometry.roller_layout,
            provisional=provisional,
            wheel_radius=_num(self.geometry.wheel_radius),
            track=_num(self.geometry.track),
            wheelbase=_num(self.geometry.wheelbase),
            gear_ratio=_num(self.geometry.gear_ratio),
            max_motor_rpm=self.limits.max_motor_rpm,
            max_linear_x=_num(self.limits.max_linear_x),
            max_linear_y=_num(self.limits.max_linear_y),
            max_angular_z=_num(self.limits.max_angular_z),
            accel_linear=_num(self.limits.accel_linear),
            decel_linear=_num(self.limits.decel_linear),
            accel_angular=_num(self.limits.accel_angular),
            decel_angular=_num(self.limits.decel_angular),
            loop_hz=_num(self.runtime.loop_hz),
            idle_timeout=_num(self.runtime.idle_timeout),
            max_comm_errors=self.runtime.max_comm_errors,
            use_batched_velocity=str(self.runtime.use_batched_velocity).lower(),
            controller_ramp_s=("null" if self.runtime.controller_ramp_s is None
                               else _num(self.runtime.controller_ramp_s)),
            use_limit_sw=self.runtime.use_limit_sw,
            auto_enable=str(self.runtime.auto_enable).lower(),
            bus_budget=self.bus_budget_note(),
        )

    def save(self, path, *, force: bool = False) -> Path:
        """Write the rendered config. Refuses to overwrite unless `force`; keeps a .bak."""
        path = Path(path)
        if path.exists():
            if not force:
                raise ConfigError(
                    f"{path} already exists — pass force=True (or --force) to overwrite")
            backup = path.with_suffix(path.suffix + ".bak")
            backup.write_text(path.read_text())
        path.write_text(self.render())
        return path


# --- parsing helpers -----------------------------------------------------------------

_TOP_LEVEL_KEYS = frozenset({
    "version", "port", "baudrate", "timeout", "wheels", "roller_layout",
    "geometry", "limits", "runtime",
})
_GEOMETRY_KEYS = frozenset({"wheel_radius", "track", "wheelbase", "gear_ratio", "roller_layout"})
_WHEEL_KEYS = frozenset({"slave_id", "channel", "sign"})


def _fields(cls) -> tuple[str, ...]:
    return tuple(cls.__dataclass_fields__)


def _is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _is_int(value) -> bool:
    """A real int. Excludes bool (True == 1) and float (1.0 == 1), both of which slip
    through a plain `value in (1, 2)` membership test."""
    return isinstance(value, int) and not isinstance(value, bool)


def _both_numbers(*values) -> bool:
    return all(_is_number(v) for v in values)


def _num(value) -> str:
    """Render a number for the template without float noise or lost float-ness."""
    if isinstance(value, int) and not isinstance(value, bool):
        return str(value)
    return repr(float(value))


def _format_problems(problems: list[str]) -> str:
    if len(problems) == 1:
        return problems[0]
    return f"{len(problems)} problems:\n  - " + "\n  - ".join(problems)


def _reject_unknown_keys(where: str, data: dict, allowed, problems: list[str]) -> None:
    for key in data:
        if key in allowed:
            continue
        close = difflib.get_close_matches(str(key), sorted(allowed), n=1)
        hint = f" — did you mean {close[0]!r}?" if close else ""
        problems.append(f"{where}: unknown key {key!r}{hint}")


def _positive_number(where: str, value, problems: list[str], default=None):
    if not _is_number(value) or not math.isfinite(value) or value <= 0:
        problems.append(f"{where}: must be a positive number, got {value!r}")
        return default
    return value


def _positive_int(where: str, value, problems: list[str], default=None):
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        problems.append(f"{where}: must be a positive integer, got {value!r}")
        return default
    return value


def _parse_wheels(raw, problems: list[str]) -> dict[str, WheelSpec]:
    if raw is None:
        problems.append("wheels: missing — there is no safe default; run the identify wizard")
        return {}
    if not isinstance(raw, dict):
        problems.append(f"wheels: must be a mapping of wheel name to "
                        f"{{slave_id, channel, sign}}, got {type(raw).__name__}")
        return {}

    _reject_unknown_keys("wheels", raw, WHEEL_NAMES, problems)

    wheels: dict[str, WheelSpec] = {}
    for name in WHEEL_NAMES:
        spec = raw.get(name)
        if spec is None:
            problems.append(f"wheels: missing {name}")
            continue
        if not isinstance(spec, dict):
            problems.append(f"wheels/{name}: must be a mapping, got {type(spec).__name__}")
            continue
        _reject_unknown_keys(f"wheels/{name}", spec, _WHEEL_KEYS, problems)

        slave_id = spec.get("slave_id")
        channel = spec.get("channel")
        sign = spec.get("sign", 1)
        ok = True
        # `_is_int` rather than `in (1, 2)`: 1.0 == 1 and True == 1 in Python, so a
        # membership test alone lets a float or a boolean through into a wire address.
        if not _is_int(slave_id) or not 1 <= slave_id <= 247:
            problems.append(f"wheels/{name}: slave_id must be an integer 1..247, got {slave_id!r}")
            ok = False
        if not _is_int(channel) or channel not in (1, 2):
            problems.append(f"wheels/{name}: channel must be 1 or 2, got {channel!r}")
            ok = False
        if not _is_int(sign) or sign not in (1, -1):
            problems.append(f"wheels/{name}: sign must be 1 or -1, got {sign!r}")
            ok = False
        if ok:
            wheels[name] = WheelSpec(slave_id=slave_id, channel=channel, sign=sign)
    return wheels


def _topology_problems(wheels: dict[str, WheelSpec]) -> list[str]:
    """Enforce the only supported shape: two controllers x two channels, no duplicates."""
    problems: list[str] = []

    seen: dict[tuple[int, int], list[str]] = {}
    for name in WHEEL_NAMES:
        seen.setdefault(wheels[name].address, []).append(name)
    for address, names in seen.items():
        if len(names) > 1:
            problems.append(
                f"wheels: {' and '.join(names)} both map to slave_id {address[0]} "
                f"channel {address[1]} — every wheel needs its own motor output")

    ids = sorted({spec.slave_id for spec in wheels.values()})
    if len(ids) != 2:
        problems.append(
            f"wheels: expected exactly 2 distinct slave ids (two dual-channel "
            f"controllers), got {len(ids)}: {ids}. This package supports only that "
            f"topology; re-ID a controller with PID_ID(133) if they clash")
    else:
        for slave_id in ids:
            channels = sorted(spec.channel for spec in wheels.values()
                              if spec.slave_id == slave_id)
            if channels != [1, 2]:
                problems.append(
                    f"wheels: slave id {slave_id} must drive channels [1, 2], got {channels}")
    return problems


def _parse_geometry(raw, top_level_layout, problems: list[str]) -> MecanumGeometry:
    fallback = MecanumGeometry(wheel_radius=0.05, track=0.30, wheelbase=0.30)
    layout = top_level_layout if top_level_layout is not None else PROVISIONAL_LAYOUT
    if layout not in ROLLER_LAYOUTS:
        problems.append(f"roller_layout: must be one of {list(ROLLER_LAYOUTS)}, got {layout!r}")
        layout = PROVISIONAL_LAYOUT

    if raw is None:
        problems.append("geometry: missing — there is no safe default; measure the robot")
        return fallback
    if not isinstance(raw, dict):
        problems.append(f"geometry: must be a mapping, got {type(raw).__name__}")
        return fallback

    _reject_unknown_keys("geometry", raw, _GEOMETRY_KEYS, problems)
    # roller_layout is a top-level key; accept it nested too, since that is where a
    # reader naturally looks for it.
    if "roller_layout" in raw:
        nested = raw["roller_layout"]
        if nested not in ROLLER_LAYOUTS:
            problems.append(f"geometry/roller_layout: must be one of "
                            f"{list(ROLLER_LAYOUTS)}, got {nested!r}")
        elif top_level_layout is not None and nested != top_level_layout:
            problems.append(f"roller_layout given twice with different values: "
                            f"{top_level_layout!r} at top level, {nested!r} under geometry")
        else:
            layout = nested

    values = {}
    for name, default in (("wheel_radius", None), ("track", None),
                          ("wheelbase", None), ("gear_ratio", 1.0)):
        value = raw.get(name, default)
        if value is None:
            problems.append(f"geometry: missing {name}")
            values[name] = getattr(fallback, name)
        else:
            values[name] = _positive_number(f"geometry: {name}", value, problems,
                                            default=getattr(fallback, name))
    try:
        return MecanumGeometry(roller_layout=layout, **values)
    except ValueError as exc:      # pragma: no cover - the checks above cover these
        problems.append(str(exc))
        return fallback


def _parse_section(where: str, raw, cls, problems: list[str]):
    """Build a defaults-carrying dataclass, rejecting unknown keys."""
    if raw is None:
        return cls()
    if not isinstance(raw, dict):
        problems.append(f"{where}: must be a mapping, got {type(raw).__name__}")
        return cls()
    _reject_unknown_keys(where, raw, _fields(cls), problems)
    known = {k: v for k, v in raw.items() if k in _fields(cls)}
    return cls(**known)


_TEMPLATE = """\
# Mecanum base configuration — 4 wheels on two dual-channel MDROBOT controllers.
# Written by `mdrobot_mecanum.identify`. Safe to hand-edit: unknown keys are an
# error, missing keys fall back to defaults.
#
# {bus_budget}
version: {version}

port: {port}                # null -> $MDROBOT_PORT
baudrate: {baudrate}
timeout: {timeout}

# Which motor output drives which wheel, and which way it turns.
# sign = +1 when a POSITIVE rpm command drives the robot FORWARD.
# The four (slave_id, channel) pairs must cover two distinct ids x channels 1 and 2.
wheels:
{wheels}

# Flips the vy column of the kinematics ONLY: a wrong value makes strafe go the wrong
# way and changes NOTHING else. It cannot be decided by eye — the roller you see from
# above projects to the mirror of the ground-contact roller that does the work. Only a
# floor test settles it.  x | o | unknown
{provisional}roller_layout: {roller_layout}

geometry:
  wheel_radius: {wheel_radius}    # m
  track: {track}           # m  LEFT-RIGHT wheel centre distance
  wheelbase: {wheelbase}       # m  FRONT-REAR wheel centre distance
  gear_ratio: {gear_ratio}       # MOTOR revolutions per WHEEL revolution (>1 = reduction)

limits:
  # THE hard cap. Enforced at the wheel by a proportional clamp that scales all four
  # speeds together, so a clamped straight line stays straight.
  max_motor_rpm: {max_motor_rpm}
  # Teleop targets — what the operator can ask for, not what the hardware accepts.
  max_linear_x: {max_linear_x}      # m/s
  max_linear_y: {max_linear_y}      # m/s
  max_angular_z: {max_angular_z}     # rad/s
  # Software twist ramp. decel must be >= accel: the robot has to stop at least as
  # fast as it starts.
  accel_linear: {accel_linear}      # m/s^2
  decel_linear: {decel_linear}      # m/s^2
  accel_angular: {accel_angular}     # rad/s^2
  decel_angular: {decel_angular}     # rad/s^2

runtime:
  loop_hz: {loop_hz}
  idle_timeout: {idle_timeout}      # s; 0 disables. Guards a dead terminal, not a released key.
  max_comm_errors: {max_comm_errors}       # consecutive drive failures before aborting
  use_batched_velocity: {use_batched_velocity}   # PID_PNT_VEL_CMD(207): halves the writes, but the
                              # word order has never been verified on a wire. Only set
                              # true after the batched probe passes on this hardware.
  controller_ramp_s: {controller_ramp_s}     # slow-start/down on all 4 channels; null leaves as-is.
                              # Keep it SHORT: the four channels ramp independently and
                              # would break the wheel-speed ratio. The real shaping is
                              # the software twist ramp above.
  use_limit_sw: {use_limit_sw}            # -1 leave as-is / 0 disable / 1 enable
  auto_enable: {auto_enable}
"""
