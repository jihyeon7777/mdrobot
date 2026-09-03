"""Keyboard teleop for a mecanum base. Latched keys, software ramping, hard stops.

Input model: **latched**, like `teleop_twist_keyboard`. A tty has no key-release
event, only bytes, so hold-to-drive would have to be emulated as "any byte within
T ms keeps going" — and T is trapped between two OS constants. Auto-repeat delivers
one byte, waits ~500 ms, then repeats at ~25-30 Hz. Pick `T < 500 ms` and a held key
stutters; pick `T > 500 ms` and the robot keeps driving ~600 ms after release, which
is worse, because the operator's mental model says "I let go, so it stopped". Latched
at least tells the truth about what it is doing.

Safety comes from four properties instead of a short timeout:

1. **Space is a hard stop** — target and ramp both to zero, written immediately.
2. **Any unmapped key is a soft stop.** A startled operator mashing keys stops.
3. **Ctrl-C / Ctrl-\\ / a lone ESC** stop, cut torque, and exit.
4. An idle watchdog, which guards a dead terminal rather than a released key.

Two hardware facts shape the loop, both measured (see `base.COMMAND_WATCHDOG_S`):

- A velocity command is **not a latch** — the controller cuts drive after about two
  seconds of bus silence. So every tick writes, unchanged or not, zero or not. That
  also makes a yanked cable *detectable* rather than silently ignored.
- The ramp is applied to the **twist**, not to the wheels. The controllers' own
  slow-start ramps run independently per channel, so during any acceleration they
  break the wheel-speed ratio and the robot curves while speeding up — worst on a
  strafe, which needs the ratio exactly. The kinematics is linear, so ramping the
  twist keeps the ratio exact at every instant.
"""

from __future__ import annotations

import argparse
import os
import select
import signal
import sys
import termios
import time
import tty
from dataclasses import dataclass, replace

from mdrobot import MdrobotError

from .base import DriveError, MecanumBase
from .config import ConfigError, MecanumConfig
from .kinematics import WHEEL_ABBREV

#: (vx, vy) pad. Each key sets translation and leaves rotation alone.
TRANSLATION_KEYS = {
    "u": (+1, +1), "i": (+1, 0), "o": (+1, -1),
    "j": (0, +1), "k": (0, 0), "l": (0, -1),
    "m": (-1, +1), ",": (-1, 0), ".": (-1, -1),
    # WASD aliases. a/d are STRAFE, not turn — the mecanum-native choice.
    "w": (+1, 0), "s": (-1, 0), "a": (0, +1), "d": (0, -1),
}

#: Rotation keys. Each sets wz and leaves translation alone.
ROTATION_KEYS = {"q": +1, "e": -1, "r": 0}

#: Escape sequences for the arrow keys: (vx, vy, wz) multipliers, None = leave alone.
ARROW_KEYS = {
    "A": ("translate", (+1, 0)),   # up
    "B": ("translate", (-1, 0)),   # down
    "D": ("rotate", +1),           # left
    "C": ("rotate", -1),           # right
}

#: Speed-scale keys: (linear delta, angular delta) as fractions.
SCALE_KEYS = {"z": (-0.1, 0.0), "c": (+0.1, 0.0),
              "v": (0.0, -0.1), "b": (0.0, +0.1)}
PRESET_KEYS = {"1": 0.2, "2": 0.4, "3": 0.6, "4": 0.8, "5": 1.0}

#: Scale never goes below this: at zero the robot would silently refuse to move.
MIN_SCALE = 0.1

HARD_STOP_KEYS = {" "}
QUIT_KEYS = {"\x03", "\x1c"}      # Ctrl-C, Ctrl-backslash

#: Seconds without any keypress before the idle countdown is shown.
COUNTDOWN_LEAD_S = 3.0

#: Smallest key-polling window, used when a tick has already overrun its period.
MIN_POLL_S = 0.005


@dataclass(frozen=True)
class Twist:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.vx, self.vy, self.wz)

    @property
    def is_zero(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


def ramp_toward(current: float, target: float, accel: float, decel: float,
                dt: float) -> float:
    """Move `current` toward `target`, using `accel` when growing, `decel` when shrinking.

    "Growing" means the magnitude is increasing OR the sign is reversing, since passing
    through zero to the other side is a stop followed by an acceleration.
    """
    if current == target:
        return target
    growing = abs(target) > abs(current) and (current == 0.0 or
                                              (target > 0) == (current > 0))
    rate = accel if growing else decel
    step = rate * dt
    if abs(target - current) <= step:
        return target
    return current + step * (1.0 if target > current else -1.0)


class TeleopState:
    """Latched target, ramped command, and the speed scales. No I/O — unit-testable."""

    def __init__(self, config: MecanumConfig, *, linear_scale: float = 1.0,
                 angular_scale: float = 1.0):
        self.config = config
        self.target = Twist()
        self.command = Twist()
        self.linear_scale = self._clamp_scale(linear_scale)
        self.angular_scale = self._clamp_scale(angular_scale)
        #: Set by keys that mean "stop right now", cleared once acted on.
        self.hard_stop = False
        self.quit = False
        #: Direction multipliers, kept so a scale change re-applies immediately.
        self._translate = (0, 0)
        self._rotate = 0

    @staticmethod
    def _clamp_scale(value: float) -> float:
        return max(MIN_SCALE, min(1.0, value))

    # --- target --------------------------------------------------------------------

    def _recompute_target(self) -> None:
        limits = self.config.limits
        fx, fy = self._translate
        self.target = Twist(
            vx=fx * limits.max_linear_x * self.linear_scale,
            vy=fy * limits.max_linear_y * self.linear_scale,
            wz=self._rotate * limits.max_angular_z * self.angular_scale,
        )

    def set_translation(self, fx: int, fy: int) -> None:
        self._translate = (fx, fy)
        self._recompute_target()

    def set_rotation(self, fz: int) -> None:
        self._rotate = fz
        self._recompute_target()

    def stop_soft(self) -> None:
        """Zero the target; the ramp brings the robot down at the decel rate."""
        self._translate, self._rotate = (0, 0), 0
        self._recompute_target()

    def stop_hard(self) -> None:
        """Zero the target AND the ramp state — the command goes out this instant."""
        self.stop_soft()
        self.command = Twist()
        self.hard_stop = True

    # --- keys ----------------------------------------------------------------------

    def handle_key(self, key: str) -> None:
        """Apply one keystroke. Anything unrecognised is a soft stop, deliberately."""
        if key in QUIT_KEYS:
            self.quit = True
            self.stop_hard()
        elif key in HARD_STOP_KEYS:
            self.stop_hard()
        elif key in TRANSLATION_KEYS:
            self.set_translation(*TRANSLATION_KEYS[key])
        elif key in ROTATION_KEYS:
            self.set_rotation(ROTATION_KEYS[key])
        elif key in SCALE_KEYS:
            dl, da = SCALE_KEYS[key]
            self.linear_scale = self._clamp_scale(self.linear_scale + dl)
            self.angular_scale = self._clamp_scale(self.angular_scale + da)
            self._recompute_target()
        elif key in PRESET_KEYS:
            self.linear_scale = self.angular_scale = PRESET_KEYS[key]
            self._recompute_target()
        else:
            # An unmapped key stops everything: a startled operator mashing the
            # keyboard must not be a no-op.
            self.stop_soft()

    def handle_arrow(self, code: str) -> None:
        action = ARROW_KEYS.get(code)
        if action is None:
            self.stop_soft()
            return
        kind, value = action
        if kind == "translate":
            self.set_translation(*value)
        else:
            self.set_rotation(value)

    # --- ramp ----------------------------------------------------------------------

    def step(self, dt: float) -> Twist:
        """Advance the ramped command toward the target by `dt` seconds."""
        limits = self.config.limits
        self.command = Twist(
            vx=ramp_toward(self.command.vx, self.target.vx,
                           limits.accel_linear, limits.decel_linear, dt),
            vy=ramp_toward(self.command.vy, self.target.vy,
                           limits.accel_linear, limits.decel_linear, dt),
            wz=ramp_toward(self.command.wz, self.target.wz,
                           limits.accel_angular, limits.decel_angular, dt),
        )
        return self.command


class KeyReader:
    """Non-blocking keystrokes from a tty, with a small arrow-key parser.

    `tty.setcbreak`, not `setraw`: cbreak leaves ISIG enabled, so Ctrl-C keeps raising
    KeyboardInterrupt through the normal path and the existing try/finally shutdown
    just works. setraw would mean hand-decoding 0x03, and a bug there strands a
    driving robot.
    """

    #: Time allowed for the rest of an escape sequence before a lone ESC means quit.
    ESCAPE_WINDOW_S = 0.02

    def __init__(self, stream=None):
        self.stream = stream or sys.stdin
        self.fd = self.stream.fileno()
        self._saved = None

    def start(self) -> "KeyReader":
        self._saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __enter__(self) -> "KeyReader":
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.restore()

    def restore(self) -> None:
        if self._saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._saved)
            self._saved = None

    def _read_ready(self, timeout: float) -> str | None:
        if select.select([self.stream], [], [], timeout)[0]:
            return self.stream.read(1)
        return None

    def poll(self, timeout: float) -> tuple[str, str] | None:
        """Return ("key", ch) or ("arrow", code) or ("quit", "") or None on timeout."""
        char = self._read_ready(timeout)
        if char is None or char == "":
            return None
        if char != "\x1b":
            return ("key", char)
        # ESC: either an arrow sequence or, with nothing following, a quit.
        if self._read_ready(self.ESCAPE_WINDOW_S) != "[":
            return ("quit", "")
        code = self._read_ready(self.ESCAPE_WINDOW_S)
        return ("arrow", code) if code else ("quit", "")


def render(state: TeleopState, result, comm_errors: int, period_ms: float,
           idle_left: float | None) -> str:
    """The status block, redrawn in place."""
    target, command = state.target, state.command
    lines = [
        f"target  vx {target.vx:+.3f}  vy {target.vy:+.3f}  wz {target.wz:+.3f}   m/s, rad/s",
        f"ramped  vx {command.vx:+.3f}  vy {command.vy:+.3f}  wz {command.wz:+.3f}",
    ]
    if result is not None:
        wheels = "  ".join(f"{n}{v:+5d}" for n, v in zip(WHEEL_ABBREV, result.wheel_rpm))
        limited = (f"   \x1b[7m LIMITED {result.scale * 100:3.0f}% \x1b[0m"
                   if result.clamped else "")
        lines.append(f"wheels  {wheels}  rpm{limited}")
    lines.append(
        f"scale   linear {state.linear_scale * 100:3.0f}%  angular "
        f"{state.angular_scale * 100:3.0f}%   loop {period_ms:5.1f} ms"
        + (f"   comm errors {comm_errors}" if comm_errors else "")
        + (f"   \x1b[7m IDLE STOP IN {idle_left:.0f}s \x1b[0m"
           if idle_left is not None else ""))
    return "\n".join(lines)


KEY_LEGEND = """\
  translate (vx, vy) — rotation is left alone
      u i o      u=(+x,+y)  i=(+x,0)  o=(+x,-y)
      j k l      j=(0,+y)   k=stop    l=(0,-y)
      m , .      m=(-x,+y)  ,=(-x,0)  .=(-x,-y)
      w/s = forward/back    a/d = STRAFE left/right
  rotate (wz) — translation is left alone
      q = left (CCW)    e = right (CW)    r = stop rotating
  arrows: up/down = +/-vx    left/right = +/-wz
  speed:  z/c linear -/+10%   v/b angular -/+10%   1..5 = 20/40/60/80/100%
  SPACE = HARD STOP    any other key = soft stop    Ctrl-C / ESC = quit\
"""


def run(base: MecanumBase, reader: KeyReader, *, printer=print, scale: float = 1.0,
        max_ticks: int | None = None) -> int:
    """The control loop. Returns a process exit code."""
    config = base.config
    state = TeleopState(config, linear_scale=scale, angular_scale=scale)
    period = 1.0 / config.runtime.loop_hz
    idle_timeout = config.runtime.idle_timeout
    max_errors = config.runtime.max_comm_errors

    comm_errors = 0
    last_key_at = time.monotonic()
    last_tick = time.monotonic()
    next_tick = time.monotonic()
    measured_ms = period * 1e3
    ticks = 0
    printer("\n" + KEY_LEGEND)      # a message: it stays on screen above the status

    while True:
        if max_ticks is not None and ticks >= max_ticks:
            return 0
        ticks += 1

        now = time.monotonic()
        dt = max(1e-3, now - last_tick)
        last_tick = now
        measured_ms = 0.8 * measured_ms + 0.2 * dt * 1e3

        # The idle watchdog guards a dead terminal or an absent operator, not a
        # released key — latched teleop legitimately drives for a long time on one
        # keypress, which is why the default is 10 s and not a fraction of a second.
        idle_for = now - last_key_at
        idle_left = None
        if idle_timeout > 0 and not state.target.is_zero:
            if idle_for >= idle_timeout:
                state.stop_soft()
                printer("\nno input — idle stop")
                last_key_at = now
            elif idle_for >= idle_timeout - COUNTDOWN_LEAD_S:
                idle_left = idle_timeout - idle_for

        command = state.command if state.hard_stop else state.step(dt)
        state.hard_stop = False

        # Write EVERY tick, unchanged or zero. The controller cuts drive after ~2 s of
        # bus silence, so re-assertion is what keeps the robot moving at all — and it
        # is what turns a yanked cable into a visible error instead of silence.
        result = None
        try:
            result = base.drive(*command.as_tuple())
            comm_errors = 0
        except DriveError as exc:
            comm_errors += 1
            printer(f"\ncomm error {comm_errors}/{max_errors}: {exc}")
            if comm_errors >= max_errors:
                printer("too many consecutive failures — stopping")
                return 1
        except MdrobotError as exc:      # pragma: no cover - defensive
            printer(f"\nunexpected error: {type(exc).__name__}: {exc}")
            return 1

        printer(render(state, result, comm_errors, measured_ms, idle_left))

        if state.quit and command.is_zero:
            return 0

        # Wait out the REST of the period while draining keystrokes. Polling for a
        # whole period and only then doing ~48 ms of bus writes would make the real
        # period period+work — a loop_hz of 10 would quietly run at 8, and raising it
        # to 15 would quietly give 10. Do the work first, then spend what is left.
        next_tick += period
        # An overrun must not leave a zero-length poll window: the operator would lose
        # key control of a moving robot exactly when the loop is already struggling.
        next_tick = max(next_tick, time.monotonic() + MIN_POLL_S)
        while True:
            remaining = next_tick - time.monotonic()
            if remaining <= 0:
                break
            event = reader.poll(remaining)
            if event is None:
                break
            kind, value = event
            last_key_at = time.monotonic()
            if kind == "quit":
                state.quit = True
                state.stop_hard()
            elif kind == "arrow":
                state.handle_arrow(value)
            else:
                state.handle_key(value)
            # A stop must not wait out the rest of the period. Everything else can.
            if state.hard_stop or state.quit:
                break


def _install_sigterm_handler() -> None:
    """Make `kill` unwind through the normal finally blocks so the motors stop."""
    def handler(signum, frame):
        raise SystemExit(128 + signum)
    try:
        signal.signal(signal.SIGTERM, handler)
    except ValueError:               # pragma: no cover - not on the main thread
        pass


class _InPlacePrinter:
    """Redraw the status block in place instead of scrolling the terminal.

    Text beginning with a newline is a MESSAGE: it scrolls, stays on screen, and the
    next status block starts fresh below it. Everything else is the status block,
    redrawn over itself and throttled so the terminal does not tear at the loop rate.
    """

    def __init__(self, min_interval: float = 0.2):
        self._last_lines = 0
        self._last_at = 0.0
        self._min_interval = min_interval

    def __call__(self, text: str = "") -> None:
        if text.startswith("\n"):
            self._message(text)
            return

        now = time.monotonic()
        if self._last_lines and now - self._last_at < self._min_interval:
            return
        lines = text.split("\n")
        if self._last_lines:
            sys.stdout.write(f"\x1b[{self._last_lines}A")
        for line in lines:
            sys.stdout.write("\r\x1b[K" + line + "\n")
        # A shorter block than last time would leave the tail of the old one on
        # screen, so wipe the leftovers and step back up over them.
        leftover = self._last_lines - len(lines)
        if leftover > 0:
            for _ in range(leftover):
                sys.stdout.write("\r\x1b[K\n")
            sys.stdout.write(f"\x1b[{leftover}A")
        sys.stdout.flush()
        self._last_lines = len(lines)
        self._last_at = now

    def _message(self, text: str) -> None:
        """Print something that must stay on screen, and let the status restart below."""
        if self._last_lines:
            sys.stdout.write("\n" * 0)   # the status block keeps its final state
        sys.stdout.write(text.rstrip("\n") + "\n")
        sys.stdout.flush()
        self._last_lines = 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Keyboard teleop for a mecanum base (the motors WILL turn).")
    ap.add_argument("--config", default="mecanum.yaml",
                    help="path to mecanum.yaml (default: ./mecanum.yaml)")
    ap.add_argument("--port", default=None, help="override the port in the config")
    ap.add_argument("--scale", type=float, default=20.0, metavar="PCT",
                    help="starting speed scale in percent (default 20)")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = MecanumConfig.load(args.config)
    except ConfigError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    if args.port:
        config = replace(config, port=args.port)

    print(config.bus_budget_note())
    if config.geometry.layout_is_provisional:
        print("roller_layout is UNVERIFIED: strafe may go the opposite way to the keys.")
    if not os.isatty(sys.stdin.fileno()):
        print("error: teleop needs a terminal (stdin is not a tty)", file=sys.stderr)
        return 2

    _install_sigterm_handler()
    try:
        base = MecanumBase.from_config(config)
    except (ValueError, OSError) as exc:
        print(f"error: cannot open the bus: {exc}", file=sys.stderr)
        return 2

    scale = max(MIN_SCALE, min(1.0, args.scale / 100.0))
    printer = _InPlacePrinter()
    reader = KeyReader()
    code = 1
    try:
        if config.runtime.auto_enable:
            print("arming controllers...")
            base.enable()
        print(f"starting at {scale * 100:.0f}% speed")
        reader.start()
        try:
            code = run(base, reader, printer=printer, scale=scale)
        except KeyboardInterrupt:
            code = 0
        finally:
            # Motors first, THEN the terminal. Restoring the terminal first would hand
            # Ctrl-C back to the operator while the stop is still retrying, and an
            # impatient Ctrl-C there would abort the shutdown of a moving robot.
            stopped = base.shutdown()
            reader.restore()
            if not stopped:
                print("\n" + "!" * 60)
                print("COULD NOT STOP THE MOTORS — CUT POWER")
                print("!" * 60)
                code = 1
    finally:
        reader.restore()      # idempotent; covers a failure before the loop started
        base.close()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
