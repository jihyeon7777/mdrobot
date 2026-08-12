"""Bring-up and identification for a mecanum base: scan, preflight, spin each wheel.

Run this before anything else drives the robot, and run it with the **wheels off the
ground**.

Phases, safest first — each one stops where it is if you pass the matching flag:

  --scan-only    inspection only. Version, voltage, status and the four settings that
                 decide whether serial drive works at all. No configuration writes.
  --preflight    the above plus conditional configuration writes. Every write is
                 gated on a read-back showing the value is wrong, so a correctly
                 configured controller is left completely untouched. Nothing turns.
  (default)      the above plus spinning ONE motor output at a time so you can see
                 which wheel each one drives and which way it goes.

Every mode issues a stop and a torque-off on the way out, in every mode and on every
exit path including a crash. That is deliberate: a motor left spinning by an earlier
run should be stopped by the next one, so the guarantee here is "nothing STARTS
turning", not "nothing is written".

There is no wheel map yet the first time you run this, so the spin phase addresses
motor outputs directly by (slave_id, channel) and does not apply any per-wheel sign —
the sign is one of the things it is measuring.
"""

from __future__ import annotations

import argparse
import sys
import time

from mdrobot import MdrobotError, registers as reg

from . import verify
from .base import MecanumBase, borrow_single
from .config import ConfigError, MecanumConfig
from .kinematics import WHEEL_NAMES

#: Motor outputs in the order they are exercised: controller 1 then controller 2.
ADDRESSES = ((1, 1), (1, 2), (2, 1), (2, 2))

#: Conservative identification defaults: slow enough to watch safely, long enough to
#: leave a clear signal. Roughly 0.8 s of every spin is start delay, and a 4-pole hall
#: produces only a few counts per second at 30 rpm, so a short spin can finish with a
#: delta of 1 — far too thin to call a direction from.
DEFAULT_RPM = 30
DEFAULT_SPIN_S = 5.0

#: Below this many counts the direction is not trustworthy — say so rather than guess.
WEAK_SIGNAL_COUNTS = 3

#: Typo guard on --rpm. These are MOTOR-shaft revolutions: behind a reduction gearbox
#: the wheel turns far slower, which is exactly why identification often needs a few
#: hundred rpm to be visible at all. Going past this needs an explicit --max-rpm, so a
#: slipped digit cannot become a command.
RPM_CEILING = 600


def bootstrap_config(port: str | None, *, baudrate: int = 19200,
                     max_motor_rpm: int = 60, **_ignored) -> MecanumConfig:
    """A placeholder config for the very first run, before any wheel map exists.

    The wheel mapping here is a guess and the geometry is a placeholder. That is safe
    for this module only because the identification path uses `spin_one`, which
    addresses a motor output directly and applies no sign — nothing it does depends on
    the mapping being right. Do NOT drive a robot with this config.
    """
    return MecanumConfig.from_dict({
        "version": 1,
        "port": port,
        "baudrate": baudrate,
        "wheels": {name: {"slave_id": sid, "channel": ch, "sign": 1}
                   for name, (sid, ch) in zip(WHEEL_NAMES, ADDRESSES)},
        "roller_layout": "unknown",
        "geometry": {"wheel_radius": 0.05, "track": 0.30, "wheelbase": 0.30},
        "limits": {"max_motor_rpm": max_motor_rpm},
        # Leave the controller ramps alone during identification: a ramp would blur
        # the start of the motion that is being watched.
        "runtime": {"controller_ramp_s": None},
    })


def confirm(question: str, *, assume_yes: bool = False) -> bool:
    if assume_yes:
        print(f"{question} [auto-yes]")
        return True
    try:
        return input(f"{question} [y/N] ").strip().lower() in ("y", "yes")
    except EOFError:
        return False


# --- phase 0: read-only ---------------------------------------------------------------

def report_settings(base: MecanumBase) -> None:
    """Print, per controller, everything that matters before a first drive. No writes."""
    for sid, info in base.identify().items():
        version, voltage = info["version"], info["voltage"]
        active = ", ".join(info["status"].active) or "none"
        print(f"\n  controller id {sid}")
        print(f"    version    : DL={version} (~v{version // 10}.{version % 10})")
        print(f"    voltage    : {voltage:.1f} V")
        print(f"    status     : {active}")
        client = base.driver(sid).client
        ppr = client.read_register(reg.PID_ENC_PPR)
        print(f"    ENC_PPR    : {ppr}"
              + ("  (hall closed loop)" if ppr == 0
                 else "  ENCODER MODE — needs a wired encoder, else it alarms"))
        print(f"    LIMIT_SW   : {client.read_register(reg.PID_USE_LIMIT_SW)} / "
              f"{client.read_register(reg.PID_USE_LIMIT_SW2)}  (motor 1 / motor 2)")
        print(f"    MAX_RPM    : {client.read_register(reg.PID_MAX_RPM)} rpm")


# --- phase 2: spin one output at a time ------------------------------------------------

def spin_and_measure(base: MecanumBase, slave_id: int, channel: int,
                     rpm: int, seconds: float) -> tuple[list[int], int]:
    """Spin one motor output for `seconds`. Returns (speed samples, position delta).

    The command is RE-SENT every sample. A velocity command is not a latch: the
    controller cuts drive after about two seconds of bus silence, so a single command
    followed by a wait produces a twitch rather than sustained motion.

    Position delta is the measurement that matters. The instantaneous speed register
    is badly quantised at identification speeds — a 4-pole hall gives only a handful of
    edges per second at 30 rpm, so single samples swing wildly — while the position
    counter accumulates and cannot lie about which way the motor went.
    """
    samples: list[int] = []
    start = _channel_position(base, slave_id, channel)
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            base.spin_one(slave_id, channel, rpm)
            time.sleep(0.1)
            monitor = base.driver(slave_id).read_monitor()
            samples.append((monitor.motor1 if channel == 1 else monitor.motor2).speed_rpm)
    finally:
        base.driver(slave_id).stop_channel(channel)
    return samples, _channel_position(base, slave_id, channel) - start


def _channel_position(base: MecanumBase, slave_id: int, channel: int) -> int:
    monitor = base.driver(slave_id).read_monitor()
    return (monitor.motor1 if channel == 1 else monitor.motor2).position


def parse_address(text: str) -> tuple[int, int]:
    """Parse a `slave_id:channel` argument, e.g. "2:1"."""
    try:
        slave_id, channel = (int(part) for part in text.split(":", 1))
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"expected slave_id:channel (e.g. 1:2), got {text!r}") from None
    if (slave_id, channel) not in ADDRESSES:
        raise argparse.ArgumentTypeError(
            f"{text!r} is not one of the four motor outputs "
            f"{', '.join(f'{s}:{c}' for s, c in ADDRESSES)}")
    return (slave_id, channel)


def spin_each_output(base: MecanumBase, rpm: int, seconds: float,
                     *, assume_yes: bool = False,
                     only: tuple[int, int] | None = None) -> dict[tuple[int, int], int]:
    """Turn each motor output in turn so the operator can see which wheel it drives.

    Returns {(slave_id, channel): position delta}. Sign of the delta is the motor's
    direction for a positive command.
    """
    targets = [only] if only else list(ADDRESSES)
    results: dict[tuple[int, int], int] = {}
    for index, (slave_id, channel) in enumerate(targets, start=1):
        print(f"\n[{index}/{len(targets)}] controller {slave_id}, channel {channel} "
              f"-> {rpm:+d} rpm for {seconds:.1f}s")
        if not confirm("    spin it?", assume_yes=assume_yes):
            print("    skipped")
            continue
        samples, delta = spin_and_measure(base, slave_id, channel, rpm, seconds)
        results[(slave_id, channel)] = delta
        if delta == 0:
            verdict = "NOTHING MOVED — dead or un-armed channel"
        elif abs(delta) < WEAK_SIGNAL_COUNTS:
            verdict = (f"only {abs(delta)} count(s) — too weak to trust. Re-run with "
                       f"--spin {seconds * 2:.0f}")
        else:
            verdict = (f"turns {'forward' if delta > 0 else 'in reverse'} "
                       f"for a +rpm command")
        print(f"    position: {delta:+d} counts   speed samples: "
              f"{min(samples)}..{max(samples)} rpm (quantised, indicative only)")
        print(f"    -> {verdict}")
        time.sleep(0.5)
    return results


def summarise(results: dict[tuple[int, int], int], commanded: int) -> int:
    """Print a summary and return a process exit code."""
    print("\n" + "=" * 70)
    print("motor output     position delta   verdict")
    dead, weak = [], []
    for slave_id, channel in ADDRESSES:
        delta = results.get((slave_id, channel))
        if delta is None:
            print(f"  id {slave_id} ch {channel}            --         skipped")
            continue
        if delta == 0:
            verdict = "NO MOTION"
            dead.append((slave_id, channel))
        elif abs(delta) < WEAK_SIGNAL_COUNTS:
            verdict = "TOO WEAK to call a direction"
            weak.append((slave_id, channel))
        elif (delta > 0) != (commanded > 0):
            verdict = "turns the other way — that is just a per-wheel sign"
        else:
            verdict = "ok"
        print(f"  id {slave_id} ch {channel}       {delta:+8d} counts   {verdict}")
    print("=" * 70)

    if weak and not dead:
        print(f"\n{len(weak)} channel(s) barely moved. Re-run with a longer --spin so the")
        print("position counter has time to give an unambiguous direction.")
        return 1
    if dead:
        print("\nSome channels did not move. Check, in this order:")
        print("  - supply voltage (logic power alone will not turn a motor)")
        print("  - ENC_PPR: must be 0 unless an encoder is physically wired")
        print("  - USE_LIMIT_SW / USE_LIMIT_SW2: usually must be 0 for serial drive")
        print("  - motor and hall wiring for that channel")
        return 1
    print("\nAll motor outputs drive. Next: note which wheel each one turned, then run")
    print("the wheel-map wizard to write mecanum.yaml.")
    return 0


# --- entry point -----------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description="Mecanum bring-up: scan, preflight, and spin each motor output.")
    ap.add_argument("--port", default=None, help="serial port; default $MDROBOT_PORT")
    ap.add_argument("--baud", type=int, default=19200)
    ap.add_argument("--config", default=None,
                    help="an existing mecanum.yaml; without it a bootstrap config is used")
    ap.add_argument("--rpm", type=int, default=DEFAULT_RPM,
                    help=f"identification speed at the MOTOR shaft (default "
                         f"{DEFAULT_RPM}). Behind a reduction gearbox the wheel turns "
                         f"far slower, so a few hundred is often needed before the "
                         f"motion is visible at all")
    ap.add_argument("--max-rpm", type=int, default=None, metavar="N",
                    help=f"raise the safety clamp above {RPM_CEILING} rpm. Only needed "
                         f"for a very high reduction ratio")
    ap.add_argument("--spin", type=float, default=DEFAULT_SPIN_S,
                    help=f"seconds per motor output (default {DEFAULT_SPIN_S}); keep it "
                         f"above ~1.5s, controllers can take a second to start")
    ap.add_argument("--scan-only", action="store_true",
                    help="inspection report, then stop. No config writes, nothing turns")
    ap.add_argument("--preflight", action="store_true",
                    help="report + conditional config writes, then stop. Nothing turns")
    ap.add_argument("--verify", action="store_true",
                    help="instead of spinning single outputs, drive +vx / +vy / +wz and "
                         "check every wheel against the kinematics. Needs --config")
    ap.add_argument("--fault-test", action="store_true",
                    help="drive all four wheels and wait for you to pull a controller "
                         "off the bus; checks that the others stop too. Needs --config")
    ap.add_argument("--motion", choices=verify.MOTION_NAMES, default=None,
                    help="run just one --verify motion. On the floor, do them one at a "
                         "time so someone can watch and report each")
    ap.add_argument("--hold", type=float, default=None, metavar="S",
                    help=f"seconds to hold each --verify motion "
                         f"(default {verify.DEFAULT_HOLD_S:.0f})")
    ap.add_argument("--effort", type=float, default=None, metavar="F",
                    help="fraction of the configured limits to use for --verify "
                         "(default 0.5)")
    ap.add_argument("--only", type=parse_address, default=None, metavar="ID:CH",
                    help="spin just one motor output, e.g. --only 1:2. Useful when "
                         "someone has to watch the wheels and answer one at a time")
    ap.add_argument("--yes", action="store_true",
                    help="do not prompt before each spin (the motors WILL turn)")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    ceiling = args.max_rpm if args.max_rpm is not None else RPM_CEILING
    if not 0 < abs(args.rpm) <= ceiling:
        print(f"error: --rpm {args.rpm} is outside 1..{ceiling}. Raise --max-rpm if "
              f"the reduction really needs it.", file=sys.stderr)
        return 2

    if args.config:
        try:
            config = MecanumConfig.load(args.config)
        except ConfigError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
        if args.port:
            config = type(config)(**{**config.__dict__, "port": args.port})
    else:
        # The clamp must not silently swallow the requested speed: being quietly
        # limited to 60 rpm while asking for 300 looks exactly like a motor that will
        # not spin up, and there is nothing on screen to say otherwise.
        limit = max(60, abs(args.rpm))
        try:
            config = bootstrap_config(args.port, baudrate=args.baud, max_motor_rpm=limit)
        except ConfigError as exc:      # pragma: no cover - the literal above is valid
            print(f"error: {exc}", file=sys.stderr)
            return 2
        print(f"using a bootstrap config (no wheel map yet) — identification only, "
              f"clamp {limit} rpm")

    if abs(args.rpm) > config.limits.max_motor_rpm:
        print(f"error: --rpm {args.rpm} exceeds max_motor_rpm "
              f"{config.limits.max_motor_rpm} in {args.config}; the clamp would quietly "
              f"reduce it and the spin would look like a motor that will not start.",
              file=sys.stderr)
        return 2

    try:
        base = MecanumBase.from_config(config)
    except (ValueError, OSError) as exc:
        print(f"error: cannot open the bus: {exc}", file=sys.stderr)
        return 2

    with base:
        print(f"\n--- phase 0: read-only report ---")
        try:
            report_settings(base)
        except MdrobotError as exc:
            print(f"\nerror: a controller did not answer: {type(exc).__name__}: {exc}",
                  file=sys.stderr)
            print("Run examples/mecanum_scan.py to check ids, wiring and baud rate.",
                  file=sys.stderr)
            return 1
        if args.scan_only:
            return 0

        print(f"\n--- phase 1: preflight (conditional writes, nothing turns) ---")
        for line in base.preflight():
            print(f"  {line}")
        if args.preflight:
            return 0

        if args.fault_test:
            print(f"\n--- phase 2: fault injection ---")
            print("  ALL FOUR MOTORS WILL TURN. Wheels off the ground.")
            if not confirm("  ready?", assume_yes=args.yes):
                print("  aborted before any motion")
                return 0
            if config.runtime.auto_enable:
                base.enable()
            effort = verify.DEFAULT_EFFORT if args.effort is None else args.effort
            return verify.fault_test(base, effort=effort)

        if args.verify:
            print(f"\n--- phase 2: verify the dispatch path on blocks ---")
            print("  THE MOTORS WILL TURN — all four together, in a coordinated motion.")
            print("  ON BLOCKS: nothing moves but the wheels.")
            print("  ON THE FLOOR: the robot travels. Each motion prints its distance")
            print("  below before it runs. Clear the space, power cut within reach.")
            if not confirm("  ready?", assume_yes=args.yes):
                print("  aborted before any motion")
                return 0
            if config.runtime.auto_enable:
                base.enable()
            effort = verify.DEFAULT_EFFORT if args.effort is None else args.effort
            hold = verify.DEFAULT_HOLD_S if args.hold is None else args.hold
            checks = verify.run(base, effort=effort, hold_s=hold, only=args.motion)
            return verify.summarise(checks, base)

        print(f"\n--- phase 2: spin each motor output ---")
        print("  THE MOTORS WILL TURN. Wheels off the ground, power cut within reach.")
        if not confirm("  ready?", assume_yes=args.yes):
            print("  aborted before any motion")
            return 0

        if config.runtime.auto_enable:
            base.enable()
        results = spin_each_output(base, args.rpm, args.spin,
                                   assume_yes=args.yes, only=args.only)
        if args.only:
            return 0 if results.get(args.only) else 1
        return summarise(results, args.rpm)


if __name__ == "__main__":
    raise SystemExit(main())
