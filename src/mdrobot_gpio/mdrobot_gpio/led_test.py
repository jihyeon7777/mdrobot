"""Blink a pin, to settle the wiring before any of the logic is in the way.

Answers one question: does this pin light that LED. No ROS, no topics, no
node — so a lamp that does not come on is a wiring problem and nothing else.

    ros2 run mdrobot_gpio led_test                 # blink GPIO 17 for 10 s
    ros2 run mdrobot_gpio led_test --pin 27
    ros2 run mdrobot_gpio led_test --on            # hold it lit, Ctrl-C to stop
    ros2 run mdrobot_gpio led_test --off           # hold it dark
    python3 -m mdrobot_gpio.led_test --pin 17      # the same, without ROS

If it blinks with --active-low and not without, the LED is wired to sink:
the pin pulls the cathode down to light it. Put active_high: false in
config/led.yaml and the node will match.

The pin is left DARK and released on the way out, whatever happens. A lamp
still lit by a program that has exited says the opposite of the truth.
"""

from __future__ import annotations

import argparse
import time


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--pin", type=int, default=17, help="BCM number (default 17)")
    # On a Pi 5 the 40-pin header is pinctrl-rp1 = /dev/gpiochip4. A Pi 4 is 0.
    parser.add_argument("--chip", type=int, default=4)
    parser.add_argument("--seconds", type=float, default=10.0)
    parser.add_argument("--period", type=float, default=1.0, help="blink period")
    parser.add_argument("--active-low", action="store_true",
                        help="the pin pulls the cathode down to light it")
    state = parser.add_mutually_exclusive_group()
    state.add_argument("--on", action="store_true", help="hold lit, do not blink")
    state.add_argument("--off", action="store_true", help="hold dark, do not blink")
    args = parser.parse_args(argv)

    try:
        import lgpio
    except ImportError:
        print("python3-lgpio is not installed:\n"
              "    sudo apt install python3-lgpio")
        return 1

    def level(on: bool) -> int:
        return int(not on) if args.active_low else int(on)

    try:
        handle = lgpio.gpiochip_open(args.chip)
    except Exception as exc:  # noqa: BLE001 - the message is the point
        print(f"cannot open /dev/gpiochip{args.chip}: {exc}\n"
              f"  ls -l /dev/gpiochip{args.chip}   # should be root:dialout\n"
              f"  cat /sys/bus/gpio/devices/gpiochip{args.chip}/../*/label")
        return 1

    info = lgpio.gpio_get_chip_info(handle)
    print(f"chip {args.chip}: {info[3]}, {info[1]} lines")
    print(f"GPIO {args.pin}, {'active low' if args.active_low else 'active high'}")

    try:
        lgpio.gpio_claim_output(handle, args.pin, level(False))
    except Exception as exc:  # noqa: BLE001
        print(f"cannot claim GPIO {args.pin}: {exc}\n"
              f"  Something else may already hold it — the led_node, or a "
              f"device tree overlay.")
        lgpio.gpiochip_close(handle)
        return 1

    try:
        if args.on or args.off:
            lit = args.on
            lgpio.gpio_write(handle, args.pin, level(lit))
            print(f"held {'LIT' if lit else 'DARK'} — Ctrl-C to release")
            while True:
                time.sleep(0.2)
        started = time.monotonic()
        lit = False
        while time.monotonic() - started < args.seconds:
            lit = not lit
            lgpio.gpio_write(handle, args.pin, level(lit))
            print(f"  {time.monotonic() - started:5.1f}s  "
                  f"{'LIT ' if lit else 'dark'}", flush=True)
            time.sleep(args.period / 2.0)
    except KeyboardInterrupt:
        print()
    finally:
        try:
            lgpio.gpio_write(handle, args.pin, level(False))
            lgpio.gpio_free(handle, args.pin)
        finally:
            lgpio.gpiochip_close(handle)
        print("pin left dark and released")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
