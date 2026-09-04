"""Indicator LEDs on the Pi's own header pins.

The machine is watched from across a room while it drives itself under a
vehicle. Whether the camera is seeing the plate could only be read off a topic,
which is no use standing next to it — so a lamp says it instead.

The lamps are driven from here rather than through the RC bridge's downlink,
which would have meant a sixth field in the frame that turns the drill on and
matching firmware on the board. A lamp is not an actuator and does not belong
in the same frame as one.

Pins
----
BCM numbering, the numbers printed on a pinout diagram. On a Pi 5 the 40-pin
header is `pinctrl-rp1`, which is /dev/gpiochip4 — hence `chip` defaulting to
4. It is root:dialout there, so this needs no sudo from a user who can already
open the serial ports.

    plate_pin  17   lit while the plate is being detected
    brake_pin  -1   off until a pin is given

Topics
------
  ~/plate_offset (geometry_msgs/Point)  a sighting. Arrival is the signal; the
                                        value is not read
  ~/command (std_msgs/Int32MultiArray)  [lift, brake, drill, actuator,
                                        solenoid]; brake is index 1
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from .lamp import Latch

BRAKE_INDEX = 1  # into ~/command, per the bridge's COMMAND_NAMES


class LedNode(Node):
    def __init__(self) -> None:
        super().__init__("mdrobot_led")
        self.declare_parameter("chip", 4)
        self.declare_parameter("plate_pin", 17)
        self.declare_parameter("brake_pin", -1)  # -1 = not fitted
        self.declare_parameter("active_high", True)
        # The detector drops out for a beat on a plate in plain view; without a
        # hold the lamp flickers and says less than the topic it replaced.
        self.declare_parameter("plate_hold", 1.0)
        self.declare_parameter("update_rate", 20.0)

        self.chip = int(self.get_parameter("chip").value)
        self.plate_pin = int(self.get_parameter("plate_pin").value)
        self.brake_pin = int(self.get_parameter("brake_pin").value)
        self.active_high = bool(self.get_parameter("active_high").value)
        self.plate = Latch(float(self.get_parameter("plate_hold").value))
        # The brake is a state, not an event: it is reported every tick, so it
        # goes dark the moment it reads 0 rather than waiting a hold out.
        self.brake = Latch(1.0)

        self._handle = None
        self._claimed: list[int] = []
        self._open()

        self.create_subscription(Point, "~/plate_offset", self._on_plate, 10)
        self.create_subscription(Int32MultiArray, "~/command", self._on_command, 10)
        rate = float(self.get_parameter("update_rate").value)
        if rate <= 0:
            raise ValueError(f"update_rate must be positive, got {rate}")
        self.create_timer(1.0 / rate, self._tick)

    def _open(self) -> None:
        try:
            import lgpio
        except ImportError as exc:
            raise RuntimeError(
                "python3-lgpio is not installed, so the LEDs cannot be driven:\n"
                "    sudo apt install python3-lgpio\n"
                "Everything else runs without it; only this node needs it."
            ) from exc
        self._lgpio = lgpio
        try:
            self._handle = lgpio.gpiochip_open(self.chip)
        except Exception as exc:  # noqa: BLE001 - the message matters more
            raise RuntimeError(
                f"cannot open /dev/gpiochip{self.chip}: {exc}. On a Pi 5 the "
                f"40-pin header is pinctrl-rp1, which is gpiochip4, and it is "
                f"root:dialout — check `ls -l /dev/gpiochip{self.chip}` and that "
                f"you are in the dialout group."
            ) from exc
        for name, pin in (("plate", self.plate_pin), ("brake", self.brake_pin)):
            if pin < 0:
                continue
            lgpio.gpio_claim_output(self._handle, pin, self._level(False))
            self._claimed.append(pin)
            self.get_logger().info(f"{name} lamp on GPIO {pin} (chip {self.chip})")
        if not self._claimed:
            self.get_logger().warn("no pins configured; this node is doing nothing")

    def _level(self, on: bool) -> int:
        return int(on) if self.active_high else int(not on)

    def _on_plate(self, _msg: Point) -> None:
        # Arrival IS the signal. plate_offset is published whenever a
        # plate-shaped region is located, which is exactly the question the
        # lamp answers.
        self.plate.signal(time.monotonic())

    def _on_command(self, msg: Int32MultiArray) -> None:
        if len(msg.data) <= BRAKE_INDEX:
            return
        if msg.data[BRAKE_INDEX]:
            self.brake.signal(time.monotonic())
        else:
            self.brake.clear()

    def _tick(self) -> None:
        now = time.monotonic()
        self._write(self.plate_pin, self.plate.lit(now))
        self._write(self.brake_pin, self.brake.lit(now))

    def _write(self, pin: int, on: bool) -> None:
        if pin < 0 or self._handle is None:
            return
        try:
            self._lgpio.gpio_write(self._handle, pin, self._level(on))
        except Exception:  # noqa: BLE001 - a lamp must never take the node down
            pass

    def destroy_node(self) -> bool:
        # Leave them dark. A lamp left lit by a node that has exited says the
        # opposite of the truth.
        for pin in self._claimed:
            self._write(pin, False)
        if self._handle is not None:
            try:
                self._lgpio.gpiochip_close(self._handle)
            except Exception:  # noqa: BLE001
                pass
            self._handle = None
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = LedNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        # A missing library or an unopenable chip: say the one useful sentence
        # rather than a traceback through lgpio.
        print(f"mdrobot_led: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
