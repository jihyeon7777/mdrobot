"""Switch the HWT901B between its 9-axis and 6-axis algorithms, and prove it.

Why this exists
---------------
In 9-axis the sensor fuses the magnetometer into the heading. This machine
drives itself under a vehicle, past four BLDC motors of its own and a drill,
and the field it sees has nothing to do with north. Measured on the real
machine, 2026-09-05: driving forward for six seconds moved the reported
heading 57 deg while the GYRO integrated 0.81 deg over the whole twenty-second
run — and once the motors stopped the heading crept back, which a real
rotation does not do. The autonomous run before it had aborted on a runaway,
and that runaway was the heading hold faithfully chasing a rotation that was
never happening.

6-axis integrates the gyro alone, so the motors cannot reach it. The price is
drift, and that has been measured too: driven for 90 s and returned to marks
on the floor, the heading came back 1.75 deg off. A phase of the sequence is
tens of seconds. That is a trade worth making.

What it writes
--------------
One register. WITMOTION's protocol is ``FF AA <reg> <low> <high>``, with a
write blocked until an unlock is sent and lost at power-off until a save is
sent. This does unlock, write, save — then READS THE REGISTER BACK and refuses
to claim success unless it reads what it wrote. A configuration tool that
reports what it *sent* rather than what the device *holds* is how a sensor
ends up quietly in the wrong mode.

    ros2 run mdrobot_imu imu_configure --show        # what mode is it in
    ros2 run mdrobot_imu imu_configure --axis 6      # switch to 6-axis
    ros2 run mdrobot_imu imu_configure --axis 9      # back again

Nothing else may hold the port. Stop the node first.
"""

from __future__ import annotations

import argparse
import struct
import time

from .protocol import PACKET_LEN, PacketFramer
from .reader import BAUDRATE

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

# WITMOTION register addresses used here.
REG_SAVE = 0x00
REG_READADDR = 0x27
REG_UNLOCK = 0x69
REG_AXIS6 = 0x24  # 0 = 9-axis, 1 = 6-axis

UNLOCK_VALUE = 0xB588
SAVE_VALUE = 0x0000
# The reply to a READADDR is a 0x55 0x5F packet carrying four registers
# starting at the address asked for.
READ_REPLY_KIND = 0x5F


def command(reg: int, value: int) -> bytes:
    return bytes([0xFF, 0xAA, reg]) + struct.pack("<H", value)


def read_register(port, reg: int, timeout: float = 1.5) -> int | None:
    """Ask for one register and return it, or None if nothing came back."""
    port.reset_input_buffer()
    port.write(command(REG_READADDR, reg))
    port.flush()
    framer = PacketFramer()
    deadline = time.time() + timeout
    while time.time() < deadline:
        for packet in framer.feed(port.read(64)):
            if packet[1] == READ_REPLY_KIND:
                # Four little-endian words; the first is the register asked for.
                return struct.unpack("<H", packet[2:4])[0]
        time.sleep(0.02)
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baudrate", type=int, default=BAUDRATE)
    parser.add_argument("--axis", type=int, choices=(6, 9),
                        help="which algorithm to switch to")
    parser.add_argument("--show", action="store_true",
                        help="report the current mode and change nothing")
    args = parser.parse_args(argv)
    if not args.show and args.axis is None:
        parser.error("give --axis 6, --axis 9, or --show")

    import serial

    try:
        port = serial.Serial(args.port, args.baudrate, timeout=0.2)
    except Exception as exc:  # noqa: BLE001 - the message is the point
        print(f"cannot open {args.port}: {exc}\n"
              f"  Something else probably holds it -- the imu_node runs from "
              f"bringup.\n"
              f"  fuser -v {args.port}")
        return 1

    def mode_of(value: int | None) -> str:
        if value is None:
            return "unreadable"
        return {0: "9-axis (magnetometer fused)", 1: "6-axis (gyro only)"}.get(
            value, f"unknown ({value})")

    with port:
        time.sleep(0.2)
        before = read_register(port, REG_AXIS6)
        print(f"  current: {mode_of(before)}")
        if before is None:
            print("  The sensor did not answer a register read. Check the baud "
                  "rate and that nothing else is on the port.")
            return 1
        if args.show:
            return 0

        want = 1 if args.axis == 6 else 0
        if before == want:
            print(f"  already {args.axis}-axis; nothing written")
            return 0

        print(f"  writing {args.axis}-axis ...")
        for message in (command(REG_UNLOCK, UNLOCK_VALUE),
                        command(REG_AXIS6, want),
                        command(REG_SAVE, SAVE_VALUE)):
            port.write(message)
            port.flush()
            time.sleep(0.25)
        time.sleep(0.6)

        after = read_register(port, REG_AXIS6)
        print(f"  now:     {mode_of(after)}")
        if after != want:
            print(f"  ** THE WRITE DID NOT TAKE ** (wanted {want}, holds "
                  f"{after}). The sensor is unchanged or half-changed; do not "
                  f"assume the mode. Try again, and check the unlock is being "
                  f"accepted.")
            return 1
        print(f"  saved. In 6-axis the heading is relative and drifts (~1.75 "
              f"deg per 90 s on this machine); zero it at the start of each "
              f"phase, which is what the sequence already does.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
