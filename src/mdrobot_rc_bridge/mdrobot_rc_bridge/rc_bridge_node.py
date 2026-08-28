#!/usr/bin/env python3
"""ROS 2 node publishing the operator's RC transmitter channels.

Sits at the bottom of the control chain:

    transmitter --RF--> STM32 board --USB serial--> THIS NODE --> ROS 2

The node only reports what the operator is asking for. It decides nothing and
commands nothing; a node above subscribes here, decides, and drives the MD
motor controller (drive axes) and the STM32 board (drill, actuator, solenoid).

Interface
---------
Parameters:
  port (str='')             serial port; '' auto-detects the board by its USB
                            product string, which survives a ttyACM renumber
  baudrate (int=115200)
  frame_id (str='rc')
  publish_rate (float=50.0) Hz; the board sends at ~50 Hz
  stale_timeout (float=0.3) s without a frame before the link is reported stale
  diag_rate (float=2.0)     Hz; diagnostics
  axis_channels (int[])     channels published as Joy axes, normalised from
                            pulse width to -1..+1. Default [0, 1] = steer,
                            throttle — the two confirmed to rest at ~1500 us.
                            Every other channel goes to Joy buttons unscaled.
  pwm_min/pwm_mid/pwm_max   (int=1000/1500/2000) normalisation range, in us
  deadband (int=15)         us around pwm_mid that reads as exactly 0.0
  invert_axes (int[])       axis channels to negate after normalising

Publishers:
  ~/channels (std_msgs/Int32MultiArray)
      All ten channels, exactly as the board sent them. Lossless — prefer this
      for anything that has to be exact.
  ~/joy (sensor_msgs/Joy)
      The same data in the standard teleop shape, for off-the-shelf tooling.
      axes    = axis_channels, normalised to -1..+1
      buttons = every remaining channel, raw
  ~/diagnostics (diagnostic_msgs/DiagnosticArray)
      link state, measured rate, checksum/parse errors, reconnect count

Only the steer and throttle channels are confirmed to carry ~1000-2000 us pulse
widths. The remaining channels were idle throughout the capture used to write
this, so their ranges are unverified: check them with
`python3 examples/read_rc_bridge.py --map` before treating any of them as an
axis, and adjust axis_channels accordingly.

Serial runs on its own daemon thread because the reader blocks; the ROS timers
only ever look at the most recent frame.
"""

from __future__ import annotations

import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray, MultiArrayDimension

from mdrobot_rc_bridge.rc_reader import (
    CHANNEL_NAMES,
    NUM_CHANNELS,
    RcBridgeReader,
    RcFrame,
)


class RcBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mdrobot_rc_bridge")

        self.declare_parameter("port", "")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "rc")
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("stale_timeout", 0.3)
        self.declare_parameter("diag_rate", 2.0)
        self.declare_parameter("axis_channels", [0, 1])
        self.declare_parameter("pwm_min", 1000)
        self.declare_parameter("pwm_mid", 1500)
        self.declare_parameter("pwm_max", 2000)
        self.declare_parameter("deadband", 15)
        self.declare_parameter("invert_axes", [-1])

        port = str(self.get_parameter("port").value) or None
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout").value)
        self.pwm_min = int(self.get_parameter("pwm_min").value)
        self.pwm_mid = int(self.get_parameter("pwm_mid").value)
        self.pwm_max = int(self.get_parameter("pwm_max").value)
        self.deadband = int(self.get_parameter("deadband").value)

        if not self.pwm_min < self.pwm_mid < self.pwm_max:
            raise ValueError(
                f"need pwm_min < pwm_mid < pwm_max, got "
                f"{self.pwm_min} / {self.pwm_mid} / {self.pwm_max}"
            )

        self.axis_channels = [int(c) for c in self.get_parameter("axis_channels").value]
        for c in self.axis_channels:
            if not 0 <= c < NUM_CHANNELS:
                raise ValueError(f"axis_channels entry {c} outside 0..{NUM_CHANNELS - 1}")
        # -1 is the "unset" placeholder, because an empty int array cannot be a
        # ROS parameter default without an explicit type.
        self.invert_axes = {int(c) for c in self.get_parameter("invert_axes").value if c >= 0}
        self.button_channels = [c for c in range(NUM_CHANNELS) if c not in self.axis_channels]

        self.pub_channels = self.create_publisher(Int32MultiArray, "~/channels", 10)
        self.pub_joy = self.create_publisher(Joy, "~/joy", 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, "~/diagnostics", 1)

        # Written by the serial thread, read by the timers.
        self._lock = threading.Lock()
        self._frame: RcFrame | None = None
        self._frame_wall = 0.0
        self._published_seq = -1
        self._seq = 0
        self._rate_mark = (time.monotonic(), 0)
        self._measured_hz = 0.0

        self.reader = RcBridgeReader(port=port,
                                     baudrate=int(self.get_parameter("baudrate").value))
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._thread.start()

        rate = float(self.get_parameter("publish_rate").value)
        self.create_timer(1.0 / rate, self._on_publish)
        self.create_timer(1.0 / float(self.get_parameter("diag_rate").value), self._on_diag)

        self.get_logger().info(
            f"reading {port or 'auto-detected RC bridge'} at "
            f"{self.get_parameter('baudrate').value} baud; "
            f"axes={[CHANNEL_NAMES[c] for c in self.axis_channels]}"
        )

    # ── serial thread ───────────────────────────────────────────────────────
    def _serial_loop(self) -> None:
        """Pull frames as fast as the board sends them; keep only the newest."""
        while not self._stop.is_set():
            try:
                for frame in self.reader.frames():
                    if self._stop.is_set():
                        break
                    with self._lock:
                        self._frame = frame
                        self._frame_wall = time.monotonic()
                        self._seq += 1
            except Exception as exc:  # noqa: BLE001 - the thread must never die
                if self._stop.is_set():
                    break
                self.get_logger().warn(f"serial reader restarting after: {exc}")
                time.sleep(0.5)

    # ── conversion ──────────────────────────────────────────────────────────
    def _normalise(self, channel: int, value: int) -> float:
        """Pulse width in microseconds -> -1..+1, with a deadband at centre."""
        if abs(value - self.pwm_mid) <= self.deadband:
            out = 0.0
        elif value >= self.pwm_mid:
            out = (value - self.pwm_mid) / (self.pwm_max - self.pwm_mid)
        else:
            out = (value - self.pwm_mid) / (self.pwm_mid - self.pwm_min)
        out = max(-1.0, min(1.0, out))
        return -out if channel in self.invert_axes else out

    # ── timers ──────────────────────────────────────────────────────────────
    def _on_publish(self) -> None:
        with self._lock:
            frame, seq = self._frame, self._seq
        if frame is None or seq == self._published_seq:
            return  # nothing new; never republish a stale frame as if it were fresh
        self._published_seq = seq

        stamp = self.get_clock().now().to_msg()

        raw = Int32MultiArray()
        dim = MultiArrayDimension()
        dim.label, dim.size, dim.stride = "channel", NUM_CHANNELS, NUM_CHANNELS
        raw.layout.dim = [dim]
        raw.data = list(frame.channels)
        self.pub_channels.publish(raw)

        joy = Joy()
        joy.header.stamp = stamp
        joy.header.frame_id = self.frame_id
        joy.axes = [self._normalise(c, frame.channels[c]) for c in self.axis_channels]
        joy.buttons = [frame.channels[c] for c in self.button_channels]
        self.pub_joy.publish(joy)

    def _on_diag(self) -> None:
        now = time.monotonic()
        with self._lock:
            age = now - self._frame_wall if self._frame_wall else None
            seq = self._seq
        mark_t, mark_seq = self._rate_mark
        if now > mark_t:
            self._measured_hz = (seq - mark_seq) / (now - mark_t)
        self._rate_mark = (now, seq)

        stats = self.reader.stats
        status = DiagnosticStatus()
        status.name = "mdrobot_rc_bridge: link"
        status.hardware_id = self.reader.port or "auto"
        if age is None:
            status.level = DiagnosticStatus.ERROR
            status.message = "no frame received yet"
        elif age > self.stale_timeout:
            status.level = DiagnosticStatus.ERROR
            status.message = f"stale: no frame for {age:.2f} s"
        else:
            status.level = DiagnosticStatus.OK
            status.message = f"ok at {self._measured_hz:.1f} Hz"
        status.values = [
            KeyValue(key="rate_hz", value=f"{self._measured_hz:.1f}"),
            KeyValue(key="frames", value=str(stats.frames)),
            KeyValue(key="bad_checksum", value=str(stats.bad_checksum)),
            KeyValue(key="unparsable", value=str(stats.unparsable)),
            KeyValue(key="wrong_length", value=str(stats.wrong_length)),
            KeyValue(key="reconnects", value=str(stats.reconnects)),
        ]
        with self._lock:
            frame = self._frame
        if frame is not None:
            status.values += [KeyValue(key=n, value=str(v))
                              for n, v in frame.as_dict().items()]

        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.pub_diag.publish(msg)

    def destroy_node(self) -> bool:
        self._stop.set()
        self.reader.close()  # unblocks the reader thread's pending read()
        self._thread.join(timeout=1.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RcBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
