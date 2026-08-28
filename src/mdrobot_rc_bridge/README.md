# mdrobot_rc_bridge

ROS 2 bridge for the STM32 RC telemetry board.

```
transmitter --RF--> STM32 board --USB serial--> rc_bridge_node --> ROS 2 decision layer
                                                                          |
                                                    +---------------------+---------------------+
                                                    |                                           |
                                              ttyUSB0 (MD)                              ttyACM0 (STM32)
                                              steer / throttle                    drill / actuator / solenoid
```

`rc_bridge_node` carries both directions and decides nothing. Upward it reports
what the operator is asking for; downward it relays whatever the decision layer
publishes on `~/command`.

Safety: with `command_timeout > 0` the node falls back to `idle_command` (all
zeros) when the decision layer goes quiet, sends it once at startup so nothing
inherits a previous run's state, and sends it again on shutdown. That is a
backstop, not a replacement for the board's own failsafe.

## Wire format

`115200 8N1`, CRLF ASCII lines at ~50 Hz, eleven integers per line:

```
steer,throttle,lift,brake,mode,drill,actuator,solenoid,limit_up,limit_down,checksum
1500, 1500,    0,   0,    -1,  0,    0,       0,       0,       0,         2999
```

`checksum == sum(the ten channels)`. Verified against 6044 captured frames, all valid.
Lines that fail to parse or fail the checksum are counted and skipped, never raised.

Downward, over the same port, five values in the same shape:

```
lift,brake,drill,actuator,solenoid,checksum
0,   0,    0,    0,       0,       0
```

The board emits CRLF but which terminator its *parser* wants was never
confirmed, so `terminator` is a parameter — try `lf` or `cr` if commands are
ignored.

## Interface

| | | |
|---|---|---|
| pub | `~/channels` | `std_msgs/Int32MultiArray` — all ten channels, exactly as sent |
| pub | `~/joy` | `sensor_msgs/Joy` — axes normalised to -1..+1, rest as buttons |
| pub | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` — link state, rate, error counts |
| sub | `~/command` | `std_msgs/Int32MultiArray` — `[lift, brake, drill, actuator, solenoid]` |

### Command ranges

| output | range | |
|---|---|---|
| `lift` | -60..60 | signed speed of the up/down motor |
| `brake` | 0/1 | |
| `drill` | 0/1 | |
| `actuator` | -1..1 | **unconfirmed** — retract/stop/extend is the usual shape, not verified against the firmware |
| `solenoid` | 0/1 | |

Values outside the range are clamped, not forwarded — this link ends at a drill
and a valve. Adjust with `command_min` / `command_max`.

### Modes

The operator's three-position `mode` channel selects base / mecanum /
autonomous. The bridge only reports its raw value; acting on it belongs to the
decision layer, which does not exist yet.

Parameters live in [config/rc_bridge.yaml](config/rc_bridge.yaml).

## Run

```bash
ros2 launch mdrobot_rc_bridge rc_bridge.launch.py
```

Without ROS, to check the link and the transmitter:

```bash
python3 examples/read_rc_bridge.py          # live view
python3 examples/read_rc_bridge.py --map    # per-channel range
```

## Known gaps

- Only `steer` and `throttle` are confirmed to carry 1000-2000 us pulse widths.
  The other eight channels sat at their idle values throughout the capture this
  was written from, so their ranges are unverified — check with `--map` before
  adding any of them to `axis_channels`.
- `mode` idles at `-1`, which is none of its three positions — probably "no
  reading yet". Confirm what the three positions report.
- The `actuator` range is unconfirmed; every other output range is.
- The downlink terminator is set to CRLF to match what the board emits. Which
  terminator its parser expects was never confirmed — switch `terminator` to
  `lf` or `cr` if commands are ignored.
- The decision layer is not written yet: nothing currently turns operator input
  into `~/command`, or splits drive off to the MD controller.
- The board was seen dropping off the USB bus and re-enumerating during early
  bring-up. The reader reopens it by its by-id path and counts the event in
  `reconnects`; a rising count there points at cabling or power.
