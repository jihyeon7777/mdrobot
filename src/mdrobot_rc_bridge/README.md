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

`rc_bridge_node` reports what the operator is asking for. It decides nothing and
commands nothing — the decision layer above it does that.

## Wire format

`115200 8N1`, CRLF ASCII lines at ~50 Hz, eleven integers per line:

```
steer,throttle,lift,brake,mode,drill,actuator,solenoid,limit_up,limit_down,checksum
1500, 1500,    0,   0,    -1,  0,    0,       0,       0,       0,         2999
```

`checksum == sum(the ten channels)`. Verified against 6044 captured frames, all valid.
Lines that fail to parse or fail the checksum are counted and skipped, never raised.

## Interface

| | | |
|---|---|---|
| pub | `~/channels` | `std_msgs/Int32MultiArray` — all ten channels, exactly as sent |
| pub | `~/joy` | `sensor_msgs/Joy` — axes normalised to -1..+1, rest as buttons |
| pub | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` — link state, rate, error counts |

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
- `mode` idles at `-1`, which does not look like one of its three positions.
- The downlink (ROS 2 -> board, for drill / actuator / solenoid) is not
  implemented yet: it needs the command format the board firmware parses.
- The board was seen dropping off the USB bus and re-enumerating during early
  bring-up. The reader reopens it by its by-id path and counts the event in
  `reconnects`; a rising count there points at cabling or power.
