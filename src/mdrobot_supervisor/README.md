# mdrobot_supervisor

The decision layer. Everything the machine does is decided here; the layers
below only report and actuate.

```
transmitter --RF--> STM32 board --> rc_bridge_node --> [ supervisor ]
                                         ^                   |
                    equipment  ~/command +                   | four wheel rpm
                                                             v
                                        md1 (slave 1)   md2 (slave 2)   ttyUSB0
```

## Interface

| | | |
|---|---|---|
| sub | `~/rc` | `std_msgs/Int32MultiArray` — the bridge's ten channels |
| pub | `~/cmd_velocity_1`, `~/cmd_velocity_2` | `std_msgs/Float64MultiArray` — `[ch1_rpm, ch2_rpm]` per controller |
| pub | `~/command` | `std_msgs/Int32MultiArray` — `[lift, brake, drill, actuator, solenoid]` |
| pub | `~/mode` | `std_msgs/String` — base / mecanum / autonomous |
| pub | `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

The launch file wires `~/rc` and `~/command` to `mdrobot_rc_bridge` and the two
drive topics to `/md1` and `/md2`.

## Drive

Four mecanum wheels on two dual-channel MD controllers. Steer and throttle
become a body twist, the twist becomes four wheel speeds
([kinematics.py](mdrobot_supervisor/kinematics.py)), and the wheel map splits
those across the controllers.

The transmitter has two axes, so `vy` (strafe) is always 0 — there is no third
axis to drive it until a mode assigns one.

Wheel speeds are capped by scaling all four together, never per wheel: the
kinematics is linear, so uniform scaling is exactly "the same path, slower",
while clipping wheels one at a time bends a commanded straight line into an arc.

## Modes

The switch reports -1, 0 or +1 and `mode_names` names them in that order. **No
behaviour is attached yet** — which position should mean what is undecided, so
the node reports the mode and drives identically in all three.

## Safety

Nothing here replaces the board's failsafe or a physical e-stop. What the node
does guarantee:

- `rc_timeout` exceeded → zero rpm to both controllers, idle equipment
- brake → drive zeroed in the same tick it is seen
- wheel speeds never exceed `max_motor_rpm`
- out-of-range equipment values never reach hardware (the bridge clamps them)

## Known gaps

- **`roller_layout` is `unknown`.** It computes as `x` so the base drives, but
  the strafe direction is unverified. It cannot be settled by eye — the roller
  you see from above is the mirror of the ground-contact one that does the work.
  Only a floor test settles it, and it matters only once something commands `vy`.
- **`lift_input` is assumed `tristate`** (-1/0/+1). The channel was never seen
  moving. If it actually carries a pulse width the node holds lift at 0 and logs
  an error rather than scaling a 1500 into full-speed lift — set `lift_input:
  pwm` in that case.
- **`limit_gating` is off**, because the switch polarity is unknown. A gate with
  the polarity backwards either blocks lift forever or never fires. Measure which
  value a *triggered* switch reports, set `limit_active_value`, then enable.
- **`max_linear_x: 0.2` asks for 611 motor rpm against a 600 cap**, so full
  throttle is always scaled to 0.98 and diagnostics sit at WARN. Both numbers are
  from `mecanum.yaml` as-is.
- The wheel map, geometry and gear ratio are copied from the untracked
  `mecanum.yaml`; the `mdrobot_mecanum` package it belonged to is no longer in
  the repository. Confirm they still describe the machine.
