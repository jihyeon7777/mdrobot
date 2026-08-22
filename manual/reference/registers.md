# Register / command / status-bit reference

Named constants for raw access via `driver.client` (Python `mdrobot.registers`) or the
`ModbusClient` (C++ `mdrobot_cpp/registers.hpp`). **Never inline the numbers** — use the
constant names. 16-bit words are big-endian; 32-bit longs are low word first. Framing
detail lives in the shipped sources: [`frame.py`](../../src/mdrobot/mdrobot/frame.py) /
[`frame.hpp`](../../src/mdrobot_cpp/include/mdrobot_cpp/frame.hpp).

Access legend: **R** read · **W** write (setting/EEPROM) · **C** command/control write
(velocity, position, start/stop, or via the `PID_COMMAND (10)` gateway). This lists the
PIDs the driver defines; the controller has more.

## PIDs (parameter IDs = register addresses)

### Single-byte command / setting PIDs (0–127)

| Name | # | Access | Meaning |
|---|---|---|---|
| `PID_VERSION` | 1 | R | Firmware/protocol version (DL=13 → v1.3); ping candidate |
| `PID_TQ_OFF` | 5 | W | Motor free / natural stop |
| `PID_BRAKE` | 6 | W | Electric brake |
| `PID_COMMAND` | 10 | W | CMD command gateway (use the `CMD_*` codes below) |
| `PID_ALARM_RESET` | 12 | W | Reset alarm |
| `PID_POSI_RESET` | 13 | W | Reset position to zero |
| `PID_INV_SIGN_CMD` | 16 | R/W | Reference command sign inverse |
| `PID_USE_LIMIT_SW` | 17 | R/W | CTRL limit-switch function (0 cancel, 1 use); default 1. With `1`, CTRL pin 6 (`DIR`) gates CW/negative rpm and pin 8 (`START/STOP`) gates CCW/positive rpm |
| `PID_HALL_TYPE` | 21 | R/W | Motor pole / hall type (0:4p, 1:8p, **2:10p**, 3:12p, 4:2p, 5:6p). Sets the hall resolution: `counts/rev = 3 × poles` |
| `PID_INPUT_TYPE` | 25 | R/W | User input type |
| `PID_POS_SEN_TYPE` | 26 | R/W | Position signal type (0 HALL, 1 EPOSI, 2 POT, 3/4 MENA); follows `PID_USE_EPOSI` automatically — no need to write it |
| `PID_USE_LIMIT_SW2` | 29 | R/W | Motor-2 limit-switch function (dual); same meaning as 17 |
| `PID_CTRL_STATUS` | 34 | R | Status bit map (status-1) |
| `PID_USE_EPOSI` | 46 | R/W | Position source (0 hall counter, 1 encoder); default 0. Encoder source: counts/rev = 4 × `ENC_PPR`, physical +/− flips vs hall — see [Using an encoder](../encoder.md) |
| `PID_DI` | 48 | R | Digital-input bits |
| `PID_IN_POSITION_OK` | 49 | R | Position control done (0/1) |
| `PID_UI_COM` | 78 | R/C | Serial-comm control (0 = CTRL I/O, 1 = serial only) |
| `PID_ENC_INV_DIR` | 80 | R/W | Encoder A/B phase swap (0 normal, 1 swap) |
| `PID_START_STOP` | 100 | C | Start/stop (0 stop, 1 CCW, 2 CW); run-latch arm |

### Word command / data PIDs (101–190)

| Name | # | Access | Meaning |
|---|---|---|---|
| `PID_VEL_CMD` | 130 | C | Velocity command, signed rpm (`set_velocity`) |
| `PID_VEL_CMD2` | 131 | C | Motor-2 velocity command (dual) |
| `PID_ID` | 133 | W | Controller ID setting (wire word `(id << 8) \| 0xAA`) |
| `PID_OPEN_VEL_CMD` | 134 | C | Open-loop output (−1023..1023) — **DANGEROUS** |
| `PID_BAUDRATE` | 135 | W | RS485 baudrate setting (write check required) |
| `PID_INT_RPM_DATA` | 138 | R | Motor speed, signed rpm (`get_speed`) |
| `PID_TQ_DATA` | 139 | R | Current, 0.1 A units (`get_current`) |
| `PID_TQ_CMD` | 140 | C | Torque/current command (−1023..1023) |
| `PID_VOLT_IN` | 143 | R | Supply voltage, 0.1 V units (`get_voltage`) |
| `PID_RETURN_TYPE` | 149 | R/W | Return type after a command |
| `PID_TAR_VEL` | 155 | R/W | Fixed target speed, rpm |
| `PID_ENC_PPR` | 156 | R/W | Encoder pulses-per-rev; **0 = no encoder** (hall closed-loop). Recent firmware ships in encoder mode → set `0` for hall drive. With an encoder wired, set the **rated PPR**: it feeds the velocity loop only, position stays on the hall counter |
| `PID_REF_RPM` | 166 | R | Reference velocity, signed rpm |
| `PID_PNT_TQ_OFF` | 174 | C | Free/tq-off both motors (DL motor1, DH motor2) |
| `PID_PNT_BRAKE` | 175 | C | Electric brake both motors (DL motor1, DH motor2) |
| `PID_TAR_POSI_VEL` | 176 | R/W | Max speed in position control, rpm |
| `PID_COM_TAR_SPEED` | 180 | C/R | Target speed used by `PID_START_STOP`, rpm |

### Acceleration / deceleration (slow-start / slow-down)

A raw value 0–1023 maps to 0..`PID_MAX_SS_TIME` seconds (full scale 15 s on tested
devices). Use `units.slow_seconds_to_raw` / `slow_raw_to_seconds`.

| Name | # | Access | Meaning |
|---|---|---|---|
| `PID_MAX_SS_TIME` | 57 | R/W | Max slow-start time, 15–60 s (full scale) |
| `PID_MIN_SSSD` | 124 | R/W | Min slow-start/down parameter, 0–1023 |
| `PID_SLOW_START` | 153 | R/W | Speed slow-start (single/global), 0–1023 |
| `PID_SLOW_DOWN` | 154 | R/W | Speed slow-down (single/global), 0–1023 |
| `PID_POSI_SS` | 178 | R/W | Position slow-start (single), 0–1023 |
| `PID_POSI_SD` | 179 | R/W | Position slow-down (single), 0–1023 |
| `PID_SLOW_START1` / `PID_SLOW_START2` | 108 / 109 | R/W | MOT1 / MOT2 speed slow-start (dual) |
| `PID_SLOW_DOWN1` / `PID_SLOW_DOWN2` | 111 / 112 | R/W | MOT1 / MOT2 speed slow-down (dual) |
| `PID_POSI_SS1` / `PID_POSI_SS2` | 113 / 114 | R/W | MOT1 / MOT2 position slow-start (dual) |
| `PID_POSI_SD1` / `PID_POSI_SD2` | 115 / 116 | R/W | MOT1 / MOT2 position slow-down (dual) |

### N-byte data / command PIDs (193–253)

| Name | # | Access | Meaning |
|---|---|---|---|
| `PID_MONITOR` | 196 | R | Single monitor (speed/current/output/position/status) |
| `PID_POSI_DATA` | 197 | R | Motor position (4-byte long) (`get_position`) |
| `PID_POSI_SET1` | 198 | C | Set motor1 position (4-byte long) |
| `PID_GAIN` | 203 | C/R | Position-P, speed-P, speed-I gains (6 bytes) |
| `PID_PNT_POSI_VEL_CMD` | 206 | C | Dual position + max speed (12 bytes) |
| `PID_PNT_VEL_CMD` | 207 | C | Dual velocity command (4 bytes: speed1, speed2) |
| `PID_PNT_OPEN_VEL_CMD` | 208 | C | Dual open-loop command (4 bytes) |
| `PID_PNT_TQ_CMD` | 209 | C | Dual torque/current command (4 bytes) |
| `PID_PNT_MAIN_DATA` | 210 | R | Dual main data (18 bytes; includes current) |
| `PID_PNT_INC_POSI_CMD` | 215 | C | Dual incremental position (8 bytes) |
| `PID_PNT_MONITOR` | 216 | R | Dual monitor (14 bytes; no current) |
| `PID_POSI_SET` | 217 | C | Set motor position (4-byte long) |
| `PID_POSI_SET2` | 218 | C | Set motor2 position (4-byte long, dual) |
| `PID_POSI_VEL_CMD` | 219 | C | Position control with max speed (6 bytes) (`move_to`) |
| `PID_INC_POSI_VEL_CMD` | 220 | C | Incremental position with max speed (6 bytes) (`move_by`) |
| `PID_MAX_RPM` | 221 | R/W | Max speed (2-byte word) |
| `PID_INC_POSI_VEL_CMD2` | 224 | C | Motor-2 incremental position + speed (dual) |
| `PID_POSI_VEL_CMD2` | 236 | C | Motor-2 position control with max speed (dual) |
| `PID_PNT_INC_POSI_VEL_CMD` | 242 | C | Dual incremental position + max speed (12 bytes) |
| `PID_POSI_CMD` | 243 | W | Target position (4-byte long) |
| `PID_INC_POSI_CMD` | 244 | W | Incremental target position (4-byte long) |
| `PID_PNT_POSI_CMD` | 246 | C | Dual absolute position (8 bytes) |
| `PID_POSI_CMD2` | 247 | C | Motor-2 target position (dual) |

## Command codes (`command(cmd)` → `PID_COMMAND (10)`)

| Name | # | Meaning |
|---|---|---|
| `CMD_TQ_OFF` | 2 | Motor free state |
| `CMD_BRAKE` | 4 | Electric brake |
| `CMD_ALARM_RESET` | 8 | Reset alarm |
| `CMD_POSI_RESET` | 10 | Position reset to zero |
| `CMD_TAR_VEL_OFF` | 20 | Erase target velocity (`PID_TAR_VEL` / `COM_TAR_SPEED`) |
| `CMD_SLOW_START_OFF` | 21 | Erase speed slow-start value |
| `CMD_SLOW_DOWN_OFF` | 22 | Erase speed slow-down value |
| `CMD_EMER_ON` | 67 | Stop + electric brake + UVW short (e-stop candidate) |
| `CMD_EMER_OFF` | 68 | Free state / Tq-off — **DANGEROUS** |
| `CMD_BRAKE1` | 69 | Electric brake on motor1 (dual) |
| `CMD_BRAKE2` | 70 | Electric brake on motor2 (dual) |
| `CMD_POSI_SS_OFF` | 71 | Stop using the position slow-start parameter |
| `CMD_POSI_SD_OFF` | 72 | Stop using the position slow-down parameter |
| `CMD_RESET_SYSTEM` | 79 | Controller reboot — **DANGEROUS** |

## Status-1 bits (`get_status()`, `PID_CTRL_STATUS (34)`, monitor)

`get_status().active` returns the set bits' names from this table; each also has a
boolean field on `StatusBits`.

| Bit | Name (`active`) | `StatusBits` field |
|---|---|---|
| 0 | `ALARM` | `alarm` |
| 1 | `CTRL_FAIL` | `ctrl_fail` |
| 2 | `OVER_VOLT` | `over_voltage` |
| 3 | `OVER_TEMP` | `over_temperature` |
| 4 | `OVER_LOAD` | `overload` |
| 5 | `HALL_OR_ENC_FAIL` | `hall_or_encoder_fail` |
| 6 | `INV_VEL` | `inverse_velocity` |
| 7 | `STALL` | `stall` |

## Status-2 bits (`PID_MONITOR` status2)

| Bit | Name |
|---|---|
| 0 | `REGEN_OVER_TEMP` |
| 1 | `ENC_FAIL` |

## Digital-input bits (`PID_DI (48)`)

| Bit | Name | | Bit | Name |
|---|---|---|---|---|
| 0 | `INT_SPEED` | | 4 | `START_STOP` |
| 1 | `ALARM_RESET` | | 5 | `ENC_B` |
| 2 | `DIR` | | 6 | `ENC_A` |
| 3 | `RUN_BRAKE` | | | |

> **Polarity.** The CTRL input bits (2 `DIR`, 3 `RUN_BRAKE`, 4 `START_STOP`) read **1
> when the pin is shorted to GND** and 0 when it is left open — the inputs are
> internally pulled up. `ENC_A` / `ENC_B` read 1 when open. Under `USE_LIMIT_SW (17) = 1`
> bits 2 and 4 tell you which rotation directions are currently permitted; see
> [Stop input](../README.md#stop-input-ctrl-connector).

> Generated from `src/mdrobot/mdrobot/registers.py` and `status.py`
> (C++ mirror: `mdrobot_cpp/registers.hpp`, `status.hpp`). If they change, update this
> table.
