"""PID / CMD named-constant map.

All PIDs/CMDs/status bits are defined as named constants; high-level code never
uses numeric literals. The names follow the MDROBOT protocol command/PID map.

This module defines the priority PIDs/CMDs actually used by the driver. To add a
new PID, look up its value in the protocol map and add it here rather than writing
the number inline.
"""

from __future__ import annotations

# --- single-byte command/setting PIDs (0-127) ---------------------------------------
PID_VERSION = 1            # R  firmware/protocol version (DL=13 -> v1.3); ping candidate
PID_TQ_OFF = 5             # W  motor free / natural stop
PID_BRAKE = 6              # W  electric brake
PID_COMMAND = 10           # W  CMD command gateway (use CMD_* below)
PID_ALARM_RESET = 12       # W  reset alarm
PID_POSI_RESET = 13        # W  reset position to zero
PID_INV_SIGN_CMD = 16      # R/W reference command sign inverse
PID_USE_LIMIT_SW = 17      # R/W CTRL limit switch function (0 cancel, 1 use); default 1
PID_HALL_TYPE = 21         # R/W motor pole / hall type (0:4p, 1:8p, 2:10p, 3:12p, 4:2p, 5:6p).
                           #    sets the hall resolution: counts/rev = 3 x poles (10 poles -> 30).
PID_INPUT_TYPE = 25        # R/W user input type
PID_POS_SEN_TYPE = 26      # R/W position signal type (0 HALL, 1 EPOSI, 2 POT, 3/4 MENA; RS485 spec
                           #    V6.55). Follows PID_USE_EPOSI automatically in BOTH directions
                           #    (hardware-verified) - no need to write it directly.
PID_USE_LIMIT_SW2 = 29     # R/W motor-2 limit switch function (dual); same meaning as PID 17
PID_CTRL_STATUS = 34       # R  status bit map
PID_USE_EPOSI = 46         # R/W position source (0 hall counter, 1 encoder); default 0. From the
                           #    RS485 spec V6.55 - absent from the 2021 Modbus protocol PDF.
                           #    hardware-verified 2026-08-22 (MD400 v8.6): reported position AND
                           #    position control switch to encoder counts (counts/rev = 4 x ENC_PPR)
                           #    and the physical +/- direction flips vs hall mode. Takes effect
                           #    immediately; stored in EEPROM. Not the CMD namespace (CMD 46 differs).
                           #    See SingleMotorDriver.set_use_encoder_position for the safety notes.
PID_DI = 48                # R  digital input bits; essential for hardware diagnostics
PID_IN_POSITION_OK = 49    # R  position control done (0/1)
PID_UI_COM = 78            # R/C serial communication control (0 = CTRL I/O, 1 = serial only)
PID_ENC_INV_DIR = 80       # R/W encoder A/B phase swap (0 normal, 1 swap)
PID_START_STOP = 100       # C  start/stop (0 stop, 1 CCW, 2 CW); run-latch arm.
                           #    verified on hardware: must be 1 to enable serial velocity drive.

# --- word command/data PIDs (101-190) -----------------------------------------------
PID_VEL_CMD = 130          # C  velocity command, signed rpm; core set_velocity
PID_VEL_CMD2 = 131         # C  motor-2 velocity command (dual)
PID_ID = 133               # W  controller ID setting (write check 0xAA)
PID_OPEN_VEL_CMD = 134     # C  open-loop output command (-1023..1023); DANGEROUS
PID_BAUDRATE = 135         # W  RS485 baudrate setting (write check required)
PID_INT_RPM_DATA = 138     # R  motor speed, signed rpm; get_speed
PID_TQ_DATA = 139          # R  current, 0.1 A units; get_current
PID_TQ_CMD = 140           # C  torque/current command (-1023..1023)
PID_VOLT_IN = 143          # R  supply voltage, 0.1 V units; get_voltage
PID_RETURN_TYPE = 149      # R/W return type after command
PID_TAR_VEL = 155          # R/W fixed target speed, rpm.
                           #    verified on hardware: NOT the speed source for START_STOP drive -> use PID_COM_TAR_SPEED.
PID_ENC_PPR = 156          # R/W encoder pulses-per-rev; 0 = no encoder (hall closed-loop).
                           #    recent firmware (e.g. MD400 v8.6) ships in encoder mode -> set 0 for hall drive.
                           #    hardware-verified: this feeds the VELOCITY loop only. Reported position
                           #    stays on the hall counter (3 x poles per rev) with or without an encoder.
                           #    See SingleMotorDriver.set_encoder_ppr for the safety note on wrong values.
PID_REF_RPM = 166          # R  reference velocity, signed rpm
PID_PNT_TQ_OFF = 174       # C  free/tq-off for both motors (DL motor1, DH motor2)
PID_PNT_BRAKE = 175        # C  electric brake for both motors (DL motor1, DH motor2)
PID_TAR_POSI_VEL = 176     # R/W max speed in position control, rpm
PID_COM_TAR_SPEED = 180    # C/R target speed used by PID_START_STOP, rpm.
                           #    verified on hardware: COM_TAR_SPEED + START_STOP(1/2) drives the motor.

# --- acceleration / deceleration: slow-start / slow-down ----------------------------
# Speed slow (153/154 single, 108/109/111/112 dual) hardware-verified 2026-06-22
# (Phase 12). A raw value 0-1023 maps to 0..PID_MAX_SS_TIME seconds (full scale 15 s
# on tested devices). Position slow (178/179, 113-116) still protocol-doc-based. Use the
# helpers in units.py (slow_seconds_to_raw / slow_raw_to_seconds) for the conversion.
PID_MAX_SS_TIME = 57       # R/W max slow-start time, 15-60 s (full scale for slow values)
PID_MIN_SSSD = 124         # R/W min slow-start/down parameter, 0-1023
PID_SLOW_START = 153       # R/W speed slow-start (single/global), 0-1023
PID_SLOW_DOWN = 154        # R/W speed slow-down (single/global), 0-1023
PID_POSI_SS = 178          # R/W position slow-start (single), 0-1023
PID_POSI_SD = 179          # R/W position slow-down (single), 0-1023
PID_SLOW_START1 = 108      # R/W MOT1 speed slow-start (dual), 0-1023
PID_SLOW_START2 = 109      # R/W MOT2 speed slow-start (dual), 0-1023
PID_SLOW_DOWN1 = 111       # R/W MOT1 speed slow-down (dual), 0-1023
PID_SLOW_DOWN2 = 112       # R/W MOT2 speed slow-down (dual), 0-1023
PID_POSI_SS1 = 113         # R/W MOT1 position slow-start (dual), 0-1023
PID_POSI_SS2 = 114         # R/W MOT2 position slow-start (dual), 0-1023
PID_POSI_SD1 = 115         # R/W MOT1 position slow-down (dual), 0-1023
PID_POSI_SD2 = 116         # R/W MOT2 position slow-down (dual), 0-1023

# --- N-byte data/command PIDs (193-253) ---------------------------------------------
PID_MONITOR = 196          # R  single monitor (12 bytes: speed/current/output/position/status)
PID_POSI_DATA = 197        # R  motor position (4-byte long); get_position
PID_POSI_SET1 = 198        # C  set motor1 position (4-byte long)
PID_GAIN = 203             # C/R position P, speed P, speed I gain (6 bytes)
PID_PNT_POSI_VEL_CMD = 206 # C  dual position + max speed (12 bytes); key dual-channel command
PID_PNT_VEL_CMD = 207      # C  dual velocity command (4 bytes: speed1, speed2)
PID_PNT_OPEN_VEL_CMD = 208 # C  dual open-loop command (4 bytes)
PID_PNT_TQ_CMD = 209       # C  dual torque/current command (4 bytes)
PID_PNT_MAIN_DATA = 210    # R  dual main data (18 bytes)
PID_PNT_INC_POSI_CMD = 215 # C  dual incremental position (8 bytes)
PID_PNT_MONITOR = 216      # R  dual monitor (14 bytes)
PID_POSI_SET = 217         # C  set motor position (4-byte long)
PID_POSI_SET2 = 218        # C  set motor2 position (4-byte long, dual)
PID_POSI_VEL_CMD = 219     # C  position control with max speed (6 bytes); core move_to
PID_INC_POSI_VEL_CMD = 220 # C  incremental position with max speed (6 bytes); core move_by
PID_MAX_RPM = 221          # R/W max speed (2-byte word)
PID_INC_POSI_VEL_CMD2 = 224  # C  motor-2 incremental position + speed (dual)
PID_POSI_VEL_CMD2 = 236    # C  motor-2 position control with max speed (dual)
PID_PNT_INC_POSI_VEL_CMD = 242  # C  dual incremental position + max speed (12 bytes)
PID_POSI_CMD = 243         # W  target position (4-byte long)
PID_INC_POSI_CMD = 244     # W  incremental target position (4-byte long)
PID_PNT_POSI_CMD = 246     # C  dual absolute position (8 bytes)
PID_POSI_CMD2 = 247        # C  motor-2 target position (dual)

# --- CMD numbers sent through the PID_COMMAND(10) gateway ----------------------------
CMD_TQ_OFF = 2             # motor free state
CMD_BRAKE = 4              # electric brake
CMD_ALARM_RESET = 8        # reset alarm
CMD_POSI_RESET = 10        # position reset to zero
CMD_TAR_VEL_OFF = 20       # erase target velocity set by PID_TAR_VEL/COM_TAR_SPEED
CMD_SLOW_START_OFF = 21    # erase speed slow-start value
CMD_SLOW_DOWN_OFF = 22     # erase speed slow-down value
CMD_EMER_ON = 67           # stop motor + electric brake + UVW short; emergency-stop candidate
CMD_EMER_OFF = 68          # free state / Tq-off; DANGEROUS
CMD_BRAKE1 = 69            # electric brake on motor1 (dual)
CMD_BRAKE2 = 70            # electric brake on motor2 (dual)
CMD_POSI_SS_OFF = 71       # do not use the position slow-start parameter
CMD_POSI_SD_OFF = 72       # do not use the position slow-down parameter
CMD_RESET_SYSTEM = 79      # controller reboot; DANGEROUS, raw-only or via an explicit API
