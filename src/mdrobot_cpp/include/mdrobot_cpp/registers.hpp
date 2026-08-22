// Copyright 2026 Taesu Yim. Licensed under Apache-2.0.
/// @file registers.hpp
/// PID / CMD named-constant map — mirrors Python registers.py.

#pragma once

#include <cstdint>

namespace mdrobot {

// --- single-byte command/setting PIDs (0-127) ---
constexpr uint16_t PID_VERSION       = 1;
constexpr uint16_t PID_TQ_OFF        = 5;
constexpr uint16_t PID_BRAKE         = 6;
constexpr uint16_t PID_COMMAND       = 10;
constexpr uint16_t PID_ALARM_RESET   = 12;
constexpr uint16_t PID_POSI_RESET    = 13;
constexpr uint16_t PID_INV_SIGN_CMD  = 16;
constexpr uint16_t PID_USE_LIMIT_SW  = 17;
constexpr uint16_t PID_HALL_TYPE     = 21;  // pole/hall type (2 = 10 poles); counts/rev = 3 x poles
constexpr uint16_t PID_INPUT_TYPE    = 25;
constexpr uint16_t PID_POS_SEN_TYPE  = 26;  // position signal type (0 HALL, 1 EPOSI; RS485 spec V6.55);
                                            // follows PID_USE_EPOSI automatically in both directions
constexpr uint16_t PID_USE_LIMIT_SW2 = 29;
constexpr uint16_t PID_CTRL_STATUS   = 34;
constexpr uint16_t PID_USE_EPOSI     = 46;  // position source (0 hall, 1 encoder); default 0. RS485 spec
                                            // V6.55 (absent from the 2021 Modbus PDF). Verified MD400 v8.6:
                                            // counts/rev becomes 4 x ENC_PPR and the +/- direction flips
                                            // vs hall mode. EEPROM; immediate effect. Not CMD namespace.
                                            // See SingleMotorDriver::set_use_encoder_position.
constexpr uint16_t PID_DI            = 48;
constexpr uint16_t PID_IN_POSITION_OK = 49;
constexpr uint16_t PID_UI_COM        = 78;
constexpr uint16_t PID_ENC_INV_DIR   = 80;  // encoder A/B phase swap (0 normal, 1 swap)
constexpr uint16_t PID_START_STOP    = 100;

// --- word command/data PIDs (101-190) ---
constexpr uint16_t PID_VEL_CMD       = 130;
constexpr uint16_t PID_VEL_CMD2      = 131;
constexpr uint16_t PID_ID            = 133;
constexpr uint16_t PID_OPEN_VEL_CMD  = 134;
constexpr uint16_t PID_BAUDRATE      = 135;
constexpr uint16_t PID_INT_RPM_DATA  = 138;
constexpr uint16_t PID_TQ_DATA       = 139;
constexpr uint16_t PID_TQ_CMD        = 140;
constexpr uint16_t PID_VOLT_IN       = 143;
constexpr uint16_t PID_RETURN_TYPE   = 149;
constexpr uint16_t PID_TAR_VEL       = 155;
constexpr uint16_t PID_ENC_PPR       = 156;  // 0 = no encoder (hall closed-loop); recent fw ships in encoder mode.
                                             // Verified: feeds the VELOCITY loop only - position stays on the hall
                                             // counter (3 x poles per rev). See SingleMotorDriver::set_encoder_ppr.
constexpr uint16_t PID_REF_RPM       = 166;
constexpr uint16_t PID_PNT_TQ_OFF    = 174;
constexpr uint16_t PID_PNT_BRAKE     = 175;
constexpr uint16_t PID_TAR_POSI_VEL  = 176;
constexpr uint16_t PID_COM_TAR_SPEED = 180;

// --- acceleration / deceleration: slow-start / slow-down ---
// NOT yet hardware-verified.
constexpr uint16_t PID_MAX_SS_TIME   = 57;
constexpr uint16_t PID_MIN_SSSD      = 124;
constexpr uint16_t PID_SLOW_START    = 153;
constexpr uint16_t PID_SLOW_DOWN     = 154;
constexpr uint16_t PID_POSI_SS       = 178;
constexpr uint16_t PID_POSI_SD       = 179;
constexpr uint16_t PID_SLOW_START1   = 108;
constexpr uint16_t PID_SLOW_START2   = 109;
constexpr uint16_t PID_SLOW_DOWN1    = 111;
constexpr uint16_t PID_SLOW_DOWN2    = 112;
constexpr uint16_t PID_POSI_SS1      = 113;
constexpr uint16_t PID_POSI_SS2      = 114;
constexpr uint16_t PID_POSI_SD1      = 115;
constexpr uint16_t PID_POSI_SD2      = 116;

// --- N-byte data/command PIDs (193-253) ---
constexpr uint16_t PID_MONITOR       = 196;
constexpr uint16_t PID_POSI_DATA     = 197;
constexpr uint16_t PID_POSI_SET1     = 198;
constexpr uint16_t PID_GAIN          = 203;
constexpr uint16_t PID_PNT_POSI_VEL_CMD = 206;
constexpr uint16_t PID_PNT_VEL_CMD   = 207;
constexpr uint16_t PID_PNT_OPEN_VEL_CMD = 208;
constexpr uint16_t PID_PNT_TQ_CMD    = 209;
constexpr uint16_t PID_PNT_MAIN_DATA = 210;
constexpr uint16_t PID_PNT_INC_POSI_CMD = 215;
constexpr uint16_t PID_PNT_MONITOR   = 216;
constexpr uint16_t PID_POSI_SET      = 217;
constexpr uint16_t PID_POSI_SET2     = 218;
constexpr uint16_t PID_POSI_VEL_CMD  = 219;
constexpr uint16_t PID_INC_POSI_VEL_CMD = 220;
constexpr uint16_t PID_MAX_RPM       = 221;
constexpr uint16_t PID_INC_POSI_VEL_CMD2 = 224;
constexpr uint16_t PID_POSI_VEL_CMD2 = 236;
constexpr uint16_t PID_PNT_INC_POSI_VEL_CMD = 242;
constexpr uint16_t PID_POSI_CMD      = 243;
constexpr uint16_t PID_INC_POSI_CMD  = 244;
constexpr uint16_t PID_PNT_POSI_CMD  = 246;
constexpr uint16_t PID_POSI_CMD2     = 247;

// --- CMD numbers sent through PID_COMMAND(10) gateway ---
constexpr uint16_t CMD_TQ_OFF         = 2;
constexpr uint16_t CMD_BRAKE          = 4;
constexpr uint16_t CMD_ALARM_RESET    = 8;
constexpr uint16_t CMD_POSI_RESET     = 10;
constexpr uint16_t CMD_TAR_VEL_OFF    = 20;
constexpr uint16_t CMD_SLOW_START_OFF = 21;
constexpr uint16_t CMD_SLOW_DOWN_OFF  = 22;
constexpr uint16_t CMD_EMER_ON        = 67;
constexpr uint16_t CMD_EMER_OFF       = 68;
constexpr uint16_t CMD_BRAKE1         = 69;
constexpr uint16_t CMD_BRAKE2         = 70;
constexpr uint16_t CMD_POSI_SS_OFF    = 71;
constexpr uint16_t CMD_POSI_SD_OFF    = 72;
constexpr uint16_t CMD_RESET_SYSTEM   = 79;

}  // namespace mdrobot
