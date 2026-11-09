/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Drive-gate configuration of the CTL-017 board port (NUCLEO-G474RE with
 * BLDC-SHIELD_IFX007T). HAL-free, so the host tests include it and check
 * that the skeleton keeps the half-bridges disabled.
 *
 * Every switch below is 0 until its owner item is closed. With any switch at
 * 0 the port never raises INH or IN on either half-bridge, whatever the core
 * commands. A move accepted on the bus then sees no Hall pulses and ends in a
 * fault (stall F03 or travel timeout F04), which is the intended visible
 * result on a bench with placeholder data.
 */
#ifndef CTL017_BOARD_DRIVE_CONFIG_H
#define CTL017_BOARD_DRIVE_CONFIG_H

#include "ctl017/drive_gate.h"

/* Current sense scale on PC1: microamps of motor current per ADC count.
 * TODO(Evans): IFX007T IS ratio and the sense resistor on the shield.
 * 0 = unknown: current reads 0 mA and the current limit cannot trip. */
#define BOARD_ISENSE_UA_PER_COUNT 0u

/* The PWM path on the IFX007T IN pins is not written yet (TODO(Evans):
 * timer and channels). board_main.c refuses to compile with 1 until it is. */
#define BOARD_HB_PWM_IMPLEMENTED 0

/* Owner confirmations, TODO(Evans). Set each to 1 only when the value it
 * names is in lift_config_default() from the DOLD data or the bench:
 *  - current limit: overcurrent_ma, overcurrent_ms, inrush_blank_ms;
 *  - speed data: hall_nm_per_count, move_speed_um_s, homing_speed_um_s,
 *    duty values, Hall-window rates, travel timeout factor and margin;
 *  - end-switch data: end_switches_wired and BOARD_LIMIT_ACTIVE_LOW. */
#define BOARD_CURRENT_LIMIT_CONFIRMED   0
#define BOARD_SPEED_DATA_CONFIRMED      0
#define BOARD_END_SWITCH_DATA_CONFIRMED 0

/* Gate from the switches above; hbridge_pins_assigned is filled by the port
 * from board_pins.h. */
static inline drive_gate_t board_drive_gate(bool hbridge_pins_assigned)
{
    drive_gate_t g;
    g.hbridge_pins_assigned = hbridge_pins_assigned;
    g.pwm_implemented = BOARD_HB_PWM_IMPLEMENTED != 0;
    g.isense_scale_set = BOARD_ISENSE_UA_PER_COUNT != 0u;
    g.current_limit_confirmed = BOARD_CURRENT_LIMIT_CONFIRMED != 0;
    g.speed_data_confirmed = BOARD_SPEED_DATA_CONFIRMED != 0;
    g.end_switch_data_confirmed = BOARD_END_SWITCH_DATA_CONFIRMED != 0;
    return g;
}

#endif /* CTL017_BOARD_DRIVE_CONFIG_H */
