/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Drive gate for the IFX007T half-bridges of CTL-017.
 *
 * The lift core computes an H-bridge command; this gate decides whether the
 * board may turn it into pin levels. The gate stays closed (both INH low,
 * both IN low, half-bridges high impedance) until every item below is true.
 * Each item stands for data or code that is still open in the skeleton
 * (TODO(Evans)): the H-bridge pins, the PWM path on IN, the current-sense
 * scale, and the owner's confirmation of the current limit, the speed data
 * behind the Hall window and travel timeout, and the DOLD end-switch data.
 *
 * Not a safety function: the hardwired PNOZ chain removes lift motor power
 * independently of this code (decisions D6, D13). The gate only keeps a
 * build with placeholder data from driving the motor.
 *
 * Portable C11, no HAL.
 */
#ifndef CTL017_DRIVE_GATE_H
#define CTL017_DRIVE_GATE_H

#include <stdbool.h>

#include "ctl017/lift.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool hbridge_pins_assigned;     /* IN_A, IN_B, INH_A, INH_B all assigned */
    bool pwm_implemented;           /* duty applied through a timer channel on IN */
    bool isense_scale_set;          /* current-sense scale known: current limit active */
    bool current_limit_confirmed;   /* overcurrent level and times from the motor data */
    bool speed_data_confirmed;      /* loaded speed and Hall scale behind the Hall window and timeout */
    bool end_switch_data_confirmed; /* DOLD end-switch wiring and polarity */
} drive_gate_t;

/* Pin levels for the two IFX007T half-bridges (true = high). */
typedef struct {
    bool inh_a;
    bool inh_b;
    bool in_a;
    bool in_b;
} ifx007t_levels_t;

/* True only if every item of the gate is true. */
bool drive_gate_open(const drive_gate_t *g);

/* Maps an H-bridge command to pin levels. With the gate closed, or with the
 * command disabled, all four levels are low. Brake: both INH high, both IN
 * low (both low sides on). Drive: both INH high, IN_A high for up, IN_B
 * high for down. */
ifx007t_levels_t ifx007t_levels(const drive_gate_t *g, const lift_hbridge_cmd_t *cmd);

#ifdef __cplusplus
}
#endif

#endif /* CTL017_DRIVE_GATE_H */
