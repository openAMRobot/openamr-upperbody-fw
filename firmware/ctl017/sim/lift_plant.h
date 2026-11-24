/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Host-only kinematic model of the DOLD column, its Hall output, the three
 * stop sensors and the end switches, with fault injection. Used by the unit
 * tests and the vcan simulator. Not a model of the real column dynamics: all
 * numbers are test values, not hardware facts.
 */
#ifndef CTL017_LIFT_PLANT_H
#define CTL017_LIFT_PLANT_H

#include <stdbool.h>
#include <stdint.h>

#include "ctl017/lift.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    /* Model parameters */
    int64_t travel_nm;            /* lower end switch at 0, upper at travel_nm */
    int32_t speed_um_s;           /* at duty 1000 */
    int32_t nm_per_count;
    int32_t sensor_pos_um[LIFT_NUM_STOPS];
    int32_t sensor_band_um;       /* sensor active within +/- band of its position */
    uint16_t running_ma;
    int32_t coast_um;             /* travel after the drive switches off */

    /* State */
    int64_t pos_nm;
    int64_t coast_left_nm;
    int dir_last;
    int32_t count_offset;
    bool driving_now;

    /* Fault injection */
    bool motor_power;
    bool hall_supply_ok;
    bool hall_frozen;             /* Hall output stops (wire break) */
    int32_t hall_rate_pct;        /* 100 = nominal counts per distance */
    bool sensor_stuck_off[LIFT_NUM_STOPS];
    bool sensor_stuck_on[LIFT_NUM_STOPS];
    bool jammed;                  /* column does not move */
    uint16_t extra_current_ma;
    int32_t hall_count_frozen_value;
    int64_t hall_scaled_nm;
} lift_plant_t;

void plant_init(lift_plant_t *p, int64_t start_pos_nm);

/* Advance the model by dt_ms under the H-bridge command. */
void plant_step(lift_plant_t *p, const lift_hbridge_cmd_t *hb, uint32_t dt_ms);

/* Fill the inputs the board port would sample. */
void plant_inputs(const lift_plant_t *p, uint32_t now_ms, lift_inputs_t *in);

#ifdef __cplusplus
}
#endif

#endif
