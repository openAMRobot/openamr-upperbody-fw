/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 */
#include "lift_plant.h"

#include <string.h>

void plant_init(lift_plant_t *p, int64_t start_pos_nm)
{
    memset(p, 0, sizeof(*p));
    p->travel_nm = 360000000;      /* test value: 5 mm + 350 mm + 5 mm */
    p->speed_um_s = 18000;
    p->nm_per_count = 209200;
    p->sensor_pos_um[0] = 5000;
    p->sensor_pos_um[1] = 180000;
    p->sensor_pos_um[2] = 355000;
    p->sensor_band_um = 1000;
    p->running_ma = 2500;
    p->coast_um = 0;
    p->pos_nm = start_pos_nm;
    p->motor_power = true;
    p->hall_supply_ok = true;
    p->hall_rate_pct = 100;
    p->hall_scaled_nm = start_pos_nm;
    p->hall_count_frozen_value = (int32_t)(start_pos_nm / p->nm_per_count);
}

static bool driving(const lift_plant_t *p, const lift_hbridge_cmd_t *hb)
{
    return hb->enable && !hb->brake && hb->dir != LIFT_DIR_NONE && hb->duty_permille > 0 && p->motor_power;
}

void plant_step(lift_plant_t *p, const lift_hbridge_cmd_t *hb, uint32_t dt_ms)
{
    int64_t dx = 0;
    p->driving_now = driving(p, hb);
    if (p->driving_now && !p->jammed) {
        int64_t v_nm_ms = (int64_t)p->speed_um_s * hb->duty_permille / 1000; /* um/s = nm/ms */
        dx = v_nm_ms * dt_ms * (int)hb->dir;
        p->dir_last = (int)hb->dir;
        p->coast_left_nm = (int64_t)p->coast_um * 1000;
    } else if (p->coast_left_nm > 0 && !p->jammed) {
        int64_t step = (int64_t)p->speed_um_s * dt_ms / 2;
        if (step > p->coast_left_nm) {
            step = p->coast_left_nm;
        }
        p->coast_left_nm -= step;
        dx = step * p->dir_last;
    }
    int64_t before = p->pos_nm;
    p->pos_nm += dx;
    if (p->pos_nm < 0) {
        p->pos_nm = 0;
    }
    if (p->pos_nm > p->travel_nm) {
        p->pos_nm = p->travel_nm;
    }
    p->hall_scaled_nm += (p->pos_nm - before) * p->hall_rate_pct / 100;
    if (!p->hall_frozen && p->hall_supply_ok) {
        p->hall_count_frozen_value = (int32_t)(p->hall_scaled_nm / p->nm_per_count) + p->count_offset;
    }
}

void plant_inputs(const lift_plant_t *p, uint32_t now_ms, lift_inputs_t *in)
{
    memset(in, 0, sizeof(*in));
    in->now_ms = now_ms;
    if (p->hall_frozen || !p->hall_supply_ok) {
        in->hall_count = p->hall_count_frozen_value;
    } else {
        in->hall_count = (int32_t)(p->hall_scaled_nm / p->nm_per_count) + p->count_offset;
    }
    in->hall_supply_ok = p->hall_supply_ok;
    in->motor_power_present = p->motor_power;
    for (unsigned k = 0; k < LIFT_NUM_STOPS; k++) {
        int64_t d = p->pos_nm - (int64_t)p->sensor_pos_um[k] * 1000;
        bool on = d <= (int64_t)p->sensor_band_um * 1000 && d >= -(int64_t)p->sensor_band_um * 1000;
        if (p->sensor_stuck_off[k]) {
            on = false;
        }
        if (p->sensor_stuck_on[k]) {
            on = true;
        }
        in->stop_sensor[k] = on;
    }
    in->lower_limit = p->pos_nm <= 0;
    in->upper_limit = p->pos_nm >= p->travel_nm;
    in->motor_current_ma = (uint16_t)(p->extra_current_ma + (p->driving_now ? p->running_ma : 0));
}
