/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Placeholder defaults. See lift_config.h for the source or the TODO(Evans)
 * of every value.
 */
#include "ctl017/lift_config.h"

#include <string.h>

void lift_config_default(lift_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->hall_nm_per_count = 209200;       /* TODO(Evans) 0.2092 mm, to confirm */
    cfg->hall_count_up_positive = true;    /* TODO(Evans) */

    cfg->stop_pos_um[0] = 5000;            /* TODO(Evans) calibrate */
    cfg->stop_pos_um[1] = 180000;          /* TODO(Evans) calibrate */
    cfg->stop_pos_um[2] = 355000;          /* TODO(Evans) calibrate */
    cfg->stop_tol_um = 2000;               /* D3 */
    cfg->xcheck_tol_um = 2000;             /* TODO(Evans) */
    cfg->sensor_edge_offset_up_um = 0;     /* TODO(Evans) */
    cfg->sensor_edge_offset_down_um = 0;   /* TODO(Evans) */
    cfg->stop_lead_um = 0;                 /* TODO(Evans) */
    cfg->sensor_overrun_um = 3000;         /* TODO(Evans) */
    cfg->rereference_on_sensor = true;

    cfg->duty_move_permille = 1000;        /* TODO(Evans) reduced speed value */
    cfg->duty_homing_permille = 500;       /* TODO(Evans) */
    cfg->move_speed_um_s = 18000;          /* P-03 item 16 no-load; TODO(Evans) loaded */
    cfg->homing_speed_um_s = 9000;         /* TODO(Evans) */
    cfg->stop_mode_brake = false;          /* I8 rev 3 Q10: halt = H-bridge disabled */

    cfg->hall_window_ms = 200;             /* TODO(Evans) */
    cfg->hall_start_grace_ms = 300;        /* TODO(Evans) */
    cfg->move_rate_min_cps = 43;           /* TODO(Evans) */
    cfg->move_rate_max_cps = 130;          /* TODO(Evans) */
    cfg->homing_rate_min_cps = 15;         /* TODO(Evans) */
    cfg->homing_rate_max_cps = 86;         /* TODO(Evans) */
    cfg->stall_ms = 300;                   /* TODO(Evans) */
    cfg->wrong_dir_counts = 10;            /* TODO(Evans) */

    cfg->travel_timeout_factor_pct = 150;  /* TODO(Evans) */
    cfg->travel_timeout_margin_ms = 2000;  /* TODO(Evans) */

    cfg->overcurrent_ma = 7000;            /* TODO(Evans) DOLD max 7 A as default */
    cfg->overcurrent_ms = 100;             /* TODO(Evans) */
    cfg->inrush_blank_ms = 200;            /* TODO(Evans) */

    cfg->sensor_debounce_ms = 5;           /* TODO(Evans) */
    cfg->input_debounce_ms = 20;           /* TODO(Evans) */
    cfg->end_switches_wired = false;       /* TODO(Evans) */

    cfg->standstill_quiet_ms = 100;        /* TODO(Evans) */
    cfg->standstill_timeout_ms = 500;      /* F08 start value, Alex 27 Oct */

    cfg->rpdo_deadline_ms = 300;           /* D15 */

    cfg->homing_continue_to_stop0 = true;  /* D10 home down; work package: to L1000 */

    cfg->init_settle_ms = 50;              /* TODO(Evans) */
}

bool lift_config_valid(const lift_config_t *cfg)
{
    if (cfg->hall_nm_per_count <= 0) {
        return false;
    }
    for (unsigned k = 1; k < LIFT_NUM_STOPS; k++) {
        if (cfg->stop_pos_um[k] <= cfg->stop_pos_um[k - 1]) {
            return false;
        }
    }
    if (cfg->stop_pos_um[0] < 0 || cfg->stop_pos_um[LIFT_NUM_STOPS - 1] > 6553000) {
        return false; /* must fit LiftStatus.position (u16, 0.1 mm) */
    }
    if (cfg->stop_tol_um <= 0 || cfg->xcheck_tol_um <= 0) {
        return false;
    }
    if (cfg->duty_move_permille == 0 || cfg->duty_move_permille > 1000 ||
        cfg->duty_homing_permille == 0 || cfg->duty_homing_permille > 1000) {
        return false;
    }
    if (cfg->move_speed_um_s <= 0 || cfg->homing_speed_um_s <= 0) {
        return false;
    }
    if (cfg->move_rate_min_cps > cfg->move_rate_max_cps ||
        cfg->homing_rate_min_cps > cfg->homing_rate_max_cps || cfg->hall_window_ms == 0) {
        return false;
    }
    if (cfg->travel_timeout_factor_pct < 100) {
        return false;
    }
    return true;
}
