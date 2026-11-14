/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * TEST DOUBLE of the lift side of firmware/ctl017/iface. Tests only.
 *
 * It never moves and has no motion logic: it records every command record
 * it is shown and reports whatever status the test scripted for that tick.
 * It does not evaluate commands, sequence numbers, heartbeats or permits.
 */
#ifndef CTL017_LIFT_DOUBLE_H
#define CTL017_LIFT_DOUBLE_H

#include <stdbool.h>
#include <stdint.h>

#include "ctl017/lift_iface.h"

#define LIFT_DOUBLE_SCRIPT_MAX 16u
#define LIFT_DOUBLE_LOG_MAX 256u

typedef struct {
    uint32_t from_ms;               /* status applies from this time on */
    ctl017_lift_status_t status;
} lift_double_step_t;

typedef struct {
    lift_double_step_t script[LIFT_DOUBLE_SCRIPT_MAX];
    unsigned n_script;
    ctl017_lift_cmd_rx_t log[LIFT_DOUBLE_LOG_MAX]; /* new command records seen */
    unsigned n_log;
    uint32_t last_rx_count;
    bool params_ok;                 /* answer of the parameter validator */
    unsigned validator_calls;
} lift_double_t;

void lift_double_init(lift_double_t *d);
/* Adds a scripted status, in time order. */
void lift_double_script(lift_double_t *d, uint32_t from_ms, const ctl017_lift_status_t *st);
/* Status for time now (the unknown status before the first script entry). */
void lift_double_status(const lift_double_t *d, uint32_t now_ms, ctl017_lift_status_t *out);
/* Records the command record if its rx_count changed. */
void lift_double_observe(lift_double_t *d, const ctl017_iface_rx_t *rx);
/* Parameter validator with a scripted answer. */
bool lift_double_params_valid(void *ctx, const ctl017_lift_params_t *p);

#endif
