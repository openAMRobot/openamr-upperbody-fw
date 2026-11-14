/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * TEST DOUBLE, tests only. No motion logic. See lift_double.h.
 */
#include "lift_double.h"

#include <string.h>

void lift_double_init(lift_double_t *d)
{
    memset(d, 0, sizeof(*d));
}

void lift_double_script(lift_double_t *d, uint32_t from_ms, const ctl017_lift_status_t *st)
{
    if (d->n_script < LIFT_DOUBLE_SCRIPT_MAX) {
        d->script[d->n_script].from_ms = from_ms;
        d->script[d->n_script].status = *st;
        d->n_script++;
    }
}

void lift_double_status(const lift_double_t *d, uint32_t now_ms, ctl017_lift_status_t *out)
{
    ctl017_lift_status_unknown(out);
    for (unsigned i = 0; i < d->n_script; i++) {
        if ((int32_t)(now_ms - d->script[i].from_ms) >= 0) {
            *out = d->script[i].status;
        }
    }
}

void lift_double_observe(lift_double_t *d, const ctl017_iface_rx_t *rx)
{
    if (rx->lift_cmd.rx_count != d->last_rx_count) {
        d->last_rx_count = rx->lift_cmd.rx_count;
        if (d->n_log < LIFT_DOUBLE_LOG_MAX) {
            d->log[d->n_log++] = rx->lift_cmd;
        }
    }
}

bool lift_double_params_valid(void *ctx, const ctl017_lift_params_t *p)
{
    lift_double_t *d = (lift_double_t *)ctx;
    (void)p;
    d->validator_calls++;
    return d->params_ok;
}
