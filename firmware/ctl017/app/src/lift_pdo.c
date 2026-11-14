/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Interface status to mapped PDO variables, and the error register.
 */
#include "ctl017/ctl017_app.h"

void ctl017_fill_status1(const ctl017_lift_status_t *st, uint8_t counter, ctl017_status1_t *out)
{
    out->state = st->state;
    out->flags = st->flags;
    out->position_01mm =
        ((st->flags & CTL017_FLAG_POSITION_VALID) != 0u) ? st->position_01mm : CTL017_POSITION_UNKNOWN;
    out->active_stop_id = st->active_stop_id;
    out->last_seq = st->last_seq;
    out->cmd_result = st->cmd_result;
    out->status_counter = counter;
}

void ctl017_fill_status2(const ctl017_lift_status_t *st, ctl017_status2_t *out)
{
    out->inputs = st->inputs;
    out->target_stop_id = st->target_stop_id;
    out->motor_current_ma = st->motor_current_ma;
    out->velocity_01mm_s = st->velocity_01mm_s;
    out->faults_lo = (uint8_t)(st->faults & 0xFFu);
    uint8_t f = (uint8_t)(st->aux_flags & (CTL017_AUX_LIGHT_ON | CTL017_AUX_GETAC_EN));
    if ((st->faults & CTL017_FAULT_F09_STOP_SENSORS) != 0u) {
        f |= CTL017_AUX_F09;
    }
    out->aux_flags = f;
}

uint8_t ctl017_error_register(uint16_t faults)
{
    uint8_t r = 0;
    if (faults != 0u) {
        r |= 0x01u; /* generic */
        r |= 0x80u; /* manufacturer specific: lift fault bitmask in 0x2002 */
    }
    if ((faults & CTL017_FAULT_F03_OVERCURRENT) != 0u) {
        r |= 0x02u; /* current */
    }
    if ((faults & CTL017_FAULT_F07_SUPERVISION) != 0u) {
        r |= 0x10u; /* communication */
    }
    return r;
}
