/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Core status to mapped PDO variables, and the error register.
 */
#include "ctl017/ctl017_app.h"

void ctl017_fill_status1(const lift_status_t *st, uint8_t counter, ctl017_status1_t *out)
{
    out->state = st->state;
    out->flags = st->flags;
    out->position_01mm = st->position_01mm;
    out->active_stop_id = st->active_stop_id;
    out->last_seq = st->last_seq;
    out->cmd_result = st->cmd_result;
    out->status_counter = counter;
}

void ctl017_fill_status2(const lift_status_t *st, const aux_io_t *aux, ctl017_status2_t *out)
{
    uint8_t in = (uint8_t)(st->stop_sensors & 0x07u);
    if (aux_estop2_pressed(aux)) {
        in |= CTL017_IN_ESTOP2;
    }
    if (st->hall_supply_ok) {
        in |= CTL017_IN_HALL_SUPPLY;
    }
    in |= (uint8_t)(aux_spare_bits(aux) << 5);
    out->inputs = in;
    out->target_stop_id = st->target_stop_id;
    out->motor_current_ma = st->motor_current_ma;
    out->velocity_01mm_s = st->velocity_01mm_s;
    out->faults_lo = (uint8_t)(st->faults & 0xFFu);
    uint8_t f = 0;
    if ((st->faults & LIFT_FAULT_F09_STOP_SENSORS) != 0u) {
        f |= CTL017_AUX_F09;
    }
    if (aux->out.head_light_on) {
        f |= CTL017_AUX_LIGHT_ON;
    }
    if (aux->out.getac_enable) {
        f |= CTL017_AUX_GETAC_EN;
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
    if ((faults & LIFT_FAULT_F03_OVERCURRENT) != 0u) {
        r |= 0x02u; /* current */
    }
    if ((faults & LIFT_FAULT_F07_SUPERVISION) != 0u) {
        r |= 0x10u; /* communication */
    }
    return r;
}
