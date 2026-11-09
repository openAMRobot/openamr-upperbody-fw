/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 */
#include "ctl017/ctl017_app.h"

#include <string.h>

_Static_assert(sizeof(bool) == 1, "BOOLEAN OD entries write one byte");

/* TPDO change detection: every byte except the status counter (TPDO1);
 * for TPDO2 the inputs, target, faults and aux flags (current and velocity
 * go out with the event timer only). */
#define TPDO1_CHANGE_MASK 0x7Fu
#define TPDO2_CHANGE_MASK 0xC3u

static ctl017_app_t *ctx_app(void *ctx)
{
    return (ctl017_app_t *)ctx;
}

static bool cb_send(void *ctx, const co_frame_t *f)
{
    ctl017_app_t *a = ctx_app(ctx);
    return (a->send != NULL) ? a->send(a->send_ctx, f) : false;
}

static void cb_before_tpdo(void *ctx, uint8_t n)
{
    ctl017_app_t *a = ctx_app(ctx);
    lift_status_t st;
    lift_get_status(&a->lift, &st);
    if (n == 0u) {
        ctl017_fill_status1(&st, a->st1.status_counter, &a->st1);
    } else {
        ctl017_fill_status2(&st, &a->aux, &a->st2);
    }
}

static void cb_tpdo_sent(void *ctx, uint8_t n)
{
    ctl017_app_t *a = ctx_app(ctx);
    if (n == 0u) {
        a->st1.status_counter++; /* wraps 255 to 0 */
    }
}

/* D12 alternative (a): the core heartbeat counts as fresh only while the
 * consumer is ACTIVE (received within 0x1016:01). WAITING (none since
 * start-up or since reset communication), LOST and OFF (consumer disabled by
 * SDO) are stale: the lift refuses new moves and stops a move in progress. */
static void sync_supervision(ctl017_app_t *a)
{
    lift_set_supervision_lost(&a->lift, a->co.hbc != CO_HBC_ACTIVE);
}

static void cb_rpdo(void *ctx, uint8_t n, uint32_t now_ms)
{
    ctl017_app_t *a = ctx_app(ctx);
    if (n == 0u) {
        sync_supervision(a);
        (void)lift_command(&a->lift, a->cmd.seq, a->cmd.op, a->cmd.stop_id, now_ms);
    } else {
        uint8_t dim = a->auxcmd.head_light_dim_pct > 100u ? 100u : a->auxcmd.head_light_dim_pct;
        aux_command(&a->aux, a->auxcmd.head_light_on != 0u, dim, a->auxcmd.getac_enable != 0u);
    }
}

static void cb_nmt(void *ctx, co_nmt_state_t from, co_nmt_state_t to)
{
    ctl017_app_t *a = ctx_app(ctx);
    if (from == CO_NMT_OPERATIONAL && to != CO_NMT_OPERATIONAL) {
        /* No more RPDOs can arrive: stop at once rather than waiting for
         * the command deadline. */
        lift_halt(&a->lift);
    }
}

static void cb_reset_node(void *ctx)
{
    ctl017_app_t *a = ctx_app(ctx);
    lift_halt(&a->lift);
    a->reset_requested = true;
}

static void cb_hb_consumer(void *ctx, bool lost)
{
    ctl017_app_t *a = ctx_app(ctx);
    sync_supervision(a);
    if (lost) {
        aux_comm_lost(&a->aux);
    }
}

bool ctl017_app_init(ctl017_app_t *a, const lift_config_t *lcfg, const aux_config_t *acfg,
                     ctl017_send_fn send, void *send_ctx, uint32_t now_ms)
{
    memset(a, 0, sizeof(*a));
    a->send = send;
    a->send_ctx = send_ctx;
    a->now_ms = now_ms;
    lift_init(&a->lift, lcfg, now_ms);
    aux_init(&a->aux, acfg, now_ms);
    a->auxcmd.head_light_on = 0;
    a->auxcmd.head_light_dim_pct = acfg->head_light_dim_default_pct;
    a->auxcmd.getac_enable = acfg->getac_enable_at_boot ? 1u : 0u;

    co_callbacks_t cb = {
        .send = cb_send,
        .before_tpdo = cb_before_tpdo,
        .tpdo_sent = cb_tpdo_sent,
        .rpdo_received = cb_rpdo,
        .nmt_changed = cb_nmt,
        .reset_node = cb_reset_node,
        .hb_consumer = cb_hb_consumer,
    };
    if (!co_init(&a->co, CTL017_NODE_ID, a->od, 0, &cb, a)) {
        return false;
    }
    a->co.hb_producer_ms = CTL017_HB_PRODUCER_MS;
    a->co.hb_consumer_entry = ((uint32_t)CTL017_CORE_NODE_ID << 16) | CTL017_HB_CONSUMER_MS;

    ctl017_od_build(a);
    a->co.od = a->od;
    a->co.od_len = a->od_len;
    if (!a->od_ok) {
        return false;
    }

    co_tpdo_cfg_t t1 = {(uint16_t)(a->tpdo_cob[0] & 0x7FFu), &a->tpdo_inhibit[0], &a->tpdo_event[0],
                        a->map_tpdo1, a->n_map_tpdo[0], TPDO1_CHANGE_MASK};
    co_tpdo_cfg_t t2 = {(uint16_t)(a->tpdo_cob[1] & 0x7FFu), &a->tpdo_inhibit[1], &a->tpdo_event[1],
                        a->map_tpdo2, a->n_map_tpdo[1], TPDO2_CHANGE_MASK};
    co_rpdo_cfg_t r1 = {(uint16_t)(a->rpdo_cob[0] & 0x7FFu), a->map_rpdo1, a->n_map_rpdo[0]};
    co_rpdo_cfg_t r2 = {(uint16_t)(a->rpdo_cob[1] & 0x7FFu), a->map_rpdo2, a->n_map_rpdo[1]};
    if (!co_add_tpdo(&a->co, &t1) || !co_add_tpdo(&a->co, &t2) || !co_add_rpdo(&a->co, &r1) ||
        !co_add_rpdo(&a->co, &r2)) {
        return false;
    }
    /* The mapped payloads must be exactly the documented 8 + 8 and 3 + 3 bytes. */
    if (a->co.tpdo[0].len != 8u || a->co.tpdo[1].len != 8u || a->co.rpdo[0].len != 3u ||
        a->co.rpdo[1].len != 3u) {
        return false;
    }
    co_start(&a->co, now_ms);
    return true;
}

void ctl017_app_rx(ctl017_app_t *a, const co_frame_t *f, uint32_t now_ms)
{
    a->now_ms = now_ms;
    co_process(&a->co, f, now_ms);
}

static void emit_emcy(ctl017_app_t *a)
{
    uint16_t raised = lift_take_new_faults(&a->lift);
    uint16_t faults = a->lift.faults;
    a->faults = faults;
    a->co.error_register = ctl017_error_register(faults);
    for (uint8_t n = 1; n <= 9u; n++) {
        if ((raised & (1u << (n - 1u))) != 0u) {
            uint8_t mfr[5] = {n, (uint8_t)faults, (uint8_t)(faults >> 8), 0, 0};
            co_emcy(&a->co, (uint16_t)(0xFF00u | n), mfr);
        }
    }
    if (faults == 0u && a->last_faults != 0u) {
        uint8_t mfr[5] = {0, 0, 0, 0, 0};
        co_emcy(&a->co, 0x0000, mfr); /* error reset */
    }
    a->last_faults = faults;
}

const ctl017_outputs_t *ctl017_app_step(ctl017_app_t *a, const lift_inputs_t *lin,
                                        const aux_inputs_t *ain)
{
    a->now_ms = lin->now_ms;
    a->hall_count_raw = lin->hall_count;
    sync_supervision(a);
    const lift_hbridge_cmd_t *hb = lift_step(&a->lift, lin);
    const aux_outputs_t *ao = aux_step(&a->aux, ain);
    emit_emcy(a);
    a->pdo_len_errors = a->co.rx_pdo_len_errors;
    co_step(&a->co, lin->now_ms);
    a->out.hbridge = *hb;
    a->out.aux = *ao;
    a->out.reset_requested = a->reset_requested;
    return &a->out;
}
