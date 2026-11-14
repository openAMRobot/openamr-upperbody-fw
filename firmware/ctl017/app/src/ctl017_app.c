/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 CANopen application layer. Data in, data out: see ctl017_app.h.
 */
#include "ctl017/ctl017_app.h"

#include <string.h>

_Static_assert(sizeof(bool) == 1, "BOOLEAN OD entries write one byte");

/* TPDO change detection: every byte except the status counter (TPDO1);
 * for TPDO2 the inputs, target, faults and aux flags (current and velocity
 * go out with the event timer only). Part of the v1 wire contract. */
#define TPDO1_CHANGE_MASK 0x7Fu
#define TPDO2_CHANGE_MASK 0xC3u

#define RPDO_LIFT   0u
#define RPDO_AUX    1u
#define RPDO_PERMIT 2u

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
    if (n == 0u) {
        ctl017_fill_status1(&a->status, a->st1.status_counter, &a->st1);
    } else {
        ctl017_fill_status2(&a->status, &a->st2);
    }
}

static void cb_tpdo_sent(void *ctx, uint8_t n)
{
    ctl017_app_t *a = ctx_app(ctx);
    if (n == 0u) {
        a->st1.status_counter++; /* wraps 255 to 0 */
    }
}

static bool aux_in_range(const ctl017_aux_cmd_t *c)
{
    return c->head_light_on <= 1u && c->head_light_dim_pct <= 100u && c->getac_enable <= 1u;
}

static void publish_aux(ctl017_app_t *a)
{
    ctl017_aux_cmd_rx_t *r = &a->rx.aux_cmd;
    r->valid = true;
    r->head_light_on = a->auxcmd.head_light_on;
    r->head_light_dim_pct = a->auxcmd.head_light_dim_pct;
    r->getac_enable = a->auxcmd.getac_enable;
    r->rx_count++;
    r->rx_ms = a->now_ms;
}

static void cb_rpdo(void *ctx, uint8_t n, uint32_t now_ms)
{
    ctl017_app_t *a = ctx_app(ctx);
    a->now_ms = now_ms;
    if (n == RPDO_LIFT) {
        /* Raw copy. Duplicates and sequence rules are the lift owner's:
         * rx_count tells a repeated frame from no frame. */
        ctl017_lift_cmd_rx_t *r = &a->rx.lift_cmd;
        r->valid = true;
        r->seq = a->cmd.seq;
        r->op = a->cmd.op;
        r->stop_id = a->cmd.stop_id;
        r->rx_count++;
        r->rx_ms = now_ms;
    } else if (n == RPDO_AUX) {
        if (!aux_in_range(&a->auxcmd)) {
            /* A malformed request is a failure, never a new sample: restore
             * the last accepted values and count it. */
            a->aux_range_errors++;
            a->auxcmd.head_light_on = a->rx.aux_cmd.head_light_on;
            a->auxcmd.head_light_dim_pct = a->rx.aux_cmd.head_light_dim_pct;
            a->auxcmd.getac_enable = a->rx.aux_cmd.getac_enable;
            return;
        }
        publish_aux(a);
    }
#ifdef CTL017_PERMIT_OD
    else if (n == RPDO_PERMIT) {
        /* Data only: copied for the lift owner, never acted on here. */
        ctl017_permit_rx_t *p = &a->rx.permit;
        p->valid = true;
        p->permit_flag = co_od_get(a->permit_od.e[0]);
        p->producer_boot_id = co_od_get(a->permit_od.e[1]);
        p->sequence = co_od_get(a->permit_od.e[2]);
        p->reason = co_od_get(a->permit_od.e[3]);
        p->rx_count++;
        p->rx_ms = now_ms;
    }
#endif
}

static void new_epoch(ctl017_app_t *a)
{
    a->rx.link.nmt_epoch++;
    /* Records received before the change belong to the old epoch. The
     * content stays for diagnostics; valid says it is not current. */
    a->rx.lift_cmd.valid = false;
#ifdef CTL017_PERMIT_OD
    a->rx.permit.valid = false;
#endif
}

static void cb_nmt(void *ctx, co_nmt_state_t from, co_nmt_state_t to)
{
    ctl017_app_t *a = ctx_app(ctx);
    (void)from;
    a->rx.link.nmt_state = (uint8_t)to;
    new_epoch(a);
}

static void cb_reset_node(void *ctx)
{
    ctl017_app_t *a = ctx_app(ctx);
    a->rx.link.reset_node_requested = true;
}

static void restore_comm(ctl017_app_t *a)
{
    a->tpdo_inhibit[0] = a->cfg.tpdo_inhibit_100us[0];
    a->tpdo_inhibit[1] = a->cfg.tpdo_inhibit_100us[1];
    a->tpdo_event[0] = a->cfg.tpdo_event_ms[0];
    a->tpdo_event[1] = a->cfg.tpdo_event_ms[1];
    a->rpdo_event_ms[0] = a->cfg.rpdo1_event_timer_ms;
    a->rpdo_event_ms[1] = 0;
#ifdef CTL017_PERMIT_OD
    a->rpdo_event_ms[2] = a->cfg.permit_event_timer_ms;
#endif
}

static void cb_reset_comm(void *ctx)
{
    restore_comm(ctx_app(ctx));
}

static void cb_hb_consumer(void *ctx, int event)
{
    ctl017_app_t *a = ctx_app(ctx);
    if (event == (int)CO_HBC_EV_TIMEOUT) {
        a->rx.link.hbc_timeouts++;
    }
}

bool ctl017_app_config_complete(const ctl017_app_config_t *c)
{
    bool ok = c->base_node_id >= 1u && c->base_node_id <= 127u && c->base_node_id != CTL017_NODE_ID &&
              c->hb_producer_ms != 0u && c->hb_consumer_ms != 0u && c->tpdo_inhibit_100us[0] != 0u &&
              c->tpdo_inhibit_100us[1] != 0u && c->tpdo_event_ms[0] != 0u && c->tpdo_event_ms[1] != 0u &&
              c->rpdo1_event_timer_ms != 0u && c->product_code != 0u && c->revision != 0u;
#ifdef CTL017_PERMIT_OD
    ok = ok && c->permit_event_timer_ms != 0u;
#endif
    return ok;
}

void ctl017_app_set_params_validator(ctl017_app_t *a, ctl017_params_valid_fn fn, void *ctx)
{
    a->params_valid = fn;
    a->params_ctx = ctx;
}

ctl017_app_result_t ctl017_app_init(ctl017_app_t *a, const ctl017_app_config_t *cfg,
                                    ctl017_send_fn send, void *send_ctx, uint32_t now_ms)
{
    memset(a, 0, sizeof(*a));
    ctl017_lift_status_unknown(&a->status);
    if (cfg == NULL || !ctl017_app_config_complete(cfg)) {
        a->init_result = CTL017_APP_NOT_CONFIGURED;
        return a->init_result;
    }
    a->cfg = *cfg;
    a->send = send;
    a->send_ctx = send_ctx;
    a->now_ms = now_ms;
    a->init_result = CTL017_APP_OD_ERROR;

    co_callbacks_t cb = {
        .send = cb_send,
        .before_tpdo = cb_before_tpdo,
        .tpdo_sent = cb_tpdo_sent,
        .rpdo_received = cb_rpdo,
        .nmt_changed = cb_nmt,
        .reset_node = cb_reset_node,
        .reset_comm = cb_reset_comm,
        .hb_consumer = cb_hb_consumer,
    };
    if (!co_init(&a->co, CTL017_NODE_ID, a->od, 0, &cb, a)) {
        return a->init_result;
    }
    co_set_comm_defaults(&a->co, cfg->hb_producer_ms,
                         ((uint32_t)cfg->base_node_id << 16) | cfg->hb_consumer_ms);
    restore_comm(a);

    bool ok = true;
    ctl017_od_build(a, &ok);
    a->co.od = a->od;
    a->co.od_len = a->od_len;
    if (!ok) {
        return a->init_result;
    }

    co_tpdo_cfg_t t1 = {(uint16_t)(a->tpdo_cob[0] & 0x7FFu), &a->tpdo_inhibit[0], &a->tpdo_event[0],
                        a->map_tpdo1, a->n_map_tpdo[0], TPDO1_CHANGE_MASK};
    co_tpdo_cfg_t t2 = {(uint16_t)(a->tpdo_cob[1] & 0x7FFu), &a->tpdo_inhibit[1], &a->tpdo_event[1],
                        a->map_tpdo2, a->n_map_tpdo[1], TPDO2_CHANGE_MASK};
    co_rpdo_cfg_t r1 = {(uint16_t)(a->rpdo_cob[0] & 0x7FFu), a->map_rpdo1, a->n_map_rpdo[0]};
    co_rpdo_cfg_t r2 = {(uint16_t)(a->rpdo_cob[1] & 0x7FFu), a->map_rpdo2, a->n_map_rpdo[1]};
    if (!co_add_tpdo(&a->co, &t1) || !co_add_tpdo(&a->co, &t2) || !co_add_rpdo(&a->co, &r1) ||
        !co_add_rpdo(&a->co, &r2)) {
        return a->init_result;
    }
    /* The v1 payloads are exactly 8 + 8 and 3 + 3 bytes. */
    if (a->co.tpdo[0].len != 8u || a->co.tpdo[1].len != 8u || a->co.rpdo[0].len != 3u ||
        a->co.rpdo[1].len != 3u) {
        return a->init_result;
    }
#ifdef CTL017_PERMIT_OD
    co_rpdo_cfg_t r3 = {(uint16_t)(a->rpdo_cob[2] & 0x7FFu), a->map_rpdo3, a->n_map_rpdo[2]};
    if (!co_add_rpdo(&a->co, &r3)) {
        return a->init_result;
    }
#endif
    a->init_result = CTL017_APP_OK;
    a->rx.link.rpdo1_event_timer_ms = a->rpdo_event_ms[0];
    co_start(&a->co, now_ms);
    return a->init_result;
}

static bool ready(const ctl017_app_t *a)
{
    return a->init_result == CTL017_APP_OK;
}

static void refresh_link(ctl017_app_t *a)
{
    ctl017_link_rx_t *l = &a->rx.link;
    l->nmt_state = (uint8_t)a->co.nmt;
    l->hbc_state = (uint8_t)a->co.hbc;
    l->hbc_remote_boots = a->co.hbc_remote_boots;
    l->hbc_last_rx_ms = a->co.hbc_last_rx_ms;
    l->rpdo1_event_timer_ms = a->rpdo_event_ms[0];
    l->rx_malformed = a->co.rx_malformed;
    l->rpdo_len_errors = a->co.rx_pdo_len_errors;
    a->pdo_len_errors = a->co.rx_pdo_len_errors;
#ifdef CTL017_PERMIT_OD
    a->rx.permit.len_errors = a->co.rx_pdo_len_errors; /* shared counter, see the contract */
#endif
}

void ctl017_app_rx(ctl017_app_t *a, const co_frame_t *f, uint32_t now_ms)
{
    if (!ready(a)) {
        return;
    }
    a->now_ms = now_ms;
    co_process(&a->co, f, now_ms);
    refresh_link(a);
}

void ctl017_app_set_status(ctl017_app_t *a, const ctl017_lift_status_t *st)
{
    a->status = *st;
    if ((a->status.flags & CTL017_FLAG_POSITION_VALID) == 0u) {
        /* Wire rule: no position value without position_valid. */
        a->status.position_01mm = CTL017_POSITION_UNKNOWN;
    }
    a->status.faults &= CTL017_FAULT_ALL;
}

static void emit_faults(ctl017_app_t *a)
{
    uint16_t faults = a->status.faults;
    uint16_t raised = (uint16_t)(faults & (uint16_t)~a->last_faults);
    a->faults = faults;
    a->hall_count_raw = a->status.hall_count_raw;
    a->reset_cause = a->status.reset_cause;
    co_set_app_error_register(&a->co, ctl017_error_register(faults));
    for (uint8_t n = 1; n <= 9u; n++) {
        if ((raised & (1u << (n - 1u))) != 0u) {
            uint8_t mfr[5] = {n, (uint8_t)faults, (uint8_t)(faults >> 8), 0, 0};
            co_emcy(&a->co, (uint16_t)(0xFF00u | n), mfr);
        }
    }
    a->last_faults = faults;
}

void ctl017_app_step(ctl017_app_t *a, uint32_t now_ms)
{
    if (!ready(a)) {
        return;
    }
    a->now_ms = now_ms;
    emit_faults(a);
    co_step(&a->co, now_ms);
    refresh_link(a);
}

const ctl017_iface_rx_t *ctl017_app_rx_view(const ctl017_app_t *a)
{
    return &a->rx;
}

const ctl017_lift_params_t *ctl017_app_params(const ctl017_app_t *a)
{
    return &a->params;
}
