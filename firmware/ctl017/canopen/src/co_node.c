/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Minimal CiA 301 slave. See co_node.h for the supported subset.
 */
#include "ctl017/co_node.h"

#include <string.h>

/* ---- Object dictionary helpers -------------------------------------------- */

uint8_t co_type_size(uint8_t type)
{
    switch (type) {
    case CO_T_BOOLEAN:
    case CO_T_INT8:
    case CO_T_UNSIGNED8:
        return 1;
    case CO_T_INT16:
    case CO_T_UNSIGNED16:
        return 2;
    case CO_T_INT32:
    case CO_T_UNSIGNED32:
        return 4;
    default:
        return 0;
    }
}

const co_od_entry_t *co_od_find(const co_od_entry_t *od, size_t n, uint16_t index, uint8_t sub)
{
    for (size_t i = 0; i < n; i++) {
        if (od[i].index == index && od[i].sub == sub) {
            return &od[i];
        }
    }
    return NULL;
}

bool co_od_index_exists(const co_od_entry_t *od, size_t n, uint16_t index)
{
    for (size_t i = 0; i < n; i++) {
        if (od[i].index == index) {
            return true;
        }
    }
    return false;
}

uint32_t co_od_get(const co_od_entry_t *e)
{
    switch (co_type_size(e->type)) {
    case 1:
        return *(const uint8_t *)e->data;
    case 2:
        return *(const uint16_t *)e->data;
    case 4:
        return *(const uint32_t *)e->data;
    default:
        return 0;
    }
}

void co_od_set(const co_od_entry_t *e, uint32_t v)
{
    switch (co_type_size(e->type)) {
    case 1:
        *(uint8_t *)e->data = (uint8_t)v;
        break;
    case 2:
        *(uint16_t *)e->data = (uint16_t)v;
        break;
    case 4:
        *(uint32_t *)e->data = v;
        break;
    default:
        break;
    }
}

static void put_le(uint8_t *p, uint32_t v, uint8_t n)
{
    for (uint8_t i = 0; i < n; i++) {
        p[i] = (uint8_t)(v >> (8u * i));
    }
}

static uint32_t get_le(const uint8_t *p, uint8_t n)
{
    uint32_t v = 0;
    for (uint8_t i = 0; i < n; i++) {
        v |= (uint32_t)p[i] << (8u * i);
    }
    return v;
}

/* ---- Init ------------------------------------------------------------------- */

bool co_init(co_node_t *co, uint8_t node_id, const co_od_entry_t *od, size_t od_len,
             const co_callbacks_t *cb, void *ctx)
{
    memset(co, 0, sizeof(*co));
    if (node_id < 1u || node_id > 127u) {
        return false;
    }
    co->node_id = node_id;
    co->nmt = CO_NMT_INITIALISING;
    co->od = od;
    co->od_len = od_len;
    co->cb = *cb;
    co->ctx = ctx;
    return true;
}

static bool resolve_map(const co_node_t *co, const uint32_t *map, uint8_t map_len,
                        const co_od_entry_t **ent, uint8_t *len)
{
    unsigned bytes = 0;
    if (map_len > CO_MAX_PDO_MAP) {
        return false;
    }
    for (uint8_t i = 0; i < map_len; i++) {
        uint16_t idx = (uint16_t)(map[i] >> 16);
        uint8_t sub = (uint8_t)(map[i] >> 8);
        uint8_t bits = (uint8_t)map[i];
        const co_od_entry_t *e = co_od_find(co->od, co->od_len, idx, sub);
        if (e == NULL || !e->pdo_mappable || co_type_size(e->type) * 8u != bits) {
            return false;
        }
        ent[i] = e;
        bytes += bits / 8u;
    }
    if (bytes > 8u) {
        return false;
    }
    *len = (uint8_t)bytes;
    return true;
}

bool co_add_tpdo(co_node_t *co, const co_tpdo_cfg_t *cfg)
{
    if (co->n_tpdo >= CO_MAX_TPDO) {
        return false;
    }
    co_tpdo_t *t = &co->tpdo[co->n_tpdo];
    memset(t, 0, sizeof(*t));
    t->cfg = *cfg;
    if (!resolve_map(co, cfg->map, cfg->map_len, t->ent, &t->len)) {
        return false;
    }
    co->n_tpdo++;
    return true;
}

bool co_add_rpdo(co_node_t *co, const co_rpdo_cfg_t *cfg)
{
    if (co->n_rpdo >= CO_MAX_RPDO) {
        return false;
    }
    co_rpdo_t *r = &co->rpdo[co->n_rpdo];
    memset(r, 0, sizeof(*r));
    r->cfg = *cfg;
    if (!resolve_map(co, cfg->map, cfg->map_len, r->ent, &r->len)) {
        return false;
    }
    co->n_rpdo++;
    return true;
}

static void send(co_node_t *co, uint16_t id, uint8_t dlc, const uint8_t *data)
{
    co_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.dlc = dlc;
    if (dlc > 0u) {
        memcpy(f.data, data, dlc);
    }
    if (co->cb.send != NULL) {
        (void)co->cb.send(co->ctx, &f);
    }
}

static void set_nmt(co_node_t *co, co_nmt_state_t s)
{
    co_nmt_state_t from = co->nmt;
    if (from == s) {
        return;
    }
    co->nmt = s;
    if (s == CO_NMT_OPERATIONAL) {
        /* First TPDO after entering OPERATIONAL is sent at once. */
        for (uint8_t i = 0; i < co->n_tpdo; i++) {
            co->tpdo[i].sent_once = false;
        }
    }
    if (co->cb.nmt_changed != NULL) {
        co->cb.nmt_changed(co->ctx, from, s);
    }
}

static void hbc_configure(co_node_t *co)
{
    uint16_t t = (uint16_t)co->hb_consumer_entry;
    uint8_t node = (uint8_t)(co->hb_consumer_entry >> 16);
    co_hbc_state_t before = co->hbc;
    co->hbc = (t == 0u || node == 0u || node > 127u) ? CO_HBC_OFF : CO_HBC_WAITING;
    if (before == CO_HBC_LOST && co->cb.hb_consumer != NULL) {
        co->cb.hb_consumer(co->ctx, false);
    }
}

void co_start(co_node_t *co, uint32_t now_ms)
{
    co->now_ms = now_ms;
    co->nmt = CO_NMT_INITIALISING;
    uint8_t boot = 0x00;
    send(co, (uint16_t)(CO_ID_HEARTBEAT + co->node_id), 1, &boot);
    co->last_hb_tx_ms = now_ms;
    hbc_configure(co);
    set_nmt(co, CO_NMT_PRE_OPERATIONAL);
}

/* ---- Standard write hooks ---------------------------------------------------- */

uint32_t co_write_err_count(co_node_t *co, const co_od_entry_t *e, uint32_t value)
{
    (void)e;
    if (value != 0u) {
        return CO_SDO_ABORT_VALUE_RANGE; /* CiA 301: only 0 (clear history) */
    }
    co->err_count = 0;
    memset(co->err_history, 0, sizeof(co->err_history));
    return 0;
}

uint32_t co_write_hb_consumer(co_node_t *co, const co_od_entry_t *e, uint32_t value)
{
    (void)e;
    if ((value & 0xFF000000u) != 0u || (value >> 16) > 127u) {
        return CO_SDO_ABORT_VALUE_RANGE;
    }
    co->hb_consumer_entry = value;
    hbc_configure(co);
    return 0;
}

/* ---- SDO server (expedited only) -------------------------------------------- */

static void sdo_abort(co_node_t *co, uint16_t idx, uint8_t sub, uint32_t code)
{
    uint8_t d[8];
    d[0] = 0x80;
    put_le(&d[1], idx, 2);
    d[3] = sub;
    put_le(&d[4], code, 4);
    send(co, (uint16_t)(CO_ID_SDO_TX + co->node_id), 8, d);
}

static uint32_t sdo_lookup(const co_node_t *co, uint16_t idx, uint8_t sub, const co_od_entry_t **out)
{
    const co_od_entry_t *e = co_od_find(co->od, co->od_len, idx, sub);
    if (e == NULL) {
        return co_od_index_exists(co->od, co->od_len, idx) ? CO_SDO_ABORT_NO_SUB : CO_SDO_ABORT_NO_OBJECT;
    }
    *out = e;
    return 0;
}

static void sdo_server(co_node_t *co, const co_frame_t *f)
{
    if (f->dlc != 8u) {
        return;
    }
    const uint8_t *d = f->data;
    uint8_t ccs = (uint8_t)(d[0] >> 5);
    uint16_t idx = (uint16_t)get_le(&d[1], 2);
    uint8_t sub = d[3];
    const co_od_entry_t *e = NULL;
    uint32_t abort;
    uint8_t r[8];

    if (ccs == 4u) {
        return; /* abort from the client: nothing pending in expedited mode */
    }
    if (ccs == 2u) { /* initiate upload */
        abort = sdo_lookup(co, idx, sub, &e);
        if (abort != 0u) {
            sdo_abort(co, idx, sub, abort);
            return;
        }
        uint8_t n = co_type_size(e->type);
        memset(r, 0, sizeof(r));
        r[0] = (uint8_t)(0x40u | ((uint8_t)(4u - n) << 2) | 0x03u);
        put_le(&r[1], idx, 2);
        r[3] = sub;
        put_le(&r[4], co_od_get(e), n);
        send(co, (uint16_t)(CO_ID_SDO_TX + co->node_id), 8, r);
        return;
    }
    if (ccs == 1u) { /* initiate download */
        bool expedited = (d[0] & 0x02u) != 0u;
        bool size_ind = (d[0] & 0x01u) != 0u;
        if (!expedited) {
            sdo_abort(co, idx, sub, CO_SDO_ABORT_UNSUPPORTED);
            return;
        }
        abort = sdo_lookup(co, idx, sub, &e);
        if (abort != 0u) {
            sdo_abort(co, idx, sub, abort);
            return;
        }
        uint8_t n = co_type_size(e->type);
        if (size_ind) {
            uint8_t given = (uint8_t)(4u - ((d[0] >> 2) & 0x03u));
            if (given != n) {
                sdo_abort(co, idx, sub, CO_SDO_ABORT_TYPE_LENGTH);
                return;
            }
        }
        switch (e->access) {
        case CO_A_CONST:
        case CO_A_RO:
            sdo_abort(co, idx, sub, CO_SDO_ABORT_READONLY);
            return;
        case CO_A_RWW:
            sdo_abort(co, idx, sub, CO_SDO_ABORT_LOCAL_CONTROL);
            return;
        case CO_A_RW_PREOP:
            if (co->nmt != CO_NMT_PRE_OPERATIONAL) {
                sdo_abort(co, idx, sub, CO_SDO_ABORT_DEVICE_STATE);
                return;
            }
            break;
        default:
            break;
        }
        uint32_t v = get_le(&d[4], n);
        if (e->type == CO_T_BOOLEAN && v > 1u) {
            sdo_abort(co, idx, sub, CO_SDO_ABORT_VALUE_RANGE);
            return;
        }
        if (e->on_write != NULL) {
            abort = e->on_write(co, e, v);
            if (abort != 0u) {
                sdo_abort(co, idx, sub, abort);
                return;
            }
        } else {
            co_od_set(e, v);
        }
        memset(r, 0, sizeof(r));
        r[0] = 0x60;
        put_le(&r[1], idx, 2);
        r[3] = sub;
        send(co, (uint16_t)(CO_ID_SDO_TX + co->node_id), 8, r);
        return;
    }
    sdo_abort(co, idx, sub, CO_SDO_ABORT_CMD);
}

/* ---- PDO --------------------------------------------------------------------- */

uint8_t co_tpdo_pack(const co_tpdo_t *t, uint8_t out[8])
{
    uint8_t off = 0;
    memset(out, 0, 8);
    for (uint8_t i = 0; i < t->cfg.map_len; i++) {
        uint8_t n = co_type_size(t->ent[i]->type);
        put_le(&out[off], co_od_get(t->ent[i]), n);
        off = (uint8_t)(off + n);
    }
    return off;
}

static void rpdo_receive(co_node_t *co, uint8_t k, const co_frame_t *f)
{
    co_rpdo_t *r = &co->rpdo[k];
    if (f->dlc < r->len) {
        co->rx_pdo_len_errors++;
        return; /* CiA 301: too short, not processed */
    }
    uint8_t off = 0;
    for (uint8_t i = 0; i < r->cfg.map_len; i++) {
        uint8_t n = co_type_size(r->ent[i]->type);
        co_od_set(r->ent[i], get_le(&f->data[off], n));
        off = (uint8_t)(off + n);
    }
    if (co->cb.rpdo_received != NULL) {
        co->cb.rpdo_received(co->ctx, k, co->now_ms);
    }
}

static void tpdo_service(co_node_t *co)
{
    for (uint8_t k = 0; k < co->n_tpdo; k++) {
        co_tpdo_t *t = &co->tpdo[k];
        if (co->cb.before_tpdo != NULL) {
            co->cb.before_tpdo(co->ctx, k);
        }
        uint8_t buf[8];
        uint8_t len = co_tpdo_pack(t, buf);
        uint32_t since = co->now_ms - t->last_tx_ms;
        uint32_t inhibit_ms = (t->cfg.inhibit_100us != NULL) ? (*t->cfg.inhibit_100us + 9u) / 10u : 0u;
        uint32_t event_ms = (t->cfg.event_ms != NULL) ? *t->cfg.event_ms : 0u;
        bool changed = false;
        for (uint8_t b = 0; b < len; b++) {
            if ((t->cfg.change_mask & (1u << b)) != 0u && buf[b] != t->last[b]) {
                changed = true;
            }
        }
        bool due = !t->sent_once || (changed && since >= inhibit_ms) ||
                   (event_ms != 0u && since >= event_ms);
        if (!due) {
            continue;
        }
        send(co, t->cfg.cob_id, len, buf);
        memcpy(t->last, buf, sizeof(buf));
        t->sent_once = true;
        t->last_tx_ms = co->now_ms;
        if (co->cb.tpdo_sent != NULL) {
            co->cb.tpdo_sent(co->ctx, k);
        }
    }
}

/* ---- NMT and heartbeat ------------------------------------------------------- */

static void nmt_command(co_node_t *co, const co_frame_t *f)
{
    if (f->dlc != 2u || (f->data[1] != 0u && f->data[1] != co->node_id)) {
        return;
    }
    switch (f->data[0]) {
    case CO_NMT_CS_START:
        set_nmt(co, CO_NMT_OPERATIONAL);
        break;
    case CO_NMT_CS_STOP:
        set_nmt(co, CO_NMT_STOPPED);
        break;
    case CO_NMT_CS_PREOP:
        set_nmt(co, CO_NMT_PRE_OPERATIONAL);
        break;
    case CO_NMT_CS_RESET_NODE:
        if (co->cb.reset_node != NULL) {
            co->cb.reset_node(co->ctx); /* application re-initialises and restarts */
        }
        break;
    case CO_NMT_CS_RESET_COMM:
        set_nmt(co, CO_NMT_INITIALISING);
        co_start(co, co->now_ms);
        break;
    default:
        break;
    }
}

static void hb_consumer_rx(co_node_t *co, const co_frame_t *f)
{
    if (co->hbc == CO_HBC_OFF || f->dlc < 1u) {
        return;
    }
    co->hbc_last_rx_ms = co->now_ms;
    co->hbc_remote_state = f->data[0];
    co_hbc_state_t before = co->hbc;
    co->hbc = CO_HBC_ACTIVE; /* before the callback, which may read it */
    if (before == CO_HBC_LOST && co->cb.hb_consumer != NULL) {
        co->cb.hb_consumer(co->ctx, false);
    }
}

void co_process(co_node_t *co, const co_frame_t *f, uint32_t now_ms)
{
    co->now_ms = now_ms;
    if (co->nmt == CO_NMT_INITIALISING) {
        return;
    }
    if (f->id == CO_ID_NMT) {
        nmt_command(co, f);
        return;
    }
    uint8_t hbc_node = (uint8_t)(co->hb_consumer_entry >> 16);
    if (co->hbc != CO_HBC_OFF && f->id == CO_ID_HEARTBEAT + hbc_node) {
        hb_consumer_rx(co, f);
        return;
    }
    if (co->nmt == CO_NMT_STOPPED) {
        return; /* only NMT and heartbeat in STOPPED */
    }
    if (f->id == CO_ID_SDO_RX + co->node_id) {
        sdo_server(co, f);
        return;
    }
    if (co->nmt == CO_NMT_OPERATIONAL) {
        for (uint8_t k = 0; k < co->n_rpdo; k++) {
            if (f->id == co->rpdo[k].cfg.cob_id) {
                rpdo_receive(co, k, f);
                return;
            }
        }
    }
}

void co_step(co_node_t *co, uint32_t now_ms)
{
    co->now_ms = now_ms;
    if (co->nmt == CO_NMT_INITIALISING) {
        return;
    }
    if (co->hb_producer_ms != 0u && (uint32_t)(now_ms - co->last_hb_tx_ms) >= co->hb_producer_ms) {
        uint8_t s = (uint8_t)co->nmt;
        send(co, (uint16_t)(CO_ID_HEARTBEAT + co->node_id), 1, &s);
        co->last_hb_tx_ms = now_ms;
    }
    if (co->hbc == CO_HBC_ACTIVE) {
        uint16_t t = (uint16_t)co->hb_consumer_entry;
        if ((uint32_t)(now_ms - co->hbc_last_rx_ms) > t) {
            co->hbc = CO_HBC_LOST;
            if (co->cb.hb_consumer != NULL) {
                co->cb.hb_consumer(co->ctx, true);
            }
        }
    }
    if (co->nmt == CO_NMT_OPERATIONAL) {
        tpdo_service(co);
    }
}

/* ---- EMCY ---------------------------------------------------------------------- */

void co_emcy(co_node_t *co, uint16_t code, const uint8_t mfr[5])
{
    if (code != 0u) {
        memmove(&co->err_history[1], &co->err_history[0],
                (CO_ERR_HISTORY - 1u) * sizeof(co->err_history[0]));
        co->err_history[0] = (uint32_t)code | ((uint32_t)mfr[0] << 16) | ((uint32_t)mfr[1] << 24);
        if (co->err_count < CO_ERR_HISTORY) {
            co->err_count++;
        }
    }
    if (co->nmt != CO_NMT_PRE_OPERATIONAL && co->nmt != CO_NMT_OPERATIONAL) {
        return;
    }
    uint8_t d[8];
    put_le(&d[0], code, 2);
    d[2] = co->error_register;
    memcpy(&d[3], mfr, 5);
    send(co, (uint16_t)(CO_ID_EMCY + co->node_id), 8, d);
}
