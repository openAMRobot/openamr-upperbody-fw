/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Shared fixture of the CTL-017 CANopen host tests: the application layer,
 * the lift test double and a capture of every transmitted frame.
 *
 * The config values below are TEST VALUES chosen to exercise the code. They
 * match the proposals in docs/ctl017_object_dictionary.md so the frames are
 * comparable, but they are not settings and the firmware has no defaults.
 */
#ifndef CTL017_TEST_FIXTURE_H
#define CTL017_TEST_FIXTURE_H

#include <string.h>

#include "ctl017/ctl017_app.h"
#include "lift_double.h"
#include "test_util.h"

#define MAXF 4096
#define TEST_BASE_NODE 0x01u
#define ID_HB_SELF (0x700u + CTL017_NODE_ID)
#define ID_HB_BASE (0x700u + TEST_BASE_NODE)
#define ID_TPDO1 (0x180u + CTL017_NODE_ID)
#define ID_TPDO2 (0x280u + CTL017_NODE_ID)
#define ID_RPDO1 (0x200u + CTL017_NODE_ID)
#define ID_RPDO2 (0x300u + CTL017_NODE_ID)
#define ID_EMCY (0x080u + CTL017_NODE_ID)
#define ID_SDO_RX (0x600u + CTL017_NODE_ID)
#define ID_SDO_TX (0x580u + CTL017_NODE_ID)

typedef struct {
    ctl017_app_t app;
    lift_double_t dbl;
    uint32_t now;
    co_frame_t tx[MAXF];
    unsigned ntx;
    bool tx_refuse;         /* send callback reports failure */
    bool base_hb;           /* emulate the base controller heartbeat */
    uint32_t last_base_hb;
} fx_t;

static fx_t F;

static inline ctl017_app_config_t test_config(void)
{
    ctl017_app_config_t c;
    memset(&c, 0, sizeof(c));
    c.base_node_id = TEST_BASE_NODE;
    c.hb_producer_ms = 100;
    c.hb_consumer_ms = 300;
    c.tpdo_inhibit_100us[0] = 100;
    c.tpdo_inhibit_100us[1] = 100;
    c.tpdo_event_ms[0] = 100;
    c.tpdo_event_ms[1] = 100;
    c.rpdo1_event_timer_ms = 300;
    c.product_code = 0x11;
    c.revision = 0x00010000u;
#ifdef CTL017_PERMIT_OD
    c.permit_event_timer_ms = 500;
#endif
    return c;
}

static inline bool cap_send(void *ctx, const co_frame_t *f)
{
    fx_t *x = (fx_t *)ctx;
    if (x->tx_refuse) {
        return false;
    }
    if (x->ntx < MAXF) {
        x->tx[x->ntx++] = *f;
    }
    return true;
}

static inline void rx_frame(fx_t *x, const co_frame_t *f)
{
    ctl017_app_rx(&x->app, f, x->now);
    lift_double_observe(&x->dbl, ctl017_app_rx_view(&x->app));
}

static inline void rx(fx_t *x, uint16_t id, uint8_t dlc, const uint8_t *d)
{
    co_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.dlc = dlc;
    if (d != NULL && dlc <= 8u) {
        memcpy(f.data, d, dlc);
    }
    rx_frame(x, &f);
}

static inline void tick(fx_t *x)
{
    x->now++;
    if (x->base_hb && x->now - x->last_base_hb >= 100) {
        uint8_t s = 0x05;
        rx(x, ID_HB_BASE, 1, &s);
        x->last_base_hb = x->now;
    }
    ctl017_lift_status_t st;
    lift_double_status(&x->dbl, x->now, &st);
    ctl017_app_set_status(&x->app, &st);
    ctl017_app_step(&x->app, x->now);
}

static inline void run(fx_t *x, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        tick(x);
    }
}

static inline void setup(void)
{
    memset(&F, 0, sizeof(F));
    F.now = 1000;
    lift_double_init(&F.dbl);
    F.dbl.params_ok = true;
    ctl017_app_config_t c = test_config();
    ctl017_app_result_t r = ctl017_app_init(&F.app, &c, cap_send, &F, F.now);
    CHECK_EQ(r, CTL017_APP_OK);
    ctl017_app_set_params_validator(&F.app, lift_double_params_valid, &F.dbl);
}

static inline void nmt(fx_t *x, uint8_t cs)
{
    uint8_t d[2] = {cs, CTL017_NODE_ID};
    rx(x, 0x000, 2, d);
}

static inline const co_frame_t *last_with_id(const fx_t *x, uint32_t id)
{
    for (unsigned i = x->ntx; i > 0; i--) {
        if (x->tx[i - 1].id == id) {
            return &x->tx[i - 1];
        }
    }
    return NULL;
}

static inline unsigned count_id(const fx_t *x, uint32_t id)
{
    unsigned n = 0;
    for (unsigned i = 0; i < x->ntx; i++) {
        if (x->tx[i].id == id) {
            n++;
        }
    }
    return n;
}

static inline unsigned count_emcy(const fx_t *x, uint16_t code)
{
    unsigned n = 0;
    for (unsigned i = 0; i < x->ntx; i++) {
        if (x->tx[i].id == ID_EMCY && (x->tx[i].data[0] | (x->tx[i].data[1] << 8)) == code) {
            n++;
        }
    }
    return n;
}

static inline const co_frame_t *sdo_read(fx_t *x, uint16_t idx, uint8_t sub)
{
    uint8_t d[8] = {0x40, (uint8_t)idx, (uint8_t)(idx >> 8), sub, 0, 0, 0, 0};
    x->ntx = 0;
    rx(x, ID_SDO_RX, 8, d);
    return last_with_id(x, ID_SDO_TX);
}

static inline uint32_t sdo_val(const co_frame_t *r)
{
    return (uint32_t)r->data[4] | ((uint32_t)r->data[5] << 8) | ((uint32_t)r->data[6] << 16) |
           ((uint32_t)r->data[7] << 24);
}

static inline const co_frame_t *sdo_write(fx_t *x, uint16_t idx, uint8_t sub, uint32_t v, uint8_t size)
{
    uint8_t cmd = (uint8_t)(0x23u | ((uint8_t)(4u - size) << 2));
    uint8_t d[8] = {cmd, (uint8_t)idx, (uint8_t)(idx >> 8), sub,
                    (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
    x->ntx = 0;
    rx(x, ID_SDO_RX, 8, d);
    return last_with_id(x, ID_SDO_TX);
}

static inline void lift_cmd(fx_t *x, uint8_t seq, uint8_t op, uint8_t stop)
{
    uint8_t d[3] = {seq, op, stop};
    rx(x, ID_RPDO1, 3, d);
}

/* A scripted "idle at stop 1" status, values arbitrary but on the wire. */
static inline ctl017_lift_status_t status_idle_at_1(void)
{
    ctl017_lift_status_t s;
    ctl017_lift_status_unknown(&s);
    s.state = CTL017_STATE_IDLE;
    s.flags = (uint8_t)(CTL017_FLAG_POSITION_VALID | CTL017_FLAG_AT_STOP | CTL017_FLAG_MOTOR_POWER_PRESENT);
    s.position_01mm = 1800;
    s.active_stop_id = 1;
    s.inputs = CTL017_IN_STOP1 | CTL017_IN_HALL_SUPPLY;
    return s;
}

static inline void go_operational(fx_t *x)
{
    x->base_hb = true;
    x->last_base_hb = x->now - 100;
    run(x, 10);
    nmt(x, CO_NMT_CS_START);
}

#endif
