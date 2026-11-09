/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Host tests of the CTL-017 CANopen slave and application: NMT, heartbeat
 * producer and consumer, expedited SDO, PDO encoding and mapping, EMCY and
 * 0x1003, parameter writes. Frames are captured from the send callback.
 */
#include <string.h>

#include "ctl017/ctl017_app.h"
#include "lift_plant.h"
#include "test_util.h"

#define MM(x) ((int64_t)(x) * 1000000)
#define MAXF 4096

typedef struct {
    ctl017_app_t app;
    lift_plant_t plant;
    uint32_t now;
    co_frame_t tx[MAXF];
    unsigned ntx;
    const ctl017_outputs_t *out;
    bool core_hb;           /* emulate the base controller heartbeat */
    uint32_t last_core_hb;
} fx_t;

static fx_t F;

static bool cap_send(void *ctx, const co_frame_t *f)
{
    fx_t *x = (fx_t *)ctx;
    if (x->ntx < MAXF) {
        x->tx[x->ntx++] = *f;
    }
    return true;
}

static void rx(fx_t *x, uint16_t id, uint8_t dlc, const uint8_t *d)
{
    co_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.dlc = dlc;
    memcpy(f.data, d, dlc);
    ctl017_app_rx(&x->app, &f, x->now);
}

static void tick(fx_t *x)
{
    x->now++;
    if (x->core_hb && x->now - x->last_core_hb >= 100) {
        uint8_t s = 0x05;
        rx(x, 0x700 + CTL017_CORE_NODE_ID, 1, &s);
        x->last_core_hb = x->now;
    }
    lift_inputs_t li;
    aux_inputs_t ai;
    plant_inputs(&x->plant, x->now, &li);
    memset(&ai, 0, sizeof(ai));
    ai.now_ms = x->now;
    x->out = ctl017_app_step(&x->app, &li, &ai);
    plant_step(&x->plant, &x->out->hbridge, 1);
}

static void run(fx_t *x, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        tick(x);
    }
}

static lift_config_t test_cfg(void)
{
    lift_config_t c;
    lift_config_default(&c);
    c.sensor_edge_offset_up_um = -1000;
    c.sensor_edge_offset_down_um = 1000;
    return c;
}

static void setup(int64_t start_nm)
{
    memset(&F, 0, sizeof(F));
    F.now = 1000;
    plant_init(&F.plant, start_nm);
    lift_config_t c = test_cfg();
    aux_config_t ac;
    aux_config_default(&ac);
    bool ok = ctl017_app_init(&F.app, &c, &ac, cap_send, &F, F.now);
    CHECK(ok);
}

static void nmt(fx_t *x, uint8_t cs)
{
    uint8_t d[2] = {cs, CTL017_NODE_ID};
    rx(x, 0x000, 2, d);
}

static const co_frame_t *last_with_id(const fx_t *x, uint16_t id)
{
    for (unsigned i = x->ntx; i > 0; i--) {
        if (x->tx[i - 1].id == id) {
            return &x->tx[i - 1];
        }
    }
    return NULL;
}

static unsigned count_id(const fx_t *x, uint16_t id)
{
    unsigned n = 0;
    for (unsigned i = 0; i < x->ntx; i++) {
        if (x->tx[i].id == id) {
            n++;
        }
    }
    return n;
}

/* SDO helpers: returns the response frame. */
static const co_frame_t *sdo_read(fx_t *x, uint16_t idx, uint8_t sub)
{
    uint8_t d[8] = {0x40, (uint8_t)idx, (uint8_t)(idx >> 8), sub, 0, 0, 0, 0};
    x->ntx = 0;
    rx(x, 0x600 + CTL017_NODE_ID, 8, d);
    return last_with_id(x, 0x580 + CTL017_NODE_ID);
}

static uint32_t sdo_val(const co_frame_t *r)
{
    return (uint32_t)r->data[4] | ((uint32_t)r->data[5] << 8) | ((uint32_t)r->data[6] << 16) |
           ((uint32_t)r->data[7] << 24);
}

static const co_frame_t *sdo_write(fx_t *x, uint16_t idx, uint8_t sub, uint32_t v, uint8_t size)
{
    uint8_t cmd = (uint8_t)(0x23u | ((uint8_t)(4u - size) << 2));
    uint8_t d[8] = {cmd, (uint8_t)idx, (uint8_t)(idx >> 8), sub,
                    (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
    x->ntx = 0;
    rx(x, 0x600 + CTL017_NODE_ID, 8, d);
    return last_with_id(x, 0x580 + CTL017_NODE_ID);
}

static void lift_cmd(fx_t *x, uint8_t seq, uint8_t op, uint8_t stop)
{
    uint8_t d[3] = {seq, op, stop};
    rx(x, 0x200 + CTL017_NODE_ID, 3, d);
}

/* ------------------------------------------------------------------------ */

static void test_boot_up_and_preop(void)
{
    setup(MM(180));
    CHECK(F.ntx >= 1);
    CHECK_EQ(F.tx[0].id, 0x710);
    CHECK_EQ(F.tx[0].dlc, 1);
    CHECK_EQ(F.tx[0].data[0], 0x00);
    CHECK_EQ(F.app.co.nmt, CO_NMT_PRE_OPERATIONAL);
    run(&F, 500);
    /* No PDO in PRE-OPERATIONAL. */
    CHECK_EQ(count_id(&F, 0x190), 0);
    CHECK_EQ(count_id(&F, 0x290), 0);
}

static void test_heartbeat_producer(void)
{
    setup(MM(180));
    F.ntx = 0;
    run(&F, 1000);
    unsigned n = count_id(&F, 0x710);
    CHECK(n >= 9 && n <= 11);
    CHECK_EQ(last_with_id(&F, 0x710)->data[0], 0x7F);
    nmt(&F, CO_NMT_CS_START);
    run(&F, 150);
    CHECK_EQ(last_with_id(&F, 0x710)->data[0], 0x05);
    nmt(&F, CO_NMT_CS_STOP);
    run(&F, 150);
    CHECK_EQ(last_with_id(&F, 0x710)->data[0], 0x04);
}

static void test_nmt_addressing(void)
{
    setup(MM(180));
    uint8_t other[2] = {CO_NMT_CS_START, 0x11};
    rx(&F, 0x000, 2, other);
    CHECK_EQ(F.app.co.nmt, CO_NMT_PRE_OPERATIONAL);
    uint8_t all[2] = {CO_NMT_CS_START, 0x00};
    rx(&F, 0x000, 2, all);
    CHECK_EQ(F.app.co.nmt, CO_NMT_OPERATIONAL);
    nmt(&F, CO_NMT_CS_RESET_COMM);
    CHECK_EQ(F.app.co.nmt, CO_NMT_PRE_OPERATIONAL);
    CHECK_EQ(last_with_id(&F, 0x710)->data[0], 0x00); /* boot-up again */
    nmt(&F, CO_NMT_CS_RESET_NODE);
    run(&F, 1);
    CHECK(F.out->reset_requested);
}

/* TPDO1 layout (T24): state, flags, position u16 LE, active_stop_id,
 * last_seq, cmd_result, status_counter. */
static void test_tpdo1_layout_and_counter(void)
{
    setup(MM(180));
    run(&F, 200);
    nmt(&F, CO_NMT_CS_START);
    F.ntx = 0;
    run(&F, 1);
    const co_frame_t *t = last_with_id(&F, 0x190);
    CHECK(t != NULL);
    if (t == NULL) {
        return;
    }
    CHECK_EQ(t->dlc, 8);
    CHECK_EQ(t->data[0], LIFT_STATE_IDLE);
    uint8_t flags = (uint8_t)(LIFT_FLAG_POSITION_VALID | LIFT_FLAG_AT_STOP | LIFT_FLAG_MOTOR_POWER_PRESENT);
    CHECK_EQ(t->data[1], flags);
    CHECK_EQ(t->data[2] | (t->data[3] << 8), 1800);
    CHECK_EQ(t->data[4], 1);
    CHECK_EQ(t->data[5], 0);
    CHECK_EQ(t->data[6], 0xFF);
    uint8_t c0 = t->data[7];
    /* Event timer: one frame per 100 ms when nothing changes. */
    F.ntx = 0;
    run(&F, 1000);
    unsigned n = count_id(&F, 0x190);
    CHECK(n >= 9 && n <= 11);
    CHECK_EQ(last_with_id(&F, 0x190)->data[7], (uint8_t)(c0 + n));
    /* Counter wraps 255 to 0 (T06). */
    F.app.st1.status_counter = 255;
    F.ntx = 0;
    run(&F, 200);
    unsigned k = 0;
    uint8_t seen[4] = {0};
    for (unsigned i = 0; i < F.ntx && k < 4; i++) {
        if (F.tx[i].id == 0x190) {
            seen[k++] = F.tx[i].data[7];
        }
    }
    CHECK(k >= 2);
    CHECK_EQ(seen[0], 255);
    CHECK_EQ(seen[1], 0);
}

static void test_tpdo2_layout(void)
{
    setup(MM(180));
    run(&F, 200);
    nmt(&F, CO_NMT_CS_START);
    F.ntx = 0;
    run(&F, 1);
    const co_frame_t *t = last_with_id(&F, 0x290);
    CHECK(t != NULL);
    if (t == NULL) {
        return;
    }
    CHECK_EQ(t->dlc, 8);
    CHECK_EQ(t->data[0], CTL017_IN_STOP1 | CTL017_IN_HALL_SUPPLY);
    CHECK_EQ(t->data[1], 0xFF);
    CHECK_EQ(t->data[2] | (t->data[3] << 8), 0);
    CHECK_EQ((int16_t)(t->data[4] | (t->data[5] << 8)), 0);
    CHECK_EQ(t->data[6], 0);
    CHECK_EQ(t->data[7], 0);
}

/* RPDO1 drives the core; TPDO1 reports the accepted command at once (change
 * detection), TPDO2 shows the target, then the status follows the move. */
static void test_rpdo1_move_and_status(void)
{
    setup(MM(5));
    F.core_hb = true;
    run(&F, 200);
    nmt(&F, CO_NMT_CS_START);
    run(&F, 50);
    F.ntx = 0;
    lift_cmd(&F, 1, LIFT_OP_MOVE_TO_STOP, 1);
    run(&F, 1);
    const co_frame_t *t = last_with_id(&F, 0x190);
    CHECK(t != NULL);
    if (t == NULL) {
        return;
    }
    CHECK_EQ(t->data[5], 1);
    CHECK_EQ(t->data[6], LIFT_RES_ACCEPTED);
    CHECK_EQ(t->data[0], LIFT_STATE_MOVING);
    CHECK_EQ(t->data[4], 0xFF);
    run(&F, 20);
    CHECK_EQ(last_with_id(&F, 0x290)->data[1], 1);
    /* Keep-alive every 100 ms until rest. */
    for (int i = 0; i < 200 && F.app.lift.state != LIFT_STATE_IDLE; i++) {
        lift_cmd(&F, 1, LIFT_OP_MOVE_TO_STOP, 1);
        run(&F, 100);
    }
    run(&F, 200);
    t = last_with_id(&F, 0x190);
    CHECK_EQ(t->data[0], LIFT_STATE_IDLE);
    CHECK_EQ(t->data[4], 1);
    int pos = t->data[2] | (t->data[3] << 8);
    CHECK(pos >= 1780 && pos <= 1820);
    CHECK_EQ(F.app.lift.faults, 0);
}

/* RPDOs are ignored outside OPERATIONAL; a short RPDO is not processed. */
static void test_rpdo_state_and_length(void)
{
    setup(MM(5));
    run(&F, 200);
    lift_cmd(&F, 1, LIFT_OP_MOVE_TO_STOP, 2);
    run(&F, 10);
    CHECK_EQ(F.app.lift.state, LIFT_STATE_IDLE);
    nmt(&F, CO_NMT_CS_START);
    uint8_t d[2] = {2, LIFT_OP_MOVE_TO_STOP};
    rx(&F, 0x210, 2, d);
    run(&F, 10);
    CHECK_EQ(F.app.lift.state, LIFT_STATE_IDLE);
    CHECK_EQ(F.app.co.rx_pdo_len_errors, 1);
}

/* T03 at the bus level: keep-alive withheld, F07 and EMCY 0xFF07, error
 * register communication bit, 0x1003 entry. */
static void test_keepalive_loss_emcy(void)
{
    setup(MM(5));
    F.core_hb = true;
    run(&F, 200);
    nmt(&F, CO_NMT_CS_START);
    lift_cmd(&F, 1, LIFT_OP_MOVE_TO_STOP, 2);
    F.ntx = 0;
    run(&F, 400);
    CHECK(F.app.lift.faults & LIFT_FAULT_F07_SUPERVISION);
    CHECK(!F.out->hbridge.enable);
    const co_frame_t *e = last_with_id(&F, 0x090);
    CHECK(e != NULL);
    if (e == NULL) {
        return;
    }
    CHECK_EQ(e->data[0] | (e->data[1] << 8), 0xFF07);
    CHECK_EQ(e->data[2], 0x01 | 0x10 | 0x80);
    CHECK_EQ(e->data[3], 7);
    CHECK_EQ(e->data[4], LIFT_FAULT_F07_SUPERVISION);
    const co_frame_t *r = sdo_read(&F, 0x1003, 0);
    CHECK_EQ(sdo_val(r), 1);
    r = sdo_read(&F, 0x1003, 1);
    CHECK_EQ(sdo_val(r) & 0xFFFF, 0xFF07);
    CHECK_EQ((sdo_val(r) >> 16) & 0xFF, 7);
    r = sdo_read(&F, 0x2002, 0);
    CHECK_EQ(sdo_val(r), LIFT_FAULT_F07_SUPERVISION);
    run(&F, 300);
    /* CLEAR_FAULT -> EMCY error reset 0x0000. */
    F.ntx = 0;
    lift_cmd(&F, 2, LIFT_OP_CLEAR_FAULT, 0);
    run(&F, 2);
    e = last_with_id(&F, 0x090);
    CHECK(e != NULL);
    if (e != NULL) {
        CHECK_EQ(e->data[0] | (e->data[1] << 8), 0x0000);
        CHECK_EQ(e->data[2], 0);
    }
    /* Clear the history with 0; any other value is refused. */
    r = sdo_write(&F, 0x1003, 0, 1, 1);
    CHECK_EQ(r->data[0], 0x80);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_VALUE_RANGE);
    r = sdo_write(&F, 0x1003, 0, 0, 1);
    CHECK_EQ(r->data[0], 0x60);
    CHECK_EQ(sdo_val(sdo_read(&F, 0x1003, 0)), 0);
}

/* T26, D12 alternative (a): core heartbeat lost during a move with the
 * keep-alive present also stops it. */
static void test_core_heartbeat_loss_stops_move(void)
{
    setup(MM(5));
    F.core_hb = true;
    run(&F, 200);
    nmt(&F, CO_NMT_CS_START);
    lift_cmd(&F, 1, LIFT_OP_MOVE_TO_STOP, 2);
    run(&F, 100);
    for (int i = 0; i < 5; i++) {
        lift_cmd(&F, 1, LIFT_OP_MOVE_TO_STOP, 2);
        run(&F, 100);
    }
    CHECK_EQ(F.app.lift.state, LIFT_STATE_MOVING);
    F.core_hb = false;
    uint32_t t0 = F.now;
    while (!(F.app.lift.faults & LIFT_FAULT_F07_SUPERVISION) && F.now - t0 < 1000) {
        if ((F.now - t0) % 100 == 0) {
            lift_cmd(&F, 1, LIFT_OP_MOVE_TO_STOP, 2);
        }
        tick(&F);
    }
    CHECK(F.app.lift.faults & LIFT_FAULT_F07_SUPERVISION);
    CHECK(F.now - t0 <= 400);
    CHECK_EQ(F.app.co.hbc, CO_HBC_LOST);
    /* Heartbeat back: consumer active again, fault stays latched. */
    F.core_hb = true;
    run(&F, 150);
    CHECK_EQ(F.app.co.hbc, CO_HBC_ACTIVE);
    CHECK(F.app.lift.faults & LIFT_FAULT_F07_SUPERVISION);
}

/* Leaving OPERATIONAL halts the move at once. */
static void test_nmt_preop_halts(void)
{
    setup(MM(5));
    F.core_hb = true;
    run(&F, 200);
    nmt(&F, CO_NMT_CS_START);
    lift_cmd(&F, 1, LIFT_OP_MOVE_TO_STOP, 2);
    run(&F, 50);
    CHECK(F.out->hbridge.enable);
    nmt(&F, CO_NMT_CS_PREOP);
    run(&F, 1);
    CHECK(!F.out->hbridge.enable);
    CHECK_EQ(F.app.lift.faults, 0);
}

static void test_sdo_upload_types(void)
{
    setup(MM(180));
    const co_frame_t *r = sdo_read(&F, 0x1000, 0);
    CHECK_EQ(r->data[0], 0x43);
    CHECK_EQ(sdo_val(r), 0);
    r = sdo_read(&F, 0x1017, 0);
    CHECK_EQ(r->data[0], 0x4B);
    CHECK_EQ(sdo_val(r), 100);
    r = sdo_read(&F, 0x1016, 1);
    CHECK_EQ(sdo_val(r), 0x0001012C);
    r = sdo_read(&F, 0x1018, 2);
    CHECK_EQ(sdo_val(r), 0x11);
    r = sdo_read(&F, 0x1A00, 3);
    CHECK_EQ(sdo_val(r), 0x20000310);
    r = sdo_read(&F, 0x2000, 1);
    CHECK_EQ(r->data[0], 0x4F);
    r = sdo_read(&F, 0x2100, 2);
    CHECK_EQ((int32_t)sdo_val(r), 180000);
    r = sdo_read(&F, 0x1800, 1);
    CHECK_EQ(sdo_val(r), 0x40000190);
}

static void test_sdo_aborts(void)
{
    setup(MM(180));
    const co_frame_t *r = sdo_read(&F, 0x1234, 0);
    CHECK_EQ(r->data[0], 0x80);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_NO_OBJECT);
    r = sdo_read(&F, 0x1018, 9);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_NO_SUB);
    r = sdo_write(&F, 0x1000, 0, 5, 4);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_READONLY);
    r = sdo_write(&F, 0x1017, 0, 5, 4); /* U16 written with 4 bytes */
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_TYPE_LENGTH);
    r = sdo_write(&F, 0x2010, 2, 1, 1); /* command only through RPDO1 */
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_LOCAL_CONTROL);
    /* segmented download not supported */
    uint8_t d[8] = {0x21, 0x00, 0x20, 0x01, 8, 0, 0, 0};
    F.ntx = 0;
    rx(&F, 0x60A + 6, 8, d);
    r = last_with_id(&F, 0x590);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_UNSUPPORTED);
    /* invalid command specifier */
    uint8_t bad[8] = {0xE0, 0x00, 0x10, 0, 0, 0, 0, 0};
    F.ntx = 0;
    rx(&F, 0x610, 8, bad);
    CHECK_EQ(sdo_val(last_with_id(&F, 0x590)), CO_SDO_ABORT_CMD);
    /* SDO is not served in STOPPED. */
    nmt(&F, CO_NMT_CS_STOP);
    F.ntx = 0;
    sdo_read(&F, 0x1000, 0);
    CHECK_EQ(count_id(&F, 0x590), 0);
}

static void test_sdo_parameter_writes(void)
{
    setup(MM(180));
    run(&F, 200);
    const co_frame_t *r = sdo_write(&F, 0x2101, 23, 6500, 2); /* overcurrent_ma */
    CHECK_EQ(r->data[0], 0x60);
    CHECK_EQ(F.app.lift.cfg.overcurrent_ma, 6500);
    /* Invalid configuration refused and reverted: stop 1 below stop 0. */
    r = sdo_write(&F, 0x2100, 2, 1000, 4);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_VALUE_RANGE);
    CHECK_EQ(F.app.lift.cfg.stop_pos_um[1], 180000);
    /* BOOLEAN accepts 0 or 1 only. */
    r = sdo_write(&F, 0x2101, 29, 2, 1);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_VALUE_RANGE);
    r = sdo_write(&F, 0x2101, 29, 1, 1);
    CHECK_EQ(r->data[0], 0x60);
    CHECK(F.app.lift.cfg.end_switches_wired);
    /* Debounce change reaches the debouncers. */
    r = sdo_write(&F, 0x2101, 26, 9, 2);
    CHECK_EQ(F.app.lift.stop_db[0].hold_ms, 9);
    /* Parameters only in PRE-OPERATIONAL. */
    nmt(&F, CO_NMT_CS_START);
    r = sdo_write(&F, 0x2101, 23, 6000, 2);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_DEVICE_STATE);
    CHECK_EQ(F.app.lift.cfg.overcurrent_ma, 6500);
    /* Event timer may change in OPERATIONAL. */
    r = sdo_write(&F, 0x1800, 5, 50, 2);
    CHECK_EQ(r->data[0], 0x60);
    CHECK_EQ(F.app.tpdo_event[0], 50);
}

/* Aux outputs over RPDO2 and SDO; head light off on boot (only during arm
 * operations), dim level clamped by SDO range check. */
static void test_aux_outputs(void)
{
    setup(MM(180));
    run(&F, 10);
    CHECK(!F.out->aux.head_light_on);
    nmt(&F, CO_NMT_CS_START);
    uint8_t d[3] = {1, 40, 1};
    rx(&F, 0x310, 3, d);
    run(&F, 2);
    CHECK(F.out->aux.head_light_on);
    CHECK_EQ(F.out->aux.head_light_duty_permille, 400);
    CHECK(F.out->aux.getac_enable);
    run(&F, 20);
    CHECK_EQ(last_with_id(&F, 0x290)->data[7], CTL017_AUX_LIGHT_ON | CTL017_AUX_GETAC_EN);
    nmt(&F, CO_NMT_CS_PREOP);
    const co_frame_t *r = sdo_write(&F, 0x2011, 2, 150, 1);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_VALUE_RANGE);
    r = sdo_write(&F, 0x2011, 1, 0, 1);
    run(&F, 1);
    CHECK(!F.out->aux.head_light_on);
}

/* Every mapped object exists, is mappable and the total is 8 bytes; the
 * generic packer puts each object at the documented offset. */
static void test_mapping_consistency(void)
{
    setup(MM(180));
    CHECK(F.app.od_ok);
    CHECK_EQ(F.app.co.tpdo[0].len, 8);
    CHECK_EQ(F.app.co.tpdo[1].len, 8);
    CHECK_EQ(F.app.co.rpdo[0].len, 3);
    CHECK_EQ(F.app.co.rpdo[1].len, 3);
    F.app.st2.inputs = 0xA5;
    F.app.st2.target_stop_id = 2;
    F.app.st2.motor_current_ma = 0x1234;
    F.app.st2.velocity_01mm_s = -180;
    F.app.st2.faults_lo = 0x42;
    F.app.st2.aux_flags = 0x05;
    uint8_t b[8];
    CHECK_EQ(co_tpdo_pack(&F.app.co.tpdo[1], b), 8);
    CHECK_EQ(b[0], 0xA5);
    CHECK_EQ(b[1], 2);
    CHECK_EQ(b[2], 0x34);
    CHECK_EQ(b[3], 0x12);
    CHECK_EQ(b[4], (uint8_t)(-180 & 0xFF));
    CHECK_EQ(b[5], (uint8_t)((-180 >> 8) & 0xFF));
    CHECK_EQ(b[6], 0x42);
    CHECK_EQ(b[7], 0x05);
    /* No duplicate (index, sub) pairs in the OD. */
    for (size_t i = 0; i < F.app.od_len; i++) {
        for (size_t j = i + 1; j < F.app.od_len; j++) {
            CHECK(!(F.app.od[i].index == F.app.od[j].index && F.app.od[i].sub == F.app.od[j].sub));
        }
    }
}

static void test_error_register(void)
{
    CHECK_EQ(ctl017_error_register(0), 0);
    CHECK_EQ(ctl017_error_register(LIFT_FAULT_F03_OVERCURRENT), 0x83);
    CHECK_EQ(ctl017_error_register(LIFT_FAULT_F02_HALL_MISMATCH), 0x81);
}

/* D12 alternative (a) at the bus level: MOVE_TO_STOP is refused with
 * cmd_result 12 while the core heartbeat 0x701 has never been received, after
 * it timed out, and after the consumer was disabled by SDO. The H-bridge
 * stays off in every case. */
static void test_move_refused_without_core_heartbeat(void)
{
    setup(MM(5));
    F.core_hb = false;
    run(&F, 200);
    nmt(&F, CO_NMT_CS_START);
    run(&F, 10);
    CHECK_EQ(F.app.co.hbc, CO_HBC_WAITING);
    lift_cmd(&F, 1, LIFT_OP_MOVE_TO_STOP, 2);
    run(&F, 300);
    CHECK_EQ(F.app.lift.cmd_result, LIFT_RES_CORE_HEARTBEAT_STALE);
    CHECK(!F.out->hbridge.enable);
    CHECK_EQ(F.app.lift.state, LIFT_STATE_IDLE);
    CHECK_EQ(F.app.st1.cmd_result, LIFT_RES_CORE_HEARTBEAT_STALE);
    CHECK_EQ(F.app.st1.last_seq, 1);

    /* Heartbeat present: accepted. */
    F.core_hb = true;
    F.last_core_hb = F.now - 100;
    run(&F, 50);
    CHECK_EQ(F.app.co.hbc, CO_HBC_ACTIVE);
    lift_cmd(&F, 2, LIFT_OP_MOVE_TO_STOP, 2);
    run(&F, 20);
    CHECK_EQ(F.app.lift.cmd_result, LIFT_RES_ACCEPTED);
    CHECK(F.out->hbridge.enable);
    lift_cmd(&F, 3, LIFT_OP_STOP, 0);
    run(&F, 1000);
    CHECK(!F.out->hbridge.enable);

    /* Heartbeat timed out at rest: refused, no fault raised at rest. */
    F.core_hb = false;
    run(&F, 400);
    CHECK_EQ(F.app.co.hbc, CO_HBC_LOST);
    lift_cmd(&F, 4, LIFT_OP_MOVE_TO_STOP, 1);
    run(&F, 300);
    CHECK_EQ(F.app.lift.cmd_result, LIFT_RES_CORE_HEARTBEAT_STALE);
    CHECK(!F.out->hbridge.enable);

    /* Heartbeat back, then the consumer disabled by SDO (0x1016:01 = 0):
     * no supervision means stale, not fresh. */
    F.core_hb = true;
    F.last_core_hb = F.now - 100;
    run(&F, 50);
    CHECK_EQ(F.app.co.hbc, CO_HBC_ACTIVE);
    nmt(&F, CO_NMT_CS_PREOP);
    const co_frame_t *r = sdo_write(&F, 0x1016, 1, 0, 4);
    CHECK_EQ(r->data[0], 0x60);
    nmt(&F, CO_NMT_CS_START);
    run(&F, 10);
    CHECK_EQ(F.app.co.hbc, CO_HBC_OFF);
    lift_cmd(&F, 5, LIFT_OP_MOVE_TO_STOP, 2);
    run(&F, 300);
    CHECK_EQ(F.app.lift.cmd_result, LIFT_RES_CORE_HEARTBEAT_STALE);
    CHECK(!F.out->hbridge.enable);
}

int main(void)
{
    RUN(test_boot_up_and_preop);
    RUN(test_heartbeat_producer);
    RUN(test_nmt_addressing);
    RUN(test_tpdo1_layout_and_counter);
    RUN(test_tpdo2_layout);
    RUN(test_rpdo1_move_and_status);
    RUN(test_rpdo_state_and_length);
    RUN(test_keepalive_loss_emcy);
    RUN(test_core_heartbeat_loss_stops_move);
    RUN(test_nmt_preop_halts);
    RUN(test_sdo_upload_types);
    RUN(test_sdo_aborts);
    RUN(test_sdo_parameter_writes);
    RUN(test_aux_outputs);
    RUN(test_mapping_consistency);
    RUN(test_error_register);
    RUN(test_move_refused_without_core_heartbeat);
    TEST_MAIN_END();
}
