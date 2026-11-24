/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Permit object extension (CTL017_PERMIT_OD). Built only when the option
 * is on; the parameter values come from the build (CTL017_PERMIT_DEFS) and
 * are not proposals. Checks that the permit is received and exposed as raw
 * data and has no effect on anything else the layer does.
 */
#include "test_fixture.h"

#define ID_PERMIT ((uint16_t)CTL017_PERMIT_COB_ID)
#define PERMIT_LEN ((CTL017_PERMIT_FLAG_BITS + CTL017_PERMIT_BOOT_BITS + CTL017_PERMIT_SEQ_BITS + \
                     CTL017_PERMIT_REASON_BITS) / 8)

static unsigned put(uint8_t *d, unsigned at, uint32_t v, unsigned bits)
{
    for (unsigned i = 0; i < bits / 8u; i++) {
        d[at + i] = (uint8_t)(v >> (8u * i));
    }
    return at + bits / 8u;
}

static void permit_frame(fx_t *x, uint32_t flag, uint32_t boot, uint32_t seq, uint32_t reason, uint8_t dlc)
{
    uint8_t d[8] = {0};
    unsigned a = put(d, 0, flag, CTL017_PERMIT_FLAG_BITS);
    a = put(d, a, boot, CTL017_PERMIT_BOOT_BITS);
    a = put(d, a, seq, CTL017_PERMIT_SEQ_BITS);
    (void)put(d, a, reason, CTL017_PERMIT_REASON_BITS);
    rx(x, ID_PERMIT, dlc, d);
}

static void test_permit_raw_passthrough(void)
{
    setup();
    go_operational(&F);
    const ctl017_iface_rx_t *v = ctl017_app_rx_view(&F.app);
    CHECK(!v->permit.valid);
    permit_frame(&F, 1, 0x21, 0x05, 0x02, PERMIT_LEN);
    CHECK(v->permit.valid);
    CHECK_EQ(v->permit.permit_flag, 1);
    CHECK_EQ(v->permit.producer_boot_id, 0x21);
    CHECK_EQ(v->permit.sequence, 0x05);
    CHECK_EQ(v->permit.reason, 0x02);
    CHECK_EQ(v->permit.rx_count, 1);
    /* Duplicate and out-of-order sequence are data, counted, not judged. */
    permit_frame(&F, 1, 0x21, 0x05, 0x02, PERMIT_LEN);
    permit_frame(&F, 1, 0x21, 0x03, 0x02, PERMIT_LEN);
    CHECK_EQ(v->permit.rx_count, 3);
    CHECK_EQ(v->permit.sequence, 0x03);
    /* Too short: not processed. */
    permit_frame(&F, 0, 0, 9, 0, (uint8_t)(PERMIT_LEN - 1));
    CHECK_EQ(v->permit.rx_count, 3);
    CHECK_EQ(v->permit.sequence, 0x03);
    /* NMT transition: no longer valid until a new frame. */
    nmt(&F, CO_NMT_CS_PREOP);
    CHECK(!v->permit.valid);
    nmt(&F, CO_NMT_CS_START);
    CHECK(!v->permit.valid);
}

/* The permit has no effect on commands, status or anything sent: the same
 * scenario run with permit frames of every flag value and without any gives
 * the same command records and the same transmitted frames. */
typedef struct {
    ctl017_lift_cmd_rx_t cmd;
    unsigned ntx;
    co_frame_t tx[MAXF];
} outcome_t;

static outcome_t out[2];

static void scenario(int with_permit, outcome_t *o)
{
    setup();
    go_operational(&F);
    F.ntx = 0;
    for (uint32_t i = 0; i < 600; i++) {
        if (with_permit && i % 50 == 0) {
            permit_frame(&F, (i / 50) & 1u, 7, i, i / 100, PERMIT_LEN);
        }
        if (i == 100) {
            lift_cmd(&F, 1, CTL017_OP_MOVE_TO_STOP, 2);
        }
        if (i == 300) {
            lift_cmd(&F, 2, CTL017_OP_START_HOMING, 0);
        }
        tick(&F);
    }
    o->cmd = ctl017_app_rx_view(&F.app)->lift_cmd;
    o->ntx = F.ntx;
    memcpy(o->tx, F.tx, sizeof(o->tx));
}

static void test_permit_has_no_effect(void)
{
    scenario(0, &out[0]);
    scenario(1, &out[1]);
    CHECK(memcmp(&out[0].cmd, &out[1].cmd, sizeof(out[0].cmd)) == 0);
    CHECK_EQ(out[0].ntx, out[1].ntx);
    CHECK(out[0].ntx > 0 && memcmp(out[0].tx, out[1].tx, out[0].ntx * sizeof(co_frame_t)) == 0);
}

static void test_permit_not_writable_by_sdo(void)
{
    setup();
    const co_frame_t *r = sdo_write(&F, (uint16_t)CTL017_PERMIT_OD_INDEX, 1, 1, CTL017_PERMIT_FLAG_BITS / 8);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_LOCAL_CONTROL);
    CHECK(!ctl017_app_rx_view(&F.app)->permit.valid);
    r = sdo_write(&F, 0x1402, 1, 0x123, 4);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_READONLY);
}

int main(void)
{
    RUN(test_permit_raw_passthrough);
    RUN(test_permit_has_no_effect);
    RUN(test_permit_not_writable_by_sdo);
    TEST_MAIN_END();
}
