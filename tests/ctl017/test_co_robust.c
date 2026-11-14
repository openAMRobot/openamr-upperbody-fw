/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Robustness of the CANopen layer: malformed frames, heartbeat consumer
 * events, reset communication, NMT epochs. Each test names the review
 * finding it covers (docs/ctl017_firmware.md, section "Review findings").
 */
#include "test_fixture.h"

/* R1: RTR and 29-bit frames are never CANopen objects. */
static void test_rtr_and_extended_dropped(void)
{
    setup();
    go_operational(&F);
    co_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = ID_RPDO1;
    f.dlc = 3;
    f.rtr = true;
    rx_frame(&F, &f);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->lift_cmd.rx_count, 0);
    f.rtr = false;
    f.ext = true;
    f.data[0] = 1;
    rx_frame(&F, &f);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->lift_cmd.rx_count, 0);
    /* 29-bit NMT stop to "id 0" does nothing. */
    memset(&f, 0, sizeof(f));
    f.ext = true;
    f.dlc = 2;
    f.data[0] = CO_NMT_CS_STOP;
    rx_frame(&F, &f);
    CHECK_EQ(F.app.co.nmt, CO_NMT_OPERATIONAL);
    /* Out-of-range id and DLC. */
    memset(&f, 0, sizeof(f));
    f.id = 0x900;
    f.dlc = 3;
    rx_frame(&F, &f);
    f.id = ID_RPDO1;
    f.dlc = 9;
    rx_frame(&F, &f);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->lift_cmd.rx_count, 0);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->link.rx_malformed, 5);
}

/* R2: NMT node control is exactly 2 bytes; unknown commands ignored. */
static void test_nmt_malformed(void)
{
    setup();
    uint8_t d[8] = {CO_NMT_CS_START, CTL017_NODE_ID, 0, 0, 0, 0, 0, 0};
    rx(&F, 0x000, 1, d);
    rx(&F, 0x000, 3, d);
    rx(&F, 0x000, 8, d);
    CHECK_EQ(F.app.co.nmt, CO_NMT_PRE_OPERATIONAL);
    uint8_t u[2] = {0x55, CTL017_NODE_ID};
    rx(&F, 0x000, 2, u);
    CHECK_EQ(F.app.co.nmt, CO_NMT_PRE_OPERATIONAL);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->link.rx_malformed, 4);
}

/* R3: a heartbeat is one byte with a valid NMT state. Anything else on the
 * COB-ID does not refresh the consumer. */
static void test_heartbeat_malformed_does_not_refresh(void)
{
    setup();
    uint8_t s = 0x05;
    rx(&F, ID_HB_BASE, 1, &s);
    CHECK_EQ(F.app.co.hbc, CO_HBC_ACTIVE);
    uint8_t junk[8] = {0x05, 1, 2, 3, 4, 5, 6, 7};
    for (int i = 0; i < 40; i++) {
        run(&F, 10);
        rx(&F, ID_HB_BASE, (uint8_t)(i % 2 == 0 ? 0 : 8), junk); /* DLC 0 and DLC 8 */
        uint8_t bad = 0x33;
        rx(&F, ID_HB_BASE, 1, &bad); /* invalid state */
    }
    CHECK_EQ(F.app.co.hbc, CO_HBC_LOST);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->link.hbc_timeouts, 1);
}

/* R4: heartbeat timeout is a communication error: EMCY 0x8130, error
 * register bit 4, error reset on recovery; events reach the link record. */
static void test_heartbeat_consumer_events(void)
{
    setup();
    uint8_t s = 0x05;
    rx(&F, ID_HB_BASE, 1, &s);
    F.ntx = 0;
    run(&F, 301);
    CHECK_EQ(F.app.co.hbc, CO_HBC_LOST);
    CHECK_EQ(count_emcy(&F, CO_EMCY_HEARTBEAT), 1);
    const co_frame_t *e = last_with_id(&F, ID_EMCY);
    CHECK_EQ(e->data[2] & 0x11, 0x11);
    CHECK_EQ(e->data[3], TEST_BASE_NODE);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->link.hbc_state, CO_HBC_LOST);
    run(&F, 500);
    CHECK_EQ(count_emcy(&F, CO_EMCY_HEARTBEAT), 1); /* once, not repeated */
    F.ntx = 0;
    rx(&F, ID_HB_BASE, 1, &s);
    CHECK_EQ(F.app.co.hbc, CO_HBC_ACTIVE);
    CHECK_EQ(F.app.co.error_register, 0);
    CHECK_EQ(count_emcy(&F, 0x0000), 1);
    /* Exactly the timeout boundary: t ms is still alive, t+1 is lost. */
    rx(&F, ID_HB_BASE, 1, &s);
    run(&F, 300);
    CHECK_EQ(F.app.co.hbc, CO_HBC_ACTIVE);
    run(&F, 1);
    CHECK_EQ(F.app.co.hbc, CO_HBC_LOST);
}

/* R5: a boot-up of the monitored node is reported, not hidden as a normal
 * heartbeat. */
static void test_remote_boot_reported(void)
{
    setup();
    uint8_t s = 0x05;
    rx(&F, ID_HB_BASE, 1, &s);
    uint8_t boot = 0x00;
    run(&F, 50);
    rx(&F, ID_HB_BASE, 1, &boot);
    run(&F, 1);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->link.hbc_remote_boots, 1);
    CHECK_EQ(F.app.co.hbc_remote_state, 0x00);
}

/* R6: reconfiguring 0x1016 while LOST does not report a recovery and does
 * not make the consumer ACTIVE: it waits for a real heartbeat. */
static void test_hbc_reconfig_no_false_recovery(void)
{
    setup();
    uint8_t s = 0x05;
    rx(&F, ID_HB_BASE, 1, &s);
    run(&F, 400);
    CHECK_EQ(F.app.co.hbc, CO_HBC_LOST);
    const co_frame_t *r = sdo_write(&F, 0x1016, 1, ((uint32_t)TEST_BASE_NODE << 16) | 500u, 4);
    CHECK_EQ(r->data[0], 0x60);
    CHECK_EQ(F.app.co.hbc, CO_HBC_WAITING);
    /* The comm error is cleared (consumer reconfigured), but the link
     * record still counts the one timeout that happened. */
    CHECK_EQ(ctl017_app_rx_view(&F.app)->link.hbc_timeouts, 1);
    run(&F, 2000);
    CHECK_EQ(F.app.co.hbc, CO_HBC_WAITING); /* never heard: no timeout either */
    r = sdo_write(&F, 0x1016, 1, 0, 4);
    CHECK_EQ(F.app.co.hbc, CO_HBC_OFF);
}

/* R7: NMT reset communication restores the power-on communication values. */
static void test_reset_comm_restores_defaults(void)
{
    setup();
    CHECK_EQ(sdo_write(&F, 0x1017, 0, 1000, 2)->data[0], 0x60);
    CHECK_EQ(sdo_write(&F, 0x1016, 1, ((uint32_t)5 << 16) | 50u, 4)->data[0], 0x60);
    CHECK_EQ(sdo_write(&F, 0x1800, 5, 999, 2)->data[0], 0x60);
    CHECK_EQ(sdo_write(&F, 0x1400, 5, 77, 2)->data[0], 0x60);
    nmt(&F, CO_NMT_CS_RESET_COMM);
    CHECK_EQ(sdo_val(sdo_read(&F, 0x1017, 0)), 100);
    CHECK_EQ(sdo_val(sdo_read(&F, 0x1016, 1)), ((uint32_t)TEST_BASE_NODE << 16) | 300u);
    CHECK_EQ(sdo_val(sdo_read(&F, 0x1800, 5)), 100);
    CHECK_EQ(sdo_val(sdo_read(&F, 0x1400, 5)), 300);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->link.rpdo1_event_timer_ms, 300);
}

/* R8: every NMT transition starts a new epoch; command and permit records
 * from before it are no longer valid. */
static void test_nmt_epoch_invalidates_records(void)
{
    setup();
    go_operational(&F);
    const ctl017_iface_rx_t *v = ctl017_app_rx_view(&F.app);
    uint32_t e0 = v->link.nmt_epoch;
    lift_cmd(&F, 4, CTL017_OP_MOVE_TO_STOP, 1);
    CHECK(v->lift_cmd.valid);
    nmt(&F, CO_NMT_CS_PREOP);
    CHECK(!v->lift_cmd.valid);
    CHECK_EQ(v->lift_cmd.seq, 4); /* content kept for diagnostics */
    CHECK(v->link.nmt_epoch > e0);
    nmt(&F, CO_NMT_CS_START);
    CHECK(!v->lift_cmd.valid);
    lift_cmd(&F, 5, CTL017_OP_STOP, 0);
    CHECK(v->lift_cmd.valid);
    uint32_t e1 = v->link.nmt_epoch;
    nmt(&F, CO_NMT_CS_RESET_COMM);
    CHECK(v->link.nmt_epoch > e1);
    CHECK(!v->lift_cmd.valid);
}

/* R9: heartbeat producer retries a refused frame on the next step. */
static void test_heartbeat_retry_after_send_failure(void)
{
    setup();
    run(&F, 99);
    F.tx_refuse = true;
    run(&F, 5);
    F.tx_refuse = false;
    F.ntx = 0;
    run(&F, 1);
    CHECK_EQ(count_id(&F, ID_HB_SELF), 1);
}

/* R10: inhibit time applies to event-timer transmissions too. */
static void test_inhibit_applies_to_event_timer(void)
{
    setup();
    CHECK_EQ(sdo_write(&F, 0x1800, 3, 500, 2)->data[0], 0x60); /* 50 ms */
    CHECK_EQ(sdo_write(&F, 0x1800, 5, 10, 2)->data[0], 0x60);  /* 10 ms */
    nmt(&F, CO_NMT_CS_START);
    F.ntx = 0;
    run(&F, 500);
    unsigned n = count_id(&F, ID_TPDO1);
    CHECK(n >= 9 && n <= 11);
}

/* R11: a frame storm of random content never crashes, never leaves
 * PRE-OPERATIONAL by itself and never produces an RPDO1 record. */
static void test_random_frames(void)
{
    setup();
    uint32_t x = 0x12345678u;
    for (int i = 0; i < 200000; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        co_frame_t f;
        memset(&f, 0, sizeof(f));
        f.id = x & 0x7FFu;
        if (f.id == 0x000 || f.id == ID_SDO_RX) {
            f.id = 0x123; /* NMT and SDO are exercised elsewhere */
        }
        f.dlc = (uint8_t)((x >> 11) % 10u);
        f.rtr = ((x >> 15) & 7u) == 0u;
        f.ext = ((x >> 18) & 7u) == 0u;
        for (int k = 0; k < 8; k++) {
            f.data[k] = (uint8_t)(x >> (k * 3));
        }
        rx_frame(&F, &f);
        if ((i & 1023) == 0) {
            run(&F, 1);
        }
    }
    CHECK_EQ(F.app.co.nmt, CO_NMT_PRE_OPERATIONAL);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->lift_cmd.rx_count, 0);
}

/* R12: random SDO requests: every 8-byte request gets exactly one 8-byte
 * answer or none (abort from client), never a crash. */
static void test_random_sdo(void)
{
    setup();
    uint32_t x = 0xCAFEF00Du;
    for (int i = 0; i < 50000; i++) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        uint8_t d[8];
        for (int k = 0; k < 8; k++) {
            d[k] = (uint8_t)(x >> (k * 4));
        }
        if ((i & 3) == 0) {
            d[1] = (uint8_t)(0x00 + (x & 0x1F)); /* hit real indexes */
            d[2] = (uint8_t)((x >> 8) & 1 ? 0x10 : 0x20);
        }
        if (d[0] == 0x23 && d[1] == 0x16 && d[2] == 0x10) {
            continue; /* would change the consumer; covered above */
        }
        F.ntx = 0;
        rx(&F, ID_SDO_RX, 8, d);
        CHECK(count_id(&F, ID_SDO_TX) <= 1);
    }
}

int main(void)
{
    RUN(test_rtr_and_extended_dropped);
    RUN(test_nmt_malformed);
    RUN(test_heartbeat_malformed_does_not_refresh);
    RUN(test_heartbeat_consumer_events);
    RUN(test_remote_boot_reported);
    RUN(test_hbc_reconfig_no_false_recovery);
    RUN(test_reset_comm_restores_defaults);
    RUN(test_nmt_epoch_invalidates_records);
    RUN(test_heartbeat_retry_after_send_failure);
    RUN(test_inhibit_applies_to_event_timer);
    RUN(test_random_frames);
    RUN(test_random_sdo);
    TEST_MAIN_END();
}
