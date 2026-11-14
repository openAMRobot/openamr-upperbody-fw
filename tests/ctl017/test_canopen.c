/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Host tests of the CTL-017 CANopen slave and application layer against
 * the lift test double: NMT, heartbeat producer and consumer, expedited
 * SDO, PDO encoding and mapping, EMCY and 0x1003, parameter storage, and
 * the not-configured state. Ported from upperbody-fw #16 where the test was
 * about CANopen; tests of lift behaviour stay with the lift core.
 */
#include "test_fixture.h"

static void test_not_configured_is_silent(void)
{
    memset(&F, 0, sizeof(F));
    ctl017_app_config_t c = test_config();
    /* Every required field, one at a time. */
    for (int k = 0; k < 11; k++) {
        ctl017_app_config_t m = c;
        switch (k) {
        case 0: m.base_node_id = 0; break;
        case 1: m.hb_producer_ms = 0; break;
        case 2: m.hb_consumer_ms = 0; break;
        case 3: m.tpdo_inhibit_100us[0] = 0; break;
        case 4: m.tpdo_inhibit_100us[1] = 0; break;
        case 5: m.tpdo_event_ms[0] = 0; break;
        case 6: m.tpdo_event_ms[1] = 0; break;
        case 7: m.rpdo1_event_timer_ms = 0; break;
        case 8: m.product_code = 0; break;
        case 9: m.revision = 0; break;
        default: m.base_node_id = CTL017_NODE_ID; break; /* own node */
        }
        F.ntx = 0;
        CHECK_EQ(ctl017_app_init(&F.app, &m, cap_send, &F, 0), CTL017_APP_NOT_CONFIGURED);
        nmt(&F, CO_NMT_CS_START);
        for (uint32_t t = 0; t < 300; t++) {
            ctl017_app_step(&F.app, t);
        }
        CHECK_EQ(F.ntx, 0);
    }
    CHECK_EQ(ctl017_app_init(&F.app, NULL, cap_send, &F, 0), CTL017_APP_NOT_CONFIGURED);
}

static void test_boot_up_and_preop(void)
{
    setup();
    CHECK(F.ntx >= 1);
    CHECK_EQ(F.tx[0].id, ID_HB_SELF);
    CHECK_EQ(F.tx[0].dlc, 1);
    CHECK_EQ(F.tx[0].data[0], 0x00);
    CHECK_EQ(F.app.co.nmt, CO_NMT_PRE_OPERATIONAL);
    run(&F, 500);
    CHECK_EQ(count_id(&F, ID_TPDO1), 0);
    CHECK_EQ(count_id(&F, ID_TPDO2), 0);
}

static void test_heartbeat_producer(void)
{
    setup();
    F.ntx = 0;
    run(&F, 1000);
    unsigned n = count_id(&F, ID_HB_SELF);
    CHECK(n >= 9 && n <= 11);
    CHECK_EQ(last_with_id(&F, ID_HB_SELF)->data[0], 0x7F);
    nmt(&F, CO_NMT_CS_START);
    run(&F, 150);
    CHECK_EQ(last_with_id(&F, ID_HB_SELF)->data[0], 0x05);
    nmt(&F, CO_NMT_CS_STOP);
    run(&F, 150);
    CHECK_EQ(last_with_id(&F, ID_HB_SELF)->data[0], 0x04);
    /* 0x1017 = 0 disables the producer. */
    nmt(&F, CO_NMT_CS_PREOP);
    CHECK_EQ(sdo_write(&F, 0x1017, 0, 0, 2)->data[0], 0x60);
    F.ntx = 0;
    run(&F, 500);
    CHECK_EQ(count_id(&F, ID_HB_SELF), 0);
}

static void test_nmt_addressing(void)
{
    setup();
    uint8_t other[2] = {CO_NMT_CS_START, 0x11};
    rx(&F, 0x000, 2, other);
    CHECK_EQ(F.app.co.nmt, CO_NMT_PRE_OPERATIONAL);
    uint8_t all[2] = {CO_NMT_CS_START, 0x00};
    rx(&F, 0x000, 2, all);
    CHECK_EQ(F.app.co.nmt, CO_NMT_OPERATIONAL);
    nmt(&F, CO_NMT_CS_RESET_COMM);
    CHECK_EQ(F.app.co.nmt, CO_NMT_PRE_OPERATIONAL);
    CHECK_EQ(last_with_id(&F, ID_HB_SELF)->data[0], 0x00); /* boot-up again */
    CHECK(!ctl017_app_rx_view(&F.app)->link.reset_node_requested);
    nmt(&F, CO_NMT_CS_RESET_NODE);
    CHECK(ctl017_app_rx_view(&F.app)->link.reset_node_requested);
}

/* TPDO1 layout: state, flags, position u16 LE, active_stop_id, last_seq,
 * cmd_result, status_counter. Values come from the double, unchanged. */
static void test_tpdo1_layout_and_counter(void)
{
    setup();
    ctl017_lift_status_t s = status_idle_at_1();
    s.last_seq = 7;
    s.cmd_result = 0;
    lift_double_script(&F.dbl, 0, &s);
    run(&F, 200);
    nmt(&F, CO_NMT_CS_START);
    F.ntx = 0;
    run(&F, 1);
    const co_frame_t *t = last_with_id(&F, ID_TPDO1);
    CHECK(t != NULL);
    if (t == NULL) {
        return;
    }
    CHECK_EQ(t->dlc, 8);
    CHECK_EQ(t->data[0], CTL017_STATE_IDLE);
    CHECK_EQ(t->data[1], s.flags);
    CHECK_EQ(t->data[2] | (t->data[3] << 8), 1800);
    CHECK_EQ(t->data[4], 1);
    CHECK_EQ(t->data[5], 7);
    CHECK_EQ(t->data[6], 0);
    uint8_t c0 = t->data[7];
    F.ntx = 0;
    run(&F, 1000);
    unsigned n = count_id(&F, ID_TPDO1);
    CHECK(n >= 9 && n <= 11);
    CHECK_EQ(last_with_id(&F, ID_TPDO1)->data[7], (uint8_t)(c0 + n));
    /* Counter wraps 255 to 0. */
    F.app.st1.status_counter = 255;
    F.ntx = 0;
    run(&F, 200);
    unsigned k = 0;
    uint8_t seen[4] = {0};
    for (unsigned i = 0; i < F.ntx && k < 4; i++) {
        if (F.tx[i].id == ID_TPDO1) {
            seen[k++] = F.tx[i].data[7];
        }
    }
    CHECK(k >= 2);
    CHECK_EQ(seen[0], 255);
    CHECK_EQ(seen[1], 0);
}

static void test_tpdo1_unknown_until_status(void)
{
    setup();
    nmt(&F, CO_NMT_CS_START);
    F.ntx = 0;
    run(&F, 1);
    const co_frame_t *t = last_with_id(&F, ID_TPDO1);
    CHECK(t != NULL);
    if (t == NULL) {
        return;
    }
    CHECK_EQ(t->data[0], CTL017_STATE_INIT);
    CHECK_EQ(t->data[1], 0);
    CHECK_EQ(t->data[2] | (t->data[3] << 8), 0xFFFF);
    CHECK_EQ(t->data[4], 0xFF);
    CHECK_EQ(t->data[6], 0xFF);
    /* A position without position_valid never reaches the wire. */
    ctl017_lift_status_t s = status_idle_at_1();
    s.flags = 0;
    s.position_01mm = 1234;
    lift_double_script(&F.dbl, F.now + 1, &s);
    F.ntx = 0;
    run(&F, 20);
    t = last_with_id(&F, ID_TPDO1);
    CHECK(t != NULL && (t->data[2] | (t->data[3] << 8)) == 0xFFFF);
}

static void test_tpdo2_layout(void)
{
    setup();
    ctl017_lift_status_t s = status_idle_at_1();
    s.inputs = CTL017_IN_STOP1 | CTL017_IN_HALL_SUPPLY | CTL017_IN_SPARE3;
    s.target_stop_id = 2;
    s.motor_current_ma = 0x1234;
    s.velocity_01mm_s = -5;
    s.faults = 0x0104; /* F03 and F09 */
    s.aux_flags = CTL017_AUX_LIGHT_ON | 0x80u; /* junk bit must not leak */
    lift_double_script(&F.dbl, 0, &s);
    run(&F, 10);
    nmt(&F, CO_NMT_CS_START);
    F.ntx = 0;
    run(&F, 1);
    const co_frame_t *t = last_with_id(&F, ID_TPDO2);
    CHECK(t != NULL);
    if (t == NULL) {
        return;
    }
    CHECK_EQ(t->dlc, 8);
    CHECK_EQ(t->data[0], s.inputs);
    CHECK_EQ(t->data[1], 2);
    CHECK_EQ(t->data[2] | (t->data[3] << 8), 0x1234);
    CHECK_EQ((int16_t)(t->data[4] | (t->data[5] << 8)), -5);
    CHECK_EQ(t->data[6], 0x04);
    CHECK_EQ(t->data[7], CTL017_AUX_F09 | CTL017_AUX_LIGHT_ON);
}

static void test_tpdo_inhibit_and_change(void)
{
    setup();
    ctl017_lift_status_t s = status_idle_at_1();
    lift_double_script(&F.dbl, 0, &s);
    run(&F, 10);
    nmt(&F, CO_NMT_CS_START);
    run(&F, 50);
    /* The status changes every ms for 30 ms: at most one frame per 10 ms
     * inhibit time (CiA 301), not one per change. */
    F.ntx = 0;
    for (int i = 0; i < 30; i++) {
        s.position_01mm = (uint16_t)(1800 + i);
        lift_double_script(&F.dbl, F.now + 1, &s);
        run(&F, 1);
    }
    unsigned n = count_id(&F, ID_TPDO1);
    CHECK(n >= 2 && n <= 4);
    /* The counter alone does not count as a change. */
    run(&F, 120);
    F.ntx = 0;
    run(&F, 95);
    CHECK(count_id(&F, ID_TPDO1) <= 1);
}

static void test_tpdo_send_failure_retried(void)
{
    setup();
    lift_double_script(&F.dbl, 0, &(ctl017_lift_status_t){0});
    nmt(&F, CO_NMT_CS_START);
    run(&F, 5);
    uint8_t c0 = F.app.st1.status_counter;
    F.tx_refuse = true;
    run(&F, 300);
    /* Nothing went out: the counter did not advance and failures counted. */
    CHECK_EQ(F.app.st1.status_counter, c0);
    CHECK(F.app.co.tx_failures > 0);
    F.tx_refuse = false;
    F.ntx = 0;
    run(&F, 1);
    const co_frame_t *t = last_with_id(&F, ID_TPDO1);
    CHECK(t != NULL && t->data[7] == c0);
}

/* RPDO1 is passed through as raw data: duplicates are counted, not
 * interpreted. The double sees every frame, including repeats. */
static void test_rpdo1_raw_passthrough(void)
{
    setup();
    go_operational(&F);
    const ctl017_iface_rx_t *v = ctl017_app_rx_view(&F.app);
    CHECK(!v->lift_cmd.valid);
    lift_cmd(&F, 1, CTL017_OP_MOVE_TO_STOP, 2);
    CHECK(v->lift_cmd.valid);
    CHECK_EQ(v->lift_cmd.seq, 1);
    CHECK_EQ(v->lift_cmd.op, CTL017_OP_MOVE_TO_STOP);
    CHECK_EQ(v->lift_cmd.stop_id, 2);
    CHECK_EQ(v->lift_cmd.rx_count, 1);
    CHECK_EQ(v->lift_cmd.rx_ms, F.now);
    run(&F, 5);
    lift_cmd(&F, 1, CTL017_OP_MOVE_TO_STOP, 2); /* duplicate */
    CHECK_EQ(v->lift_cmd.rx_count, 2);
    CHECK_EQ(F.dbl.n_log, 2);
    /* Unknown op values are data too: the lift owner answers them. */
    lift_cmd(&F, 2, 0xEE, 0x77);
    CHECK_EQ(v->lift_cmd.op, 0xEE);
    CHECK_EQ(F.dbl.n_log, 3);
}

static void test_rpdo_state_and_length(void)
{
    setup();
    const ctl017_iface_rx_t *v = ctl017_app_rx_view(&F.app);
    /* PRE-OPERATIONAL: RPDO ignored. */
    lift_cmd(&F, 1, CTL017_OP_MOVE_TO_STOP, 2);
    CHECK_EQ(v->lift_cmd.rx_count, 0);
    go_operational(&F);
    /* Too short: not processed, counted, EMCY 0x8210 once. */
    F.ntx = 0;
    uint8_t d[2] = {1, 1};
    rx(&F, ID_RPDO1, 2, d);
    rx(&F, ID_RPDO1, 2, d);
    CHECK_EQ(v->lift_cmd.rx_count, 0);
    CHECK_EQ(v->link.rpdo_len_errors, 2);
    CHECK_EQ(count_emcy(&F, CO_EMCY_PDO_LENGTH), 1);
    CHECK((F.app.co.error_register & 0x11u) == 0x11u);
    run(&F, 1);
    sdo_read(&F, 0x2300, 3);
    CHECK_EQ(sdo_val(last_with_id(&F, ID_SDO_TX)), 2);
    /* A valid RPDO clears the communication error: error reset EMCY. */
    F.ntx = 0;
    lift_cmd(&F, 1, CTL017_OP_STOP, 0);
    CHECK_EQ(v->lift_cmd.rx_count, 1);
    CHECK_EQ(F.app.co.error_register, 0);
    CHECK_EQ(count_emcy(&F, 0x0000), 1);
    /* Too long: the first bytes are used, EMCY 0x8220. */
    F.ntx = 0;
    uint8_t l[8] = {2, CTL017_OP_STOP, 0, 9, 9, 9, 9, 9};
    rx(&F, ID_RPDO1, 8, l);
    CHECK_EQ(v->lift_cmd.rx_count, 2);
    CHECK_EQ(v->lift_cmd.seq, 2);
    CHECK_EQ(count_emcy(&F, CO_EMCY_PDO_LENGTH_EX), 1);
}

static void test_sdo_upload_types(void)
{
    setup();
    const co_frame_t *r = sdo_read(&F, 0x1018, 2);
    CHECK_EQ(r->data[0], 0x43);
    CHECK_EQ(sdo_val(r), 0x11);
    r = sdo_read(&F, 0x1017, 0);
    CHECK_EQ(r->data[0], 0x4B);
    CHECK_EQ(sdo_val(r), 100);
    r = sdo_read(&F, 0x1001, 0);
    CHECK_EQ(r->data[0], 0x4F);
    r = sdo_read(&F, 0x1016, 1);
    CHECK_EQ(sdo_val(r), (TEST_BASE_NODE << 16) | 300);
    r = sdo_read(&F, 0x1400, 5);
    CHECK_EQ(sdo_val(r), 300);
}

static void test_sdo_aborts(void)
{
    setup();
    const co_frame_t *r = sdo_read(&F, 0x5555, 0);
    CHECK_EQ(r->data[0], 0x80);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_NO_OBJECT);
    r = sdo_read(&F, 0x1018, 9);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_NO_SUB);
    r = sdo_write(&F, 0x1018, 1, 5, 4);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_READONLY);
    r = sdo_write(&F, 0x1017, 0, 5, 4); /* wrong size indicated */
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_TYPE_LENGTH);
    r = sdo_write(&F, 0x2010, 2, 1, 1); /* command object is RPDO only */
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_LOCAL_CONTROL);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->lift_cmd.rx_count, 0);
    r = sdo_write(&F, 0x2101, 2, 2, 1); /* BOOLEAN > 1 */
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_VALUE_RANGE);
    r = sdo_write(&F, 0x1003, 0, 3, 1);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_VALUE_RANGE);
    /* Consumer of its own heartbeat. */
    r = sdo_write(&F, 0x1016, 1, ((uint32_t)CTL017_NODE_ID << 16) | 300u, 4);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_PARAM_INCOMPAT);
    /* Segmented download initiate. */
    uint8_t seg[8] = {0x21, 0x17, 0x10, 0, 4, 0, 0, 0};
    F.ntx = 0;
    rx(&F, ID_SDO_RX, 8, seg);
    CHECK_EQ(sdo_val(last_with_id(&F, ID_SDO_TX)), CO_SDO_ABORT_UNSUPPORTED);
    /* Unknown command specifier. */
    uint8_t bad[8] = {0xE0, 0x00, 0x10, 0, 0, 0, 0, 0};
    F.ntx = 0;
    rx(&F, ID_SDO_RX, 8, bad);
    CHECK_EQ(sdo_val(last_with_id(&F, ID_SDO_TX)), CO_SDO_ABORT_CMD);
    /* Short SDO frame: dropped silently (CiA 301: SDO is 8 bytes). */
    F.ntx = 0;
    rx(&F, ID_SDO_RX, 4, bad);
    CHECK_EQ(F.ntx, 0);
    /* Client abort: no answer. */
    uint8_t ab[8] = {0x80, 0x00, 0x10, 0, 0, 0, 0, 0};
    rx(&F, ID_SDO_RX, 8, ab);
    CHECK_EQ(F.ntx, 0);
    /* STOPPED: SDO not served. */
    nmt(&F, CO_NMT_CS_STOP);
    r = sdo_read(&F, 0x1000, 0);
    CHECK(r == NULL);
}

static void test_sdo_parameter_writes(void)
{
    setup();
    /* No validator: nothing writable. */
    ctl017_app_set_params_validator(&F.app, NULL, NULL);
    const co_frame_t *r = sdo_write(&F, 0x2100, 1, 1234, 4);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_DEVICE_STATE);
    CHECK_EQ(ctl017_app_params(&F.app)->stop_pos_um[0], 0);
    ctl017_app_set_params_validator(&F.app, lift_double_params_valid, &F.dbl);
    r = sdo_write(&F, 0x2100, 1, 1234, 4);
    CHECK_EQ(r->data[0], 0x60);
    CHECK_EQ(ctl017_app_params(&F.app)->stop_pos_um[0], 1234);
    CHECK_EQ(ctl017_app_params(&F.app)->write_count, 1);
    /* Validator refuses: value reverted. */
    F.dbl.params_ok = false;
    r = sdo_write(&F, 0x2100, 1, 99, 4);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_VALUE_RANGE);
    CHECK_EQ(ctl017_app_params(&F.app)->stop_pos_um[0], 1234);
    F.dbl.params_ok = true;
    r = sdo_write(&F, 0x2101, 9, 777, 2);
    CHECK_EQ(r->data[0], 0x60);
    CHECK_EQ(ctl017_app_params(&F.app)->duty_move_permille, 777);
    r = sdo_write(&F, 0x2101, 29, 1, 1);
    CHECK(ctl017_app_params(&F.app)->end_switches_wired);
    /* OPERATIONAL: PRE-OP-only objects refused. */
    nmt(&F, CO_NMT_CS_START);
    r = sdo_write(&F, 0x2100, 1, 5, 4);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_DEVICE_STATE);
    r = sdo_write(&F, 0x1400, 5, 250, 2);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_DEVICE_STATE);
    nmt(&F, CO_NMT_CS_PREOP);
    r = sdo_write(&F, 0x1400, 5, 250, 2);
    CHECK_EQ(r->data[0], 0x60);
    CHECK_EQ(ctl017_app_rx_view(&F.app)->link.rpdo1_event_timer_ms, 250);
}

static void test_aux_records(void)
{
    setup();
    go_operational(&F);
    const ctl017_iface_rx_t *v = ctl017_app_rx_view(&F.app);
    CHECK(!v->aux_cmd.valid);
    uint8_t d[3] = {1, 40, 1};
    rx(&F, ID_RPDO2, 3, d);
    CHECK(v->aux_cmd.valid);
    CHECK_EQ(v->aux_cmd.head_light_on, 1);
    CHECK_EQ(v->aux_cmd.head_light_dim_pct, 40);
    CHECK_EQ(v->aux_cmd.getac_enable, 1);
    CHECK_EQ(v->aux_cmd.rx_count, 1);
    /* Out of range: not a new sample, last accepted values stay. */
    uint8_t bad[3] = {2, 140, 0};
    rx(&F, ID_RPDO2, 3, bad);
    CHECK_EQ(v->aux_cmd.rx_count, 1);
    CHECK_EQ(v->aux_cmd.head_light_dim_pct, 40);
    CHECK_EQ(F.app.aux_range_errors, 1);
    const co_frame_t *r = sdo_read(&F, 0x2011, 2);
    CHECK_EQ(sdo_val(r), 40);
    /* SDO writes land in the same record. */
    r = sdo_write(&F, 0x2011, 2, 101, 1);
    CHECK_EQ(sdo_val(r), CO_SDO_ABORT_VALUE_RANGE);
    r = sdo_write(&F, 0x2011, 1, 0, 1);
    CHECK_EQ(r->data[0], 0x60);
    CHECK_EQ(v->aux_cmd.head_light_on, 0);
    CHECK_EQ(v->aux_cmd.rx_count, 2);
}

static void test_emcy_and_error_history(void)
{
    setup();
    nmt(&F, CO_NMT_CS_START);
    ctl017_lift_status_t s = status_idle_at_1();
    s.faults = 0x0004; /* F03 */
    lift_double_script(&F.dbl, F.now + 1, &s);
    F.ntx = 0;
    run(&F, 5);
    CHECK_EQ(count_emcy(&F, 0xFF03), 1);
    const co_frame_t *e = last_with_id(&F, ID_EMCY);
    CHECK_EQ(e->data[2], 0x83); /* generic, current, manufacturer */
    CHECK_EQ(e->data[3], 3);
    CHECK_EQ(e->data[4], 0x04);
    CHECK_EQ(F.app.co.err_count, 1);
    CHECK_EQ(sdo_val(sdo_read(&F, 0x2002, 0)), 0x0004);
    /* F07 and F09 added: one EMCY each, no repeat for F03. */
    s.faults = 0x0144;
    lift_double_script(&F.dbl, F.now + 1, &s);
    F.ntx = 0;
    run(&F, 5);
    CHECK_EQ(count_emcy(&F, 0xFF03), 0);
    CHECK_EQ(count_emcy(&F, 0xFF07), 1);
    CHECK_EQ(count_emcy(&F, 0xFF09), 1);
    CHECK_EQ(F.app.co.err_count, 3);
    CHECK_EQ(F.app.co.err_history[0] & 0xFFFFu, 0xFF09);
    /* Cleared: one error reset EMCY. */
    s.faults = 0;
    lift_double_script(&F.dbl, F.now + 1, &s);
    F.ntx = 0;
    run(&F, 5);
    CHECK_EQ(count_emcy(&F, 0x0000), 1);
    CHECK_EQ(F.app.co.error_register, 0);
    /* 0x1003:00 = 0 clears the history. */
    nmt(&F, CO_NMT_CS_PREOP);
    CHECK_EQ(sdo_write(&F, 0x1003, 0, 0, 1)->data[0], 0x60);
    CHECK_EQ(F.app.co.err_count, 0);
}

static void test_mapping_consistency(void)
{
    setup();
    const uint32_t ids[4] = {0x1600, 0x1601, 0x1A00, 0x1A01};
    const uint8_t lens[4] = {3, 3, 8, 8};
    for (int k = 0; k < 4; k++) {
        uint32_t n = sdo_val(sdo_read(&F, (uint16_t)ids[k], 0));
        unsigned bytes = 0;
        for (uint8_t s = 1; s <= n; s++) {
            uint32_t m = sdo_val(sdo_read(&F, (uint16_t)ids[k], s));
            const co_od_entry_t *e = co_od_find(F.app.od, F.app.od_len, (uint16_t)(m >> 16), (uint8_t)(m >> 8));
            CHECK(e != NULL && e->pdo_mappable && co_type_size(e->type) * 8u == (m & 0xFFu));
            bytes += (m & 0xFFu) / 8u;
        }
        CHECK_EQ(bytes, lens[k]);
    }
    /* Every sub 0 "highest sub-index" equals the largest sub present. */
    for (size_t i = 0; i < F.app.od_len; i++) {
        const co_od_entry_t *e = &F.app.od[i];
        if (e->sub != 0 || e->index <= 0x1003 || co_type_size(e->type) != 1) { /* 0x1003:00 = number of errors */
            continue;
        }
        uint8_t max = 0;
        bool record = false;
        for (size_t j = 0; j < F.app.od_len; j++) {
            if (F.app.od[j].index == e->index && F.app.od[j].sub > 0) {
                record = true;
                if (F.app.od[j].sub > max) {
                    max = F.app.od[j].sub;
                }
            }
        }
        if (record) {
            CHECK_EQ(co_od_get(e), max);
        }
    }
}

static void test_error_register(void)
{
    CHECK_EQ(ctl017_error_register(0), 0);
    CHECK_EQ(ctl017_error_register(0x0004), 0x83);
    CHECK_EQ(ctl017_error_register(0x0040), 0x91);
    CHECK_EQ(ctl017_error_register(0x0100), 0x81);
}

int main(void)
{
    RUN(test_not_configured_is_silent);
    RUN(test_boot_up_and_preop);
    RUN(test_heartbeat_producer);
    RUN(test_nmt_addressing);
    RUN(test_tpdo1_layout_and_counter);
    RUN(test_tpdo1_unknown_until_status);
    RUN(test_tpdo2_layout);
    RUN(test_tpdo_inhibit_and_change);
    RUN(test_tpdo_send_failure_retried);
    RUN(test_rpdo1_raw_passthrough);
    RUN(test_rpdo_state_and_length);
    RUN(test_sdo_upload_types);
    RUN(test_sdo_aborts);
    RUN(test_sdo_parameter_writes);
    RUN(test_aux_records);
    RUN(test_emcy_and_error_history);
    RUN(test_mapping_consistency);
    RUN(test_error_register);
    TEST_MAIN_END();
}
