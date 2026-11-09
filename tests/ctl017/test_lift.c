/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Host unit tests of the CTL-017 lift core against the kinematic plant.
 * Test IDs in comments refer to the verification matrix of the CAN3 lift
 * message set (T01 to T34).
 */
#include <string.h>

#include "ctl017/lift.h"
#include "lift_plant.h"
#include "test_util.h"

#define MM(x) ((int64_t)(x) * 1000000)

typedef struct {
    lift_t lift;
    lift_plant_t plant;
    lift_config_t cfg;
    uint32_t now;
    const lift_hbridge_cmd_t *hb;
    /* keep-alive generator */
    bool keepalive;
    uint8_t seq, op, stop_id;
    uint32_t last_ka;
    /* raw input overrides */
    bool force_lower, force_upper;
    bool track_max_enable;
    bool ever_enabled;
} fx_t;

static void fx_cfg(lift_config_t *c)
{
    lift_config_default(c);
    /* The plant's sensors switch on 1 mm before the stop in each direction. */
    c->sensor_edge_offset_up_um = -1000;
    c->sensor_edge_offset_down_um = 1000;
}

static void check_invariants(fx_t *f)
{
    lift_status_t st;
    lift_get_status(&f->lift, &st);
    bool at_stop = (st.flags & LIFT_FLAG_AT_STOP) != 0;
    CHECK(at_stop == (st.active_stop_id != LIFT_STOP_NONE));
    if (at_stop) {
        CHECK(st.flags & LIFT_FLAG_POSITION_VALID);
        CHECK(!(st.flags & LIFT_FLAG_MOVING));
    }
    if (!(st.flags & LIFT_FLAG_POSITION_VALID)) {
        CHECK_EQ(st.position_01mm, LIFT_POSITION_UNKNOWN);
    }
    CHECK(((st.flags & LIFT_FLAG_FAULT_PRESENT) != 0) == (st.faults != 0));
}

static void fx_tick(fx_t *f)
{
    lift_inputs_t in;
    f->now += 1;
    if (f->keepalive && (uint32_t)(f->now - f->last_ka) >= 100) {
        lift_command(&f->lift, f->seq, f->op, f->stop_id, f->now);
        f->last_ka = f->now;
    }
    plant_inputs(&f->plant, f->now, &in);
    if (f->force_lower) {
        in.lower_limit = true;
    }
    if (f->force_upper) {
        in.upper_limit = true;
    }
    f->hb = lift_step(&f->lift, &in);
    if (f->hb->enable && !f->hb->brake) {
        f->ever_enabled = true;
    }
    plant_step(&f->plant, f->hb, 1);
    check_invariants(f);
}

static void fx_run(fx_t *f, uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++) {
        fx_tick(f);
    }
}

/* Runs until the lift is in one of the rest states or the limit passes. */
static void fx_run_until_rest(fx_t *f, uint32_t limit_ms)
{
    for (uint32_t i = 0; i < limit_ms; i++) {
        fx_tick(f);
        lift_state_t s = f->lift.state;
        if ((s == LIFT_STATE_IDLE || s == LIFT_STATE_UNHOMED || s == LIFT_STATE_FAULT) &&
            !lift_motion_in_progress(&f->lift)) {
            return;
        }
    }
}

static void fx_init_cfg(fx_t *f, int64_t start_nm, const lift_config_t *cfg)
{
    memset(f, 0, sizeof(*f));
    f->cfg = *cfg;
    f->now = 1000;
    plant_init(&f->plant, start_nm);
    lift_init(&f->lift, &f->cfg, f->now);
    fx_run(f, 200); /* settle: INIT done and standstill confirmed */
}

static void fx_init(fx_t *f, int64_t start_nm)
{
    lift_config_t c;
    fx_cfg(&c);
    fx_init_cfg(f, start_nm, &c);
}

static uint8_t fx_cmd(fx_t *f, uint8_t op, uint8_t stop_id)
{
    f->seq++;
    f->op = op;
    f->stop_id = stop_id;
    f->last_ka = f->now;
    f->keepalive = true;
    return lift_command(&f->lift, f->seq, op, stop_id, f->now);
}

static lift_status_t fx_status(fx_t *f)
{
    lift_status_t st;
    lift_get_status(&f->lift, &st);
    return st;
}

static int64_t abs64(int64_t v)
{
    return v < 0 ? -v : v;
}

/* ------------------------------------------------------------------------ */

static void test_config_defaults_valid(void)
{
    lift_config_t c;
    lift_config_default(&c);
    CHECK(lift_config_valid(&c));
    c.stop_pos_um[1] = c.stop_pos_um[0];
    CHECK(!lift_config_valid(&c));
}

/* P-03 item 24: a stop sensor active at power-up gives a valid position. */
static void test_boot_at_sensor_is_valid(void)
{
    fx_t f;
    fx_init(&f, MM(180));
    lift_status_t st = fx_status(&f);
    CHECK_EQ(st.state, LIFT_STATE_IDLE);
    CHECK(st.flags & LIFT_FLAG_POSITION_VALID);
    CHECK(!(st.flags & LIFT_FLAG_RECOVERY_REQUIRED));
    CHECK_EQ(st.active_stop_id, 1);
    CHECK_EQ(st.position_01mm, 1800);
    CHECK_EQ(st.last_seq, 0);          /* T27 */
    CHECK_EQ(st.cmd_result, 0xFF);
}

static void test_boot_between_stops_is_unhomed(void)
{
    fx_t f;
    fx_init(&f, MM(100));
    lift_status_t st = fx_status(&f);
    CHECK_EQ(st.state, LIFT_STATE_UNHOMED);
    CHECK(st.flags & LIFT_FLAG_RECOVERY_REQUIRED);
    CHECK_EQ(st.position_01mm, LIFT_POSITION_UNKNOWN);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2), LIFT_RES_RECOVERY_REQUIRED); /* T16 */
    fx_run(&f, 50);
    CHECK(!f.ever_enabled); /* T18 */
}

static void test_homing_from_between_stops_ends_at_L1000(void)
{
    fx_t f;
    fx_init(&f, MM(100));
    CHECK_EQ(fx_cmd(&f, LIFT_OP_START_HOMING, 0), LIFT_RES_ACCEPTED);
    fx_run(&f, 5);
    CHECK_EQ(f.lift.state, LIFT_STATE_HOMING);
    CHECK_EQ(f.hb->dir, LIFT_DIR_DOWN);
    CHECK_EQ(f.hb->duty_permille, f.cfg.duty_homing_permille);
    fx_run_until_rest(&f, 40000);
    lift_status_t st = fx_status(&f);
    CHECK_EQ(st.state, LIFT_STATE_IDLE);
    CHECK_EQ(st.active_stop_id, 0);
    CHECK(!(st.flags & LIFT_FLAG_RECOVERY_REQUIRED));
    CHECK(abs64(f.plant.pos_nm - MM(5)) <= MM(2));
    CHECK_EQ(st.faults, 0);
}

/* Homing references at the first sensor met (L1175), cross-checks L1000. */
static void test_homing_from_above_references_at_first_sensor(void)
{
    fx_t f;
    fx_init(&f, MM(250));
    CHECK_EQ(fx_cmd(&f, LIFT_OP_START_HOMING, 0), LIFT_RES_ACCEPTED);
    bool valid_seen_above_l1000 = false;
    for (int i = 0; i < 60000 && f.lift.state == LIFT_STATE_HOMING; i++) {
        fx_tick(&f);
        if (f.lift.position_valid && f.plant.pos_nm > MM(150)) {
            valid_seen_above_l1000 = true;
        }
    }
    fx_run_until_rest(&f, 2000);
    CHECK(valid_seen_above_l1000);
    CHECK_EQ(f.lift.state, LIFT_STATE_IDLE);
    CHECK_EQ(fx_status(&f).active_stop_id, 0);
}

static void test_homing_without_continue_stops_at_first_sensor(void)
{
    fx_t f;
    lift_config_t c;
    fx_cfg(&c);
    c.homing_continue_to_stop0 = false;
    fx_init_cfg(&f, MM(250), &c);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_START_HOMING, 0), LIFT_RES_ACCEPTED);
    fx_run_until_rest(&f, 40000);
    CHECK_EQ(f.lift.state, LIFT_STATE_IDLE);
    CHECK_EQ(fx_status(&f).active_stop_id, 1);
}

/* T31: full move L1000 -> L1350 passing L1175; active_stop_id 0xFF while
 * moving, target in TPDO2, arrival within +/- 2 mm. */
static void test_move_up_full_travel(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    CHECK_EQ(fx_status(&f).active_stop_id, 0);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2), LIFT_RES_ACCEPTED);
    fx_run(&f, 50);
    lift_status_t st = fx_status(&f);
    CHECK_EQ(st.state, LIFT_STATE_MOVING);
    CHECK(st.flags & LIFT_FLAG_MOVING);
    CHECK_EQ(st.active_stop_id, LIFT_STOP_NONE);
    CHECK_EQ(st.target_stop_id, 2);
    CHECK_EQ(f.hb->dir, LIFT_DIR_UP);
    fx_run(&f, 2000);
    CHECK(fx_status(&f).velocity_01mm_s > 150); /* about 18 mm/s */
    fx_run_until_rest(&f, 30000);
    st = fx_status(&f);
    CHECK_EQ(st.state, LIFT_STATE_IDLE);
    CHECK_EQ(st.active_stop_id, 2);
    CHECK_EQ(st.target_stop_id, LIFT_STOP_NONE);
    CHECK_EQ(st.faults, 0);
    CHECK(abs64(f.plant.pos_nm - MM(355)) <= MM(2));
    CHECK(abs64(lift_position_nm(&f.lift) - f.plant.pos_nm) <= MM(1));
    CHECK_EQ(st.velocity_01mm_s, 0);
}

static void test_move_down_one_stop(void)
{
    fx_t f;
    fx_init(&f, MM(355));
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 1), LIFT_RES_ACCEPTED);
    fx_run(&f, 50);
    CHECK_EQ(f.hb->dir, LIFT_DIR_DOWN);
    CHECK(fx_status(&f).velocity_01mm_s == 0 || fx_status(&f).velocity_01mm_s < 0);
    fx_run_until_rest(&f, 30000);
    CHECK_EQ(f.lift.state, LIFT_STATE_IDLE);
    CHECK_EQ(fx_status(&f).active_stop_id, 1);
    CHECK(abs64(f.plant.pos_nm - MM(180)) <= MM(2));
}

/* T32: MOVE_TO_STOP to the stop already occupied is an accepted no-op. */
static void test_move_to_current_stop_is_noop(void)
{
    fx_t f;
    fx_init(&f, MM(180));
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 1), LIFT_RES_ACCEPTED);
    fx_run(&f, 100);
    CHECK(!f.ever_enabled);
    CHECK_EQ(f.lift.state, LIFT_STATE_IDLE);
}

/* T03: command keep-alive withheld during a move: F07 within the deadline,
 * target discarded, no resume. */
static void test_keepalive_loss_raises_f07(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 1000);
    CHECK_EQ(f.lift.state, LIFT_STATE_MOVING);
    f.keepalive = false;
    uint32_t t0 = f.now;
    while (!(f.lift.faults & LIFT_FAULT_F07_SUPERVISION) && f.now - t0 < 1000) {
        fx_tick(&f);
    }
    uint32_t dt = f.now - t0;
    CHECK(dt <= 300); /* last keep-alive was at most 100 ms before t0 */
    CHECK(dt >= 200);
    CHECK(!f.hb->enable);
    CHECK_EQ(f.lift.state, LIFT_STATE_FAULT);
    CHECK_EQ(fx_status(&f).target_stop_id, LIFT_STOP_NONE);
    /* F07 does not invalidate the position. */
    CHECK(f.lift.position_valid);
    fx_run(&f, 300);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2), LIFT_RES_FAULT_ACTIVE);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_CLEAR_FAULT, 0), LIFT_RES_ACCEPTED);
    f.keepalive = false;
    CHECK_EQ(f.lift.state, LIFT_STATE_IDLE); /* position valid, homed, no fault */
    fx_run(&f, 500);
    CHECK(!f.hb->enable); /* T15: clearing never moves */
}

/* T26 under D12 alternative (a): core heartbeat lost with keep-alive present
 * also stops the move. */
static void test_core_heartbeat_loss_raises_f07(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 1);
    fx_run(&f, 500);
    lift_set_supervision_lost(&f.lift, true);
    fx_run(&f, 2);
    CHECK(f.lift.faults & LIFT_FAULT_F07_SUPERVISION);
    CHECK(!f.hb->enable);
}

/* T34: same seq different content rejected with 10 and no state change; a
 * late keep-alive of a completed command is not executed again. */
static void test_seq_semantics(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 1);
    fx_run_until_rest(&f, 30000);
    CHECK_EQ(fx_status(&f).active_stop_id, 1);
    f.keepalive = false;
    f.ever_enabled = false;
    /* Late keep-alive of the completed move. */
    CHECK_EQ(lift_command(&f.lift, f.seq, LIFT_OP_MOVE_TO_STOP, 1, f.now), LIFT_RES_ACCEPTED);
    fx_run(&f, 100);
    CHECK(!f.ever_enabled);
    /* Same seq, different content. */
    CHECK_EQ(lift_command(&f.lift, f.seq, LIFT_OP_MOVE_TO_STOP, 2, f.now), LIFT_RES_SEQ_CONFLICT);
    fx_run(&f, 100);
    CHECK(!f.ever_enabled);
    CHECK_EQ(fx_status(&f).last_seq, f.seq);
    CHECK_EQ(fx_status(&f).cmd_result, LIFT_RES_SEQ_CONFLICT);
}

static void test_malformed_and_invalid_stop(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    CHECK_EQ(fx_cmd(&f, 7, 0), LIFT_RES_MALFORMED);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 3), LIFT_RES_INVALID_STOP);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 0xFF), LIFT_RES_INVALID_STOP);
    f.keepalive = false;
    fx_run(&f, 100);
    CHECK(!f.ever_enabled);
}

static void test_busy_while_moving(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 500);
    f.seq++;
    CHECK_EQ(lift_command(&f.lift, f.seq, LIFT_OP_MOVE_TO_STOP, 1, f.now), LIFT_RES_BUSY);
    CHECK_EQ(f.lift.target_stop, 2); /* target unchanged */
    CHECK_EQ(lift_command(&f.lift, (uint8_t)(f.seq + 1), LIFT_OP_START_HOMING, 0, f.now), LIFT_RES_BUSY);
    CHECK_EQ(lift_command(&f.lift, (uint8_t)(f.seq + 2), LIFT_OP_CLEAR_FAULT, 0, f.now), LIFT_RES_BUSY);
}

/* Travel timeout per move (P-03 item 30 A7): F04. */
static void test_travel_timeout_f04(void)
{
    fx_t f;
    lift_config_t c;
    fx_cfg(&c);
    c.move_speed_um_s = 36000; /* expects twice the plant speed */
    c.travel_timeout_factor_pct = 100;
    c.travel_timeout_margin_ms = 0;
    fx_init_cfg(&f, MM(5), &c);
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 1);
    uint32_t t0 = f.now;
    fx_run_until_rest(&f, 30000);
    CHECK_EQ(f.lift.faults, LIFT_FAULT_F04_MOVE_TIMEOUT);
    uint32_t expect = 175000 * 1000 / 36000; /* 4861 ms */
    CHECK(f.now - t0 >= expect);
    CHECK(f.now - t0 <= expect + 200);
    CHECK(f.lift.position_valid); /* F04 keeps the count */
}

/* Hall-window check: rate above the window raises F02 and invalidates. */
static void test_hall_window_high_rate_f02(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    f.plant.hall_rate_pct = 200;
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 1000);
    CHECK(f.lift.faults & LIFT_FAULT_F02_HALL_MISMATCH);
    CHECK(!f.lift.position_valid);
    CHECK(fx_status(&f).flags & LIFT_FLAG_RECOVERY_REQUIRED);
    CHECK(!f.hb->enable);
}

static void test_hall_window_low_rate_f02(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    f.plant.hall_rate_pct = 30;
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 1000);
    CHECK(f.lift.faults & LIFT_FAULT_F02_HALL_MISMATCH);
}

static void test_hall_window_nominal_no_fault_during_grace(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    f.plant.hall_rate_pct = 300; /* wrong rate, but only checked after the grace */
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, f.cfg.hall_start_grace_ms - 10);
    CHECK_EQ(f.lift.faults, 0);
    fx_run(&f, f.cfg.hall_window_ms + 20);
    CHECK(f.lift.faults & LIFT_FAULT_F02_HALL_MISMATCH);
}

/* No Hall pulses while driving: stall, F03 (Alex 24 Oct). */
static void test_hall_frozen_stall_f03(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 1000);
    f.plant.hall_frozen = true;
    fx_run(&f, f.cfg.stall_ms + 5);
    CHECK_EQ(f.lift.faults, LIFT_FAULT_F03_OVERCURRENT);
    CHECK(!f.hb->enable);
}

static void test_jammed_column_f03(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    f.plant.jammed = true;
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 1);
    fx_run(&f, f.cfg.hall_start_grace_ms + f.cfg.stall_ms + 5);
    CHECK(f.lift.faults & LIFT_FAULT_F03_OVERCURRENT);
}

static void test_wrong_direction_f02(void)
{
    fx_t f;
    fx_init(&f, MM(180));
    f.plant.hall_rate_pct = -100;
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 200);
    CHECK(f.lift.faults & LIFT_FAULT_F02_HALL_MISMATCH);
}

static void test_overcurrent_f03_and_inrush_blank(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    f.plant.extra_current_ma = 8000;
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, f.cfg.inrush_blank_ms - 20);
    f.plant.extra_current_ma = 0; /* inrush over */
    fx_run(&f, 500);
    CHECK_EQ(f.lift.faults, 0);
    f.plant.extra_current_ma = 8000;
    fx_run(&f, f.cfg.overcurrent_ms - 10);
    CHECK_EQ(f.lift.faults, 0);
    fx_run(&f, 20);
    CHECK(f.lift.faults & LIFT_FAULT_F03_OVERCURRENT);
    /* Clearing is refused while the current stays above the limit. */
    fx_run(&f, 300);
    f.keepalive = false;
    f.plant.extra_current_ma = 9000;
    fx_run(&f, 1);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_CLEAR_FAULT, 0), LIFT_RES_CLEAR_CONDITIONS);
    f.plant.extra_current_ma = 0;
    fx_run(&f, 1);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_CLEAR_FAULT, 0), LIFT_RES_ACCEPTED);
}

/* Stop sensor priority: the move ends only when the target sensor is active,
 * and Hall drift is removed at each sensor edge. */
static void test_stop_sensor_priority_corrects_drift(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 3000);
    f.plant.count_offset += 7; /* about 1.5 mm of lost counts */
    fx_run_until_rest(&f, 30000);
    CHECK_EQ(f.lift.faults, 0);
    CHECK_EQ(fx_status(&f).active_stop_id, 2);
    CHECK(abs64(f.plant.pos_nm - MM(355)) <= MM(2));
    CHECK(abs64(lift_position_nm(&f.lift) - f.plant.pos_nm) <= MM(1));
}

static void test_hall_drift_beyond_window_f02(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 3000);
    f.plant.count_offset += 20; /* about 4.2 mm */
    fx_run_until_rest(&f, 30000);
    CHECK(f.lift.faults & LIFT_FAULT_F02_HALL_MISMATCH);
    CHECK(f.plant.pos_nm < MM(200)); /* stopped at the L1175 cross-check */
}

/* Open wire on the target stop sensor: Hall passes the stop, F02 after the
 * overrun limit, before the end switch. */
static void test_target_sensor_missing_overrun_f02(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    f.plant.sensor_stuck_off[1] = true;
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 1);
    fx_run_until_rest(&f, 30000);
    CHECK(f.lift.faults & LIFT_FAULT_F02_HALL_MISMATCH);
    int64_t over = f.plant.pos_nm - MM(180);
    CHECK(over > MM(2));
    CHECK(over <= MM(4));
}

/* Two stop sensors active: F09 (proposal), recovery required, clearing
 * refused while it persists (T15), then UNHOMED and not IDLE (T16, T30). */
static void test_two_sensors_f09_and_clearing(void)
{
    fx_t f;
    fx_init(&f, MM(180));
    f.plant.sensor_stuck_on[2] = true;
    fx_run(&f, 20);
    CHECK(f.lift.faults & LIFT_FAULT_F09_STOP_SENSORS);
    CHECK_EQ(f.lift.state, LIFT_STATE_FAULT);
    CHECK(fx_status(&f).flags & LIFT_FLAG_RECOVERY_REQUIRED);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_START_HOMING, 0), LIFT_RES_FAULT_ACTIVE);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_CLEAR_FAULT, 0), LIFT_RES_CLEAR_CONDITIONS);
    CHECK(f.lift.faults & LIFT_FAULT_F09_STOP_SENSORS);
    f.plant.sensor_stuck_on[2] = false;
    fx_run(&f, 20);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_CLEAR_FAULT, 0), LIFT_RES_ACCEPTED);
    CHECK_EQ(f.lift.state, LIFT_STATE_UNHOMED);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 0), LIFT_RES_RECOVERY_REQUIRED);
    /* At a sensor: homing references without motion. With continue-to-L1000
     * it then drives down. */
    CHECK_EQ(fx_cmd(&f, LIFT_OP_START_HOMING, 0), LIFT_RES_ACCEPTED);
    fx_run_until_rest(&f, 30000);
    CHECK_EQ(f.lift.state, LIFT_STATE_IDLE);
    CHECK_EQ(fx_status(&f).active_stop_id, 0);
}

static void test_two_sensors_at_boot_f09(void)
{
    fx_t f;
    memset(&f, 0, sizeof(f));
    fx_cfg(&f.cfg);
    f.now = 1000;
    plant_init(&f.plant, MM(5));
    f.plant.sensor_stuck_on[1] = true;
    lift_init(&f.lift, &f.cfg, f.now);
    fx_run(&f, 200);
    CHECK(f.lift.faults & LIFT_FAULT_F09_STOP_SENSORS);
    CHECK(!f.lift.position_valid);
}

/* cmd_result 11 and E-stop during a move (D4, D13): stop, keep the count, no
 * fault, no resume. */
static void test_motor_power(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    f.plant.motor_power = false;
    fx_run(&f, 2);
    CHECK(!(fx_status(&f).flags & LIFT_FLAG_MOTOR_POWER_PRESENT));
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2), LIFT_RES_MOTOR_POWER_ABSENT);
    f.plant.motor_power = true;
    fx_run(&f, 2);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2), LIFT_RES_ACCEPTED);
    fx_run(&f, 3000);
    f.plant.motor_power = false; /* E-stop */
    fx_run_until_rest(&f, 2000);
    CHECK_EQ(f.lift.faults, 0);
    CHECK_EQ(f.lift.state, LIFT_STATE_IDLE);
    CHECK(f.lift.position_valid);
    CHECK_EQ(fx_status(&f).active_stop_id, LIFT_STOP_NONE);
    CHECK(abs64(lift_position_nm(&f.lift) - f.plant.pos_nm) <= MM(1));
    f.plant.motor_power = true;
    f.ever_enabled = false;
    fx_run(&f, 1000);
    CHECK(!f.ever_enabled); /* no resume */
}

static void test_motor_power_absent_homing(void)
{
    fx_t f;
    fx_init(&f, MM(100));
    f.plant.motor_power = false;
    fx_run(&f, 2);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_START_HOMING, 0), LIFT_RES_MOTOR_POWER_ABSENT);
}

/* Hall supply loss clears position_valid (Alex 24 Oct); during motion F05. */
static void test_hall_supply_loss(void)
{
    fx_t f;
    fx_init(&f, MM(180));
    f.plant.hall_supply_ok = false;
    fx_run(&f, 2);
    CHECK(!f.lift.position_valid);
    CHECK_EQ(f.lift.faults, 0);
    CHECK_EQ(f.lift.state, LIFT_STATE_UNHOMED);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_START_HOMING, 0), LIFT_RES_HOMING_PRECONDITIONS);
    f.plant.hall_supply_ok = true;
    fx_run(&f, 2);
    CHECK_EQ(f.lift.state, LIFT_STATE_UNHOMED);

    fx_init(&f, MM(5));
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 1000);
    f.plant.hall_supply_ok = false;
    fx_run(&f, 2);
    CHECK(f.lift.faults & LIFT_FAULT_F05_POSITION_LOST);
    CHECK(!f.hb->enable);
}

/* F08: standstill not confirmed after the drive is switched off. */
static void test_standstill_timeout_f08(void)
{
    fx_t f;
    fx_init(&f, MM(5));
    f.plant.coast_um = 20000; /* 20 mm of coast at half speed: > 500 ms */
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run(&f, 2000);
    fx_cmd(&f, LIFT_OP_STOP, 0);
    CHECK_EQ(f.lift.state, LIFT_STATE_STOPPING);
    fx_run(&f, f.cfg.standstill_timeout_ms - 5);
    CHECK_EQ(f.lift.faults, 0);
    fx_run(&f, 10);
    CHECK(f.lift.faults & LIFT_FAULT_F08_STANDSTILL);
    CHECK(fx_status(&f).flags & LIFT_FLAG_MOVING);
}

/* T33: STOP during homing aborts it with recovery_required unchanged; STOP
 * with a fault active at standstill keeps FAULT. */
static void test_stop_during_homing_and_with_fault(void)
{
    fx_t f;
    fx_init(&f, MM(100));
    fx_cmd(&f, LIFT_OP_START_HOMING, 0);
    fx_run(&f, 1000);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_STOP, 0), LIFT_RES_ACCEPTED);
    CHECK(!f.hb->enable || f.lift.state == LIFT_STATE_STOPPING);
    fx_run_until_rest(&f, 2000);
    CHECK_EQ(f.lift.state, LIFT_STATE_UNHOMED);
    CHECK(fx_status(&f).flags & LIFT_FLAG_RECOVERY_REQUIRED);

    fx_init(&f, MM(180));
    f.plant.sensor_stuck_on[0] = true;
    fx_run(&f, 20);
    CHECK_EQ(f.lift.state, LIFT_STATE_FAULT);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_STOP, 0), LIFT_RES_ACCEPTED);
    fx_run(&f, 10);
    CHECK_EQ(f.lift.state, LIFT_STATE_FAULT);
}

/* T13, T14: end switches when wired. */
static void test_end_switches(void)
{
    fx_t f;
    lift_config_t c;
    fx_cfg(&c);
    c.end_switches_wired = true;
    fx_init_cfg(&f, MM(355), &c);
    f.force_upper = true;
    fx_run(&f, 20);
    lift_status_t st = fx_status(&f);
    CHECK(st.flags & LIFT_FLAG_UPPER_LIMIT);
    f.force_lower = true;
    fx_run(&f, 20);
    CHECK(f.lift.faults & LIFT_FAULT_F01_BOTH_LIMITS);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_CLEAR_FAULT, 0), LIFT_RES_CLEAR_CONDITIONS);
    f.force_lower = false;
    fx_run(&f, 20);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_CLEAR_FAULT, 0), LIFT_RES_ACCEPTED);
    f.keepalive = false;
    f.force_upper = false;

    /* Reaching the end switch in the direction of travel: F02. */
    fx_init_cfg(&f, MM(5), &c);
    f.plant.sensor_stuck_off[2] = true;
    f.cfg.sensor_overrun_um = 20000;
    f.lift.cfg.sensor_overrun_um = 20000;
    fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2);
    fx_run_until_rest(&f, 40000);
    CHECK(f.lift.faults & LIFT_FAULT_F02_HALL_MISMATCH);
    CHECK(f.plant.pos_nm >= MM(359));
}

static void test_limit_already_active_rejects_direction(void)
{
    fx_t f;
    lift_config_t c;
    fx_cfg(&c);
    c.end_switches_wired = true;
    fx_init_cfg(&f, MM(180), &c);
    f.force_upper = true;
    fx_run(&f, 20);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_MOVE_TO_STOP, 2), LIFT_RES_LIMIT_ACTIVE);
    f.keepalive = false;
    fx_run(&f, 50);
    CHECK(!f.ever_enabled);
}

static void test_clear_without_fault_is_noop(void)
{
    fx_t f;
    fx_init(&f, MM(180));
    CHECK_EQ(fx_cmd(&f, LIFT_OP_CLEAR_FAULT, 0), LIFT_RES_ACCEPTED);
    f.keepalive = false;
    fx_run(&f, 50);
    CHECK(!f.ever_enabled);
    CHECK_EQ(f.lift.state, LIFT_STATE_IDLE);
}

/* Rejected commands during start-up. */
static void test_commands_during_init(void)
{
    fx_t f;
    memset(&f, 0, sizeof(f));
    fx_cfg(&f.cfg);
    plant_init(&f.plant, MM(180));
    lift_init(&f.lift, &f.cfg, 0);
    CHECK_EQ(f.lift.state, LIFT_STATE_INIT);
    CHECK_EQ(lift_command(&f.lift, 1, LIFT_OP_MOVE_TO_STOP, 2, 0), LIFT_RES_BUSY);
    CHECK_EQ(lift_command(&f.lift, 2, LIFT_OP_STOP, 0, 0), LIFT_RES_ACCEPTED);
}

static void test_brake_mode(void)
{
    fx_t f;
    lift_config_t c;
    fx_cfg(&c);
    c.stop_mode_brake = true;
    fx_init_cfg(&f, MM(180), &c);
    CHECK(f.hb->enable && f.hb->brake);
}

/* T29: homing interrupted by a new fault completes nothing:
 * recovery_required stays set and the state is FAULT, then UNHOMED. */
static void test_homing_interrupted_by_fault(void)
{
    fx_t f;
    fx_init(&f, MM(100));
    CHECK_EQ(fx_cmd(&f, LIFT_OP_START_HOMING, 0), LIFT_RES_ACCEPTED);
    fx_run(&f, 1000);
    f.plant.jammed = true;
    fx_run(&f, f.cfg.stall_ms + 5);
    CHECK(f.lift.faults & LIFT_FAULT_F03_OVERCURRENT);
    CHECK_EQ(f.lift.state, LIFT_STATE_FAULT);
    CHECK(fx_status(&f).flags & LIFT_FLAG_RECOVERY_REQUIRED);
    f.plant.jammed = false;
    f.keepalive = false;
    fx_run(&f, 300);
    CHECK_EQ(fx_cmd(&f, LIFT_OP_CLEAR_FAULT, 0), LIFT_RES_ACCEPTED);
    f.keepalive = false;
    CHECK_EQ(f.lift.state, LIFT_STATE_UNHOMED);
}

int main(void)
{
    RUN(test_config_defaults_valid);
    RUN(test_boot_at_sensor_is_valid);
    RUN(test_boot_between_stops_is_unhomed);
    RUN(test_homing_from_between_stops_ends_at_L1000);
    RUN(test_homing_from_above_references_at_first_sensor);
    RUN(test_homing_without_continue_stops_at_first_sensor);
    RUN(test_move_up_full_travel);
    RUN(test_move_down_one_stop);
    RUN(test_move_to_current_stop_is_noop);
    RUN(test_keepalive_loss_raises_f07);
    RUN(test_core_heartbeat_loss_raises_f07);
    RUN(test_seq_semantics);
    RUN(test_malformed_and_invalid_stop);
    RUN(test_busy_while_moving);
    RUN(test_travel_timeout_f04);
    RUN(test_hall_window_high_rate_f02);
    RUN(test_hall_window_low_rate_f02);
    RUN(test_hall_window_nominal_no_fault_during_grace);
    RUN(test_hall_frozen_stall_f03);
    RUN(test_jammed_column_f03);
    RUN(test_wrong_direction_f02);
    RUN(test_overcurrent_f03_and_inrush_blank);
    RUN(test_stop_sensor_priority_corrects_drift);
    RUN(test_hall_drift_beyond_window_f02);
    RUN(test_target_sensor_missing_overrun_f02);
    RUN(test_two_sensors_f09_and_clearing);
    RUN(test_two_sensors_at_boot_f09);
    RUN(test_motor_power);
    RUN(test_motor_power_absent_homing);
    RUN(test_hall_supply_loss);
    RUN(test_standstill_timeout_f08);
    RUN(test_stop_during_homing_and_with_fault);
    RUN(test_homing_interrupted_by_fault);
    RUN(test_end_switches);
    RUN(test_limit_already_active_rejects_direction);
    RUN(test_clear_without_fault_is_noop);
    RUN(test_commands_during_init);
    RUN(test_brake_mode);
    TEST_MAIN_END();
}
