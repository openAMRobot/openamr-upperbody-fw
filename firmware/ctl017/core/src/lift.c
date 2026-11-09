/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 lift node core. See lift.h and docs/ctl017_lift_core.md.
 */
#include "ctl017/lift.h"

#include <string.h>

#define VELOCITY_WINDOW_MS 100u

static int32_t hall_sign(const lift_t *l)
{
    return l->cfg.hall_count_up_positive ? 1 : -1;
}

int64_t lift_position_nm(const lift_t *l)
{
    int64_t dc = (int64_t)hall_sign(l) * ((int64_t)l->last_count - (int64_t)l->ref_count);
    return l->ref_pos_nm + dc * (int64_t)l->cfg.hall_nm_per_count;
}

static int64_t stop_nm(const lift_t *l, unsigned k)
{
    return (int64_t)l->cfg.stop_pos_um[k] * 1000;
}

static void set_reference(lift_t *l, int64_t pos_nm)
{
    l->ref_count = l->last_count;
    l->ref_pos_nm = pos_nm;
}

static bool standstill(const lift_t *l)
{
    return (uint32_t)(l->now_ms - l->last_count_change_ms) >= l->cfg.standstill_quiet_ms;
}

static bool both_limits(const lift_t *l)
{
    return l->lower_db.state && l->upper_db.state;
}

static unsigned active_sensor_count(const lift_t *l, unsigned *which)
{
    unsigned n = 0;
    for (unsigned k = 0; k < LIFT_NUM_STOPS; k++) {
        if (l->stop_db[k].state) {
            n++;
            if (which != NULL) {
                *which = k;
            }
        }
    }
    return n;
}

static void apply_hbridge(lift_t *l)
{
    memset(&l->hb, 0, sizeof(l->hb));
    if (l->drive_on) {
        l->hb.enable = true;
        l->hb.dir = l->dir;
        l->hb.duty_permille = (l->state == LIFT_STATE_HOMING) ? l->cfg.duty_homing_permille
                                                              : l->cfg.duty_move_permille;
    } else if (l->cfg.stop_mode_brake) {
        l->hb.enable = true;
        l->hb.brake = true;
    }
}

static void drive_start(lift_t *l, lift_dir_t dir, uint32_t timeout_ms)
{
    l->dir = dir;
    l->drive_on = true;
    l->arriving = false;
    l->drive_on_ms = l->now_ms;
    l->move_deadline_ms = l->now_ms + timeout_ms;
    l->win_start_ms = l->now_ms;
    l->win_start_count = l->last_count;
    l->last_pulse_ms = l->now_ms;
    l->dir_ref_count = l->last_count;
    l->over_active = false;
    l->f08_armed = false;
}

static void drive_stop(lift_t *l)
{
    if (l->drive_on) {
        l->drive_on = false;
        l->drive_off_ms = l->now_ms;
        l->f08_armed = true;
    }
}

static void discard_target(lift_t *l)
{
    l->target_stop = LIFT_STOP_NONE;
    l->arriving = false;
}

/* State when no motion is in progress (idle readiness, S2 section 5.3). */
static lift_state_t rest_state(const lift_t *l)
{
    if (l->faults != 0u) {
        return LIFT_STATE_FAULT;
    }
    if (l->position_valid && l->homed && !both_limits(l)) {
        return LIFT_STATE_IDLE;
    }
    return LIFT_STATE_UNHOMED;
}

static void raise_fault(lift_t *l, uint16_t bits)
{
    l->new_faults |= (uint16_t)(bits & (uint16_t)~l->faults);
    l->faults |= bits;
    if ((bits & LIFT_FAULTS_INVALIDATE_POSITION) != 0u) {
        l->position_valid = false;
        l->homed = false;
    }
    drive_stop(l);
    discard_target(l);
    l->state = LIFT_STATE_FAULT;
}

static uint32_t travel_timeout_ms(const lift_t *l, int64_t distance_nm, int32_t speed_um_s)
{
    if (distance_nm < 0) {
        distance_nm = -distance_nm;
    }
    /* nm / (um/s) = ms */
    int64_t t = distance_nm / speed_um_s;
    t = t * l->cfg.travel_timeout_factor_pct / 100 + l->cfg.travel_timeout_margin_ms;
    if (t > 0x7FFFFFFF) {
        t = 0x7FFFFFFF;
    }
    return (uint32_t)t;
}

static void set_target(lift_t *l, unsigned k)
{
    l->target_stop = (uint8_t)k;
    l->target_nm = stop_nm(l, k);
}

/* ---------------------------------------------------------------------------
 * Start-up
 */
void lift_init(lift_t *l, const lift_config_t *cfg, uint32_t now_ms)
{
    memset(l, 0, sizeof(*l));
    l->cfg = *cfg;
    l->state = LIFT_STATE_INIT;
    l->target_stop = LIFT_STOP_NONE;
    l->cmd_result = LIFT_RES_NONE;
    l->last_seq = 0;
    l->init_ms = now_ms;
    l->now_ms = now_ms;
    for (unsigned k = 0; k < LIFT_NUM_STOPS; k++) {
        debounce_init(&l->stop_db[k], false, cfg->sensor_debounce_ms, now_ms);
    }
    debounce_init(&l->lower_db, false, cfg->sensor_debounce_ms, now_ms);
    debounce_init(&l->upper_db, false, cfg->sensor_debounce_ms, now_ms);
    apply_hbridge(l);
}

static void finish_init(lift_t *l)
{
    unsigned k = 0;
    unsigned n = active_sensor_count(l, &k);
    if (both_limits(l)) {
        raise_fault(l, LIFT_FAULT_F01_BOTH_LIMITS);
    }
    if (n > 1u) {
        raise_fault(l, LIFT_FAULT_F09_STOP_SENSORS);
    } else if (n == 1u && l->hall_supply_ok && l->faults == 0u) {
        /* P-03 item 24: a sensor active at power-up gives a valid position
         * without homing. */
        set_reference(l, stop_nm(l, k));
        l->homed = true;
        l->position_valid = true;
    }
    l->state = rest_state(l);
}

/* ---------------------------------------------------------------------------
 * Commands
 */
static bool motion_state(lift_state_t s)
{
    return s == LIFT_STATE_MOVING || s == LIFT_STATE_HOMING || s == LIFT_STATE_STOPPING;
}

bool lift_motion_in_progress(const lift_t *l)
{
    return l->drive_on || motion_state(l->state) || !standstill(l);
}

static int active_stop(const lift_t *l)
{
    if (!l->position_valid || l->drive_on || !standstill(l)) {
        return -1;
    }
    int64_t pos = lift_position_nm(l);
    int64_t tol = (int64_t)l->cfg.stop_tol_um * 1000;
    for (unsigned k = 0; k < LIFT_NUM_STOPS; k++) {
        int64_t d = pos - stop_nm(l, k);
        if (l->stop_db[k].state && d <= tol && d >= -tol) {
            return (int)k;
        }
    }
    return -1;
}

static uint8_t cmd_stop(lift_t *l)
{
    if (l->drive_on || l->state == LIFT_STATE_MOVING || l->state == LIFT_STATE_HOMING) {
        drive_stop(l);
        discard_target(l);
        if (l->state != LIFT_STATE_FAULT) {
            l->state = LIFT_STATE_STOPPING;
        }
    } else {
        discard_target(l);
    }
    return LIFT_RES_ACCEPTED;
}

static uint8_t cmd_move(lift_t *l, uint8_t stop_id)
{
    if (stop_id >= LIFT_NUM_STOPS) {
        return LIFT_RES_INVALID_STOP;
    }
    if (l->faults != 0u) {
        return LIFT_RES_FAULT_ACTIVE;
    }
    if (l->state == LIFT_STATE_INIT || lift_motion_in_progress(l)) {
        return LIFT_RES_BUSY;
    }
    if (!l->homed) {
        return LIFT_RES_RECOVERY_REQUIRED;
    }
    if (!l->position_valid) {
        return LIFT_RES_POSITION_INVALID;
    }
    if (active_stop(l) == (int)stop_id) {
        return LIFT_RES_ACCEPTED; /* already there: no-op (S2 6.6) */
    }
    if (!l->motor_power) {
        return LIFT_RES_MOTOR_POWER_ABSENT;
    }
    int64_t diff = stop_nm(l, stop_id) - lift_position_nm(l);
    lift_dir_t dir = (diff >= 0) ? LIFT_DIR_UP : LIFT_DIR_DOWN;
    if ((dir == LIFT_DIR_UP && l->upper_db.state) || (dir == LIFT_DIR_DOWN && l->lower_db.state)) {
        return LIFT_RES_LIMIT_ACTIVE;
    }
    set_target(l, stop_id);
    l->state = LIFT_STATE_MOVING;
    drive_start(l, dir, travel_timeout_ms(l, diff, l->cfg.move_speed_um_s));
    return LIFT_RES_ACCEPTED;
}

static uint8_t cmd_clear(lift_t *l)
{
    if (lift_motion_in_progress(l)) {
        return LIFT_RES_BUSY;
    }
    if (l->faults == 0u) {
        return LIFT_RES_ACCEPTED; /* D9: no-op */
    }
    uint16_t keep = 0;
    if (both_limits(l)) {
        keep |= LIFT_FAULT_F01_BOTH_LIMITS;
    }
    if (active_sensor_count(l, NULL) > 1u) {
        keep |= LIFT_FAULT_F09_STOP_SENSORS;
    }
    if (l->current_ma > l->cfg.overcurrent_ma) {
        keep |= LIFT_FAULT_F03_OVERCURRENT;
    }
    if (!l->hall_supply_ok) {
        keep |= LIFT_FAULT_F05_POSITION_LOST;
    }
    /* CLEAR_FAULT clears every clearable fault (Alex 24 Oct); it never moves
     * the column and never clears recovery_required. */
    l->faults &= keep;
    l->state = rest_state(l);
    return (l->faults != 0u) ? LIFT_RES_CLEAR_CONDITIONS : LIFT_RES_ACCEPTED;
}

static uint8_t cmd_home(lift_t *l)
{
    if (l->faults != 0u) {
        return LIFT_RES_FAULT_ACTIVE;
    }
    if (l->state == LIFT_STATE_INIT || lift_motion_in_progress(l)) {
        return LIFT_RES_BUSY;
    }
    if (both_limits(l) || !l->hall_supply_ok || active_sensor_count(l, NULL) > 1u) {
        return LIFT_RES_HOMING_PRECONDITIONS;
    }
    unsigned k = 0;
    if (active_sensor_count(l, &k) == 1u) {
        /* Already at a sensor: reference without motion. */
        if (!l->homed) {
            set_reference(l, stop_nm(l, k));
            l->homed = true;
            l->position_valid = true;
        }
        if (k == 0u || !l->cfg.homing_continue_to_stop0) {
            l->state = rest_state(l);
            return LIFT_RES_ACCEPTED;
        }
    }
    if (!l->motor_power) {
        return LIFT_RES_MOTOR_POWER_ABSENT;
    }
    if (l->lower_db.state) {
        return LIFT_RES_LIMIT_ACTIVE;
    }
    /* Distance bound: the whole travel between the outer stops plus one
     * stop tolerance on each side. */
    int64_t span = stop_nm(l, LIFT_NUM_STOPS - 1u) - stop_nm(l, 0) +
                   2 * (int64_t)l->cfg.stop_tol_um * 1000;
    /* Target: stop 0 if homing continues there and the position is already
     * known; otherwise set at the first stop sensor reached. */
    if (l->homed && l->cfg.homing_continue_to_stop0) {
        set_target(l, 0u);
    } else {
        discard_target(l);
    }
    l->state = LIFT_STATE_HOMING;
    drive_start(l, LIFT_DIR_DOWN, travel_timeout_ms(l, span, l->cfg.homing_speed_um_s));
    return LIFT_RES_ACCEPTED;
}

uint8_t lift_command(lift_t *l, uint8_t seq, uint8_t op, uint8_t stop_id, uint32_t now_ms)
{
    l->now_ms = now_ms;
    if (l->any_cmd && seq == l->last_seq) {
        if (op == l->last_op && stop_id == l->last_stop_id) {
            /* Keep-alive: refresh the deadline, do not execute again. */
            l->last_keepalive_ms = now_ms;
            return l->cmd_result;
        }
        /* Same seq, different content: rejected, no refresh (S2 6.6). */
        l->cmd_result = LIFT_RES_SEQ_CONFLICT;
        return l->cmd_result;
    }
    l->any_cmd = true;
    l->last_seq = seq;
    l->last_op = op;
    l->last_stop_id = stop_id;

    uint8_t res;
    switch (op) {
    case LIFT_OP_STOP:
        res = cmd_stop(l);
        break;
    case LIFT_OP_MOVE_TO_STOP:
        res = cmd_move(l, stop_id);
        break;
    case LIFT_OP_CLEAR_FAULT:
        res = cmd_clear(l);
        break;
    case LIFT_OP_START_HOMING:
        res = cmd_home(l);
        break;
    default:
        res = LIFT_RES_MALFORMED;
        break;
    }
    if (res == LIFT_RES_ACCEPTED) {
        l->last_keepalive_ms = now_ms;
    }
    l->cmd_result = res;
    apply_hbridge(l);
    return res;
}

void lift_set_supervision_lost(lift_t *l, bool lost)
{
    l->supervision_lost = lost;
}

/* ---------------------------------------------------------------------------
 * Control tick
 */
static void sample_inputs(lift_t *l, const lift_inputs_t *in)
{
    for (unsigned k = 0; k < LIFT_NUM_STOPS; k++) {
        l->stop_prev[k] = l->stop_db[k].state;
        debounce_update(&l->stop_db[k], in->stop_sensor[k], in->now_ms);
    }
    bool wired = l->cfg.end_switches_wired;
    debounce_update(&l->lower_db, wired && in->lower_limit, in->now_ms);
    debounce_update(&l->upper_db, wired && in->upper_limit, in->now_ms);
    l->motor_power = in->motor_power_present;
    l->hall_supply_ok = in->hall_supply_ok;
    l->current_ma = in->motor_current_ma;

    if (in->hall_count != l->last_count) {
        l->last_count = in->hall_count;
        l->last_count_change_ms = in->now_ms;
        l->last_pulse_ms = in->now_ms;
    }
}

static void update_velocity(lift_t *l)
{
    uint32_t dt = l->now_ms - l->vel_t0_ms;
    if (dt < VELOCITY_WINDOW_MS) {
        if (!l->drive_on && standstill(l)) {
            l->velocity_01mm_s = 0;
        }
        return;
    }
    int64_t dc = (int64_t)hall_sign(l) * ((int64_t)l->last_count - (int64_t)l->vel_c0);
    /* nm per ms = um/s; 0.1 mm/s = 100 um/s */
    int64_t v = dc * l->cfg.hall_nm_per_count / (int64_t)dt / 100;
    if (v > 32767) {
        v = 32767;
    } else if (v < -32767) {
        v = -32767;
    }
    l->velocity_01mm_s = (int16_t)v;
    l->vel_t0_ms = l->now_ms;
    l->vel_c0 = l->last_count;
}

static void check_global(lift_t *l)
{
    if (both_limits(l) && (l->faults & LIFT_FAULT_F01_BOTH_LIMITS) == 0u) {
        raise_fault(l, LIFT_FAULT_F01_BOTH_LIMITS);
    }
    if (active_sensor_count(l, NULL) > 1u && (l->faults & LIFT_FAULT_F09_STOP_SENSORS) == 0u) {
        raise_fault(l, LIFT_FAULT_F09_STOP_SENSORS);
    }
    if (!l->hall_supply_ok && (l->homed || l->position_valid)) {
        /* Alex 24 Oct: Hall supply loss clears position_valid. During motion
         * the count is lost as well: F05. */
        bool moving = l->drive_on || !standstill(l);
        l->homed = false;
        l->position_valid = false;
        if (moving) {
            raise_fault(l, LIFT_FAULT_F05_POSITION_LOST);
        } else if (!motion_state(l->state)) {
            l->state = rest_state(l);
        }
    }
    if (l->f08_armed) {
        if (standstill(l)) {
            l->f08_armed = false;
        } else if ((uint32_t)(l->now_ms - l->drive_off_ms) >= l->cfg.standstill_timeout_ms) {
            l->f08_armed = false;
            raise_fault(l, LIFT_FAULT_F08_STANDSTILL);
        }
    }
}

static void check_drive(lift_t *l)
{
    const lift_config_t *c = &l->cfg;
    uint32_t on_for = l->now_ms - l->drive_on_ms;

    /* Command supervision: RPDO deadline and core heartbeat (D12 alt. a). */
    if (l->supervision_lost || (uint32_t)(l->now_ms - l->last_keepalive_ms) >= c->rpdo_deadline_ms) {
        raise_fault(l, LIFT_FAULT_F07_SUPERVISION);
        return;
    }

    /* Motor power removed by the hardwired chain: stop driving, keep
     * counting, no fault and no resume (decision D4, D13). */
    if (!l->motor_power) {
        drive_stop(l);
        discard_target(l);
        l->state = LIFT_STATE_STOPPING;
        return;
    }

    /* Current limit on the shared current sense. */
    if (on_for >= c->inrush_blank_ms && l->current_ma > c->overcurrent_ma) {
        if (!l->over_active) {
            l->over_active = true;
            l->over_since_ms = l->now_ms;
        } else if ((uint32_t)(l->now_ms - l->over_since_ms) >= c->overcurrent_ms) {
            raise_fault(l, LIFT_FAULT_F03_OVERCURRENT);
            return;
        }
    } else {
        l->over_active = false;
    }

    /* End switch in the direction of travel: the stops sit inside the
     * switches, so reaching one means the count or a stop sensor is wrong. */
    if ((l->dir == LIFT_DIR_UP && l->upper_db.state) || (l->dir == LIFT_DIR_DOWN && l->lower_db.state)) {
        raise_fault(l, LIFT_FAULT_F02_HALL_MISMATCH);
        return;
    }

    /* Travel timeout per move. */
    if (time_reached(l->now_ms, l->move_deadline_ms)) {
        raise_fault(l, LIFT_FAULT_F04_MOVE_TIMEOUT);
        return;
    }

    /* Direction check: count running against the commanded direction. */
    int32_t progress = hall_sign(l) * (l->last_count - l->dir_ref_count) * (int32_t)l->dir;
    if (progress > 0) {
        l->dir_ref_count = l->last_count;
    } else if (-progress > (int32_t)c->wrong_dir_counts) {
        raise_fault(l, LIFT_FAULT_F02_HALL_MISMATCH);
        return;
    }

    /* Hall-window check (P-03 item 30 A7). */
    if (on_for < c->hall_start_grace_ms) {
        l->win_start_ms = l->now_ms;
        l->win_start_count = l->last_count;
    } else {
        if ((uint32_t)(l->now_ms - l->last_pulse_ms) >= c->stall_ms) {
            raise_fault(l, LIFT_FAULT_F03_OVERCURRENT); /* stall: no pulses while driving */
            return;
        }
        uint32_t w = l->now_ms - l->win_start_ms;
        if (w >= c->hall_window_ms) {
            int32_t dc = l->last_count - l->win_start_count;
            if (dc < 0) {
                dc = -dc;
            }
            uint32_t cps = (uint32_t)((uint64_t)(uint32_t)dc * 1000u / w);
            bool homing = (l->state == LIFT_STATE_HOMING);
            uint32_t lo = homing ? c->homing_rate_min_cps : c->move_rate_min_cps;
            uint32_t hi = homing ? c->homing_rate_max_cps : c->move_rate_max_cps;
            /* A rate of zero, or pulses that stopped during the window, is
             * left to the stall check so that a stall reports F03. */
            bool pulses_recent = (uint32_t)(l->now_ms - l->last_pulse_ms) < c->hall_window_ms / 2u;
            if (cps > hi || (cps != 0u && pulses_recent && cps < lo)) {
                raise_fault(l, LIFT_FAULT_F02_HALL_MISMATCH);
                return;
            }
            l->win_start_ms = l->now_ms;
            l->win_start_count = l->last_count;
        }
    }
}

/* Stop sensor edges while driving: cross-check the Hall count (F02) and,
 * during homing, set the first reference. */
static void handle_sensor_edges(lift_t *l)
{
    for (unsigned k = 0; k < LIFT_NUM_STOPS && l->drive_on; k++) {
        if (!l->stop_db[k].state || l->stop_prev[k]) {
            continue;
        }
        int32_t edge_um = (l->dir == LIFT_DIR_UP) ? l->cfg.sensor_edge_offset_up_um
                                                  : l->cfg.sensor_edge_offset_down_um;
        int64_t expect = stop_nm(l, k) + (int64_t)edge_um * 1000;
        if (!l->homed) {
            if (l->state != LIFT_STATE_HOMING || l->dir != LIFT_DIR_DOWN) {
                continue;
            }
            set_reference(l, expect);
            l->homed = true;
            l->position_valid = true;
            set_target(l, l->cfg.homing_continue_to_stop0 ? 0u : k);
            continue;
        }
        int64_t err = lift_position_nm(l) - expect;
        int64_t tol = (int64_t)l->cfg.xcheck_tol_um * 1000;
        if (err > tol || err < -tol) {
            raise_fault(l, LIFT_FAULT_F02_HALL_MISMATCH);
            return;
        }
        if (l->cfg.rereference_on_sensor) {
            set_reference(l, expect);
        }
        if (l->state == LIFT_STATE_HOMING && l->target_stop == LIFT_STOP_NONE) {
            set_target(l, l->cfg.homing_continue_to_stop0 ? 0u : k);
        }
    }
}

/* Target logic: the target stop sensor must be active and the Hall position
 * must have reached the stop (minus the coast lead). Hall position past the
 * stop without the sensor raises F02. */
static void handle_target(lift_t *l)
{
    if (!l->drive_on || l->target_stop == LIFT_STOP_NONE || !l->position_valid) {
        return;
    }
    int64_t remaining = (l->target_nm - lift_position_nm(l)) * (int64_t)l->dir;
    int64_t lead = (int64_t)l->cfg.stop_lead_um * 1000;
    int64_t overrun = (int64_t)l->cfg.sensor_overrun_um * 1000;
    if (l->stop_db[l->target_stop].state && remaining <= lead) {
        drive_stop(l);
        l->arriving = true;
        l->state = LIFT_STATE_STOPPING;
    } else if (remaining < -overrun) {
        raise_fault(l, LIFT_FAULT_F02_HALL_MISMATCH);
    }
}

const lift_hbridge_cmd_t *lift_step(lift_t *l, const lift_inputs_t *in)
{
    l->now_ms = in->now_ms;
    if (!l->started) {
        l->started = true;
        l->last_count = in->hall_count;
        l->ref_count = in->hall_count;
        l->vel_c0 = in->hall_count;
        l->vel_t0_ms = in->now_ms;
        l->last_count_change_ms = in->now_ms;
    }
    sample_inputs(l, in);
    update_velocity(l);

    if (l->state == LIFT_STATE_INIT) {
        if ((uint32_t)(l->now_ms - l->init_ms) >= l->cfg.init_settle_ms) {
            finish_init(l);
        }
        apply_hbridge(l);
        return &l->hb;
    }

    check_global(l);
    if (l->drive_on) {
        check_drive(l);
    }
    if (l->drive_on) {
        handle_sensor_edges(l);
    }
    if (l->drive_on) {
        handle_target(l);
    }

    if (l->state == LIFT_STATE_STOPPING && !l->drive_on && standstill(l)) {
        discard_target(l);
        l->state = rest_state(l);
    } else if (!motion_state(l->state) && l->state != LIFT_STATE_FAULT) {
        l->state = rest_state(l);
    }

    apply_hbridge(l);
    return &l->hb;
}

/* ---------------------------------------------------------------------------
 * Status
 */
void lift_get_status(const lift_t *l, lift_status_t *st)
{
    memset(st, 0, sizeof(*st));
    st->state = (uint8_t)l->state;
    uint8_t f = 0;
    int as = active_stop(l);
    if (l->position_valid) {
        f |= LIFT_FLAG_POSITION_VALID;
    }
    if (as >= 0) {
        f |= LIFT_FLAG_AT_STOP;
    }
    if (l->drive_on || !standstill(l)) {
        f |= LIFT_FLAG_MOVING;
    }
    if (l->lower_db.state) {
        f |= LIFT_FLAG_LOWER_LIMIT;
    }
    if (l->upper_db.state) {
        f |= LIFT_FLAG_UPPER_LIMIT;
    }
    if (l->faults != 0u) {
        f |= LIFT_FLAG_FAULT_PRESENT;
    }
    if (l->motor_power) {
        f |= LIFT_FLAG_MOTOR_POWER_PRESENT;
    }
    if (!l->homed) {
        f |= LIFT_FLAG_RECOVERY_REQUIRED;
    }
    st->flags = f;

    if (l->position_valid) {
        int64_t p = lift_position_nm(l) / 100000; /* 0.1 mm */
        if (p < 0) {
            p = 0;
        } else if (p > 0xFFFE) {
            p = 0xFFFE;
        }
        st->position_01mm = (uint16_t)p;
    } else {
        st->position_01mm = LIFT_POSITION_UNKNOWN;
    }
    st->active_stop_id = (as >= 0) ? (uint8_t)as : LIFT_STOP_NONE;
    st->last_seq = l->last_seq;
    st->cmd_result = l->cmd_result;
    st->target_stop_id = l->target_stop;
    for (unsigned k = 0; k < LIFT_NUM_STOPS; k++) {
        if (l->stop_db[k].state) {
            st->stop_sensors |= (uint8_t)(1u << k);
        }
    }
    st->motor_current_ma = l->current_ma;
    st->velocity_01mm_s = l->velocity_01mm_s;
    st->faults = l->faults;
    st->hall_supply_ok = l->hall_supply_ok;
}

uint16_t lift_take_new_faults(lift_t *l)
{
    uint16_t f = l->new_faults;
    l->new_faults = 0;
    return f;
}
