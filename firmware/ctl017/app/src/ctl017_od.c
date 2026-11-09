/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 object dictionary. This table is the single source for the EDS
 * (docs/ctl017.eds) and the tables in docs/ctl017_object_dictionary.md;
 * tools/ctl017_od/gen_od_docs.py regenerates both from the dump of this
 * table and a ctest fails if they drift apart.
 */
#include <string.h>

#include "ctl017/ctl017_app.h"

static ctl017_app_t *app_of(co_node_t *co)
{
    return (ctl017_app_t *)co->ctx;
}

/* Lift parameters (0x2100, 0x2101, 0x1400:05): only in PRE-OPERATIONAL,
 * never during motion, and only if the result is a valid configuration. */
static uint32_t write_lift_param(co_node_t *co, const co_od_entry_t *e, uint32_t v)
{
    ctl017_app_t *a = app_of(co);
    if (lift_motion_in_progress(&a->lift)) {
        return CO_SDO_ABORT_DEVICE_STATE;
    }
    uint32_t old = co_od_get(e);
    co_od_set(e, v);
    if (!lift_config_valid(&a->lift.cfg)) {
        co_od_set(e, old);
        return CO_SDO_ABORT_VALUE_RANGE;
    }
    lift_config_changed(&a->lift);
    return 0;
}

static uint32_t write_aux_cmd(co_node_t *co, const co_od_entry_t *e, uint32_t v)
{
    ctl017_app_t *a = app_of(co);
    if (e->sub == 2u && v > 100u) {
        return CO_SDO_ABORT_VALUE_RANGE;
    }
    if (e->sub != 2u && v > 1u) {
        return CO_SDO_ABORT_VALUE_RANGE;
    }
    co_od_set(e, v);
    aux_command(&a->aux, a->auxcmd.head_light_on != 0u, a->auxcmd.head_light_dim_pct,
                a->auxcmd.getac_enable != 0u);
    return 0;
}

static uint32_t write_hb_producer(co_node_t *co, const co_od_entry_t *e, uint32_t v)
{
    (void)co;
    co_od_set(e, v);
    return 0;
}

typedef struct {
    ctl017_app_t *a;
    bool ok;
} od_builder_t;

static void add(od_builder_t *b, uint16_t idx, uint8_t sub, uint8_t type, uint8_t access, bool map,
                void *data, const char *name, co_write_fn wr)
{
    ctl017_app_t *a = b->a;
    if (a->od_len >= CTL017_OD_MAX) {
        b->ok = false;
        return;
    }
    co_od_entry_t *e = &a->od[a->od_len++];
    e->index = idx;
    e->sub = sub;
    e->type = type;
    e->access = access;
    e->pdo_mappable = map;
    e->data = data;
    e->name = name;
    e->on_write = wr;
}

#define U8 CO_T_UNSIGNED8
#define U16 CO_T_UNSIGNED16
#define U32 CO_T_UNSIGNED32
#define I16 CO_T_INT16
#define I32 CO_T_INT32
#define BOOL CO_T_BOOLEAN

#define MAPENT(idx, sub, bits) (((uint32_t)(idx) << 16) | ((uint32_t)(sub) << 8) | (uint32_t)(bits))

void ctl017_od_build(ctl017_app_t *a)
{
    od_builder_t b = {a, true};
    co_node_t *co = &a->co;
    lift_config_t *c = &a->lift.cfg;
    const uint8_t id = CTL017_NODE_ID;

    a->od_len = 0;
    memset(a->od, 0, sizeof(a->od));

    /* Communication values */
    a->device_type = 0x00000000u;                 /* no CiA device profile (D1) */
    a->cob_emcy = 0x080u + id;
    a->n_sub_1003 = CO_ERR_HISTORY;
    a->n_sub_1016 = 1;
    a->n_sub_1018 = 4;
    a->vendor_id = 0x00000000u;                   /* TODO(Evans): no CiA vendor ID assigned */
    a->product_code = 0x00000011u;                /* proposal: 17 = CTL-017 */
    a->revision = 0x00010000u;                    /* v1.0 */
    a->n_sub_rpdo_comm = 5;
    a->n_sub_tpdo_comm = 5;
    a->rpdo_cob[0] = 0x200u + id;
    a->rpdo_cob[1] = 0x300u + id;
    a->tpdo_cob[0] = 0x40000000u | (0x180u + id); /* bit 30: no RTR */
    a->tpdo_cob[1] = 0x40000000u | (0x280u + id);
    a->trans_type = 0xFE;                         /* event driven */
    a->rpdo2_event_ms = 0;                        /* no deadline on aux outputs */
    a->tpdo_inhibit[0] = 100;                     /* 10 ms, TODO(Evans) */
    a->tpdo_inhibit[1] = 100;                     /* 10 ms, TODO(Evans) */
    a->tpdo_event[0] = 100;                       /* 100 ms (S2 section 5) */
    a->tpdo_event[1] = 100;                       /* TODO(Evans) */
    a->n_map_rpdo[0] = 3;
    a->n_map_rpdo[1] = 3;
    a->n_map_tpdo[0] = 7;
    a->n_map_tpdo[1] = 6;
    a->map_rpdo1[0] = MAPENT(0x2010, 1, 8);
    a->map_rpdo1[1] = MAPENT(0x2010, 2, 8);
    a->map_rpdo1[2] = MAPENT(0x2010, 3, 8);
    a->map_rpdo2[0] = MAPENT(0x2011, 1, 8);
    a->map_rpdo2[1] = MAPENT(0x2011, 2, 8);
    a->map_rpdo2[2] = MAPENT(0x2011, 3, 8);
    a->map_tpdo1[0] = MAPENT(0x2000, 1, 8);
    a->map_tpdo1[1] = MAPENT(0x2000, 2, 8);
    a->map_tpdo1[2] = MAPENT(0x2000, 3, 16);
    a->map_tpdo1[3] = MAPENT(0x2000, 4, 8);
    a->map_tpdo1[4] = MAPENT(0x2000, 5, 8);
    a->map_tpdo1[5] = MAPENT(0x2000, 6, 8);
    a->map_tpdo1[6] = MAPENT(0x2000, 7, 8);
    a->map_tpdo2[0] = MAPENT(0x2001, 1, 8);
    a->map_tpdo2[1] = MAPENT(0x2001, 2, 8);
    a->map_tpdo2[2] = MAPENT(0x2001, 3, 16);
    a->map_tpdo2[3] = MAPENT(0x2001, 4, 16);
    a->map_tpdo2[4] = MAPENT(0x2001, 5, 8);
    a->map_tpdo2[5] = MAPENT(0x2001, 6, 8);
    a->n_sub_2000 = 7;
    a->n_sub_2001 = 6;
    a->n_sub_2010 = 3;
    a->n_sub_2011 = 3;
    a->n_sub_2100 = LIFT_NUM_STOPS;
    a->n_sub_2101 = 32;
    a->n_sub_2300 = 3;

    /* ---- CiA 301 communication area ---- */
    add(&b, 0x1000, 0, U32, CO_A_CONST, false, &a->device_type, "Device type", NULL);
    add(&b, 0x1001, 0, U8, CO_A_RO, false, &co->error_register, "Error register", NULL);
    add(&b, 0x1003, 0, U8, CO_A_RW, false, &co->err_count, "Pre-defined error field: number of errors", co_write_err_count);
    static const char *const err_names[CO_ERR_HISTORY] = {
        "Standard error field 1", "Standard error field 2", "Standard error field 3", "Standard error field 4",
        "Standard error field 5", "Standard error field 6", "Standard error field 7", "Standard error field 8"};
    for (uint8_t i = 0; i < CO_ERR_HISTORY; i++) {
        add(&b, 0x1003, (uint8_t)(i + 1u), U32, CO_A_RO, false, &co->err_history[i], err_names[i], NULL);
    }
    add(&b, 0x1014, 0, U32, CO_A_CONST, false, &a->cob_emcy, "COB-ID EMCY", NULL);
    add(&b, 0x1016, 0, U8, CO_A_CONST, false, &a->n_sub_1016, "Consumer heartbeat time: highest sub-index", NULL);
    add(&b, 0x1016, 1, U32, CO_A_RW, false, &co->hb_consumer_entry, "Consumer heartbeat time 1 (node << 16 | ms)", co_write_hb_consumer);
    add(&b, 0x1017, 0, U16, CO_A_RW, false, &co->hb_producer_ms, "Producer heartbeat time (ms)", write_hb_producer);
    add(&b, 0x1018, 0, U8, CO_A_CONST, false, &a->n_sub_1018, "Identity: highest sub-index", NULL);
    add(&b, 0x1018, 1, U32, CO_A_CONST, false, &a->vendor_id, "Vendor-ID", NULL);
    add(&b, 0x1018, 2, U32, CO_A_CONST, false, &a->product_code, "Product code", NULL);
    add(&b, 0x1018, 3, U32, CO_A_CONST, false, &a->revision, "Revision number", NULL);
    add(&b, 0x1018, 4, U32, CO_A_RO, false, &a->serial, "Serial number", NULL);

    /* RPDO communication and mapping */
    add(&b, 0x1400, 0, U8, CO_A_CONST, false, &a->n_sub_rpdo_comm, "RPDO1 comm: highest sub-index", NULL);
    add(&b, 0x1400, 1, U32, CO_A_CONST, false, &a->rpdo_cob[0], "RPDO1 COB-ID", NULL);
    add(&b, 0x1400, 2, U8, CO_A_CONST, false, &a->trans_type, "RPDO1 transmission type", NULL);
    add(&b, 0x1400, 5, U16, CO_A_RW_PREOP, false, &c->rpdo_deadline_ms, "RPDO1 event timer: command deadline (ms)", write_lift_param);
    add(&b, 0x1401, 0, U8, CO_A_CONST, false, &a->n_sub_rpdo_comm, "RPDO2 comm: highest sub-index", NULL);
    add(&b, 0x1401, 1, U32, CO_A_CONST, false, &a->rpdo_cob[1], "RPDO2 COB-ID", NULL);
    add(&b, 0x1401, 2, U8, CO_A_CONST, false, &a->trans_type, "RPDO2 transmission type", NULL);
    add(&b, 0x1401, 5, U16, CO_A_CONST, false, &a->rpdo2_event_ms, "RPDO2 event timer (ms, 0 = no deadline)", NULL);
    add(&b, 0x1600, 0, U8, CO_A_CONST, false, &a->n_map_rpdo[0], "RPDO1 mapping: number of entries", NULL);
    static const char *const rp1[3] = {"RPDO1 mapping 1", "RPDO1 mapping 2", "RPDO1 mapping 3"};
    static const char *const rp2[3] = {"RPDO2 mapping 1", "RPDO2 mapping 2", "RPDO2 mapping 3"};
    for (uint8_t i = 0; i < 3u; i++) {
        add(&b, 0x1600, (uint8_t)(i + 1u), U32, CO_A_CONST, false, &a->map_rpdo1[i], rp1[i], NULL);
    }
    add(&b, 0x1601, 0, U8, CO_A_CONST, false, &a->n_map_rpdo[1], "RPDO2 mapping: number of entries", NULL);
    for (uint8_t i = 0; i < 3u; i++) {
        add(&b, 0x1601, (uint8_t)(i + 1u), U32, CO_A_CONST, false, &a->map_rpdo2[i], rp2[i], NULL);
    }

    /* TPDO communication and mapping */
    add(&b, 0x1800, 0, U8, CO_A_CONST, false, &a->n_sub_tpdo_comm, "TPDO1 comm: highest sub-index", NULL);
    add(&b, 0x1800, 1, U32, CO_A_CONST, false, &a->tpdo_cob[0], "TPDO1 COB-ID", NULL);
    add(&b, 0x1800, 2, U8, CO_A_CONST, false, &a->trans_type, "TPDO1 transmission type", NULL);
    add(&b, 0x1800, 3, U16, CO_A_RW_PREOP, false, &a->tpdo_inhibit[0], "TPDO1 inhibit time (100 us)", NULL);
    add(&b, 0x1800, 5, U16, CO_A_RW, false, &a->tpdo_event[0], "TPDO1 event timer (ms)", NULL);
    add(&b, 0x1801, 0, U8, CO_A_CONST, false, &a->n_sub_tpdo_comm, "TPDO2 comm: highest sub-index", NULL);
    add(&b, 0x1801, 1, U32, CO_A_CONST, false, &a->tpdo_cob[1], "TPDO2 COB-ID", NULL);
    add(&b, 0x1801, 2, U8, CO_A_CONST, false, &a->trans_type, "TPDO2 transmission type", NULL);
    add(&b, 0x1801, 3, U16, CO_A_RW_PREOP, false, &a->tpdo_inhibit[1], "TPDO2 inhibit time (100 us)", NULL);
    add(&b, 0x1801, 5, U16, CO_A_RW, false, &a->tpdo_event[1], "TPDO2 event timer (ms)", NULL);
    static const char *const tp1[7] = {"TPDO1 mapping 1", "TPDO1 mapping 2", "TPDO1 mapping 3", "TPDO1 mapping 4",
                                       "TPDO1 mapping 5", "TPDO1 mapping 6", "TPDO1 mapping 7"};
    static const char *const tp2[6] = {"TPDO2 mapping 1", "TPDO2 mapping 2", "TPDO2 mapping 3",
                                       "TPDO2 mapping 4", "TPDO2 mapping 5", "TPDO2 mapping 6"};
    add(&b, 0x1A00, 0, U8, CO_A_CONST, false, &a->n_map_tpdo[0], "TPDO1 mapping: number of entries", NULL);
    for (uint8_t i = 0; i < 7u; i++) {
        add(&b, 0x1A00, (uint8_t)(i + 1u), U32, CO_A_CONST, false, &a->map_tpdo1[i], tp1[i], NULL);
    }
    add(&b, 0x1A01, 0, U8, CO_A_CONST, false, &a->n_map_tpdo[1], "TPDO2 mapping: number of entries", NULL);
    for (uint8_t i = 0; i < 6u; i++) {
        add(&b, 0x1A01, (uint8_t)(i + 1u), U32, CO_A_CONST, false, &a->map_tpdo2[i], tp2[i], NULL);
    }

    /* ---- Manufacturer area: lift status (TPDO1, LiftStatus) ---- */
    add(&b, 0x2000, 0, U8, CO_A_CONST, false, &a->n_sub_2000, "LiftStatus: highest sub-index", NULL);
    add(&b, 0x2000, 1, U8, CO_A_RO, true, &a->st1.state, "state", NULL);
    add(&b, 0x2000, 2, U8, CO_A_RO, true, &a->st1.flags, "flags", NULL);
    add(&b, 0x2000, 3, U16, CO_A_RO, true, &a->st1.position_01mm, "position (0.1 mm above lower end, 0xFFFF unknown)", NULL);
    add(&b, 0x2000, 4, U8, CO_A_RO, true, &a->st1.active_stop_id, "active_stop_id (0xFF none)", NULL);
    add(&b, 0x2000, 5, U8, CO_A_RO, true, &a->st1.last_seq, "last_seq", NULL);
    add(&b, 0x2000, 6, U8, CO_A_RO, true, &a->st1.cmd_result, "cmd_result (0xFF none since start-up)", NULL);
    add(&b, 0x2000, 7, U8, CO_A_RO, true, &a->st1.status_counter, "status_counter", NULL);

    /* Lift and aux detail (TPDO2) */
    add(&b, 0x2001, 0, U8, CO_A_CONST, false, &a->n_sub_2001, "LiftStatus2: highest sub-index", NULL);
    add(&b, 0x2001, 1, U8, CO_A_RO, true, &a->st2.inputs, "inputs (stop sensors, second E-stop, Hall supply, spares)", NULL);
    add(&b, 0x2001, 2, U8, CO_A_RO, true, &a->st2.target_stop_id, "target_stop_id (0xFF none)", NULL);
    add(&b, 0x2001, 3, U16, CO_A_RO, true, &a->st2.motor_current_ma, "motor current (mA)", NULL);
    add(&b, 0x2001, 4, I16, CO_A_RO, true, &a->st2.velocity_01mm_s, "velocity (0.1 mm/s, positive up)", NULL);
    add(&b, 0x2001, 5, U8, CO_A_RO, true, &a->st2.faults_lo, "faults F01..F08 (bit n-1 = Fn)", NULL);
    add(&b, 0x2001, 6, U8, CO_A_RO, true, &a->st2.aux_flags, "aux flags (F09, head light on, Getac enable)", NULL);
    add(&b, 0x2002, 0, U16, CO_A_RO, false, &a->faults, "Lift fault bitmask (bit n-1 = Fn)", NULL);

    /* Commands (RPDO1 LiftCommand, RPDO2 aux outputs) */
    add(&b, 0x2010, 0, U8, CO_A_CONST, false, &a->n_sub_2010, "LiftCommand: highest sub-index", NULL);
    add(&b, 0x2010, 1, U8, CO_A_RWW, true, &a->cmd.seq, "seq", NULL);
    add(&b, 0x2010, 2, U8, CO_A_RWW, true, &a->cmd.op, "op (0 STOP, 1 MOVE_TO_STOP, 2 CLEAR_FAULT, 3 START_HOMING)", NULL);
    add(&b, 0x2010, 3, U8, CO_A_RWW, true, &a->cmd.stop_id, "stop_id (0 L1000, 1 L1175, 2 L1350)", NULL);
    add(&b, 0x2011, 0, U8, CO_A_CONST, false, &a->n_sub_2011, "AuxCommand: highest sub-index", NULL);
    add(&b, 0x2011, 1, U8, CO_A_RW, true, &a->auxcmd.head_light_on, "head light on (0/1)", write_aux_cmd);
    add(&b, 0x2011, 2, U8, CO_A_RW, true, &a->auxcmd.head_light_dim_pct, "head light dim level (0..100 %)", write_aux_cmd);
    add(&b, 0x2011, 3, U8, CO_A_RW, true, &a->auxcmd.getac_enable, "Getac power or charging enable (0/1)", write_aux_cmd);

    /* Stop calibration */
    add(&b, 0x2100, 0, U8, CO_A_CONST, false, &a->n_sub_2100, "Stop positions: highest sub-index", NULL);
    add(&b, 0x2100, 1, I32, CO_A_RW_PREOP, false, &c->stop_pos_um[0], "stop 0 L1000 position (um above lower end)", write_lift_param);
    add(&b, 0x2100, 2, I32, CO_A_RW_PREOP, false, &c->stop_pos_um[1], "stop 1 L1175 position (um above lower end)", write_lift_param);
    add(&b, 0x2100, 3, I32, CO_A_RW_PREOP, false, &c->stop_pos_um[2], "stop 2 L1350 position (um above lower end)", write_lift_param);

    /* Lift parameters (volatile; defaults compiled in) */
    add(&b, 0x2101, 0, U8, CO_A_CONST, false, &a->n_sub_2101, "Lift parameters: highest sub-index", NULL);
    struct {
        uint8_t type;
        void *p;
        const char *n;
    } prm[] = {
        {I32, &c->hall_nm_per_count, "hall_nm_per_count"},
        {BOOL, &c->hall_count_up_positive, "hall_count_up_positive"},
        {I32, &c->stop_tol_um, "stop_tol_um"},
        {I32, &c->xcheck_tol_um, "xcheck_tol_um"},
        {I32, &c->sensor_edge_offset_up_um, "sensor_edge_offset_up_um"},
        {I32, &c->sensor_edge_offset_down_um, "sensor_edge_offset_down_um"},
        {I32, &c->stop_lead_um, "stop_lead_um"},
        {I32, &c->sensor_overrun_um, "sensor_overrun_um"},
        {U16, &c->duty_move_permille, "duty_move_permille"},
        {U16, &c->duty_homing_permille, "duty_homing_permille"},
        {I32, &c->move_speed_um_s, "move_speed_um_s"},
        {I32, &c->homing_speed_um_s, "homing_speed_um_s"},
        {U16, &c->hall_window_ms, "hall_window_ms"},
        {U16, &c->hall_start_grace_ms, "hall_start_grace_ms"},
        {U16, &c->move_rate_min_cps, "move_rate_min_cps"},
        {U16, &c->move_rate_max_cps, "move_rate_max_cps"},
        {U16, &c->homing_rate_min_cps, "homing_rate_min_cps"},
        {U16, &c->homing_rate_max_cps, "homing_rate_max_cps"},
        {U16, &c->stall_ms, "stall_ms"},
        {U16, &c->wrong_dir_counts, "wrong_dir_counts"},
        {U16, &c->travel_timeout_factor_pct, "travel_timeout_factor_pct"},
        {U16, &c->travel_timeout_margin_ms, "travel_timeout_margin_ms"},
        {U16, &c->overcurrent_ma, "overcurrent_ma"},
        {U16, &c->overcurrent_ms, "overcurrent_ms"},
        {U16, &c->inrush_blank_ms, "inrush_blank_ms"},
        {U16, &c->sensor_debounce_ms, "sensor_debounce_ms"},
        {U16, &c->standstill_quiet_ms, "standstill_quiet_ms"},
        {U16, &c->standstill_timeout_ms, "standstill_timeout_ms"},
        {BOOL, &c->end_switches_wired, "end_switches_wired"},
        {BOOL, &c->homing_continue_to_stop0, "homing_continue_to_stop0"},
        {BOOL, &c->stop_mode_brake, "stop_mode_brake"},
        {BOOL, &c->rereference_on_sensor, "rereference_on_sensor"},
    };
    for (uint8_t i = 0; i < (uint8_t)(sizeof(prm) / sizeof(prm[0])); i++) {
        add(&b, 0x2101, (uint8_t)(i + 1u), prm[i].type, CO_A_RW_PREOP, false, prm[i].p, prm[i].n, write_lift_param);
    }
    if (sizeof(prm) / sizeof(prm[0]) != a->n_sub_2101) {
        b.ok = false;
    }

    /* Diagnostics */
    add(&b, 0x2300, 0, U8, CO_A_CONST, false, &a->n_sub_2300, "Diagnostics: highest sub-index", NULL);
    add(&b, 0x2300, 1, I32, CO_A_RO, false, &a->hall_count_raw, "Hall count (raw)", NULL);
    add(&b, 0x2300, 2, U8, CO_A_RO, false, &a->reset_cause, "Reset cause (board port, TODO(Evans) encoding)", NULL);
    add(&b, 0x2300, 3, U32, CO_A_RO, false, &a->pdo_len_errors, "RPDO length errors", NULL);

    a->od_ok = b.ok;
}
