/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Host tests of the IFX007T drive gate and of the skeleton board
 * configuration: while the current limit, speed and end-switch data are
 * TODO(Evans), the NUCLEO-G474RE build must never enable a half-bridge.
 */
#include <string.h>

#include "board_drive_config.h"
#include "ctl017/drive_gate.h"
#include "ctl017/lift.h"
#include "lift_plant.h"
#include "test_util.h"

#define MM(x) ((int64_t)(x) * 1000000)

static drive_gate_t all_open(void)
{
    drive_gate_t g = {true, true, true, true, true, true};
    return g;
}

static bool any_high(ifx007t_levels_t lv)
{
    return lv.inh_a || lv.inh_b || lv.in_a || lv.in_b;
}

/* The skeleton configuration keeps the gate closed, with or without
 * H-bridge pins assigned. */
static void test_skeleton_gate_closed(void)
{
    drive_gate_t none;
    memset(&none, 0, sizeof(none));
    CHECK(!drive_gate_open(&none));
    drive_gate_t g = board_drive_gate(false);
    CHECK(!drive_gate_open(&g));
    g = board_drive_gate(true);
    CHECK(!drive_gate_open(&g));
    CHECK_EQ(BOARD_ISENSE_UA_PER_COUNT, 0u);
    CHECK_EQ(BOARD_HB_PWM_IMPLEMENTED, 0);
    CHECK_EQ(BOARD_CURRENT_LIMIT_CONFIRMED, 0);
    CHECK_EQ(BOARD_SPEED_DATA_CONFIRMED, 0);
    CHECK_EQ(BOARD_END_SWITCH_DATA_CONFIRMED, 0);
}

/* Any single missing item keeps every pin low for every command. */
static void test_each_item_closes_gate(void)
{
    const lift_hbridge_cmd_t cmds[] = {
        {true, LIFT_DIR_UP, 1000, false},
        {true, LIFT_DIR_DOWN, 500, false},
        {true, LIFT_DIR_NONE, 0, true},
    };
    for (unsigned item = 0; item < 6; item++) {
        drive_gate_t g = all_open();
        bool *f[6] = {&g.hbridge_pins_assigned, &g.pwm_implemented, &g.isense_scale_set,
                      &g.current_limit_confirmed, &g.speed_data_confirmed,
                      &g.end_switch_data_confirmed};
        *f[item] = false;
        CHECK(!drive_gate_open(&g));
        for (unsigned c = 0; c < sizeof(cmds) / sizeof(cmds[0]); c++) {
            CHECK(!any_high(ifx007t_levels(&g, &cmds[c])));
        }
    }
}

/* With the gate open the mapping follows the IFX007T truth table. */
static void test_open_gate_mapping(void)
{
    drive_gate_t g = all_open();
    CHECK(drive_gate_open(&g));
    lift_hbridge_cmd_t off = {false, LIFT_DIR_UP, 1000, false};
    CHECK(!any_high(ifx007t_levels(&g, &off)));
    lift_hbridge_cmd_t up = {true, LIFT_DIR_UP, 1000, false};
    ifx007t_levels_t lv = ifx007t_levels(&g, &up);
    CHECK(lv.inh_a && lv.inh_b && lv.in_a && !lv.in_b);
    lift_hbridge_cmd_t down = {true, LIFT_DIR_DOWN, 1000, false};
    lv = ifx007t_levels(&g, &down);
    CHECK(lv.inh_a && lv.inh_b && !lv.in_a && lv.in_b);
    lift_hbridge_cmd_t brake = {true, LIFT_DIR_NONE, 0, true};
    lv = ifx007t_levels(&g, &brake);
    CHECK(lv.inh_a && lv.inh_b && !lv.in_a && !lv.in_b);
    lift_hbridge_cmd_t nodir = {true, LIFT_DIR_NONE, 1000, false};
    CHECK(!any_high(ifx007t_levels(&g, &nodir)));
    lift_hbridge_cmd_t zero = {true, LIFT_DIR_UP, 0, false};
    CHECK(!any_high(ifx007t_levels(&g, &zero)));
}

/* End to end with the core: a move the core accepts and drives (enable
 * true) still gives all-low pins through the skeleton gate on every tick,
 * also in brake stop mode. The column (plant fed with the gated command)
 * does not move, and the core ends in a fault, not in motion. */
static void run_gated_move(bool brake_mode)
{
    lift_config_t c;
    lift_config_default(&c);
    c.sensor_edge_offset_up_um = -1000;
    c.sensor_edge_offset_down_um = 1000;
    c.stop_mode_brake = brake_mode;
    lift_t l;
    lift_plant_t p;
    plant_init(&p, MM(5));
    uint32_t now = 1000;
    lift_init(&l, &c, now);
    lift_set_supervision_lost(&l, false);
    drive_gate_t g = board_drive_gate(true);
    bool core_enabled = false;
    bool accepted = false;
    for (uint32_t t = 0; t < 20000; t++) {
        now++;
        lift_inputs_t in;
        plant_inputs(&p, now, &in);
        const lift_hbridge_cmd_t *hb = lift_step(&l, &in);
        core_enabled |= hb->enable;
        ifx007t_levels_t lv = ifx007t_levels(&g, hb);
        CHECK(!any_high(lv));
        lift_hbridge_cmd_t applied = {lv.inh_a && lv.inh_b, LIFT_DIR_NONE, 0, false};
        plant_step(&p, &applied, 1);
        if (t == 300) {
            accepted = lift_command(&l, 1, LIFT_OP_MOVE_TO_STOP, 2, now) == LIFT_RES_ACCEPTED;
        } else if (t > 300 && t % 100 == 0) {
            (void)lift_command(&l, 1, LIFT_OP_MOVE_TO_STOP, 2, now); /* keep-alive */
        }
    }
    CHECK(accepted);
    CHECK(core_enabled);
    CHECK_EQ(l.state, LIFT_STATE_FAULT);
    CHECK(l.faults & (LIFT_FAULT_F03_OVERCURRENT | LIFT_FAULT_F04_MOVE_TIMEOUT));
    CHECK(p.pos_nm == MM(5));
}

static void test_skeleton_never_drives_with_core(void)
{
    run_gated_move(false);
    run_gated_move(true);
}

int main(void)
{
    RUN(test_skeleton_gate_closed);
    RUN(test_each_item_closes_gate);
    RUN(test_open_gate_mapping);
    RUN(test_skeleton_never_drives_with_core);
    TEST_MAIN_END();
}
