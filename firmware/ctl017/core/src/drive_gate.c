/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 */
#include "ctl017/drive_gate.h"

bool drive_gate_open(const drive_gate_t *g)
{
    return g->hbridge_pins_assigned && g->pwm_implemented && g->isense_scale_set &&
           g->current_limit_confirmed && g->speed_data_confirmed && g->end_switch_data_confirmed;
}

ifx007t_levels_t ifx007t_levels(const drive_gate_t *g, const lift_hbridge_cmd_t *cmd)
{
    ifx007t_levels_t lv = {false, false, false, false};
    if (!drive_gate_open(g) || !cmd->enable) {
        return lv;
    }
    if (cmd->brake) {
        lv.inh_a = true;
        lv.inh_b = true;
        return lv;
    }
    if (cmd->dir == LIFT_DIR_NONE || cmd->duty_permille == 0u) {
        return lv;
    }
    lv.inh_a = true;
    lv.inh_b = true;
    lv.in_a = (cmd->dir == LIFT_DIR_UP);
    lv.in_b = (cmd->dir == LIFT_DIR_DOWN);
    return lv;
}
