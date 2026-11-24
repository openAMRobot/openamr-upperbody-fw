/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 */
#include "ctl017/aux_io.h"

#include <string.h>

void aux_config_default(aux_config_t *cfg)
{
    cfg->input_debounce_ms = 20;            /* TODO(Evans) */
    cfg->getac_enable_at_boot = false;      /* TODO(Evans) */
    cfg->head_light_dim_default_pct = 100;  /* TODO(Evans) */
    cfg->hold_outputs_on_comm_loss = true;  /* TODO(Evans) */
}

static void update_outputs(aux_io_t *a)
{
    a->out.head_light_on = a->light_on;
    a->out.head_light_duty_permille = a->light_on ? (uint16_t)(a->light_dim_pct * 10u) : 0u;
    a->out.getac_enable = a->getac_enable;
}

void aux_init(aux_io_t *a, const aux_config_t *cfg, uint32_t now_ms)
{
    memset(a, 0, sizeof(*a));
    a->cfg = *cfg;
    debounce_init(&a->estop2_db, false, cfg->input_debounce_ms, now_ms);
    for (unsigned i = 0; i < AUX_NUM_SPARE_IN; i++) {
        debounce_init(&a->spare_db[i], false, cfg->input_debounce_ms, now_ms);
    }
    a->light_on = false; /* head light only during arm operations */
    a->light_dim_pct = cfg->head_light_dim_default_pct;
    a->getac_enable = cfg->getac_enable_at_boot;
    update_outputs(a);
}

void aux_command(aux_io_t *a, bool light_on, uint8_t dim_pct, bool getac_enable)
{
    a->light_on = light_on;
    a->light_dim_pct = (dim_pct > 100u) ? 100u : dim_pct;
    a->getac_enable = getac_enable;
    update_outputs(a);
}

void aux_comm_lost(aux_io_t *a)
{
    if (!a->cfg.hold_outputs_on_comm_loss) {
        a->light_on = false;
        update_outputs(a);
    }
}

const aux_outputs_t *aux_step(aux_io_t *a, const aux_inputs_t *in)
{
    debounce_update(&a->estop2_db, in->estop2_no_closed, in->now_ms);
    for (unsigned i = 0; i < AUX_NUM_SPARE_IN; i++) {
        debounce_update(&a->spare_db[i], in->spare_in[i], in->now_ms);
    }
    return &a->out;
}

bool aux_estop2_pressed(const aux_io_t *a)
{
    return a->estop2_db.state;
}

uint8_t aux_spare_bits(const aux_io_t *a)
{
    uint8_t b = 0;
    for (unsigned i = 0; i < AUX_NUM_SPARE_IN; i++) {
        if (a->spare_db[i].state) {
            b |= (uint8_t)(1u << i);
        }
    }
    return b;
}
