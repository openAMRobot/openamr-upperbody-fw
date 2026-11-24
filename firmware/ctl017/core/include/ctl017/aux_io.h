/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 auxiliary I/O (P-03 item 29 CTL-017 scope, decision D-11):
 * head light low-side PWM output, Getac ZX80 power or charging enable,
 * second E-stop spare NO contact (diagnostics only, I8 rev 3 section 5.3),
 * spare functional inputs. None of these is a safety function.
 */
#ifndef CTL017_AUX_IO_H
#define CTL017_AUX_IO_H

#include <stdbool.h>
#include <stdint.h>

#include "ctl017/debounce.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AUX_NUM_SPARE_IN 3u

typedef struct {
    uint16_t input_debounce_ms;   /* TODO(Evans) */
    bool getac_enable_at_boot;    /* TODO(Evans) */
    uint8_t head_light_dim_default_pct; /* TODO(Evans) */
    /* On loss of the core heartbeat: true = keep the last commanded outputs,
     * false = head light off (Getac enable is never dropped by comms loss).
     * TODO(Evans). */
    bool hold_outputs_on_comm_loss;
} aux_config_t;

typedef struct {
    uint32_t now_ms;
    bool estop2_no_closed;        /* logical: spare NO contact closed = pressed */
    bool spare_in[AUX_NUM_SPARE_IN];
} aux_inputs_t;

typedef struct {
    bool head_light_on;
    uint16_t head_light_duty_permille;
    bool getac_enable;
} aux_outputs_t;

typedef struct {
    aux_config_t cfg;
    debounce_t estop2_db;
    debounce_t spare_db[AUX_NUM_SPARE_IN];
    bool light_on;
    uint8_t light_dim_pct;
    bool getac_enable;
    aux_outputs_t out;
} aux_io_t;

void aux_config_default(aux_config_t *cfg);
void aux_init(aux_io_t *a, const aux_config_t *cfg, uint32_t now_ms);

/* RPDO2 content. dim_pct above 100 is clamped. */
void aux_command(aux_io_t *a, bool light_on, uint8_t dim_pct, bool getac_enable);

void aux_comm_lost(aux_io_t *a);

const aux_outputs_t *aux_step(aux_io_t *a, const aux_inputs_t *in);

bool aux_estop2_pressed(const aux_io_t *a);
uint8_t aux_spare_bits(const aux_io_t *a);

#ifdef __cplusplus
}
#endif

#endif /* CTL017_AUX_IO_H */
