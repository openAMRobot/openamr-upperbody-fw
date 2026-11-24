/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 */
#include "ctl017/debounce.h"

void debounce_init(debounce_t *d, bool initial, uint16_t hold_ms, uint32_t now_ms)
{
    d->state = initial;
    d->candidate = initial;
    d->since_ms = now_ms;
    d->hold_ms = hold_ms;
}

bool debounce_update(debounce_t *d, bool raw, uint32_t now_ms)
{
    if (raw != d->candidate) {
        d->candidate = raw;
        d->since_ms = now_ms;
    }
    if (d->candidate != d->state && (uint32_t)(now_ms - d->since_ms) >= d->hold_ms) {
        d->state = d->candidate;
    }
    return d->state;
}
