/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Time-based debouncer: the output follows the raw input once the raw input
 * has held the new level for hold_ms. Wrap-safe on a uint32_t millisecond
 * clock.
 */
#ifndef CTL017_DEBOUNCE_H
#define CTL017_DEBOUNCE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool state;       /* debounced output */
    bool candidate;   /* last raw level seen */
    uint32_t since_ms;/* time the raw level last changed */
    uint16_t hold_ms;
} debounce_t;

void debounce_init(debounce_t *d, bool initial, uint16_t hold_ms, uint32_t now_ms);

/* Feeds one raw sample; returns the debounced state. */
bool debounce_update(debounce_t *d, bool raw, uint32_t now_ms);

/* Wrap-safe "a is at or after b". */
static inline bool time_reached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

#ifdef __cplusplus
}
#endif

#endif /* CTL017_DEBOUNCE_H */
