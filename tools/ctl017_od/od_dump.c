/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Prints the CTL-017 object dictionary as one JSON object per line, as
 * built by the firmware at start-up. Input of gen_od_docs.py.
 *
 * The OD is built with an empty config: entries set by an integration
 * parameter carry its name in "param" and their value is not printed as a
 * default (there is none, TODO(Evans)).
 */
#include <stdio.h>
#include <string.h>

#include "ctl017/ctl017_app.h"

static const char *access_name(uint8_t a)
{
    switch (a) {
    case CO_A_CONST:
        return "const";
    case CO_A_RO:
        return "ro";
    case CO_A_RW:
        return "rw";
    case CO_A_RWW:
        return "rww";
    case CO_A_RW_PREOP:
        return "rw_preop";
    default:
        return "?";
    }
}

static ctl017_app_t app;

int main(void)
{
    bool ok = true;
    ctl017_od_build(&app, &ok);
    if (!ok) {
        fprintf(stderr, "OD inconsistent\n");
        return 1;
    }
    for (size_t i = 0; i < app.od_len; i++) {
        const co_od_entry_t *e = &app.od[i];
        uint32_t v = co_od_get(e);
        long long sv = (long long)v;
        if (e->type == CO_T_INT32) {
            sv = (int32_t)v;
        } else if (e->type == CO_T_INT16) {
            sv = (int16_t)(uint16_t)v;
        }
        printf("{\"index\": %u, \"sub\": %u, \"type\": %u, \"access\": \"%s\", \"pdo\": %s, "
               "\"value\": %lld, \"name\": \"%s\", \"param\": %s%s%s}\n",
               e->index, e->sub, e->type, access_name(e->access), e->pdo_mappable ? "true" : "false",
               sv, e->name, e->param ? "\"" : "", e->param ? e->param : "null", e->param ? "\"" : "");
    }
    return 0;
}
