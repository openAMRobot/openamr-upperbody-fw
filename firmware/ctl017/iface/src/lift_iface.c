/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 */
#include "ctl017/lift_iface.h"

#include <string.h>

void ctl017_lift_status_unknown(ctl017_lift_status_t *st)
{
    memset(st, 0, sizeof(*st));
    st->state = CTL017_STATE_INIT;
    st->position_01mm = CTL017_POSITION_UNKNOWN;
    st->active_stop_id = CTL017_STOP_NONE;
    st->cmd_result = CTL017_RESULT_NONE;
    st->target_stop_id = CTL017_STOP_NONE;
}
