/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 lift node: tunable parameters.
 *
 * Every value marked TODO(Evans) is a placeholder that keeps the code and the
 * host tests running. It is not a hardware fact. The owner sets it from the
 * DOLD data for the ordered column (BS-HM240G350530-MFF24) and from the bench.
 * Values without a TODO come from a recorded decision, cited next to them.
 */
#ifndef CTL017_LIFT_CONFIG_H
#define CTL017_LIFT_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LIFT_NUM_STOPS 3u /* L1000, L1175, L1350 (decision D3) */

typedef struct {
    /* --- Hall position measurement --------------------------------------- */
    /* Travel per Hall count in nanometres. TODO(Evans): 0.2092 mm per Hall
     * signal is the DOLD catalogue figure on the lift sheet; confirm it for the
     * ordered model and confirm whether it is per edge of the two-channel
     * (quadrature) count that TIM5 produces or per pulse of one channel. */
    int32_t hall_nm_per_count;
    /* true if the port's count increases while the column extends.
     * TODO(Evans): set from the DOLD Hall wiring. */
    bool hall_count_up_positive;

    /* --- Stops (positions above the lower end of travel, micrometres) ----- */
    /* TODO(Evans): calibrate. Defaults: stop 0 sits 5 mm inside the lower
     * switch and the stops are 175 mm apart (Alex 27 Oct; P-03 item 24).
     * That implies a 360 mm switch-to-switch travel against the 350 mm
     * catalogue stroke: stroke check is open. */
    int32_t stop_pos_um[LIFT_NUM_STOPS];
    /* at_stop tolerance: plus or minus 2 mm for every stop (decision D3,
     * Alex 27 Oct). */
    int32_t stop_tol_um;
    /* Hall cross-check window when a stop sensor switches on (F02).
     * TODO(Evans): D10 uses plus or minus 2 mm for the stroke cross-check;
     * confirm it also applies at every stop sensor edge. */
    int32_t xcheck_tol_um;
    /* Where a stop sensor switches on relative to its stop position, per
     * travel direction (the steel flag has a finite length).
     * TODO(Evans): measure on the bench (drawing LIFT-SNS-01). */
    int32_t sensor_edge_offset_up_um;
    int32_t sensor_edge_offset_down_um;
    /* Distance before the stop position at which the H-bridge is switched
     * off, to absorb the coast. TODO(Evans): measure on the bench. */
    int32_t stop_lead_um;
    /* How far the Hall position may pass the target stop without the target
     * stop sensor switching on before F02 is raised. Must stay below the
     * 5 mm margin to the end switch. TODO(Evans). */
    int32_t sensor_overrun_um;
    /* After a valid sensor edge, set the Hall reference so that the position
     * equals the expected edge position (removes accumulated drift). */
    bool rereference_on_sensor;

    /* --- Drive --------------------------------------------------------------- */
    /* PWM duty in permille for a move between stops and for homing.
     * TODO(Evans): the record asks for "reduced speed" without a value. */
    uint16_t duty_move_permille;
    uint16_t duty_homing_permille;
    /* Expected speed for the travel timeout, micrometres per second.
     * 18 mm/s is the DOLD no-load speed at 24 V (P-03 item 16).
     * TODO(Evans): replace with the loaded speed at the chosen duty. */
    int32_t move_speed_um_s;
    int32_t homing_speed_um_s;
    /* Halt mode: false = H-bridge disabled (column holds mechanically,
     * I8 rev 3 Q10), true = both low sides on (short-circuit brake). */
    bool stop_mode_brake;

    /* --- Hall-window check (P-03 item 30 A7) ------------------------------- */
    /* Pulse rate window, counts per second, evaluated every hall_window_ms
     * once hall_start_grace_ms has passed after the drive was switched on.
     * Below min (but not zero) or above max raises F02 (implausible pulse
     * rate, Alex 24 Oct). No pulse at all for stall_ms raises F03.
     * TODO(Evans): all five values. Defaults are 0.5x to 1.5x of the rate
     * implied by 18 mm/s and 0.2092 mm per count (86 counts/s). */
    uint16_t hall_window_ms;
    uint16_t hall_start_grace_ms;
    uint16_t move_rate_min_cps;
    uint16_t move_rate_max_cps;
    uint16_t homing_rate_min_cps;
    uint16_t homing_rate_max_cps;
    uint16_t stall_ms;
    /* Counts against the commanded direction that raise F02. TODO(Evans). */
    uint16_t wrong_dir_counts;

    /* --- Travel timeout per move (F04, P-03 item 30 A7) -------------------- */
    /* timeout = distance / speed * factor + margin. Homing uses the full
     * travel between stop 0 and stop 2 plus one stop margin as distance.
     * TODO(Evans): factor and margin. */
    uint16_t travel_timeout_factor_pct;
    uint16_t travel_timeout_margin_ms;

    /* --- Current limit (F03) ---------------------------------------------- */
    /* TODO(Evans): 7 A is the DOLD maximum current (P-03 item 16), used here
     * as the default trip level; set from the motor data and F-LIFT 15 A. */
    uint16_t overcurrent_ma;
    uint16_t overcurrent_ms;      /* TODO(Evans) */
    uint16_t inrush_blank_ms;     /* TODO(Evans) */

    /* --- Inputs ------------------------------------------------------------- */
    uint16_t sensor_debounce_ms;  /* stop sensors and end switches, TODO(Evans) */
    uint16_t input_debounce_ms;   /* second E-stop NO and spare inputs, TODO(Evans) */
    /* Whether the DOLD built-in end switches are wired to the node.
     * TODO(Evans): DOLD end-switch data are open. */
    bool end_switches_wired;

    /* --- Standstill (F08) --------------------------------------------------- */
    /* No Hall count change for standstill_quiet_ms means standstill.
     * TODO(Evans): quiet time. F08 is raised if standstill is not confirmed
     * standstill_timeout_ms after the drive was switched off (start value
     * 500 ms, Alex 27 Oct; final value from the bench). */
    uint16_t standstill_quiet_ms;
    uint16_t standstill_timeout_ms;

    /* --- Supervision (decisions D12, D15) ----------------------------------- */
    uint16_t rpdo_deadline_ms;    /* 300 ms command keep-alive deadline */

    /* --- Homing (decision D10, work package: home to L1000) ---------------- */
    /* Homing drives down. The first stop sensor reached sets the reference.
     * If true, homing then continues to L1000 and cross-checks it. */
    bool homing_continue_to_stop0;

    /* --- Start-up ----------------------------------------------------------- */
    uint16_t init_settle_ms;      /* time to settle the debouncers, TODO(Evans) */
} lift_config_t;

/* Fills cfg with the placeholder defaults documented above. */
void lift_config_default(lift_config_t *cfg);

/* Returns true if cfg is internally consistent (ordered stops, non-zero
 * scales, windows with min <= max). */
bool lift_config_valid(const lift_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* CTL017_LIFT_CONFIG_H */
