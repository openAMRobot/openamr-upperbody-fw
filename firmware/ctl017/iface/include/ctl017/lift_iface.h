/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 lift interface: the only contact between the CANopen application
 * layer (firmware/ctl017/app) and the lift owner's code.
 *
 * Plain data only. The application layer fills a ctl017_iface_rx_t with what
 * arrived on CAN3 (commands, auxiliary requests, link observations and, when
 * built with CTL017_PERMIT_OD, the permit record) and reads one
 * ctl017_lift_status_t per tick to put on the bus. It never calls into the
 * lift, never interprets a command and never decides whether anything may
 * move. Wire values (states, flags, ops, results, fault bits) follow
 * docs/ctl017_object_dictionary.md, the v1 wire contract (upperbody-fw #15).
 *
 * Freshness rule: every record carries a receive counter and the local
 * receipt time. A record whose counter did not change since the consumer
 * last looked is not a new sample, even if the content is identical.
 */
#ifndef CTL017_LIFT_IFACE_H
#define CTL017_LIFT_IFACE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CTL017_NUM_STOPS 3u

/* LiftStatus.state (wire values) */
#define CTL017_STATE_INIT      0u
#define CTL017_STATE_UNHOMED   1u
#define CTL017_STATE_HOMING    2u
#define CTL017_STATE_IDLE      3u
#define CTL017_STATE_MOVING    4u
#define CTL017_STATE_STOPPING  5u
#define CTL017_STATE_FAULT     6u

/* LiftStatus.flags (wire values) */
#define CTL017_FLAG_POSITION_VALID      (1u << 0)
#define CTL017_FLAG_AT_STOP             (1u << 1)
#define CTL017_FLAG_MOVING              (1u << 2)
#define CTL017_FLAG_LOWER_LIMIT         (1u << 3)
#define CTL017_FLAG_UPPER_LIMIT         (1u << 4)
#define CTL017_FLAG_FAULT_PRESENT       (1u << 5)
#define CTL017_FLAG_MOTOR_POWER_PRESENT (1u << 6)
#define CTL017_FLAG_RECOVERY_REQUIRED   (1u << 7)

/* LiftCommand.op (wire values; the application never interprets them) */
#define CTL017_OP_STOP          0u
#define CTL017_OP_MOVE_TO_STOP  1u
#define CTL017_OP_CLEAR_FAULT   2u
#define CTL017_OP_START_HOMING  3u

#define CTL017_RESULT_NONE      0xFFu
#define CTL017_STOP_NONE        0xFFu
#define CTL017_POSITION_UNKNOWN 0xFFFFu

/* Fault bitmask, bit n-1 = Fn (wire values, F09 is a proposal). */
#define CTL017_FAULT_F03_OVERCURRENT (1u << 2)
#define CTL017_FAULT_F07_SUPERVISION (1u << 6)
#define CTL017_FAULT_F09_STOP_SENSORS (1u << 8)
#define CTL017_FAULT_ALL             0x01FFu

/* TPDO2 inputs byte */
#define CTL017_IN_STOP0       (1u << 0)
#define CTL017_IN_STOP1       (1u << 1)
#define CTL017_IN_STOP2       (1u << 2)
#define CTL017_IN_ESTOP2      (1u << 3) /* diagnostic observation only */
#define CTL017_IN_HALL_SUPPLY (1u << 4)
#define CTL017_IN_SPARE1      (1u << 5)
#define CTL017_IN_SPARE2      (1u << 6)
#define CTL017_IN_SPARE3      (1u << 7)
/* TPDO2 aux flags byte */
#define CTL017_AUX_F09        (1u << 0)
#define CTL017_AUX_LIGHT_ON   (1u << 1)
#define CTL017_AUX_GETAC_EN   (1u << 2)

/* ---- Status out: filled by the lift owner, sent by the application ---- */
typedef struct {
    uint8_t state;
    uint8_t flags;
    uint16_t position_01mm;     /* CTL017_POSITION_UNKNOWN unless position_valid */
    uint8_t active_stop_id;     /* CTL017_STOP_NONE if none */
    uint8_t last_seq;
    uint8_t cmd_result;         /* CTL017_RESULT_NONE before the first command */
    uint8_t target_stop_id;     /* CTL017_STOP_NONE if none */
    uint8_t inputs;             /* CTL017_IN_* */
    uint8_t aux_flags;          /* CTL017_AUX_* except F09, which comes from faults */
    uint16_t motor_current_ma;
    int16_t velocity_01mm_s;
    uint16_t faults;            /* latched bitmask, bit n-1 = Fn */
    int32_t hall_count_raw;     /* diagnostics 0x2300:01 */
    uint8_t reset_cause;        /* diagnostics 0x2300:02, encoding TODO(Evans) */
} ctl017_lift_status_t;

/* Status the application sends until the lift owner provides one: INIT,
 * position unknown, no stop, no command result. Never a nominal value. */
void ctl017_lift_status_unknown(ctl017_lift_status_t *st);

/* ---- Commands in: written by the application, read by the lift owner ---- */
typedef struct {
    bool valid;                 /* false until the first RPDO1 of this NMT epoch */
    uint8_t seq;                /* raw RPDO1 bytes, not interpreted */
    uint8_t op;
    uint8_t stop_id;
    uint32_t rx_count;          /* +1 for every RPDO1 frame accepted, duplicates included */
    uint32_t rx_ms;             /* local receipt time of the last one */
} ctl017_lift_cmd_rx_t;

typedef struct {
    bool valid;                 /* false until the first RPDO2 or SDO write */
    uint8_t head_light_on;      /* 0 or 1 */
    uint8_t head_light_dim_pct; /* 0..100 */
    uint8_t getac_enable;       /* 0 or 1 */
    uint32_t rx_count;
    uint32_t rx_ms;
} ctl017_aux_cmd_rx_t;

/* Observations of the CANopen link. The application reports them; it does
 * not react to them. */
typedef struct {
    uint8_t nmt_state;          /* co_nmt_state_t value */
    uint32_t nmt_epoch;         /* +1 on start-up, reset communication and every NMT state change */
    uint8_t hbc_state;          /* co_hbc_state_t value of the consumer of the base controller */
    uint32_t hbc_timeouts;      /* heartbeat consumer timeouts seen */
    uint32_t hbc_remote_boots;  /* boot-up messages seen from the base controller */
    uint32_t hbc_last_rx_ms;
    uint16_t rpdo1_event_timer_ms; /* 0x1400:05 value, TODO(Evans); not supervised here */
    uint32_t rx_malformed;
    uint32_t rpdo_len_errors;
    bool reset_node_requested;  /* NMT reset node received; the board port resets */
} ctl017_link_rx_t;

/* Lift parameters (0x2100, 0x2101): storage only. Every value is a
 * TODO(Evans) parameter with no default; 0 means "not set". The lift owner
 * validates them; the application stores what the validator accepts. */
typedef struct {
    int32_t stop_pos_um[CTL017_NUM_STOPS]; /* 0x2100:01..03 */
    /* 0x2101:01..32 in this order (wire types in the OD document) */
    int32_t hall_nm_per_count;
    bool hall_count_up_positive;
    int32_t stop_tol_um;
    int32_t xcheck_tol_um;
    int32_t sensor_edge_offset_up_um;
    int32_t sensor_edge_offset_down_um;
    int32_t stop_lead_um;
    int32_t sensor_overrun_um;
    uint16_t duty_move_permille;
    uint16_t duty_homing_permille;
    int32_t move_speed_um_s;
    int32_t homing_speed_um_s;
    uint16_t hall_window_ms;
    uint16_t hall_start_grace_ms;
    uint16_t move_rate_min_cps;
    uint16_t move_rate_max_cps;
    uint16_t homing_rate_min_cps;
    uint16_t homing_rate_max_cps;
    uint16_t stall_ms;
    uint16_t wrong_dir_counts;
    uint16_t travel_timeout_factor_pct;
    uint16_t travel_timeout_margin_ms;
    uint16_t overcurrent_ma;
    uint16_t overcurrent_ms;
    uint16_t inrush_blank_ms;
    uint16_t sensor_debounce_ms;
    uint16_t standstill_quiet_ms;
    uint16_t standstill_timeout_ms;
    bool end_switches_wired;
    bool homing_continue_to_stop0;
    bool stop_mode_brake;
    bool rereference_on_sensor;
    uint32_t write_count;       /* +1 per accepted SDO write */
} ctl017_lift_params_t;

#ifdef CTL017_PERMIT_OD
/* Permit record (docs/ctl017_permit_contract.md). Received and exposed as
 * data only. The application never acts on it, never derives a decision
 * from it and never refreshes it on its own. */
typedef struct {
    bool valid;                 /* false until the first permit PDO of this NMT epoch */
    uint32_t permit_flag;       /* raw field, width CTL017_PERMIT_FLAG_BITS */
    uint32_t producer_boot_id;  /* raw field, width CTL017_PERMIT_BOOT_BITS */
    uint32_t sequence;          /* raw field, width CTL017_PERMIT_SEQ_BITS */
    uint32_t reason;            /* raw field, width CTL017_PERMIT_REASON_BITS */
    uint32_t rx_count;
    uint32_t rx_ms;
    uint32_t len_errors;        /* frames shorter than the mapping */
} ctl017_permit_rx_t;
#endif

typedef struct {
    ctl017_lift_cmd_rx_t lift_cmd;
    ctl017_aux_cmd_rx_t aux_cmd;
    ctl017_link_rx_t link;
#ifdef CTL017_PERMIT_OD
    ctl017_permit_rx_t permit;
#endif
} ctl017_iface_rx_t;

/* Optional validator for parameter writes, supplied by the lift owner.
 * Returns true if the parameter set is acceptable. */
typedef bool (*ctl017_params_valid_fn)(void *ctx, const ctl017_lift_params_t *p);

#ifdef __cplusplus
}
#endif

#endif /* CTL017_LIFT_IFACE_H */
