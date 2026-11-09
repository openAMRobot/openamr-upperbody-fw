/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 lift node core: lift state machine, Hall position, stop sensors,
 * Hall-window check, travel timeout, current limit, standstill and command
 * supervision.
 *
 * Portable C11. No HAL, no registers, no CAN stack. The board port fills a
 * lift_inputs_t every control tick, calls lift_step() and applies the
 * returned H-bridge command. Not a safety function: the hardwired PNOZ chain
 * removes motor power independently of this code (decision D6, D13).
 *
 * Message semantics follow the CAN3 lift message set (openamr-platform-fw #12,
 * decisions D1 to D20 of 24 Oct 2026 and the 27 Oct changes): states, flags,
 * cmd_result codes and fault IDs below are the wire values.
 */
#ifndef CTL017_LIFT_H
#define CTL017_LIFT_H

#include <stdbool.h>
#include <stdint.h>

#include "ctl017/debounce.h"
#include "ctl017/lift_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* LiftStatus.state */
typedef enum {
    LIFT_STATE_INIT = 0,
    LIFT_STATE_UNHOMED = 1,
    LIFT_STATE_HOMING = 2,
    LIFT_STATE_IDLE = 3,
    LIFT_STATE_MOVING = 4,
    LIFT_STATE_STOPPING = 5,
    LIFT_STATE_FAULT = 6,
} lift_state_t;

/* LiftStatus.flags (after Alex 27 Oct: bit 6 motor_power_present replaces
 * homed, bit 7 recovery_required = not homed). */
#define LIFT_FLAG_POSITION_VALID      (1u << 0)
#define LIFT_FLAG_AT_STOP             (1u << 1)
#define LIFT_FLAG_MOVING              (1u << 2)
#define LIFT_FLAG_LOWER_LIMIT         (1u << 3)
#define LIFT_FLAG_UPPER_LIMIT         (1u << 4)
#define LIFT_FLAG_FAULT_PRESENT       (1u << 5)
#define LIFT_FLAG_MOTOR_POWER_PRESENT (1u << 6)
#define LIFT_FLAG_RECOVERY_REQUIRED   (1u << 7)

/* LiftCommand.op */
typedef enum {
    LIFT_OP_STOP = 0,
    LIFT_OP_MOVE_TO_STOP = 1,
    LIFT_OP_CLEAR_FAULT = 2,
    LIFT_OP_START_HOMING = 3,
} lift_op_t;

/* LiftStatus.cmd_result */
typedef enum {
    LIFT_RES_ACCEPTED = 0,
    LIFT_RES_INVALID_STOP = 1,
    LIFT_RES_POSITION_INVALID = 2,
    LIFT_RES_FAULT_ACTIVE = 3,
    LIFT_RES_LIMIT_ACTIVE = 4,
    LIFT_RES_BUSY = 5,
    LIFT_RES_RECOVERY_REQUIRED = 6,
    LIFT_RES_CLEAR_CONDITIONS = 7,
    LIFT_RES_HOMING_PRECONDITIONS = 8,
    LIFT_RES_MALFORMED = 9,
    LIFT_RES_SEQ_CONFLICT = 10,
    LIFT_RES_MOTOR_POWER_ABSENT = 11,
    LIFT_RES_NONE = 0xFF, /* no command since start-up */
} lift_result_t;

/* Latched fault bitmask (decision D9). Bit n-1 is fault Fn. */
#define LIFT_FAULT_F01_BOTH_LIMITS     (1u << 0)
#define LIFT_FAULT_F02_HALL_MISMATCH   (1u << 1)
#define LIFT_FAULT_F03_OVERCURRENT     (1u << 2)
#define LIFT_FAULT_F04_MOVE_TIMEOUT    (1u << 3)
#define LIFT_FAULT_F05_POSITION_LOST   (1u << 4)
#define LIFT_FAULT_F06_STORED_RECORD   (1u << 5) /* unused in v1: no stored position */
#define LIFT_FAULT_F07_SUPERVISION     (1u << 6)
#define LIFT_FAULT_F08_STANDSTILL      (1u << 7)
/* Proposal, not in the D1 to D20 list: two or more stop sensors active
 * (P-03 item 24 says "two active sensors is a fault" without an ID). */
#define LIFT_FAULT_F09_STOP_SENSORS    (1u << 8)
#define LIFT_FAULT_ALL                 0x01FFu

/* Faults after which the Hall reference can no longer be trusted. */
#define LIFT_FAULTS_INVALIDATE_POSITION \
    (LIFT_FAULT_F01_BOTH_LIMITS | LIFT_FAULT_F02_HALL_MISMATCH | \
     LIFT_FAULT_F05_POSITION_LOST | LIFT_FAULT_F09_STOP_SENSORS)

#define LIFT_STOP_NONE 0xFFu
#define LIFT_POSITION_UNKNOWN 0xFFFFu

typedef enum { LIFT_DIR_NONE = 0, LIFT_DIR_UP = 1, LIFT_DIR_DOWN = -1 } lift_dir_t;

/* H-bridge command for the board port (IFX007T half-bridges). */
typedef struct {
    bool enable;            /* false: both half-bridges inhibited (high impedance) */
    lift_dir_t dir;         /* direction while enable and !brake */
    uint16_t duty_permille; /* 0..1000 */
    bool brake;             /* both low sides on; only when stop_mode_brake */
} lift_hbridge_cmd_t;

/* Raw inputs sampled by the board port each control tick. Logical polarity:
 * true means "active" (target present, switch made, supply good). The port
 * applies the electrical polarity (NPN NO through the divider pulls the pin
 * low when the target is present, P-03 item 30 A7). */
typedef struct {
    uint32_t now_ms;
    int32_t hall_count;           /* free-running signed count from TIM5 */
    bool hall_supply_ok;          /* Hall supply monitor (functional status) */
    bool motor_power_present;     /* motor-supply sense (functional status) */
    bool stop_sensor[LIFT_NUM_STOPS];
    bool lower_limit;             /* used only if end_switches_wired */
    bool upper_limit;
    uint16_t motor_current_ma;    /* shared current sense, PC1 */
} lift_inputs_t;

typedef struct {
    lift_config_t cfg;

    lift_state_t state;
    uint16_t faults;              /* latched bitmask */
    uint16_t new_faults;          /* bits raised since the last lift_take_new_faults() */
    bool homed;                   /* recovery_required = !homed (D9) */
    bool position_valid;

    /* Hall reference: position_nm = (count - ref_count) * nm_per_count + ref_pos_nm */
    int32_t ref_count;
    int64_t ref_pos_nm;
    int32_t last_count;
    uint32_t last_count_change_ms;

    /* Debounced inputs */
    debounce_t stop_db[LIFT_NUM_STOPS];
    debounce_t lower_db;
    debounce_t upper_db;
    bool stop_prev[LIFT_NUM_STOPS];
    bool motor_power;
    bool hall_supply_ok;
    uint16_t current_ma;

    /* Motion */
    lift_dir_t dir;
    uint8_t target_stop;          /* LIFT_STOP_NONE if none */
    int64_t target_nm;
    bool drive_on;
    bool arriving;                /* target reached, waiting for standstill */
    uint32_t drive_on_ms;
    uint32_t drive_off_ms;
    uint32_t move_deadline_ms;
    bool f08_armed;

    /* Hall window */
    uint32_t win_start_ms;
    int32_t win_start_count;
    uint32_t last_pulse_ms;
    int32_t dir_ref_count;

    /* Current limit */
    uint32_t over_since_ms;
    bool over_active;

    /* Velocity estimate */
    uint32_t vel_t0_ms;
    int32_t vel_c0;
    int16_t velocity_01mm_s;

    /* Command bookkeeping */
    bool any_cmd;
    uint8_t last_seq;
    uint8_t last_op;
    uint8_t last_stop_id;
    uint8_t cmd_result;
    uint32_t last_keepalive_ms;
    bool supervision_lost;        /* core heartbeat lost (heartbeat consumer) */

    uint32_t init_ms;
    uint32_t now_ms;
    bool started;

    lift_hbridge_cmd_t hb;
} lift_t;

/* Snapshot used by the CANopen layer and by tests. */
typedef struct {
    uint8_t state;
    uint8_t flags;
    uint16_t position_01mm;       /* 0.1 mm above the lower end, 0xFFFF unknown */
    uint8_t active_stop_id;       /* confirmed occupied stop, 0xFF none */
    uint8_t last_seq;
    uint8_t cmd_result;
    uint8_t target_stop_id;       /* target of the move in progress, 0xFF none */
    uint8_t stop_sensors;         /* bit k = stop sensor k active (debounced) */
    uint16_t motor_current_ma;
    int16_t velocity_01mm_s;      /* signed, positive = up */
    uint16_t faults;
    bool hall_supply_ok;
} lift_status_t;

/* Initialise with a configuration (copied). now_ms is the boot time. */
void lift_init(lift_t *l, const lift_config_t *cfg, uint32_t now_ms);

/* One control tick. Must be called periodically (1 ms to 10 ms). Returns the
 * H-bridge command to apply now. */
const lift_hbridge_cmd_t *lift_step(lift_t *l, const lift_inputs_t *in);

/* Handle a LiftCommand (RPDO1). A frame with the same seq and identical
 * content as the previous one is a keep-alive: it refreshes the command
 * deadline and is not executed again. Returns the resulting cmd_result. */
uint8_t lift_command(lift_t *l, uint8_t seq, uint8_t op, uint8_t stop_id, uint32_t now_ms);

/* The core heartbeat (heartbeat consumer) was lost or came back. While lost,
 * any move or homing stops with F07 and new moves are refused. */
void lift_set_supervision_lost(lift_t *l, bool lost);

/* True while the motor is driven or the column is not yet at standstill
 * after a drive. */
bool lift_motion_in_progress(const lift_t *l);

void lift_get_status(const lift_t *l, lift_status_t *st);

/* Returns fault bits raised since the previous call and clears that record
 * (used to emit EMCY once per new fault). */
uint16_t lift_take_new_faults(lift_t *l);

/* Position in nanometres above the lower end; only meaningful while
 * position_valid. */
int64_t lift_position_nm(const lift_t *l);

#ifdef __cplusplus
}
#endif

#endif /* CTL017_LIFT_H */
