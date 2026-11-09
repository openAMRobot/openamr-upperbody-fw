/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 application: binds the lift core and the auxiliary I/O to the
 * CANopen slave and its object dictionary (docs/ctl017_object_dictionary.md).
 * Board independent: the board port feeds frames, inputs and the time, and
 * applies the returned outputs.
 */
#ifndef CTL017_APP_H
#define CTL017_APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ctl017/aux_io.h"
#include "ctl017/co_node.h"
#include "ctl017/lift.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CTL017_NODE_ID        0x10u /* decision D1/P-03 item 29 */
#define CTL017_CORE_NODE_ID   0x01u /* base controller (Alex 24 Oct) */
#define CTL017_HB_PRODUCER_MS 100u  /* D15 */
#define CTL017_HB_CONSUMER_MS 300u  /* D15 */
#define CTL017_OD_MAX         128u

/* Mapped OD variables. Field order is the PDO byte order. */
typedef struct {
    uint8_t state;
    uint8_t flags;
    uint16_t position_01mm;
    uint8_t active_stop_id;
    uint8_t last_seq;
    uint8_t cmd_result;
    uint8_t status_counter;
} ctl017_status1_t;   /* 0x2000, TPDO1 */

/* 0x2001:01 inputs bits */
#define CTL017_IN_STOP0       (1u << 0)
#define CTL017_IN_STOP1       (1u << 1)
#define CTL017_IN_STOP2       (1u << 2)
#define CTL017_IN_ESTOP2      (1u << 3) /* second E-stop spare NO closed = pressed */
#define CTL017_IN_HALL_SUPPLY (1u << 4)
#define CTL017_IN_SPARE1      (1u << 5)
#define CTL017_IN_SPARE2      (1u << 6)
#define CTL017_IN_SPARE3      (1u << 7)
/* 0x2001:06 aux flags bits */
#define CTL017_AUX_F09        (1u << 0)
#define CTL017_AUX_LIGHT_ON   (1u << 1)
#define CTL017_AUX_GETAC_EN   (1u << 2)

typedef struct {
    uint8_t inputs;
    uint8_t target_stop_id;
    uint16_t motor_current_ma;
    int16_t velocity_01mm_s;
    uint8_t faults_lo;    /* F01..F08 */
    uint8_t aux_flags;
} ctl017_status2_t;   /* 0x2001, TPDO2 */

typedef struct {
    uint8_t seq;
    uint8_t op;
    uint8_t stop_id;
} ctl017_lift_cmd_t;  /* 0x2010, RPDO1 */

typedef struct {
    uint8_t head_light_on;
    uint8_t head_light_dim_pct;
    uint8_t getac_enable;
} ctl017_aux_cmd_t;   /* 0x2011, RPDO2 */

typedef struct {
    lift_hbridge_cmd_t hbridge;
    aux_outputs_t aux;
    bool reset_requested;   /* NMT reset node: the port performs a system reset */
} ctl017_outputs_t;

typedef bool (*ctl017_send_fn)(void *ctx, const co_frame_t *f);

typedef struct {
    lift_t lift;
    aux_io_t aux;
    co_node_t co;

    co_od_entry_t od[CTL017_OD_MAX];
    size_t od_len;

    /* OD variables */
    uint32_t device_type;
    uint32_t cob_emcy;
    uint8_t n_sub_1003, n_sub_1016, n_sub_1018, n_sub_rpdo_comm, n_sub_tpdo_comm;
    uint32_t vendor_id, product_code, revision, serial;
    uint32_t rpdo_cob[2], tpdo_cob[2];
    uint8_t trans_type;
    uint16_t rpdo2_event_ms;
    uint16_t tpdo_inhibit[2], tpdo_event[2];
    uint8_t n_map_rpdo[2], n_map_tpdo[2];
    uint32_t map_rpdo1[3], map_rpdo2[3], map_tpdo1[7], map_tpdo2[6];
    uint8_t n_sub_2000, n_sub_2001, n_sub_2010, n_sub_2011, n_sub_2100, n_sub_2101, n_sub_2300;
    ctl017_status1_t st1;
    ctl017_status2_t st2;
    uint16_t faults;
    ctl017_lift_cmd_t cmd;
    ctl017_aux_cmd_t auxcmd;
    int32_t hall_count_raw;
    uint8_t reset_cause;
    uint32_t pdo_len_errors;

    ctl017_send_fn send;
    void *send_ctx;
    uint16_t last_faults;
    bool reset_requested;
    bool od_ok;
    uint32_t now_ms;
    ctl017_outputs_t out;
} ctl017_app_t;

/* Initialises the core, aux I/O, object dictionary and CANopen node, sends
 * the boot-up message. Returns false if the OD or PDO mapping is
 * inconsistent (a build error, caught by the host tests). */
bool ctl017_app_init(ctl017_app_t *a, const lift_config_t *lcfg, const aux_config_t *acfg,
                     ctl017_send_fn send, void *send_ctx, uint32_t now_ms);

void ctl017_app_rx(ctl017_app_t *a, const co_frame_t *f, uint32_t now_ms);

/* One control tick. */
const ctl017_outputs_t *ctl017_app_step(ctl017_app_t *a, const lift_inputs_t *lin,
                                        const aux_inputs_t *ain);

/* Builds the OD into a->od (called by ctl017_app_init). */
void ctl017_od_build(ctl017_app_t *a);

/* Fill the mapped status variables from the core (lift_pdo.c). */
void ctl017_fill_status1(const lift_status_t *st, uint8_t counter, ctl017_status1_t *out);
void ctl017_fill_status2(const lift_status_t *st, const aux_io_t *aux, ctl017_status2_t *out);

/* Error register 0x1001 for a fault bitmask. */
uint8_t ctl017_error_register(uint16_t faults);

#ifdef __cplusplus
}
#endif

#endif /* CTL017_APP_H */
