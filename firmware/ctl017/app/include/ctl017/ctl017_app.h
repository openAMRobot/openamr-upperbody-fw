/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 CANopen application layer: binds the object dictionary
 * (docs/ctl017_object_dictionary.md) to the CANopen slave and to the plain
 * data interface of firmware/ctl017/iface.
 *
 * What it does: decodes RPDOs and SDO writes into ctl017_iface_rx_t records,
 * encodes the status supplied by the lift owner into TPDO1, TPDO2, 0x2002,
 * 0x1001 and EMCY, and reports link observations.
 * What it never does: interpret a lift command, start or stop anything,
 * supervise a command deadline or a heartbeat, or act on the permit record.
 * Those belong to the lift owner's code, which reads the records.
 *
 * Not configured is a state: every timing and identity value comes from
 * ctl017_app_config_t, which has no defaults. ctl017_app_init() returns
 * CTL017_APP_NOT_CONFIGURED and the node stays silent if any is unset.
 */
#ifndef CTL017_APP_H
#define CTL017_APP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ctl017/co_node.h"
#include "ctl017/lift_iface.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CTL017_NODE_ID 0x10u /* P-03 item 29, D-11 */
#define CTL017_OD_MAX  160u

#ifdef CTL017_PERMIT_OD
/* Named parameters of the permit contract (docs/ctl017_permit_contract.md).
 * No value is proposed here; the build must define every one. */
#if !defined(CTL017_PERMIT_OD_INDEX) || !defined(CTL017_PERMIT_COB_ID) || \
    !defined(CTL017_PERMIT_FLAG_BITS) || !defined(CTL017_PERMIT_BOOT_BITS) || \
    !defined(CTL017_PERMIT_SEQ_BITS) || !defined(CTL017_PERMIT_REASON_BITS)
#error "CTL017_PERMIT_OD needs CTL017_PERMIT_OD_INDEX, _COB_ID, _FLAG_BITS, _BOOT_BITS, _SEQ_BITS and _REASON_BITS (TODO(Evans))"
#endif
#endif

typedef enum {
    CTL017_APP_OK = 0,
    CTL017_APP_NOT_CONFIGURED = 1, /* a config value is unset: nothing is sent */
    CTL017_APP_OD_ERROR = 2,       /* OD or PDO mapping inconsistent (build error) */
} ctl017_app_result_t;

/* Every field is TODO(Evans): a value that is 0 is "not set". The values
 * documented in docs/ctl017_object_dictionary.md are proposals. */
typedef struct {
    uint8_t base_node_id;          /* node whose heartbeat 0x1016:01 observes */
    uint16_t hb_producer_ms;       /* 0x1017 power-on value */
    uint16_t hb_consumer_ms;       /* 0x1016:01 time part, power-on value */
    uint16_t tpdo_inhibit_100us[2];/* 0x1800:03, 0x1801:03 */
    uint16_t tpdo_event_ms[2];     /* 0x1800:05, 0x1801:05 */
    uint16_t rpdo1_event_timer_ms; /* 0x1400:05: stored and exposed, not supervised here */
    uint32_t product_code;         /* 0x1018:02 */
    uint32_t revision;             /* 0x1018:03 */
    uint32_t vendor_id;            /* 0x1018:01; 0 is allowed (no CiA vendor ID yet) */
    uint32_t serial;               /* 0x1018:04; 0 is allowed */
#ifdef CTL017_PERMIT_OD
    uint16_t permit_event_timer_ms;/* 0x1402:05: stored and exposed, not supervised here */
#endif
} ctl017_app_config_t;

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

#ifdef CTL017_PERMIT_OD
typedef union {
    uint8_t u8;
    uint16_t u16;
    uint32_t u32;
} ctl017_od_var_t;

typedef struct {
    ctl017_od_var_t f[4];               /* flag, boot id, sequence, reason */
    const co_od_entry_t *e[4];          /* their OD entries, read with co_od_get() */
} ctl017_permit_od_t;
#endif

typedef bool (*ctl017_send_fn)(void *ctx, const co_frame_t *f);

typedef struct {
    co_node_t co;
    ctl017_app_config_t cfg;
    ctl017_app_result_t init_result;

    co_od_entry_t od[CTL017_OD_MAX];
    size_t od_len;

    /* OD variables */
    uint32_t device_type;
    uint32_t cob_emcy;
    uint8_t n_sub_1003, n_sub_1016, n_sub_1018, n_sub_rpdo_comm, n_sub_tpdo_comm;
    uint32_t vendor_id, product_code, revision, serial;
    uint32_t rpdo_cob[3], tpdo_cob[2];
    uint8_t trans_type;
    uint16_t rpdo_event_ms[3];
    uint16_t tpdo_inhibit[2], tpdo_event[2];
    uint8_t n_map_rpdo[3], n_map_tpdo[2];
    uint32_t map_rpdo1[3], map_rpdo2[3], map_rpdo3[4], map_tpdo1[7], map_tpdo2[6];
    uint8_t n_sub_2000, n_sub_2001, n_sub_2010, n_sub_2011, n_sub_2100, n_sub_2101, n_sub_2300;
    ctl017_status1_t st1;
    ctl017_status2_t st2;
    uint16_t faults;                 /* 0x2002 */
    ctl017_lift_cmd_t cmd;           /* RPDO1 landing area */
    ctl017_aux_cmd_t auxcmd;         /* RPDO2 and SDO landing area */
    int32_t hall_count_raw;
    uint8_t reset_cause;
    uint32_t pdo_len_errors;
#ifdef CTL017_PERMIT_OD
    uint8_t n_sub_permit;
    ctl017_permit_od_t permit_od;    /* RPDO3 landing area */
#endif

    /* Lift parameter storage and its validator (lift owner). */
    ctl017_lift_params_t params;
    ctl017_params_valid_fn params_valid;
    void *params_ctx;

    /* Interface records */
    ctl017_iface_rx_t rx;
    ctl017_lift_status_t status;
    uint16_t last_faults;
    uint32_t aux_range_errors;       /* RPDO2 frames with a field out of range, not applied */

    ctl017_send_fn send;
    void *send_ctx;
    uint32_t now_ms;
} ctl017_app_t;

/* Validates the config, builds the OD, sends the boot-up message. With
 * CTL017_APP_NOT_CONFIGURED or CTL017_APP_OD_ERROR the node never sends and
 * ignores every frame. */
ctl017_app_result_t ctl017_app_init(ctl017_app_t *a, const ctl017_app_config_t *cfg,
                                    ctl017_send_fn send, void *send_ctx, uint32_t now_ms);

/* True when every required config field is set. */
bool ctl017_app_config_complete(const ctl017_app_config_t *cfg);

/* Registers the lift owner's parameter validator. Without one, SDO writes
 * to 0x2100 and 0x2101 are refused (abort 0x08000022). */
void ctl017_app_set_params_validator(ctl017_app_t *a, ctl017_params_valid_fn fn, void *ctx);

/* One received frame. */
void ctl017_app_rx(ctl017_app_t *a, const co_frame_t *f, uint32_t now_ms);

/* Latest status from the lift owner; copied, sent on the next step. */
void ctl017_app_set_status(ctl017_app_t *a, const ctl017_lift_status_t *st);

/* Periodic service: heartbeat, TPDOs, EMCY. */
void ctl017_app_step(ctl017_app_t *a, uint32_t now_ms);

/* What arrived, as plain data, for the lift owner. */
const ctl017_iface_rx_t *ctl017_app_rx_view(const ctl017_app_t *a);
const ctl017_lift_params_t *ctl017_app_params(const ctl017_app_t *a);

/* Builds the OD into a->od (called by ctl017_app_init). */
void ctl017_od_build(ctl017_app_t *a, bool *ok);

/* Encoders from the interface status to the mapped variables. */
void ctl017_fill_status1(const ctl017_lift_status_t *st, uint8_t counter, ctl017_status1_t *out);
void ctl017_fill_status2(const ctl017_lift_status_t *st, ctl017_status2_t *out);

/* Application part of the error register 0x1001 for a fault bitmask. */
uint8_t ctl017_error_register(uint16_t faults);

#ifdef __cplusplus
}
#endif

#endif /* CTL017_APP_H */
