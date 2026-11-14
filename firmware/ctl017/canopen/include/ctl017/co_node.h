/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * Minimal CiA 301 CANopen slave for CTL-017: NMT slave, heartbeat producer
 * and one heartbeat consumer, expedited SDO server, static-mapped TPDOs and
 * RPDOs (mapping read from the object dictionary), EMCY producer with the
 * pre-defined error field 0x1003.
 *
 * Portable C11: frames in and out through a send callback, time from the
 * caller. Not supported: segmented and block SDO, SYNC, TIME, LSS, dynamic
 * PDO mapping, non-volatile storage (0x1010/0x1011).
 */
#ifndef CTL017_CO_NODE_H
#define CTL017_CO_NODE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Frames ---------------------------------------------------------------- */
typedef struct {
    uint32_t id;      /* identifier as received; must be <= 0x7FF for CANopen */
    bool ext;         /* 29-bit identifier: never a CANopen object here */
    bool rtr;         /* remote frame: carries no data, never processed */
    uint8_t dlc;      /* 0..8 */
    uint8_t data[8];
} co_frame_t;

#define CO_ID_NMT       0x000u
#define CO_ID_EMCY      0x080u
#define CO_ID_TPDO1     0x180u
#define CO_ID_RPDO1     0x200u
#define CO_ID_TPDO2     0x280u
#define CO_ID_RPDO2     0x300u
#define CO_ID_SDO_TX    0x580u
#define CO_ID_SDO_RX    0x600u
#define CO_ID_HEARTBEAT 0x700u

/* ---- NMT ------------------------------------------------------------------- */
typedef enum {
    CO_NMT_INITIALISING = 0x00,
    CO_NMT_STOPPED = 0x04,
    CO_NMT_OPERATIONAL = 0x05,
    CO_NMT_PRE_OPERATIONAL = 0x7F,
} co_nmt_state_t;

#define CO_NMT_CS_START       0x01u
#define CO_NMT_CS_STOP        0x02u
#define CO_NMT_CS_PREOP       0x80u
#define CO_NMT_CS_RESET_NODE  0x81u
#define CO_NMT_CS_RESET_COMM  0x82u

/* ---- Object dictionary ----------------------------------------------------- */
/* CiA 301 data type codes (used in the EDS). */
typedef enum {
    CO_T_BOOLEAN = 0x0001,
    CO_T_INT8 = 0x0002,
    CO_T_INT16 = 0x0003,
    CO_T_INT32 = 0x0004,
    CO_T_UNSIGNED8 = 0x0005,
    CO_T_UNSIGNED16 = 0x0006,
    CO_T_UNSIGNED32 = 0x0007,
} co_type_t;

typedef enum {
    CO_A_CONST = 0, /* read only, never changes */
    CO_A_RO,        /* read only, changes at run time */
    CO_A_RW,        /* read and write */
    CO_A_RWW,       /* written by RPDO; SDO write refused (0x08000021) */
    CO_A_RW_PREOP,  /* writable by SDO in PRE-OPERATIONAL only */
} co_access_t;

typedef struct co_od_entry co_od_entry_t;
typedef struct co_node co_node_t;

/* Write hook: validate and apply (including storing the value); returns 0
 * or an SDO abort code. When NULL the value is stored as is. */
typedef uint32_t (*co_write_fn)(co_node_t *co, const co_od_entry_t *e, uint32_t value);

struct co_od_entry {
    uint16_t index;
    uint8_t sub;
    uint8_t type;      /* co_type_t */
    uint8_t access;    /* co_access_t */
    bool pdo_mappable;
    void *data;        /* points to a variable of the matching C type */
    const char *name;  /* for the EDS and the documentation dump */
    co_write_fn on_write;
    /* Name of the integration parameter that sets this value, or NULL when
     * the value is fixed by the object dictionary. Documentation only. */
    const char *param;
};

uint8_t co_type_size(uint8_t type);
const co_od_entry_t *co_od_find(const co_od_entry_t *od, size_t n, uint16_t index, uint8_t sub);
bool co_od_index_exists(const co_od_entry_t *od, size_t n, uint16_t index);
uint32_t co_od_get(const co_od_entry_t *e);
void co_od_set(const co_od_entry_t *e, uint32_t v);

/* ---- SDO abort codes ------------------------------------------------------- */
#define CO_SDO_ABORT_CMD            0x05040001u
#define CO_SDO_ABORT_UNSUPPORTED    0x06010000u
#define CO_SDO_ABORT_WRITEONLY      0x06010001u
#define CO_SDO_ABORT_READONLY       0x06010002u
#define CO_SDO_ABORT_NO_OBJECT      0x06020000u
#define CO_SDO_ABORT_TYPE_LENGTH    0x06070010u
#define CO_SDO_ABORT_NO_SUB         0x06090011u
#define CO_SDO_ABORT_PARAM_INCOMPAT 0x06040043u
#define CO_SDO_ABORT_VALUE_RANGE    0x06090030u
#define CO_SDO_ABORT_GENERAL        0x08000000u
#define CO_SDO_ABORT_LOCAL_CONTROL  0x08000021u
#define CO_SDO_ABORT_DEVICE_STATE   0x08000022u

/* ---- PDO ------------------------------------------------------------------- */
#define CO_MAX_PDO_MAP 8u

typedef struct {
    uint16_t cob_id;
    uint16_t *inhibit_100us;   /* OD variable (TPDO) */
    uint16_t *event_ms;        /* OD variable (TPDO event timer) */
    const uint32_t *map;       /* mapping entries, 0xIIIISSLL */
    uint8_t map_len;
    uint8_t change_mask;       /* bit b set: byte b counts for change detection */
} co_tpdo_cfg_t;

typedef struct {
    uint16_t cob_id;
    const uint32_t *map;
    uint8_t map_len;
} co_rpdo_cfg_t;

typedef struct {
    co_tpdo_cfg_t cfg;
    const co_od_entry_t *ent[CO_MAX_PDO_MAP];
    uint8_t len;
    uint8_t last[8];
    bool sent_once;
    uint32_t last_tx_ms;
} co_tpdo_t;

typedef struct {
    co_rpdo_cfg_t cfg;
    const co_od_entry_t *ent[CO_MAX_PDO_MAP];
    uint8_t len;
} co_rpdo_t;

#define CO_MAX_TPDO 2u
#define CO_MAX_RPDO 3u
#define CO_ERR_HISTORY 8u

/* ---- Callbacks to the application ----------------------------------------- */
typedef struct {
    bool (*send)(void *ctx, const co_frame_t *f);
    /* Called before every TPDO build so the application can refresh the
     * mapped OD variables. */
    void (*before_tpdo)(void *ctx, uint8_t n);
    /* Called after a TPDO frame was sent. */
    void (*tpdo_sent)(void *ctx, uint8_t n);
    /* Called after a valid RPDO was written into the mapped OD variables. */
    void (*rpdo_received)(void *ctx, uint8_t n, uint32_t now_ms);
    void (*nmt_changed)(void *ctx, co_nmt_state_t from, co_nmt_state_t to);
    void (*reset_node)(void *ctx);
    /* NMT reset communication: the application restores the power-on values
     * of the communication objects it owns (0x1000 to 0x1FFF). Called before
     * the boot-up message is sent again. */
    void (*reset_comm)(void *ctx);
    /* Heartbeat consumer events (data only, see co_hbc_event_t). */
    void (*hb_consumer)(void *ctx, int event);
} co_callbacks_t;

typedef enum {
    CO_HBC_OFF = 0,     /* not configured (time 0 or node 0) */
    CO_HBC_WAITING,     /* configured, no heartbeat seen since start or reconfiguration */
    CO_HBC_ACTIVE,      /* heartbeat received within the consumer time */
    CO_HBC_LOST,        /* consumer time elapsed since the last heartbeat */
} co_hbc_state_t;

/* Events reported through co_callbacks_t.hb_consumer. The CANopen layer
 * only observes; what the node does about them is not decided here. */
typedef enum {
    CO_HBC_EV_TIMEOUT = 1,     /* ACTIVE to LOST */
    CO_HBC_EV_RECOVERED = 2,   /* LOST to ACTIVE after a valid heartbeat */
    CO_HBC_EV_REMOTE_BOOT = 3, /* boot-up message (state 0x00) of the monitored node */
    CO_HBC_EV_STARTED = 4,     /* WAITING to ACTIVE, first heartbeat */
} co_hbc_event_t;

/* Communication error bits kept by the node (error register bit 4). */
#define CO_COMM_ERR_RPDO_LEN  (1u << 0) /* EMCY 0x8210: RPDO shorter than its mapping */
#define CO_COMM_ERR_HB        (1u << 1) /* EMCY 0x8130: heartbeat consumer timeout */
#define CO_EMCY_PDO_LENGTH    0x8210u
#define CO_EMCY_PDO_LENGTH_EX 0x8220u
#define CO_EMCY_HEARTBEAT     0x8130u
#define CO_EMCY_NO_ERROR      0x0000u

struct co_node {
    uint8_t node_id;
    co_nmt_state_t nmt;
    const co_od_entry_t *od;
    size_t od_len;
    co_callbacks_t cb;
    void *ctx;

    /* OD-backed communication variables */
    uint16_t hb_producer_ms;      /* 0x1017 */
    uint32_t hb_consumer_entry;   /* 0x1016:01 = node << 16 | time_ms */
    uint16_t hb_producer_default; /* power-on values restored by reset communication */
    uint32_t hb_consumer_default;
    uint8_t error_register;       /* 0x1001 = app_error_register | communication bits */
    uint8_t app_error_register;   /* set with co_set_app_error_register() */
    uint8_t comm_errors;          /* CO_COMM_ERR_* */
    uint8_t err_count;            /* 0x1003:00 */
    uint32_t err_history[CO_ERR_HISTORY]; /* 0x1003:01..08 */

    uint32_t last_hb_tx_ms;
    co_hbc_state_t hbc;
    uint32_t hbc_last_rx_ms;
    uint8_t hbc_remote_state;
    uint32_t hbc_remote_boots;    /* boot-up messages seen from the monitored node */

    co_tpdo_t tpdo[CO_MAX_TPDO];
    uint8_t n_tpdo;
    co_rpdo_t rpdo[CO_MAX_RPDO];
    uint8_t n_rpdo;

    uint32_t now_ms;
    uint32_t rx_pdo_len_errors;   /* RPDO shorter than the mapping, not processed */
    uint32_t rx_pdo_len_excess;   /* RPDO longer than the mapping, first bytes used */
    uint32_t rx_malformed;        /* frames dropped: RTR, 29-bit, DLC > 8, bad NMT/SDO/heartbeat length or value */
    uint32_t tx_failures;         /* send callback refused a frame; retried where periodic */
};

/* Standard write hooks for the communication objects. */
uint32_t co_write_err_count(co_node_t *co, const co_od_entry_t *e, uint32_t value); /* 0x1003:00 */
uint32_t co_write_hb_consumer(co_node_t *co, const co_od_entry_t *e, uint32_t value); /* 0x1016:01 */

/* Prepares the node. The OD must outlive the node. Returns false if a PDO
 * mapping names an object that is missing, not mappable, of the wrong
 * length, or if a PDO exceeds 8 bytes. */
bool co_init(co_node_t *co, uint8_t node_id, const co_od_entry_t *od, size_t od_len,
             const co_callbacks_t *cb, void *ctx);
bool co_add_tpdo(co_node_t *co, const co_tpdo_cfg_t *cfg);
bool co_add_rpdo(co_node_t *co, const co_rpdo_cfg_t *cfg);

/* Sets the power-on values of 0x1017 and 0x1016:01 (restored by NMT reset
 * communication) and applies them. Call before co_start(). */
void co_set_comm_defaults(co_node_t *co, uint16_t hb_producer_ms, uint32_t hb_consumer_entry);

/* Sends the boot-up message and enters PRE-OPERATIONAL. */
void co_start(co_node_t *co, uint32_t now_ms);

/* Application part of the error register 0x1001 (bits other than 4). When
 * the combined register returns to 0 the node sends EMCY 0x0000. */
void co_set_app_error_register(co_node_t *co, uint8_t bits);

void co_process(co_node_t *co, const co_frame_t *f, uint32_t now_ms);
void co_step(co_node_t *co, uint32_t now_ms);

/* EMCY: error code, 5 manufacturer bytes. Appends to 0x1003 unless code is
 * 0 (error reset). Sent in PRE-OPERATIONAL and OPERATIONAL only. */
void co_emcy(co_node_t *co, uint16_t code, const uint8_t mfr[5]);

/* Packs a TPDO from the OD (for tests and tools). Returns the length. */
uint8_t co_tpdo_pack(const co_tpdo_t *t, uint8_t out[8]);

#ifdef __cplusplus
}
#endif

#endif /* CTL017_CO_NODE_H */
