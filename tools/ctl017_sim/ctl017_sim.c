/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * SocketCAN harness of the CTL-017 CANopen application layer (Linux only).
 *
 * It runs ctl017_app on a vcan interface with the lift replaced by a fixed
 * status: no lift core, no plant model, nothing moves. It exists to check
 * the wire behaviour (NMT, heartbeat, SDO, PDO, EMCY) against a real
 * SocketCAN stack. The config values are TEST VALUES given on the command
 * line; there are no defaults.
 *
 *   ctl017_sim --iface vcan0 --base-node 1 --hb-ms 100 --hbc-ms 300 \
 *              --inhibit 100 --event-ms 100 --rpdo1-ms 300 \
 *              --product 0x11 --revision 0x10000 [--duration-s 5] [--faults 0x4]
 */
#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "ctl017/ctl017_app.h"

static volatile sig_atomic_t stop_flag;

static void on_signal(int s)
{
    (void)s;
    stop_flag = 1;
}

static uint32_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
}

static int open_can(const char *iface)
{
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        perror("socket");
        return -1;
    }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, iface, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        close(fd);
        return -1;
    }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(fd);
        return -1;
    }
    return fd;
}

static bool can_send(void *ctx, const co_frame_t *f)
{
    int fd = *(int *)ctx;
    struct can_frame cf;
    memset(&cf, 0, sizeof(cf));
    cf.can_id = f->id;
    cf.can_dlc = f->dlc;
    memcpy(cf.data, f->data, f->dlc);
    return write(fd, &cf, sizeof(cf)) == (ssize_t)sizeof(cf);
}

static unsigned long num(const char *s)
{
    return strtoul(s, NULL, 0);
}

static ctl017_app_t app;

int main(int argc, char **argv)
{
    const char *iface = "vcan0";
    double duration_s = 0;
    uint16_t faults = 0;
    ctl017_app_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    for (int i = 1; i + 1 < argc; i += 2) {
        const char *k = argv[i];
        const char *v = argv[i + 1];
        if (strcmp(k, "--iface") == 0) {
            iface = v;
        } else if (strcmp(k, "--duration-s") == 0) {
            duration_s = atof(v);
        } else if (strcmp(k, "--base-node") == 0) {
            cfg.base_node_id = (uint8_t)num(v);
        } else if (strcmp(k, "--hb-ms") == 0) {
            cfg.hb_producer_ms = (uint16_t)num(v);
        } else if (strcmp(k, "--hbc-ms") == 0) {
            cfg.hb_consumer_ms = (uint16_t)num(v);
        } else if (strcmp(k, "--inhibit") == 0) {
            cfg.tpdo_inhibit_100us[0] = cfg.tpdo_inhibit_100us[1] = (uint16_t)num(v);
        } else if (strcmp(k, "--event-ms") == 0) {
            cfg.tpdo_event_ms[0] = cfg.tpdo_event_ms[1] = (uint16_t)num(v);
        } else if (strcmp(k, "--rpdo1-ms") == 0) {
            cfg.rpdo1_event_timer_ms = (uint16_t)num(v);
        } else if (strcmp(k, "--product") == 0) {
            cfg.product_code = (uint32_t)num(v);
        } else if (strcmp(k, "--revision") == 0) {
            cfg.revision = (uint32_t)num(v);
        } else if (strcmp(k, "--faults") == 0) {
            faults = (uint16_t)num(v);
        } else {
            fprintf(stderr, "unknown option %s\n", k);
            return 2;
        }
    }
    int fd = open_can(iface);
    if (fd < 0) {
        return 1;
    }
    signal(SIGTERM, on_signal);
    signal(SIGINT, on_signal);
    ctl017_app_result_t r = ctl017_app_init(&app, &cfg, can_send, &fd, now_ms());
    if (r != CTL017_APP_OK) {
        fprintf(stderr, "ctl017_app_init: %s, staying silent\n",
                r == CTL017_APP_NOT_CONFIGURED ? "not configured" : "OD error");
    }
    /* Fixed status: unknown position, no motion (there is no lift here). */
    ctl017_lift_status_t st;
    ctl017_lift_status_unknown(&st);
    st.faults = faults;
    uint32_t t0 = now_ms();
    while (!stop_flag && (duration_s <= 0 || (now_ms() - t0) < (uint32_t)(duration_s * 1000.0))) {
        struct pollfd p = {fd, POLLIN, 0};
        int n = poll(&p, 1, 1);
        if (n > 0 && (p.revents & POLLIN) != 0) {
            struct can_frame cf;
            if (read(fd, &cf, sizeof(cf)) == (ssize_t)sizeof(cf)) {
                co_frame_t f;
                memset(&f, 0, sizeof(f));
                f.ext = (cf.can_id & CAN_EFF_FLAG) != 0u;
                f.rtr = (cf.can_id & CAN_RTR_FLAG) != 0u;
                f.id = cf.can_id & (f.ext ? CAN_EFF_MASK : CAN_SFF_MASK);
                f.dlc = cf.can_dlc;
                memcpy(f.data, cf.data, sizeof(f.data));
                ctl017_app_rx(&app, &f, now_ms());
            }
        } else if (n < 0 && errno != EINTR) {
            perror("poll");
            break;
        }
        ctl017_app_set_status(&app, &st);
        ctl017_app_step(&app, now_ms());
        if (ctl017_app_rx_view(&app)->link.reset_node_requested) {
            (void)ctl017_app_init(&app, &cfg, can_send, &fd, now_ms());
        }
    }
    close(fd);
    return 0;
}
