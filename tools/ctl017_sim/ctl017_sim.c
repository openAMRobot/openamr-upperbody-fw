/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 OpenAMRobot (Botshare LTD)
 *
 * CTL-017 software-in-the-loop node for Linux SocketCAN (vcan or a real
 * adapter): runs the unchanged firmware application (lift core, aux I/O,
 * CANopen slave) against the kinematic lift plant in real time, 1 ms tick.
 *
 * Test-only control frames on CAN-ID 0x7F0 (never used by the firmware):
 *   byte 0 = 1: motor power, byte 1 = 0/1
 *   byte 0 = 2: Hall supply, byte 1 = 0/1
 *   byte 0 = 3: stop sensor byte 1 stuck on, byte 2 = 0/1
 *   byte 0 = 4: stop sensor byte 1 stuck off, byte 2 = 0/1
 *   byte 0 = 5: column jammed, byte 1 = 0/1
 *   byte 0 = 6: second E-stop NO closed, byte 1 = 0/1
 *
 * Usage: ctl017_sim [--iface vcan0] [--start-mm 180] [--duration-s 0]
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "ctl017/ctl017_app.h"
#include "lift_plant.h"

#define SIM_CTRL_ID 0x7F0u

static volatile sig_atomic_t g_stop;

static void on_signal(int s)
{
    (void)s;
    g_stop = 1;
}

static uint32_t mono_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u);
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

static int open_can(const char *iface)
{
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        perror("socket");
        return -1;
    }
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", iface);
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
    int fl = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, fl | O_NONBLOCK);
    return fd;
}

static void sim_control(lift_plant_t *p, bool *estop2, const struct can_frame *cf)
{
    if (cf->can_dlc < 2) {
        return;
    }
    uint8_t k = cf->data[1];
    bool v2 = cf->can_dlc >= 3 && cf->data[2] != 0;
    switch (cf->data[0]) {
    case 1:
        p->motor_power = cf->data[1] != 0;
        break;
    case 2:
        p->hall_supply_ok = cf->data[1] != 0;
        break;
    case 3:
        if (k < LIFT_NUM_STOPS) {
            p->sensor_stuck_on[k] = v2;
        }
        break;
    case 4:
        if (k < LIFT_NUM_STOPS) {
            p->sensor_stuck_off[k] = v2;
        }
        break;
    case 5:
        p->jammed = cf->data[1] != 0;
        break;
    case 6:
        *estop2 = cf->data[1] != 0;
        break;
    default:
        break;
    }
}

static ctl017_app_t g_app;

int main(int argc, char **argv)
{
    const char *iface = "vcan0";
    double start_mm = 180.0;
    double duration_s = 0.0;
    for (int i = 1; i + 1 < argc; i += 2) {
        if (strcmp(argv[i], "--iface") == 0) {
            iface = argv[i + 1];
        } else if (strcmp(argv[i], "--start-mm") == 0) {
            start_mm = atof(argv[i + 1]);
        } else if (strcmp(argv[i], "--duration-s") == 0) {
            duration_s = atof(argv[i + 1]);
        } else {
            fprintf(stderr, "unknown option %s\n", argv[i]);
            return 2;
        }
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    int fd = open_can(iface);
    if (fd < 0) {
        return 1;
    }

    lift_plant_t plant;
    plant_init(&plant, (int64_t)(start_mm * 1e6));
    lift_config_t lc;
    lift_config_default(&lc);
    /* The plant's stop sensors switch on 1 mm before each stop. */
    lc.sensor_edge_offset_up_um = -1000;
    lc.sensor_edge_offset_down_um = 1000;
    aux_config_t ac;
    aux_config_default(&ac);

    uint32_t t0 = mono_ms();
    uint32_t now = t0;
    if (!ctl017_app_init(&g_app, &lc, &ac, can_send, &fd, now)) {
        fprintf(stderr, "application init failed\n");
        return 1;
    }
    fprintf(stderr, "ctl017_sim: node 0x%02X on %s, start %.1f mm\n", CTL017_NODE_ID, iface, start_mm);

    bool estop2 = false;
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    while (!g_stop) {
        struct can_frame cf;
        while (read(fd, &cf, sizeof(cf)) == (ssize_t)sizeof(cf)) {
            if ((cf.can_id & CAN_EFF_FLAG) != 0u || (cf.can_id & CAN_RTR_FLAG) != 0u) {
                continue;
            }
            uint16_t id = (uint16_t)(cf.can_id & CAN_SFF_MASK);
            if (id == SIM_CTRL_ID) {
                sim_control(&plant, &estop2, &cf);
                continue;
            }
            co_frame_t f;
            memset(&f, 0, sizeof(f));
            f.id = id;
            f.dlc = cf.can_dlc > 8 ? 8 : cf.can_dlc;
            memcpy(f.data, cf.data, f.dlc);
            ctl017_app_rx(&g_app, &f, mono_ms());
        }

        uint32_t t = mono_ms();
        uint32_t dt = t - now;
        now = t;
        if (dt > 0u) {
            plant_step(&plant, &g_app.out.hbridge, dt);
        }
        lift_inputs_t li;
        aux_inputs_t ai;
        plant_inputs(&plant, now, &li);
        memset(&ai, 0, sizeof(ai));
        ai.now_ms = now;
        ai.estop2_no_closed = estop2;
        const ctl017_outputs_t *out = ctl017_app_step(&g_app, &li, &ai);
        if (out->reset_requested) {
            fprintf(stderr, "ctl017_sim: NMT reset node, restarting\n");
            if (!ctl017_app_init(&g_app, &lc, &ac, can_send, &fd, mono_ms())) {
                return 1;
            }
        }
        if (duration_s > 0.0 && (double)(now - t0) >= duration_s * 1000.0) {
            break;
        }
        next.tv_nsec += 1000000;
        if (next.tv_nsec >= 1000000000) {
            next.tv_nsec -= 1000000000;
            next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }
    close(fd);
    return 0;
}
