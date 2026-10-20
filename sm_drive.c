/*
 * sm_drive.c - minimal C consumer of libsparkmax.
 * Ramps duty cycle 0 -> 0.2 -> 0 on one device and prints telemetry.
 *
 *   sm_drive --slcan /dev/ttyUSB0 --dev 5
 *   sm_drive --socketcan can0 --dev 5
 *
 * Bench use only. This sends the non-roboRIO heartbeat: never run it on a
 * bus with a roboRIO, and never on an FRC robot (it bypasses the DS e-stop).
 */
#define _DEFAULT_SOURCE
#include "sparkmax.h"

#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static atomic_bool g_run = true;
static void on_sig(int s) { (void)s; g_run = false; }

typedef struct { sm_bus *bus; } ctx_t;

/* Heartbeat + rx pump on one thread so the enable signal never starves
 * behind application logic. 20 ms period vs ~100 ms device timeout. */
static void *io_thread(void *arg)
{
    ctx_t *c = arg;
    while (g_run) {
        if (sm_bus_tick(c->bus, 20) < 0) { fprintf(stderr, "heartbeat tx failed\n"); g_run = false; }
        int rc = sm_bus_poll(c->bus, 5, NULL);
        if (rc != SM_OK && rc != SM_ETIMEOUT) { fprintf(stderr, "rx error %d\n", rc); g_run = false; }
    }
    return NULL;
}

int main(int argc, char **argv)
{
    const char *tty = NULL, *ifn = NULL;
    int dev = -1;
    for (int i = 1; i + 1 < argc; i += 2) {
        if      (!strcmp(argv[i], "--slcan"))     tty = argv[i + 1];
        else if (!strcmp(argv[i], "--socketcan")) ifn = argv[i + 1];
        else if (!strcmp(argv[i], "--dev"))       dev = atoi(argv[i + 1]);
    }
    if ((!tty == !ifn) || dev < 1 || dev > 62) {
        fprintf(stderr, "usage: %s (--slcan TTY | --socketcan IF) --dev 1..62\n", argv[0]);
        return 2;
    }

    sm_transport t;
    int rc = tty ? slcan_open(&t, tty, 0, SLCAN_1M) : socketcan_open(&t, ifn);
    if (rc != SM_OK) { fprintf(stderr, "open failed: %d\n", rc); return 1; }

    sm_bus *bus = sm_bus_create(t);
    if (!bus) return 1;

    struct sigaction sa;
    memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_sig;
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);

    ctx_t c = { bus };
    pthread_t th;
    pthread_create(&th, NULL, io_thread, &c);

    sm_bus_set_enabled(bus, (uint8_t)dev, true);

    /* Setpoints "stick" on modern firmware, but resend periodically anyway:
     * a dropped frame should cost you 20 ms, not the whole ramp. */
    for (int step = 0; g_run && step < 400; step++) {   /* 400 * 20ms = 8 s */
        float phase = (float)(step < 200 ? step : 400 - step) / 200.0f;
        sm_set_duty(bus, (uint8_t)dev, 0.2f * phase);

        if (step % 10 == 0) {
            sm_device_state s;
            if (sm_get_state(bus, (uint8_t)dev, &s))
                printf("out=%+.3f vel=%8.1frpm pos=%9.3f V=%5.2f I=%5.2fA T=%uC\n",
                       s.s0.applied_output, s.s1.velocity_rpm, s.s2.position_rot,
                       s.s1.bus_voltage, s.s1.output_current, s.s1.temperature_c);
            else
                printf("no status from dev %d yet (wrong ID? wrong firmware frame map?)\n", dev);
        }
        usleep(20000);
    }

    g_run = false;
    pthread_join(th, NULL);
    sm_bus_destroy(bus);   /* zero setpoint, drop enable mask, close */
    return 0;
}
