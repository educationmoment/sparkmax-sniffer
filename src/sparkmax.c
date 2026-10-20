#define _POSIX_C_SOURCE 200809L
#include "sparkmax.h"

#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

uint64_t sm_monotonic_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

/* ----------------------------- codec ------------------------------------ */

void sm_put_f32le(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, sizeof u);           /* the only legal way to pun in C */
    p[0] = (uint8_t)(u);
    p[1] = (uint8_t)(u >> 8);
    p[2] = (uint8_t)(u >> 16);
    p[3] = (uint8_t)(u >> 24);
}

float sm_get_f32le(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float v;
    memcpy(&v, &u, sizeof v);
    return v;
}

static inline uint16_t get_u16le(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

const char *sm_api_name(uint16_t api)
{
    switch (api) {
    case SM_API_SETPOINT_DUTY:      return "SET_DUTY";
    case SM_API_SETPOINT_VELOCITY:  return "SET_VELOCITY";
    case SM_API_SETPOINT_SMART_VEL: return "SET_SMART_VEL";
    case SM_API_SETPOINT_POSITION:  return "SET_POSITION";
    case SM_API_SETPOINT_VOLTAGE:   return "SET_VOLTAGE";
    case SM_API_SETPOINT_CURRENT:   return "SET_CURRENT";
    case SM_API_SETPOINT_SMART_MOT: return "SET_SMART_MOTION";
    case SM_API_STATUS_0:           return "STATUS_0";
    case SM_API_STATUS_1:           return "STATUS_1";
    case SM_API_STATUS_2:           return "STATUS_2";
    case SM_API_STATUS_3:           return "STATUS_3";
    case SM_API_STATUS_4:           return "STATUS_4";
    case SM_API_HEARTBEAT:          return "HEARTBEAT";
    default:                        return "UNKNOWN";
    }
}

static int check_status(const sm_can_frame *f, uint16_t api, uint8_t min_dlc)
{
    if (!f || !sm_is_sparkmax_frame(f)) return SM_EINVAL;
    if (sm_split_id(f->id).api != api)  return SM_EINVAL;
    if (f->dlc < min_dlc)               return SM_EPARSE;
    return SM_OK;
}

int sm_decode_status0(const sm_can_frame *f, sm_status0 *o)
{
    int rc = check_status(f, SM_API_STATUS_0, 8);
    if (rc) return rc;
    o->applied_output = (float)(int16_t)get_u16le(&f->data[0]) / 32768.0f;
    o->faults         = get_u16le(&f->data[2]);
    o->sticky_faults  = get_u16le(&f->data[4]);
    o->raw6 = f->data[6];
    o->raw7 = f->data[7];
    return SM_OK;
}

int sm_decode_status1(const sm_can_frame *f, sm_status1 *o)
{
    int rc = check_status(f, SM_API_STATUS_1, 8);
    if (rc) return rc;
    const uint8_t *d = f->data;
    o->velocity_rpm   = sm_get_f32le(&d[0]);
    o->temperature_c  = d[4];
    /* bytes 5..7 hold two packed 12-bit fields:
     *   voltage = d5 | (d6 & 0x0F) << 8      (LSB = 1/128 V)
     *   current = d6 >> 4 | d7 << 4          (LSB = 1/32  A) */
    uint16_t v12 = (uint16_t)(d[5] | ((d[6] & 0x0F) << 8));
    uint16_t c12 = (uint16_t)((d[6] >> 4) | (d[7] << 4));
    o->bus_voltage    = (float)v12 / 128.0f;
    o->output_current = (float)c12 / 32.0f;
    return SM_OK;
}

int sm_decode_status2(const sm_can_frame *f, sm_status2 *o)
{
    int rc = check_status(f, SM_API_STATUS_2, 4);
    if (rc) return rc;
    o->position_rot = sm_get_f32le(&f->data[0]);
    return SM_OK;
}

int sm_encode_setpoint(sm_can_frame *f, uint8_t device, sm_api mode,
                       float sp, int16_t arb_ff, uint8_t slot)
{
    if (!f || device == 0 || device > 62 || slot > 3) return SM_EINVAL;
    if (sp != sp) return SM_EINVAL;                     /* reject NaN */
    if (mode == SM_API_SETPOINT_DUTY) {                 /* clamp, don't trust callers */
        if (sp > 1.0f) sp = 1.0f;
        if (sp < -1.0f) sp = -1.0f;
    }
    memset(f, 0, sizeof *f);
    f->id = sm_make_id((uint16_t)mode, device);
    f->extended = 1;
    f->dlc = 8;
    sm_put_f32le(&f->data[0], sp);
    f->data[4] = (uint8_t)((uint16_t)arb_ff);
    f->data[5] = (uint8_t)((uint16_t)arb_ff >> 8);
    f->data[6] = (uint8_t)(slot & 0x03);
    return SM_OK;
}

void sm_encode_heartbeat(sm_can_frame *f, const uint8_t mask[8])
{
    memset(f, 0, sizeof *f);
    f->id = sm_make_id(SM_API_HEARTBEAT, 0);
    f->extended = 1;
    f->dlc = 8;
    memcpy(f->data, mask, 8);
}

/* ----------------------------- bus -------------------------------------- */

struct sm_bus {
    sm_transport     t;
    pthread_mutex_t  tx_lock;     /* transport send is not re-entrant */
    pthread_mutex_t  state_lock;  /* guards mask + cache */
    uint8_t          mask[8];
    uint64_t         last_hb_us;
    sm_device_state  dev[SM_MAX_DEVICES];
};

sm_bus *sm_bus_create(sm_transport t)
{
    sm_bus *b = calloc(1, sizeof *b);
    if (!b) return NULL;
    b->t = t;
    pthread_mutex_init(&b->tx_lock, NULL);
    pthread_mutex_init(&b->state_lock, NULL);
    return b;
}

int sm_send_raw(sm_bus *b, const sm_can_frame *f)
{
    pthread_mutex_lock(&b->tx_lock);
    int rc = b->t.send(b->t.ctx, f);
    pthread_mutex_unlock(&b->tx_lock);
    return rc;
}

static int send_heartbeat_now(sm_bus *b)
{
    sm_can_frame f;
    uint8_t m[8];
    pthread_mutex_lock(&b->state_lock);
    memcpy(m, b->mask, 8);
    b->last_hb_us = sm_monotonic_us();
    pthread_mutex_unlock(&b->state_lock);
    sm_encode_heartbeat(&f, m);
    return sm_send_raw(b, &f);
}

void sm_bus_destroy(sm_bus *b)
{
    if (!b) return;
    /* Fail safe: zero every enabled device, then drop the enable mask. */
    for (int d = 1; d < 63; d++)
        if (b->mask[d >> 3] & (1u << (d & 7)))
            sm_set_duty(b, (uint8_t)d, 0.0f);
    sm_bus_disable_all(b);
    send_heartbeat_now(b);
    if (b->t.close) b->t.close(b->t.ctx);
    pthread_mutex_destroy(&b->tx_lock);
    pthread_mutex_destroy(&b->state_lock);
    free(b);
}

void sm_bus_set_enabled(sm_bus *b, uint8_t d, bool en)
{
    if (d == 0 || d > 62) return;
    pthread_mutex_lock(&b->state_lock);
    if (en) b->mask[d >> 3] |=  (uint8_t)(1u << (d & 7));
    else    b->mask[d >> 3] &= (uint8_t)~(1u << (d & 7));
    pthread_mutex_unlock(&b->state_lock);
}

void sm_bus_disable_all(sm_bus *b)
{
    pthread_mutex_lock(&b->state_lock);
    memset(b->mask, 0, 8);
    pthread_mutex_unlock(&b->state_lock);
}

int sm_bus_tick(sm_bus *b, unsigned period_ms)
{
    uint64_t now = sm_monotonic_us();
    pthread_mutex_lock(&b->state_lock);
    bool due = (now - b->last_hb_us) >= (uint64_t)period_ms * 1000u;
    pthread_mutex_unlock(&b->state_lock);
    if (!due) return 0;
    int rc = send_heartbeat_now(b);
    return rc < 0 ? rc : 1;
}

int sm_send_setpoint(sm_bus *b, uint8_t dev, sm_api mode, float sp,
                     int16_t arb_ff, uint8_t slot)
{
    sm_can_frame f;
    int rc = sm_encode_setpoint(&f, dev, mode, sp, arb_ff, slot);
    return rc ? rc : sm_send_raw(b, &f);
}

int sm_set_duty(sm_bus *b, uint8_t d, float v)
{ return sm_send_setpoint(b, d, SM_API_SETPOINT_DUTY, v, 0, 0); }
int sm_set_voltage(sm_bus *b, uint8_t d, float v)
{ return sm_send_setpoint(b, d, SM_API_SETPOINT_VOLTAGE, v, 0, 0); }
int sm_set_velocity(sm_bus *b, uint8_t d, float rpm, uint8_t s)
{ return sm_send_setpoint(b, d, SM_API_SETPOINT_VELOCITY, rpm, 0, s); }
int sm_set_position(sm_bus *b, uint8_t d, float rot, uint8_t s)
{ return sm_send_setpoint(b, d, SM_API_SETPOINT_POSITION, rot, 0, s); }
int sm_set_current(sm_bus *b, uint8_t d, float a)
{ return sm_send_setpoint(b, d, SM_API_SETPOINT_CURRENT, a, 0, 0); }

int sm_bus_poll(sm_bus *b, int timeout_ms, sm_can_frame *out)
{
    sm_can_frame f;
    int rc = b->t.recv(b->t.ctx, &f, timeout_ms);
    if (rc != SM_OK) return rc;
    if (out) *out = f;
    if (!sm_is_sparkmax_frame(&f)) return SM_OK;

    sm_frc_id id = sm_split_id(f.id);
    sm_device_state *st = &b->dev[id.device];
    pthread_mutex_lock(&b->state_lock);
    switch (id.api) {
    case SM_API_STATUS_0:
        if (sm_decode_status0(&f, &st->s0) == SM_OK) st->rx_us[0] = f.ts_us;
        break;
    case SM_API_STATUS_1:
        if (sm_decode_status1(&f, &st->s1) == SM_OK) st->rx_us[1] = f.ts_us;
        break;
    case SM_API_STATUS_2:
        if (sm_decode_status2(&f, &st->s2) == SM_OK) st->rx_us[2] = f.ts_us;
        break;
    default: break;
    }
    pthread_mutex_unlock(&b->state_lock);
    return SM_OK;
}

bool sm_get_state(sm_bus *b, uint8_t d, sm_device_state *out)
{
    if (d >= SM_MAX_DEVICES) return false;
    pthread_mutex_lock(&b->state_lock);
    *out = b->dev[d];
    pthread_mutex_unlock(&b->state_lock);
    return out->rx_us[0] || out->rx_us[1] || out->rx_us[2];
}
