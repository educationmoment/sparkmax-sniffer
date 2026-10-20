/*
 * sparkmax.h - C library for REV SPARK MAX over raw CAN.
 *
 * Two layers:
 *   1. Codec  (sm_encode_* / sm_decode_*): pure functions, no I/O, no state.
 *      Unit-testable, usable on an MCU with zero dependencies.
 *   2. Bus    (sm_bus_*): owns a transport, a heartbeat schedule, a mutex,
 *      and a cache of the latest status per device.
 *
 * ---------------------------------------------------------------------------
 * PROTOCOL PROVENANCE - read this before trusting any number below.
 * REV has never published a full payload spec for the SPARK MAX. The arbitration
 * IDs here follow the FRC CAN layout (documented by REV/FIRST) and the
 * community-reverse-engineered frame map for pre-2025 firmware (heartbeat
 * 0x2052C80, setpoints 0x205xxxx, status 0x20518xx). Firmware 25.x reworked
 * status frames. Heartbeat and duty-cycle setpoint reportedly kept their IDs;
 * everything else must be verified with tools/sm_sniff against YOUR firmware
 * while the REV Hardware Client drives the controller.
 * ---------------------------------------------------------------------------
 *
 * FRC 29-bit ID:  [28:24] device type   (2  = motor controller)
 *                 [23:16] manufacturer  (5  = REV)
 *                 [15:10] API class
 *                 [ 9: 6] API index
 *                 [ 5: 0] device number (1..62; 0 = unconfigured, never enables)
 */
#ifndef SPARKMAX_H
#define SPARKMAX_H

#include "sm_can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SM_DEVTYPE_MOTOR_CTRL  2u
#define SM_MFR_REV             5u
#define SM_MAX_DEVICES         64

/* 10-bit API id = (class << 4) | index */
#define SM_API(cls, idx) ((uint16_t)(((cls) << 4) | (idx)))

typedef enum {
    SM_API_SETPOINT_DUTY      = SM_API(0, 2),   /* 0x2050080 */
    SM_API_SETPOINT_VELOCITY  = SM_API(1, 2),   /* 0x2050480 */
    SM_API_SETPOINT_SMART_VEL = SM_API(1, 3),   /* 0x20504C0 */
    SM_API_SETPOINT_POSITION  = SM_API(3, 2),   /* 0x2050C80 */
    SM_API_SETPOINT_VOLTAGE   = SM_API(4, 2),   /* 0x2051080 */
    SM_API_SETPOINT_CURRENT   = SM_API(4, 3),   /* 0x20510C0 */
    SM_API_SETPOINT_SMART_MOT = SM_API(5, 2),   /* 0x2051480 */
    SM_API_STATUS_0           = SM_API(6, 0),   /* 0x2051800 */
    SM_API_STATUS_1           = SM_API(6, 1),   /* 0x2051840 */
    SM_API_STATUS_2           = SM_API(6, 2),   /* 0x2051880 */
    SM_API_STATUS_3           = SM_API(6, 3),   /* 0x20518C0 */
    SM_API_STATUS_4           = SM_API(6, 4),   /* 0x2051900 */
    SM_API_HEARTBEAT          = SM_API(11, 2)   /* 0x2052C80, device field = 0 */
} sm_api;

/* The FRC roboRIO universal heartbeat. A SPARK that sees this ignores the
 * non-roboRIO heartbeat above. Never transmit it yourself. */
#define SM_ROBORIO_HEARTBEAT_ID 0x01011840u

typedef struct {
    uint8_t  devtype, mfr, api_class, api_index, device;
    uint16_t api;   /* (class<<4)|index */
} sm_frc_id;

static inline uint32_t sm_make_id(uint16_t api, uint8_t device)
{
    return ((uint32_t)SM_DEVTYPE_MOTOR_CTRL << 24) |
           ((uint32_t)SM_MFR_REV << 16) |
           ((uint32_t)(api & 0x3FFu) << 6) |
           (uint32_t)(device & 0x3Fu);
}

static inline sm_frc_id sm_split_id(uint32_t id)
{
    sm_frc_id r;
    r.devtype   = (uint8_t)((id >> 24) & 0x1F);
    r.mfr       = (uint8_t)((id >> 16) & 0xFF);
    r.api_class = (uint8_t)((id >> 10) & 0x3F);
    r.api_index = (uint8_t)((id >> 6)  & 0x0F);
    r.api       = (uint16_t)((id >> 6) & 0x3FF);
    r.device    = (uint8_t)(id & 0x3F);
    return r;
}

static inline bool sm_is_sparkmax_frame(const sm_can_frame *f)
{
    if (!f->extended) return false;
    sm_frc_id s = sm_split_id(f->id);
    return s.devtype == SM_DEVTYPE_MOTOR_CTRL && s.mfr == SM_MFR_REV;
}

const char *sm_api_name(uint16_t api);   /* "STATUS_1", "UNKNOWN", ... */

/* ----------------------------- codec ------------------------------------ */

typedef struct {
    float    applied_output;    /* -1..1 */
    uint16_t faults;
    uint16_t sticky_faults;
    uint8_t  raw6, raw7;        /* flag bits, layout varies by firmware */
} sm_status0;

typedef struct {
    float   velocity_rpm;
    uint8_t temperature_c;
    float   bus_voltage;        /* 12-bit, /128 */
    float   output_current;     /* 12-bit, /32  */
} sm_status1;

typedef struct {
    float position_rot;
} sm_status2;

int sm_decode_status0(const sm_can_frame *f, sm_status0 *out);
int sm_decode_status1(const sm_can_frame *f, sm_status1 *out);
int sm_decode_status2(const sm_can_frame *f, sm_status2 *out);

/* Setpoint frame: [0..3] float32 LE setpoint, [4..5] int16 LE arbitrary
 * feed-forward, [6] bits0-1 PID slot, [7] reserved. */
int sm_encode_setpoint(sm_can_frame *f, uint8_t device, sm_api mode,
                       float setpoint, int16_t arb_ff, uint8_t pid_slot);

/* Non-roboRIO heartbeat: 8-byte bitmask, bit n => device n enabled. */
void sm_encode_heartbeat(sm_can_frame *f, const uint8_t enable_mask[8]);

/* Little-endian float helpers (portable: no type punning through pointers). */
void  sm_put_f32le(uint8_t *p, float v);
float sm_get_f32le(const uint8_t *p);

/* ----------------------------- bus -------------------------------------- */

typedef struct sm_bus sm_bus;

typedef struct {
    uint64_t   rx_us[3];        /* last receive time of status 0,1,2; 0 = never */
    sm_status0 s0;
    sm_status1 s1;
    sm_status2 s2;
} sm_device_state;

/* Takes ownership of the transport. */
sm_bus *sm_bus_create(sm_transport t);
void    sm_bus_destroy(sm_bus *b);        /* disables all, closes transport */

/* Enable/disable a device in the heartbeat mask. Disabled = motor coast/brake
 * per its idle mode within ~100 ms of the last enabling heartbeat. */
void sm_bus_set_enabled(sm_bus *b, uint8_t device, bool enabled);
void sm_bus_disable_all(sm_bus *b);

/* Send a heartbeat if period_ms elapsed. Call it from your loop or a thread.
 * The SPARK times out at ~100 ms; use <= 25 ms. Returns 1 if sent, 0 if not
 * due, <0 on error. */
int  sm_bus_tick(sm_bus *b, unsigned period_ms);

int  sm_set_duty(sm_bus *b, uint8_t dev, float duty);          /* -1..1 */
int  sm_set_voltage(sm_bus *b, uint8_t dev, float volts);
int  sm_set_velocity(sm_bus *b, uint8_t dev, float rpm, uint8_t slot);
int  sm_set_position(sm_bus *b, uint8_t dev, float rotations, uint8_t slot);
int  sm_set_current(sm_bus *b, uint8_t dev, float amps);
int  sm_send_setpoint(sm_bus *b, uint8_t dev, sm_api mode, float sp,
                      int16_t arb_ff, uint8_t slot);
int  sm_send_raw(sm_bus *b, const sm_can_frame *f);

/* Receive one frame, update the status cache if it is SPARK status.
 * Returns transport codes. Raw frame copied to *out if non-NULL. */
int  sm_bus_poll(sm_bus *b, int timeout_ms, sm_can_frame *out);

/* Copy of the cached state. Returns false if nothing ever received. */
bool sm_get_state(sm_bus *b, uint8_t dev, sm_device_state *out);

#ifdef __cplusplus
}
#endif
#endif /* SPARKMAX_H */
