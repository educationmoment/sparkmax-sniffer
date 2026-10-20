/*
 * sm_can.h - transport-agnostic CAN frame + transport vtable.
 *
 * The protocol layer (sparkmax.c) never touches a file descriptor. It talks
 * to an sm_transport. That is what lets the same library run over a Lawicel
 * CANUSB (SLCAN ASCII over serial), Linux SocketCAN, or an MCU CAN peripheral
 * you write a 30-line shim for.
 */
#ifndef SM_CAN_H
#define SM_CAN_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SM_CAN_EFF_MASK 0x1FFFFFFFu   /* 29-bit extended id */
#define SM_CAN_SFF_MASK 0x000007FFu   /* 11-bit standard id */

typedef struct {
    uint32_t id;        /* 11 or 29 bit identifier, no flag bits */
    uint8_t  extended;  /* 1 = 29-bit id */
    uint8_t  rtr;       /* 1 = remote frame */
    uint8_t  dlc;       /* 0..8 */
    uint8_t  data[8];
    uint64_t ts_us;     /* host receive timestamp, monotonic microseconds */
} sm_can_frame;

/* Return codes shared by all transports. */
enum {
    SM_OK        =  0,
    SM_ETIMEOUT  = -1,
    SM_EIO       = -2,
    SM_EPARSE    = -3,
    SM_EINVAL    = -4,
    SM_EADAPTER  = -5   /* adapter replied BELL (0x07) - command rejected */
};

typedef struct sm_transport {
    void *ctx;
    /* Non-blocking-ish send. Must be safe to call from one thread at a time;
     * sm_bus serialises calls with its own mutex. */
    int  (*send)(void *ctx, const sm_can_frame *f);
    /* Blocks up to timeout_ms (-1 = forever). Returns SM_OK, SM_ETIMEOUT, or <0. */
    int  (*recv)(void *ctx, sm_can_frame *f, int timeout_ms);
    void (*close)(void *ctx);
} sm_transport;

uint64_t sm_monotonic_us(void);

/* ---------------- SLCAN (Lawicel CANUSB, CANable slcan fw, etc.) ---------- */

/* SLCAN bitrate codes: S0..S8 */
typedef enum {
    SLCAN_10K = 0, SLCAN_20K, SLCAN_50K, SLCAN_100K, SLCAN_125K,
    SLCAN_250K, SLCAN_500K, SLCAN_800K, SLCAN_1M
} slcan_bitrate;

/*
 * Open an SLCAN adapter on a serial device and put it on the bus.
 * serial_baud: the UART/VCP baud. Lawicel CANUSB in VCP mode is typically run
 *              at 3000000; CDC-ACM adapters (CANable) ignore it. Pass 0 to
 *              use 3000000.
 * SPARK MAX / FRC CAN runs at 1 Mbit/s -> SLCAN_1M.
 */
int  slcan_open(sm_transport *out, const char *tty, unsigned serial_baud,
                slcan_bitrate br);

/* Pure parser, exposed for testing: parses one SLCAN line WITHOUT the '\r'. */
int  slcan_parse_frame(const char *line, int len, sm_can_frame *f);
/* Pure encoder: writes "Tiiiiiiiildd..\r" into buf, returns length or <0. */
int  slcan_format_frame(const sm_can_frame *f, char *buf, int buflen);

/* ---------------- SocketCAN (Linux only) ---------------------------------- */
#ifdef __linux__
int  socketcan_open(sm_transport *out, const char *ifname);
#endif

#ifdef __cplusplus
}
#endif
#endif /* SM_CAN_H */
