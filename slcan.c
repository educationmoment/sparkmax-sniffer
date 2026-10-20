/*
 * slcan.c - Lawicel SLCAN ASCII protocol over a POSIX serial port.
 *
 * Wire format (each command/frame terminated by '\r', errors are '\a'):
 *   C            close channel          O        open channel
 *   Sn           bitrate code 0..8      Z0/Z1    timestamps off/on
 *   tiiiLdd..    std frame              Tiiiiiiiildd..  ext frame
 *   riiiL        std RTR                Riiiiiiiil      ext RTR
 * Adapter acks a transmit with "z\r" (std) / "Z\r" (ext).
 * With timestamps on, 4 hex digits (ms, 0..0xEA5F) trail each rx frame.
 */
#define _DEFAULT_SOURCE
#include "sm_can.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

typedef struct {
    int  fd;
    char buf[512];
    int  len;
    unsigned errors;   /* count of '\a' replies seen */
} slcan_ctx;

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static int parse_hex(const char *s, int n, uint32_t *out)
{
    uint32_t v = 0;
    for (int i = 0; i < n; i++) {
        int h = hexval(s[i]);
        if (h < 0) return -1;
        v = (v << 4) | (uint32_t)h;
    }
    *out = v;
    return 0;
}

int slcan_parse_frame(const char *line, int len, sm_can_frame *f)
{
    if (len < 1) return SM_EPARSE;
    memset(f, 0, sizeof *f);
    int idlen;
    switch (line[0]) {
    case 't': idlen = 3; break;
    case 'r': idlen = 3; f->rtr = 1; break;
    case 'T': idlen = 8; f->extended = 1; break;
    case 'R': idlen = 8; f->extended = 1; f->rtr = 1; break;
    default:  return SM_EPARSE;
    }
    if (len < 1 + idlen + 1) return SM_EPARSE;

    uint32_t id, dlc;
    if (parse_hex(line + 1, idlen, &id)) return SM_EPARSE;
    if (parse_hex(line + 1 + idlen, 1, &dlc) || dlc > 8) return SM_EPARSE;
    if (id > (f->extended ? SM_CAN_EFF_MASK : SM_CAN_SFF_MASK)) return SM_EPARSE;

    int pos = 2 + idlen;
    int need = pos + (f->rtr ? 0 : (int)dlc * 2);
    /* Exactly the frame, or the frame plus a 4-digit timestamp. */
    if (len != need && len != need + 4) return SM_EPARSE;

    if (!f->rtr) {
        for (uint32_t i = 0; i < dlc; i++, pos += 2) {
            uint32_t byte;
            if (parse_hex(line + pos, 2, &byte)) return SM_EPARSE;
            f->data[i] = (uint8_t)byte;
        }
    }
    f->id = id;
    f->dlc = (uint8_t)dlc;
    return SM_OK;
}

int slcan_format_frame(const sm_can_frame *f, char *buf, int buflen)
{
    if (f->dlc > 8) return SM_EINVAL;
    char kind = f->extended ? (f->rtr ? 'R' : 'T') : (f->rtr ? 'r' : 't');
    int n = f->extended
        ? snprintf(buf, (size_t)buflen, "%c%08X%u", kind, f->id & SM_CAN_EFF_MASK, f->dlc)
        : snprintf(buf, (size_t)buflen, "%c%03X%u", kind, f->id & SM_CAN_SFF_MASK, f->dlc);
    if (n < 0 || n >= buflen) return SM_EINVAL;
    if (!f->rtr) {
        for (int i = 0; i < f->dlc; i++) {
            if (n + 3 >= buflen) return SM_EINVAL;
            n += snprintf(buf + n, (size_t)(buflen - n), "%02X", f->data[i]);
        }
    }
    if (n + 2 > buflen) return SM_EINVAL;
    buf[n++] = '\r';
    buf[n] = '\0';
    return n;
}

static int write_all(int fd, const char *s, int n)
{
    while (n > 0) {
        ssize_t w = write(fd, s, (size_t)n);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN) {               /* tx buffer full: wait */
                struct pollfd p = { fd, POLLOUT, 0 };
                if (poll(&p, 1, 100) <= 0) return SM_ETIMEOUT;
                continue;
            }
            return SM_EIO;
        }
        s += w; n -= (int)w;
    }
    return SM_OK;
}

static int cmd(slcan_ctx *c, const char *s)
{
    return write_all(c->fd, s, (int)strlen(s));
}

static int slcan_send(void *vc, const sm_can_frame *f)
{
    slcan_ctx *c = vc;
    char line[32];
    int n = slcan_format_frame(f, line, sizeof line);
    if (n < 0) return n;
    /* Do NOT block on the Z ack: rx frames interleave with acks, and waiting
     * here would stall the heartbeat. Acks/errors are consumed in recv. */
    return write_all(c->fd, line, n);
}

/* Pop one '\r'/'\a'-terminated token out of the buffer. */
static int take_line(slcan_ctx *c, char *out, int outcap, char *term)
{
    for (int i = 0; i < c->len; i++) {
        if (c->buf[i] == '\r' || c->buf[i] == '\a') {
            int n = i < outcap - 1 ? i : outcap - 1;
            memcpy(out, c->buf, (size_t)n);
            out[n] = '\0';
            *term = c->buf[i];
            memmove(c->buf, c->buf + i + 1, (size_t)(c->len - i - 1));
            c->len -= i + 1;
            return n;
        }
    }
    if (c->len == (int)sizeof c->buf) c->len = 0;  /* garbage w/o terminator: resync */
    return -1;
}

static int slcan_recv(void *vc, sm_can_frame *f, int timeout_ms)
{
    slcan_ctx *c = vc;
    uint64_t deadline = timeout_ms < 0 ? 0 : sm_monotonic_us() + (uint64_t)timeout_ms * 1000u;

    for (;;) {
        char line[64], term;
        int n;
        while ((n = take_line(c, line, sizeof line, &term)) >= 0) {
            if (term == '\a') { c->errors++; continue; }
            if (n == 0) continue;                         /* bare OK */
            if (line[0] == 'z' || line[0] == 'Z') {       /* tx ack; may share line w/ frame on some fw */
                if (n == 1) continue;
                memmove(line, line + 1, (size_t)n); n--;
            }
            if (slcan_parse_frame(line, n, f) == SM_OK) {
                f->ts_us = sm_monotonic_us();
                return SM_OK;
            }
            /* Unknown response (V/N/F replies): ignore. */
        }

        int wait = -1;
        if (timeout_ms >= 0) {
            uint64_t now = sm_monotonic_us();
            if (now >= deadline) return SM_ETIMEOUT;
            wait = (int)((deadline - now + 999) / 1000);
        }
        struct pollfd p = { c->fd, POLLIN, 0 };
        int pr = poll(&p, 1, wait);
        if (pr == 0) return SM_ETIMEOUT;
        if (pr < 0) { if (errno == EINTR) return SM_ETIMEOUT; return SM_EIO; }
        if (p.revents & (POLLERR | POLLHUP)) return SM_EIO;

        ssize_t r = read(c->fd, c->buf + c->len, sizeof c->buf - (size_t)c->len);
        if (r < 0) { if (errno == EAGAIN || errno == EINTR) continue; return SM_EIO; }
        if (r == 0) return SM_EIO;
        c->len += (int)r;
    }
}

static void slcan_close(void *vc)
{
    slcan_ctx *c = vc;
    if (!c) return;
    cmd(c, "C\r");
    tcdrain(c->fd);
    close(c->fd);
    free(c);
}

static speed_t to_speed(unsigned baud)
{
    switch (baud) {
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
#ifdef B3000000
    case 3000000: return B3000000;
#endif
    default:      return 0;
    }
}

int slcan_open(sm_transport *out, const char *tty, unsigned baud, slcan_bitrate br)
{
    if (!out || !tty || br > SLCAN_1M) return SM_EINVAL;
    if (baud == 0) baud = 3000000;
    speed_t sp = to_speed(baud);
    if (!sp) return SM_EINVAL;

    int fd = open(tty, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return SM_EIO;

    struct termios tio;
    if (tcgetattr(fd, &tio) < 0) { close(fd); return SM_EIO; }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CRTSCTS;
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);
    if (tcsetattr(fd, TCSANOW, &tio) < 0) { close(fd); return SM_EIO; }
    tcflush(fd, TCIOFLUSH);

    slcan_ctx *c = calloc(1, sizeof *c);
    if (!c) { close(fd); return SM_EIO; }
    c->fd = fd;

    /* Flush any half-typed command, close in case it was left open, set
     * bitrate, disable timestamps (we stamp on the host), open. */
    char sbuf[8];
    snprintf(sbuf, sizeof sbuf, "S%d\r", (int)br);
    const char *seq[] = { "\r\r\r", "C\r", sbuf, "Z0\r", "O\r" };
    for (size_t i = 0; i < sizeof seq / sizeof *seq; i++) {
        if (cmd(c, seq[i]) != SM_OK) { close(fd); free(c); return SM_EIO; }
        usleep(10000);
    }
    /* "C" while already closed returns BELL - that is expected; drop the
     * startup replies. A rejected "O" would surface later as no traffic. */
    tcflush(fd, TCIFLUSH);
    c->len = 0;

    out->ctx = c;
    out->send = slcan_send;
    out->recv = slcan_recv;
    out->close = slcan_close;
    return SM_OK;
}
