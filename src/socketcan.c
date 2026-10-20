/* socketcan.c - Linux SocketCAN backend (use with `slcand` or native CAN HW). */
#ifdef __linux__
#define _DEFAULT_SOURCE
#include "sm_can.h"

#include <errno.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

typedef struct { int fd; } sc_ctx;

static int sc_send(void *vc, const sm_can_frame *f)
{
    sc_ctx *c = vc;
    struct can_frame cf;
    memset(&cf, 0, sizeof cf);
    cf.can_id = f->extended ? ((f->id & CAN_EFF_MASK) | CAN_EFF_FLAG)
                            : (f->id & CAN_SFF_MASK);
    if (f->rtr) cf.can_id |= CAN_RTR_FLAG;
    cf.can_dlc = f->dlc;
    memcpy(cf.data, f->data, 8);
    ssize_t w = write(c->fd, &cf, sizeof cf);
    if (w == (ssize_t)sizeof cf) return SM_OK;
    return (errno == ENOBUFS || errno == EAGAIN) ? SM_ETIMEOUT : SM_EIO;
}

static int sc_recv(void *vc, sm_can_frame *f, int timeout_ms)
{
    sc_ctx *c = vc;
    struct pollfd p = { c->fd, POLLIN, 0 };
    int pr = poll(&p, 1, timeout_ms);
    if (pr == 0) return SM_ETIMEOUT;
    if (pr < 0) return errno == EINTR ? SM_ETIMEOUT : SM_EIO;

    struct can_frame cf;
    ssize_t r = read(c->fd, &cf, sizeof cf);
    if (r != (ssize_t)sizeof cf) return SM_EIO;
    if (cf.can_id & CAN_ERR_FLAG) return SM_ETIMEOUT;   /* error frames: skip */
    memset(f, 0, sizeof *f);
    f->extended = (cf.can_id & CAN_EFF_FLAG) != 0;
    f->rtr      = (cf.can_id & CAN_RTR_FLAG) != 0;
    f->id       = cf.can_id & (f->extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    f->dlc      = cf.can_dlc > 8 ? 8 : cf.can_dlc;
    memcpy(f->data, cf.data, f->dlc);
    f->ts_us    = sm_monotonic_us();
    return SM_OK;
}

static void sc_close(void *vc)
{
    sc_ctx *c = vc;
    if (!c) return;
    close(c->fd);
    free(c);
}

int socketcan_open(sm_transport *out, const char *ifname)
{
    int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) return SM_EIO;
    struct ifreq ifr;
    memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) { close(fd); return SM_EIO; }
    struct sockaddr_can addr;
    memset(&addr, 0, sizeof addr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (struct sockaddr *)&addr, sizeof addr) < 0) { close(fd); return SM_EIO; }

    sc_ctx *c = calloc(1, sizeof *c);
    if (!c) { close(fd); return SM_EIO; }
    c->fd = fd;
    out->ctx = c;
    out->send = sc_send;
    out->recv = sc_recv;
    out->close = sc_close;
    return SM_OK;
}
#endif
