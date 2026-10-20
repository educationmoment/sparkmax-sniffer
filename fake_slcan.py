#!/usr/bin/env python3
"""Fake Lawicel SLCAN adapter on a pty, with one simulated SPARK MAX.

Lets you exercise sm_sniff / sm_drive / your own code with no hardware:
    python3 tests/fake_slcan.py /tmp/fakecan &
    ./build/sm_drive --slcan /tmp/fakecan --dev 5

The fake SPARK only spins when (a) the bus is open, (b) a heartbeat with its
bit set arrived in the last 100 ms, (c) a duty setpoint was received. That
mirrors the real enable logic, so heartbeat bugs show up here first.
"""
import os, pty, select, struct, sys, time, tty

DEV = 5
HB_ID = 0x02052C80
def mk(api, dev): return (2 << 24) | (5 << 16) | (api << 6) | dev
DUTY = mk(0x02, DEV)
ST0, ST1, ST2 = mk(0x60, DEV), mk(0x61, DEV), mk(0x62, DEV)

link = sys.argv[1] if len(sys.argv) > 1 else "/tmp/fakecan"
master, slave = pty.openpty()
tty.setraw(slave)
try: os.unlink(link)
except FileNotFoundError: pass
os.symlink(os.ttyname(slave), link)

is_open, bitrate = False, None
duty, last_hb, pos, vel = 0.0, 0.0, 0.0, 0.0
buf = b""
stats = {"hb": 0, "duty": 0, "bad": 0}
t_last_status = time.monotonic()

def emit(fid, data):
    os.write(master, b"T%08X%d%s\r" % (fid, len(data), data.hex().upper().encode()))

def handle(line):
    global is_open, bitrate, duty, last_hb
    if not line: return b"\r"
    c = line[:1]
    if c == b"S": bitrate = int(line[1:]); return b"\r"
    if c == b"O": is_open = True; return b"\r"
    if c == b"C":
        if not is_open: return b"\a"
        is_open = False; return b"\r"
    if c == b"Z": return b"\r"
    if c == b"T" and is_open:
        fid, dlc = int(line[1:9], 16), int(line[9:10], 16)
        data = bytes.fromhex(line[10:10 + 2 * dlc].decode())
        if fid == HB_ID:
            stats["hb"] += 1
            if data[DEV >> 3] & (1 << (DEV & 7)): last_hb = time.monotonic()
        elif fid == DUTY:
            stats["duty"] += 1
            duty = struct.unpack("<f", data[:4])[0]
        return b"Z\r"
    stats["bad"] += 1
    return b"\a"

try:
    while True:
        r, _, _ = select.select([master], [], [], 0.002)
        if r:
            try: buf += os.read(master, 4096)
            except OSError: buf = b""
            while b"\r" in buf:
                line, buf = buf.split(b"\r", 1)
                os.write(master, handle(line))
        now = time.monotonic()
        if is_open and now - t_last_status >= 0.010:
            dt = now - t_last_status; t_last_status = now
            enabled = (now - last_hb) < 0.100
            out = duty if enabled else 0.0
            vel = 5676.0 * out                     # NEO free speed
            pos += vel / 60.0 * dt
            v12, c12 = int(12.3 * 128), int(abs(out) * 20 * 32)
            emit(ST0, struct.pack("<hHHBB", int(out * 32767), 0, 0, 0, 0))
            emit(ST1, struct.pack("<fB", vel, 31) + bytes([v12 & 0xFF, ((v12 >> 8) & 0xF) | ((c12 & 0xF) << 4), (c12 >> 4) & 0xFF]))
            emit(ST2, struct.pack("<fI", pos, 0))
except KeyboardInterrupt:
    pass
finally:
    print(f"fake_slcan: bitrate=S{bitrate} {stats}", file=sys.stderr)
    os.unlink(link)
