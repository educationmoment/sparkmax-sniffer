// sm_sniff.cpp - read CAN from a CANUSB (SLCAN) or SocketCAN interface and
// decode SPARK MAX traffic. Doubles as a reverse-engineering tool: every
// REV frame the codec doesn't understand is printed raw, and per-ID rate
// statistics are dumped on exit so you can map firmware-25 status frames.
//
//   sm_sniff --slcan /dev/ttyUSB0 [--baud 3000000]
//   sm_sniff --socketcan can0
//   options: --dev N  --all (non-REV frames too)  --csv out.csv  --quiet
#include "sparkmax.h"

#include <atomic>
#include <cinttypes>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

std::atomic<bool> g_run{true};
void on_sigint(int) { g_run = false; }

// RAII over the C transport. The C++ side owns lifetime; the C side owns protocol.
class Transport {
public:
    static Transport slcan(const std::string& tty, unsigned baud) {
        Transport t;
        if (slcan_open(&t.t_, tty.c_str(), baud, SLCAN_1M) != SM_OK)
            throw std::runtime_error("slcan_open failed on " + tty);
        return t;
    }
#ifdef __linux__
    static Transport socketcan(const std::string& ifn) {
        Transport t;
        if (socketcan_open(&t.t_, ifn.c_str()) != SM_OK)
            throw std::runtime_error("socketcan_open failed on " + ifn);
        return t;
    }
#endif
    Transport(Transport&& o) noexcept : t_(o.t_) { o.t_ = {}; }
    Transport(const Transport&) = delete;
    Transport& operator=(const Transport&) = delete;
    Transport& operator=(Transport&&) = delete;
    ~Transport() { if (t_.close) t_.close(t_.ctx); }

    int recv(sm_can_frame& f, int timeout_ms) { return t_.recv(t_.ctx, &f, timeout_ms); }

private:
    Transport() = default;
    sm_transport t_{};
};

struct IdStats {
    uint64_t count = 0, first_us = 0, last_us = 0;
    uint8_t  last_data[8]{};
    uint8_t  last_dlc = 0;
    uint8_t  changed_bits[8]{};   // OR of (prev ^ cur): which bits ever toggle
};

std::string hex(const uint8_t* d, int n) {
    char b[3 * 8 + 1] = {};
    for (int i = 0; i < n; ++i) std::snprintf(b + 3 * i, 4, "%02X ", d[i]);
    return b;
}

void usage(const char* a0) {
    std::fprintf(stderr,
        "usage: %s (--slcan TTY [--baud N] | --socketcan IF) [--dev N] [--all] "
        "[--csv FILE] [--quiet]\n", a0);
}

} // namespace

int main(int argc, char** argv) {
    std::string tty, ifname, csv_path;
    unsigned baud = 0;
    int dev_filter = -1;
    bool all = false, quiet = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { usage(argv[0]); std::exit(2); }
            return argv[++i];
        };
        if      (a == "--slcan")     tty = next();
        else if (a == "--socketcan") ifname = next();
        else if (a == "--baud")      baud = static_cast<unsigned>(std::stoul(next()));
        else if (a == "--dev")       dev_filter = std::stoi(next());
        else if (a == "--csv")       csv_path = next();
        else if (a == "--all")       all = true;
        else if (a == "--quiet")     quiet = true;
        else { usage(argv[0]); return 2; }
    }
    if (tty.empty() == ifname.empty()) { usage(argv[0]); return 2; }

    std::signal(SIGINT, on_sigint);
    std::signal(SIGTERM, on_sigint);

    std::unique_ptr<Transport> tp;
    try {
        if (!tty.empty()) tp = std::make_unique<Transport>(Transport::slcan(tty, baud));
#ifdef __linux__
        else              tp = std::make_unique<Transport>(Transport::socketcan(ifname));
#endif
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }

    std::ofstream csv;
    if (!csv_path.empty()) {
        csv.open(csv_path);
        csv << "t_us,id,ext,dlc,d0,d1,d2,d3,d4,d5,d6,d7\n";
    }

    std::map<uint32_t, IdStats> stats;
    const uint64_t t0 = sm_monotonic_us();
    sm_can_frame f;

    while (g_run) {
        int rc = tp->recv(f, 200);
        if (rc == SM_ETIMEOUT) continue;
        if (rc != SM_OK) { std::fprintf(stderr, "transport error %d\n", rc); break; }

        const bool rev = sm_is_sparkmax_frame(&f);
        const sm_frc_id id = sm_split_id(f.id);
        if (!rev && !all) continue;
        if (rev && dev_filter >= 0 && id.device != dev_filter && id.api != SM_API_HEARTBEAT) continue;

        auto& s = stats[f.id | (f.extended ? 0x80000000u : 0)];
        if (s.count) for (int i = 0; i < 8; ++i) s.changed_bits[i] |= s.last_data[i] ^ f.data[i];
        else s.first_us = f.ts_us;
        ++s.count; s.last_us = f.ts_us; s.last_dlc = f.dlc;
        std::memcpy(s.last_data, f.data, 8);

        if (csv.is_open()) {
            csv << (f.ts_us - t0) << ",0x" << std::hex << f.id << std::dec << ','
                << int(f.extended) << ',' << int(f.dlc);
            for (int i = 0; i < 8; ++i) csv << ',' << int(f.data[i]);
            csv << '\n';
        }
        if (quiet) continue;

        const double t = (f.ts_us - t0) / 1e6;
        if (!rev) {
            std::printf("%10.4f  %08X        %-14s %s\n", t, f.id, "(non-REV)", hex(f.data, f.dlc).c_str());
            continue;
        }

        std::printf("%10.4f  %08X dev%-2u %-14s %-24s", t, f.id, id.device,
                    sm_api_name(id.api), hex(f.data, f.dlc).c_str());
        switch (id.api) {
        case SM_API_STATUS_0: {
            sm_status0 s0;
            if (sm_decode_status0(&f, &s0) == SM_OK)
                std::printf("out=%+.3f faults=%04X sticky=%04X", s0.applied_output, s0.faults, s0.sticky_faults);
            break;
        }
        case SM_API_STATUS_1: {
            sm_status1 s1;
            if (sm_decode_status1(&f, &s1) == SM_OK)
                std::printf("vel=%.1frpm T=%uC V=%.2f I=%.2fA", s1.velocity_rpm, s1.temperature_c,
                            s1.bus_voltage, s1.output_current);
            break;
        }
        case SM_API_STATUS_2: {
            sm_status2 s2;
            if (sm_decode_status2(&f, &s2) == SM_OK) std::printf("pos=%.4frot", s2.position_rot);
            break;
        }
        case SM_API_HEARTBEAT: {
            std::printf("enabled:");
            for (int d = 0; d < 64; ++d)
                if (f.data[d >> 3] & (1u << (d & 7))) std::printf(" %d", d);
            break;
        }
        default:
            if (id.api >= SM_API_SETPOINT_DUTY && id.api <= SM_API_SETPOINT_SMART_MOT && f.dlc >= 4)
                std::printf("setpoint=%g slot=%u", sm_get_f32le(f.data), f.dlc > 6 ? f.data[6] & 3 : 0);
            else
                std::printf("[cls=%u idx=%u - unmapped]", id.api_class, id.api_index);
        }
        std::printf("\n");
    }

    // Rate + bit-activity table: the RE payoff. Bits that toggle while the
    // motor spins are your velocity/current fields; static bytes are flags/ids.
    std::fprintf(stderr, "\n%-9s %-16s %7s %8s  %-24s %s\n", "ID", "API", "count", "period", "last", "toggling bits");
    for (const auto& [key, s] : stats) {
        uint32_t raw = key & 0x1FFFFFFF;
        sm_frc_id id = sm_split_id(raw);
        double period = s.count > 1 ? (s.last_us - s.first_us) / 1e3 / double(s.count - 1) : 0;
        std::fprintf(stderr, "%08X  %-16s %7" PRIu64 " %6.1fms  %-24s %s\n", raw,
                     (key & 0x80000000u) && sm_split_id(raw).mfr == SM_MFR_REV ? sm_api_name(id.api) : "-",
                     s.count, period, hex(s.last_data, s.last_dlc).c_str(),
                     hex(s.changed_bits, s.last_dlc).c_str());
    }
    return 0;
}
