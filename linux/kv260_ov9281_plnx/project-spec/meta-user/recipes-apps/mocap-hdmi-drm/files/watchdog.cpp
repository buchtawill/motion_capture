#include "watchdog.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include <unistd.h>

#include "signals.hpp"

using namespace mocap;

std::atomic<bool> g_recover_req{false};

static std::mutex g_cap_mtx;
static CaptureBeat g_cap;

void publish_capture_beat(uint32_t sequence) {
    std::lock_guard<std::mutex> lk(g_cap_mtx);
    g_cap.sequence = sequence;
    g_cap.dq_count++;
}
CaptureBeat read_capture_beat() {
    std::lock_guard<std::mutex> lk(g_cap_mtx);
    return g_cap;
}

std::string resolve_irq_label(const std::string &uio_dev) {
    auto slash = uio_dev.rfind('/');
    std::string base = (slash == std::string::npos) ? uio_dev
                                                    : uio_dev.substr(slash + 1);
    if (base.empty())
        return {};
    char buf[512] = {};
    std::string link = "/sys/class/uio/" + base + "/device";
    ssize_t n = readlink(link.c_str(), buf, sizeof(buf) - 1);
    if (n > 0) {
        std::string t(buf, n);
        auto s = t.rfind('/');
        return (s == std::string::npos) ? t : t.substr(s + 1);
    }
    std::ifstream nf("/sys/class/uio/" + base + "/name");
    std::string name;
    if (nf && std::getline(nf, name) && !name.empty())
        return name;
    return {};
}

long read_irq_count(const std::string &label) {
    if (label.empty())
        return -1;
    std::ifstream f("/proc/interrupts");
    std::string line;
    while (std::getline(f, line)) {
        if (line.find(label) == std::string::npos)
            continue;
        // After the leading "NN:" come the per-CPU counts (integers); the first
        // non-integer token is the controller name, at which point `is >> v`
        // fails and stops the sum. That yields the total interrupt count.
        std::istringstream is(line);
        std::string irq_tok;
        if (!(is >> irq_tok)) // "NN:"
            continue;
        long total = 0, v;
        while (is >> v)
            total += v;
        return total;
    }
    return -1;
}

// Watchdog cadence and how long a signal may be frozen before we call it stalled.
static constexpr int kWdPollMs = 150;
static constexpr int kWdStallMs = 750;

void watchdog_thread(MocapPipeline *mp, std::string irq_label,
                     bool trigger_mode) {
    using clock = std::chrono::steady_clock;
    auto ms_since = [](clock::time_point t) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   clock::now() - t)
            .count();
    };

    const auto t0 = clock::now();
    // Last-changed timestamps per signal; seed to now so we don't false-trip in
    // the first window before any frames have flowed.
    clock::time_point last_frame_chg = t0, last_irq_chg = t0, last_cap_chg = t0;
    uint32_t prev_frame = mp->frame_id();
    long prev_irq = read_irq_count(irq_label);
    CaptureBeat prev_cap = read_capture_beat();
    uint64_t prev_cycles = mp->snapshot_cycles().cycles;

    enum State { HEALTHY, CLOCK_DEAD, PIPELINE_HANG, IRQ_LOST, CAPTURE_STALL };
    State state = HEALTHY;
    auto name = [](State s) {
        switch (s) {
        case CLOCK_DEAD:    return "PL CLOCK/RESET STUCK";
        case PIPELINE_HANG: return "MOCAP PIPELINE HANG";
        case IRQ_LOST:      return "IRQ DELIVERY LOST";
        case CAPTURE_STALL: return "CAPTURE/VDMA STALL";
        default:            return "healthy";
        }
    };

    while (!g_quit) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kWdPollMs));
        if (g_quit)
            break;

        const uint32_t frame = mp->frame_id();
        const uint32_t status = mp->status();
        const uint32_t dropped = mp->dropped_frames();
        const uint64_t cycles = mp->snapshot_cycles().cycles;
        const long irq = read_irq_count(irq_label);
        const CaptureBeat cap = read_capture_beat();

        if (frame != prev_frame) { prev_frame = frame; last_frame_chg = clock::now(); }
        if (irq != prev_irq)     { prev_irq = irq;     last_irq_chg = clock::now(); }
        if (cap.dq_count != prev_cap.dq_count) { prev_cap = cap; last_cap_chg = clock::now(); }

        const bool clock_dead = (cycles == prev_cycles); // 64-bit free-runner:
                                                         // any live clock moves it
        prev_cycles = cycles;

        const bool cam_alive     = ms_since(last_cap_chg)   < kWdStallMs;
        const bool frame_stalled = ms_since(last_frame_chg) > kWdStallMs;
        const bool irq_stalled   = ms_since(last_irq_chg)   > kWdStallMs;

        // Grace period: don't classify until the pipeline has had time to spin up
        // after STREAMON (first-frame latency must not read as a stall).
        if (ms_since(t0) < 2000)
            continue;

        // In external-trigger mode only a dead PL clock is a real fault; the
        // frame/capture/IRQ signals legitimately pause between FSIN pulses.
        State next = HEALTHY;
        if (clock_dead)
            next = CLOCK_DEAD;
        else if (trigger_mode)
            next = HEALTHY;       // suppress frame/capture/IRQ stall classes
        else if (cam_alive && frame_stalled)
            next = PIPELINE_HANG; // sensor delivering + clock alive, no new publish
        else if (!cam_alive)
            next = CAPTURE_STALL; // no frames from the sensor/VDMA at all
        else if (irq_stalled && irq >= 0)
            next = IRQ_LOST;      // HW progressing but the IRQ isn't reaching us

        if (next != state) {
            if (next == HEALTHY) {
                std::cerr << "[watchdog] recovered -> healthy\n";
            } else {
                std::cerr << "\n[watchdog] *** " << name(next) << " ***\n"
                          << "  cycles="   << cycles
                          << " frame_id="  << frame
                          << " dropped="   << dropped
                          << " irq="       << irq
                          << " v4l2_seq="  << cap.sequence
                          << " dq="        << cap.dq_count
                          << " status=0x"  << std::hex << status << std::dec
                          << "\n"
                          << "  BLOB_FIFO_OVFL=" << ((status & MOCAP_REGS__STATUS_REG__BLOB_FIFO_OVFL_bm) ? 1 : 0)
                          << " BLOB_OVERFLOW="   << ((status & MOCAP_REGS__STATUS_REG__BLOB_OVERFLOW_bm) ? 1 : 0)
                          << " OVERRUN="         << ((status & MOCAP_REGS__STATUS_REG__OVERRUN_bm) ? 1 : 0)
                          << "\n";
                if (next == PIPELINE_HANG)
                    std::cerr << "  -> sensor+clock alive but FRAME_ID frozen: "
                                 "blob/ownership FSM deadlock (or, if dropped is "
                                 "climbing with FIFO_OVFL, sustained overload).\n";
                else if (next == IRQ_LOST)
                    std::cerr << "  -> FRAME_ID moving but UIO irq count frozen: "
                                 "frame-done interrupt lost (UIO ack/arm window).\n";
                else if (next == CLOCK_DEAD)
                    std::cerr << "  -> cycle counter frozen: PL clock stopped or "
                                 "the block is held in reset.\n";
                else if (next == CAPTURE_STALL)
                    std::cerr << "  -> no new V4L2 frames: capture/VDMA upstream "
                                 "of mocap has stalled (not the mocap core).\n";
            }
            state = next;
            // Ask the main thread to soft-reset the block on a pipeline hang
            // (the only class a reset can clear; clock-dead/capture-stall/irq-
            // lost are not fixed by pulsing RESET).
            if (next == PIPELINE_HANG)
                g_recover_req.store(true);
        }
    }
}
