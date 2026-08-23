// watchdog.hpp: the hardware-hang watchdog.
//
// A hang is not one failure but several, and each shows a different signature
// across four independent progress signals:
//   * the free-running cycle counter (raw PL clock / AXI-Lite liveness),
//   * FRAME_ID           (mocap blob+ownership pipeline published a frame),
//   * the UIO interrupt count in /proc/interrupts (the IRQ actually reached SW),
//   * the V4L2 capture sequence (frames are still arriving from the sensor).
// The watchdog samples all four on its own thread and classifies a stall so the
// log says *which* stage died instead of just "it froze".
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include <mocap/mocap_pipeline.hpp>

// Set by the watchdog when it detects a recoverable MOCAP PIPELINE HANG; the
// main thread consumes it and performs the soft reset, so every mocap register
// write (disable/reset/arm/arm_irq) stays on one thread (no cross-thread CMD
// race). This is a symptom-level mitigation to keep the field system alive --
// the real fix is in the RTL frame-sync (run_extractor frames by beat count, so
// a dropped snoop-FIFO beat under overload desyncs it forever).
extern std::atomic<bool> g_recover_req;

// Latest V4L2 capture progress, published by the render thread and read by the
// watchdog thread. Guarded by an internal mutex (the "semaphore" around buf
// access): the render thread writes it right after each VIDIOC_DQBUF; the
// watchdog snapshots it under the same lock. dq_count is a monotonic DQBUF
// tally that works even if a driver leaves buf.sequence at 0.
struct CaptureBeat {
    uint32_t sequence = 0;   // last v4l2_buffer.sequence
    uint64_t dq_count = 0;   // number of frames dequeued since STREAMON
};

void publish_capture_beat(uint32_t sequence);
CaptureBeat read_capture_beat();

// Resolve the /proc/interrupts label for a /dev/uioN device. uio_pdrv_genirq
// requests its IRQ under the platform device name (e.g. "a0010000.mocap"), which
// is the basename of /sys/class/uio/uioN/device. Falls back to the UIO "name".
std::string resolve_irq_label(const std::string &uio_dev);

// Sum the per-CPU interrupt counts on the first /proc/interrupts line whose text
// contains `label`. Returns -1 if the label is empty or no line matches.
long read_irq_count(const std::string &label);

// trigger_mode: when true (external-trigger capture), only CLOCK_DEAD is
// classified -- FRAME_ID/capture/IRQ "stalls" are expected during the legitimate
// gaps between FSIN pulses, so those classes (and their soft-reset request) are
// suppressed.
void watchdog_thread(mocap::MocapPipeline *mp, std::string irq_label,
                     bool trigger_mode = false);
