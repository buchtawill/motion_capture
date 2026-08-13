// mocap/mocap_pipeline.hpp: UIO-based Linux driver for the mocap_wrapper
// pipeline (fused ISP histogram + RLE blob detector, one AXI4-Lite slave, one
// frame-done interrupt).
//
// This is the *pipeline* half of the driver: it owns the UIO device (open,
// mmap, IRQ) and manages pipeline-level control -- resolution, enable,
// threshold, reset, the histogram readback, the free-running cycle counter,
// and the per-frame lifecycle (block on the frame-done IRQ, then RESULTS_ACK
// to release the hardware-owned result buffer). Blob-result readback lives in
// the separate, modular mocap::BlobDetector (blob_detect.hpp), which borrows
// this object's register mapping -- see that header.
//
// Register-map background (see agents/AAR_blob_detection.md): CTRL @ 0x00 holds
// only sticky R/W settings (ENABLE / *_AUTOINC / THRESHOLD); the write-only
// single-pulse commands (RESET / RESULTS_ACK / CYCLE_SNAPSHOT) live in the
// separate CMD register @ 0x68. Because pulses no longer share CTRL, a
// per-frame ACK never disturbs the settings, and we keep a simple CTRL shadow
// instead of a read-modify-write on every settings change.
//
// RAII -- acquire via open()/discover(), released in the destructor (munmap +
// close). Non-copyable, movable. Mirrors mocap/isp_stats.hpp.

#pragma once

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>

#include <mocap/mocap_regs.h>

namespace mocap {

typedef uint32_t mocap_hist_t[256];

// A coherent snapshot of the free-running 64-bit cycle counter.
struct MocapCycleSnapshot {
    uint64_t cycles;
};

class MocapPipeline {
public:
    static constexpr uint16_t NUM_BINS = 256;

    // --- Factory methods ----------------------------------------------------

    static std::unique_ptr<MocapPipeline> open(const std::string &uio_path) {
        int fd = ::open(uio_path.c_str(), O_RDWR | O_SYNC);
        if (fd == -1) {
            std::cerr << "MocapPipeline: cannot open " << uio_path << ": "
                      << strerror(errno) << "\n";
            return nullptr;
        }
        void *map = mmap(nullptr, kMapSize, PROT_READ | PROT_WRITE, MAP_SHARED,
                         fd, 0);
        if (map == MAP_FAILED) {
            std::cerr << "MocapPipeline: mmap failed: " << strerror(errno)
                      << "\n";
            ::close(fd);
            return nullptr;
        }
        std::cout << "MocapPipeline: opened " << uio_path << "\n";
        auto p = std::unique_ptr<MocapPipeline>(
            new MocapPipeline(fd, static_cast<volatile mocap_regs_t *>(map)));
        p->uio_dev_path_ = uio_path;
        return p;
    }

    static std::unique_ptr<MocapPipeline> discover(
        const std::string &name_substr = "mocap_wrapper") {
        std::string path = find_uio_device(name_substr);
        if (path.empty()) {
            std::cerr << "MocapPipeline: no UIO device matching '" << name_substr
                      << "'\n";
            return nullptr;
        }
        return open(path);
    }

    // --- Lifecycle ----------------------------------------------------------

    ~MocapPipeline() {
        if (regs_)
            munmap(const_cast<mocap_regs_t *>(regs_), kMapSize);
        if (fd_ != -1)
            ::close(fd_);
    }

    MocapPipeline(const MocapPipeline &) = delete;
    MocapPipeline &operator=(const MocapPipeline &) = delete;

    MocapPipeline(MocapPipeline &&o) noexcept
        : fd_(o.fd_), regs_(o.regs_), ctrl_shadow_(o.ctrl_shadow_) {
        o.fd_ = -1;
        o.regs_ = nullptr;
    }
    MocapPipeline &operator=(MocapPipeline &&o) noexcept {
        if (this != &o) {
            if (regs_)
                munmap(const_cast<mocap_regs_t *>(regs_), kMapSize);
            if (fd_ != -1)
                ::close(fd_);
            fd_ = o.fd_;
            regs_ = o.regs_;
            ctrl_shadow_ = o.ctrl_shadow_;
            o.fd_ = -1;
            o.regs_ = nullptr;
        }
        return *this;
    }

    // Borrowed register view for collaborators (e.g. BlobDetector). The
    // MocapPipeline retains ownership of the mapping for its whole lifetime.
    volatile mocap_regs_t *regs() const { return regs_; }

    // --- Configuration / control --------------------------------------------

    // Program resolution + threshold and start continuous capture. HW then
    // free-runs, publishing a coherent {histogram, blobs, frame_id} buffer and
    // raising the frame-done IRQ once per frame until disable().
    //
    // NOTE (inline pipeline): mocap_wrapper sits IN the CSI->VDMA datapath and
    // gates passthrough on ENABLE -- the blob core only accepts stream beats
    // while enabled and correctly framed. A capture app must therefore arm() the
    // block (with hres/vres matching the sensor frame) for video to reach the
    // sink at all; leaving it disabled stalls the stream. hres must be a multiple
    // of 4.
    void arm(uint16_t hres, uint16_t vres, uint8_t threshold) {
        regs_->HRES = hres;
        regs_->VRES = vres;
        ctrl_shadow_ = MOCAP_REGS__CTRL_REG__ENABLE_bm |
                       MOCAP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm |
                       MOCAP_REGS__CTRL_REG__BLOB_ADDR_AUTOINC_bm |
                       ((static_cast<uint32_t>(threshold)
                         << MOCAP_REGS__CTRL_REG__THRESHOLD_bp) &
                        MOCAP_REGS__CTRL_REG__THRESHOLD_bm);
        regs_->CTRL = ctrl_shadow_;
    }

    // Stop continuous capture after the current frame (settings preserved).
    void disable() {
        ctrl_shadow_ &= ~MOCAP_REGS__CTRL_REG__ENABLE_bm;
        regs_->CTRL = ctrl_shadow_;
    }

    // Update the foreground threshold (pixel >= THRESHOLD) in-place.
    void set_threshold(uint8_t threshold) {
        ctrl_shadow_ = (ctrl_shadow_ & ~MOCAP_REGS__CTRL_REG__THRESHOLD_bm) |
                       ((static_cast<uint32_t>(threshold)
                         << MOCAP_REGS__CTRL_REG__THRESHOLD_bp) &
                        MOCAP_REGS__CTRL_REG__THRESHOLD_bm);
        regs_->CTRL = ctrl_shadow_;
    }
    uint8_t threshold() const {
        return static_cast<uint8_t>((ctrl_shadow_ &
                                     MOCAP_REGS__CTRL_REG__THRESHOLD_bm) >>
                                    MOCAP_REGS__CTRL_REG__THRESHOLD_bp);
    }

    // Soft reset: clears counters/buffers/status/FRAME_ID/DROPPED_FRAMES. Does
    // NOT clear HRES/VRES/THRESHOLD/AUTOINC or the free-running cycle counter,
    // and is orthogonal to ENABLE -- if you want the block to land in READY,
    // disable() first (otherwise it re-arms immediately).
    void reset() { pulse_cmd(MOCAP_REGS__CMD_REG__RESET_bm); }

    void set_resolution(uint16_t hres, uint16_t vres) {
        regs_->HRES = hres;
        regs_->VRES = vres;
    }
    void get_resolution(uint16_t &hres, uint16_t &vres) const {
        hres = static_cast<uint16_t>(regs_->HRES & MOCAP_REGS__HRES_REG__HRES_bm);
        vres = static_cast<uint16_t>(regs_->VRES & MOCAP_REGS__VRES_REG__VRES_bm);
    }

    // Reports the synthesized MAX_BLOBS parameter (blob capacity per frame).
    uint16_t max_blobs() const {
        return static_cast<uint16_t>(regs_->MAX_BLOBS_CFG & 0xFFFF);
    }

    // --- Status -------------------------------------------------------------

    uint32_t status() const { return regs_->STATUS; }
    bool is_ready() const {
        return (regs_->STATUS & MOCAP_REGS__STATUS_REG__READY_bm) != 0;
    }
    bool results_valid() const {
        return (regs_->STATUS & MOCAP_REGS__STATUS_REG__RESULTS_VALID_bm) != 0;
    }
    bool frame_done_pending() const {
        return (regs_->STATUS & MOCAP_REGS__STATUS_REG__FRAME_DONE_IRQ_bm) != 0;
    }
    bool blob_overflow() const {
        return (regs_->STATUS & MOCAP_REGS__STATUS_REG__BLOB_OVERFLOW_bm) != 0;
    }
    bool overrun() const {
        return (regs_->STATUS & MOCAP_REGS__STATUS_REG__OVERRUN_bm) != 0;
    }
    // Sticky: the blob snoop FIFO dropped a beat because the blob core could not
    // keep up with a busy frame. The video passthrough is unaffected; only the
    // blob result for such a frame is best-effort/incomplete. Cleared by reset().
    bool blob_fifo_overflow() const {
        return (regs_->STATUS & MOCAP_REGS__STATUS_REG__BLOB_FIFO_OVFL_bm) != 0;
    }

    uint32_t frame_id() const { return regs_->FRAME_ID; }
    uint32_t dropped_frames() const { return regs_->DROPPED_FRAMES; }
    uint32_t pixel_sum() const { return regs_->PIXEL_SUM; }

    double avg_brightness() const {
        uint16_t h, v;
        get_resolution(h, v);
        uint32_t npix = static_cast<uint32_t>(h) * v;
        return npix ? static_cast<double>(pixel_sum()) / npix : 0.0;
    }

    // --- Per-frame lifecycle (IRQ / ACK) ------------------------------------

    // Block until the hardware publishes a new frame (frame-done IRQ), then
    // return true. The published buffer (histogram + blobs + FRAME_ID) is held
    // by hardware for software until ack(). Falls back to polling STATUS if the
    // UIO IRQ write() path is unavailable. timeout_ms < 0 blocks indefinitely.
    bool wait_frame(int timeout_ms = 1000) {
        if (frame_done_pending())
            return true;

        uint32_t reenable = 1;
        if (::write(fd_, &reenable, sizeof(reenable)) ==
            static_cast<ssize_t>(sizeof(reenable))) {
            pollfd pfd{fd_, POLLIN, 0};
            int ret = ::poll(&pfd, 1, timeout_ms);
            if (ret > 0) {
                uint32_t count;
                ::read(fd_, &count, sizeof(count));
                return frame_done_pending();
            }
            return frame_done_pending();
        }

        // No IRQ path: spin on STATUS.
        int spins = (timeout_ms < 0) ? 1 << 30 : timeout_ms * 100;
        for (int i = 0; i < spins; i++) {
            if (frame_done_pending())
                return true;
            usleep(10);
        }
        return false;
    }

    // Acknowledge/consume the published buffer: clears FRAME_DONE_IRQ and
    // releases the buffer back to hardware. Call after reading blobs/histogram.
    void ack() { regs_->CMD = MOCAP_REGS__CMD_REG__RESULTS_ACK_bm; }

    // --- Integration into an external poll() loop ---------------------------
    // For apps that already multiplex several fds (e.g. V4L2 + DRM), add uio_fd()
    // to the poll set with POLLIN. Sequence: arm_irq() once up front; when poll
    // reports POLLIN, drain_irq() to consume the event, then read blobs, ack(),
    // and arm_irq() again to re-enable the next frame-done interrupt.
    int uio_fd() const { return fd_; }

    // The /dev/uioN path this pipeline was opened on. Useful to a supervisor that
    // wants to correlate hardware progress against the UIO interrupt count in
    // /proc/interrupts (resolve the label via /sys/class/uio/uioN/device).
    const std::string &uio_dev_path() const { return uio_dev_path_; }
    bool arm_irq() {
        uint32_t one = 1;
        return ::write(fd_, &one, sizeof(one)) ==
               static_cast<ssize_t>(sizeof(one));
    }
    void drain_irq() {
        uint32_t count;
        ssize_t n = ::read(fd_, &count, sizeof(count));
        (void)n;
    }

    // --- Histogram readback (published bank) --------------------------------

    // Dump all 256 bins via the HIST_ADDR/HIST_DATA autoinc window. Valid
    // between wait_frame() and ack().
    void dump_histogram(mocap_hist_t &hist) {
        bool autoinc =
            (ctrl_shadow_ & MOCAP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm) != 0;
        if (!autoinc)
            set_hist_autoinc(true);
        regs_->HIST_ADDR = 0;
        for (int i = 0; i < NUM_BINS; i++)
            hist[i] = regs_->HIST_DATA & MOCAP_REGS__HIST_DATA_REG__HIST_DATA_bm;
        if (!autoinc)
            set_hist_autoinc(false);
    }

    // --- Cycle counter ------------------------------------------------------

    // Atomically snapshot the free-running 64-bit cycle counter (CMD pulse),
    // then read the two halves back (they never tear -- snap regs only change
    // on the pulse).
    MocapCycleSnapshot snapshot_cycles() {
        regs_->CMD = MOCAP_REGS__CMD_REG__CYCLE_SNAPSHOT_bm;
        uint32_t hi = regs_->CYCLE_SNAP_HI;
        uint32_t lo = regs_->CYCLE_SNAP_LO;
        return {(static_cast<uint64_t>(hi) << 32) | lo};
    }

    static double compute_fps(const MocapCycleSnapshot &t1,
                              const MocapCycleSnapshot &t2, uint32_t frames,
                              double clock_hz) {
        uint64_t cd = t2.cycles - t1.cycles;
        return cd ? (static_cast<double>(frames) * clock_hz) / cd : 0.0;
    }

private:
    static constexpr size_t kMapSize = 0x1000;
    int fd_ = -1;
    volatile mocap_regs_t *regs_ = nullptr;
    // Shadow of the sticky CTRL settings so a settings change never needs a
    // read of the (volatile) register, and pulses (in CMD) never touch it.
    uint32_t ctrl_shadow_ = MOCAP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm |
                            MOCAP_REGS__CTRL_REG__BLOB_ADDR_AUTOINC_bm |
                            (MOCAP_REGS__CTRL_REG__THRESHOLD_reset
                             << MOCAP_REGS__CTRL_REG__THRESHOLD_bp);

    std::string uio_dev_path_;

    MocapPipeline(int fd, volatile mocap_regs_t *regs) : fd_(fd), regs_(regs) {}

    void set_hist_autoinc(bool enable) {
        if (enable)
            ctrl_shadow_ |= MOCAP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm;
        else
            ctrl_shadow_ &= ~MOCAP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm;
        regs_->CTRL = ctrl_shadow_;
    }

    void pulse_cmd(uint32_t pulse_bm) { regs_->CMD = pulse_bm; }

    static std::string find_uio_device(const std::string &name_substr) {
        DIR *d = opendir("/sys/class/uio");
        if (!d)
            return {};
        for (dirent *de = readdir(d); de; de = readdir(d)) {
            if (de->d_name[0] == '.')
                continue;
            std::string npath =
                "/sys/class/uio/" + std::string(de->d_name) + "/name";
            int nfd = ::open(npath.c_str(), O_RDONLY);
            if (nfd == -1)
                continue;
            char buf[256] = {};
            ssize_t n = ::read(nfd, buf, sizeof(buf) - 1);
            ::close(nfd);
            if (n > 0) {
                if (buf[n - 1] == '\n')
                    buf[n - 1] = '\0';
                if (std::string(buf).find(name_substr) != std::string::npos) {
                    closedir(d);
                    return "/dev/" + std::string(de->d_name);
                }
            }
        }
        closedir(d);
        return {};
    }
};

}  // namespace mocap
