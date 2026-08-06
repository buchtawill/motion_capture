// mocap/blob_detect.hpp: blob-result readback for the mocap_wrapper block.
//
// This is the *blob* half of the driver, kept deliberately separate from the
// pipeline driver (mocap/mocap_pipeline.hpp) for modularity. It owns no
// resources: it is a thin *view* over the register mapping already opened by a
// MocapPipeline, reading only the blob-result registers. The pipeline driver
// owns the UIO device, the IRQ/ACK frame lifecycle, and all pipeline control;
// this class just reads whichever result bank is currently published.
//
// Usage (single-threaded app):
//     auto pipe  = mocap::MocapPipeline::discover();
//     mocap::BlobDetector blobs(*pipe);
//     pipe->arm(w, h, threshold);
//     for (;;) {
//         pipe->wait_frame();                 // block on frame-done IRQ
//         std::vector<mocap::Blob> v;
//         blobs.read_all(v);                  // read published bank
//         pipe->ack();                        // release buffer to HW
//         ... use v ...
//     }
//
// Blob descriptor field order matches the hardware / golden model:
//   count, sum_x, sum_y, xmin, xmax, ymin, ymax  (centroid = sum/count).

#pragma once

#include <cstdint>
#include <vector>

#include <mocap/mocap_pipeline.hpp>
#include <mocap/mocap_regs.h>

namespace mocap {

struct Blob {
    uint32_t count = 0;  // foreground pixel count (blob "mass")
    uint32_t sum_x = 0;  // sum of x over all pixels
    uint32_t sum_y = 0;  // sum of y over all pixels
    uint16_t xmin = 0, xmax = 0, ymin = 0, ymax = 0;  // bounding box (inclusive)
    float cx = 0.0f, cy = 0.0f;  // centroid = sum/count

    uint16_t width() const { return static_cast<uint16_t>(xmax - xmin + 1); }
    uint16_t height() const { return static_cast<uint16_t>(ymax - ymin + 1); }
};

class BlobDetector {
public:
    // Borrows the pipeline's register mapping; the pipeline must outlive this.
    explicit BlobDetector(MocapPipeline &pipe)
        : regs_(pipe.regs()), max_blobs_(pipe.max_blobs()) {}

    // Number of blobs in the currently published bank (from STATUS).
    uint32_t count() const {
        return (regs_->STATUS & MOCAP_REGS__STATUS_REG__BLOB_COUNT_bm) >>
               MOCAP_REGS__STATUS_REG__BLOB_COUNT_bp;
    }

    // True if the published frame overflowed the MAX_BLOBS label budget (sticky
    // until CMD.RESET). Blob table is then truncated -- draw what we have.
    bool overflow() const {
        return (regs_->STATUS & MOCAP_REGS__STATUS_REG__BLOB_OVERFLOW_bm) != 0;
    }

    // Read every blob descriptor in the published bank into `out`. Returns the
    // number appended. Must be called between MocapPipeline::wait_frame() and
    // ack() (that window is when hardware holds the bank for software).
    //
    // BLOB_ADDR auto-increment is disabled for the readback: with autoinc on,
    // reading BLOB_COUNT_RD would advance BLOB_ADDR mid-descriptor and the
    // seven fields would come from different indices (see the TB read_blobs
    // discipline). We set BLOB_ADDR explicitly per blob, then restore the
    // caller's CTRL. Safe now that CTRL holds no single-pulse bits.
    size_t read_all(std::vector<Blob> &out) {
        out.clear();
        uint32_t n = count();
        if (max_blobs_ && n > max_blobs_)
            n = max_blobs_;  // guard against a stale/garbage STATUS read

        const uint32_t saved_ctrl = regs_->CTRL;
        regs_->CTRL = saved_ctrl & ~MOCAP_REGS__CTRL_REG__BLOB_ADDR_AUTOINC_bm;

        out.reserve(n);
        for (uint32_t b = 0; b < n; ++b) {
            regs_->BLOB_ADDR = b;
            Blob blob;
            blob.count = regs_->BLOB_COUNT_RD;
            blob.sum_x = regs_->BLOB_SX;
            blob.sum_y = regs_->BLOB_SY;
            blob.xmin = static_cast<uint16_t>(regs_->BLOB_XMIN & 0xFFFF);
            blob.xmax = static_cast<uint16_t>(regs_->BLOB_XMAX & 0xFFFF);
            blob.ymin = static_cast<uint16_t>(regs_->BLOB_YMIN & 0xFFFF);
            blob.ymax = static_cast<uint16_t>(regs_->BLOB_YMAX & 0xFFFF);
            if (blob.count == 0)
                continue;  // empty/merged-away slot
            blob.cx = static_cast<float>(blob.sum_x) / blob.count;
            blob.cy = static_cast<float>(blob.sum_y) / blob.count;
            out.push_back(blob);
        }

        regs_->CTRL = saved_ctrl;  // restore autoinc setting
        return out.size();
    }

private:
    volatile mocap_regs_t *regs_ = nullptr;
    uint16_t max_blobs_ = 0;
};

}  // namespace mocap
