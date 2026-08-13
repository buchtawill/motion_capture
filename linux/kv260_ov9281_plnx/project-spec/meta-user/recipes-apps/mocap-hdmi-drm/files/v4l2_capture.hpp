// v4l2_capture.hpp: V4L2 capture setup that imports the DRM-exported luma
// dma-bufs (VB2_DMABUF) instead of allocating its own buffers, plus the
// dma-buf CPU-coherency sync helper used both by V4L2 (AE reads) and the
// render loop's luma diagnostics.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "drm_display.hpp"

// Bracket a CPU read of a dma-buf so the kernel makes the DMA-written bytes
// coherent for the CPU (invalidate on START, no-op on END for a read). The
// luma buffers are written by the capture DMA; without this the CPU may read a
// stale cached view (near-zero) and auto-exposure never converges. Best-effort:
// if the driver's exporter doesn't implement sync, the ioctl fails harmlessly.
void dmabuf_sync_read(int fd, bool start);

struct V4l2Capture {
    int fd = -1;
    uint32_t buf_type = 0;
    bool mplane = false;
    unsigned n_planes = 1;
    unsigned width = 0, height = 0; // negotiated capture size (may differ from
                                    // the requested size only in error)
    unsigned bytesperline = 0;
    uint32_t pixfmt = 0;
};

// Open `device`, negotiate `width`x`height`/`pixfmt` with the capture stride
// forced to `luma_stride` (so the DMA lands in the top-left of the
// disp_w x disp_h scanout buffer), and validate the result. Prints the
// "Capture: ..." status line. Dies on any mismatch.
V4l2Capture v4l2_open_capture(const std::string &device, unsigned width,
                               unsigned height, uint32_t pixfmt,
                               uint32_t luma_stride, unsigned disp_w,
                               unsigned disp_h);

// VIDIOC_REQBUFS (DMABUF import) for nbuf slots, then queue every slot's
// dma-buf back to the capture DMA.
void v4l2_setup_dmabuf_buffers(V4l2Capture &cap, std::vector<LumaSlot> &slot,
                                unsigned nbuf);

// Queue slot i's luma dma-buf back to the capture DMA (also used to requeue a
// buffer once it is no longer on screen).
void v4l2_qbuf(const V4l2Capture &cap, const std::vector<LumaSlot> &slot,
               int i);
