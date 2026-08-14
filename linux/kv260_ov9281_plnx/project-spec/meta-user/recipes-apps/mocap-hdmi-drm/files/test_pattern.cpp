#include "test_pattern.hpp"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>

#include <fcntl.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>

#include <mocap/ov9281_pipeline.hpp>

#include "drm_display.hpp"
#include "signals.hpp"

using namespace mocap;

int run_test_pattern(const std::string &drm_dev, const std::string &conn_name,
                      unsigned w, unsigned h) {
    DrmDisplay d;
    d.fd = open(drm_dev.c_str(), O_RDWR | O_CLOEXEC);
    if (d.fd == -1)
        fail("open " + drm_dev);
    if (drmSetMaster(d.fd) != 0)
        fail("drmSetMaster (is a compositor/other KMS client holding it?)");
    d.master = true;
    if (drmSetClientCap(d.fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0)
        fail("drmSetClientCap UNIVERSAL_PLANES");
    if (drmSetClientCap(d.fd, DRM_CLIENT_CAP_ATOMIC, 1) != 0)
        fail("drmSetClientCap ATOMIC");

    // The ZynqMP DPSUB video layer must exactly match the panel's active size
    // (kernel: "Layer width:height must be 1920:1080") and the CRTC mode can't
    // be changed to 720p ("failed to set mode: Function not implemented"). So
    // the NV12 overlay path only works at the native 1920x1080; ignore the
    // requested size here.
    w = 1920;
    h = 1080;

    drm_pick_output(d, conn_name, w, h);
    drm_pick_plane(d);
    drm_cache_props(d);
    d.saved_crtc = drmModeGetCrtc(d.fd, d.crtc_id);
    drm_make_chroma(d, w, h);

    // Luma dumb buffer filled with N vertical grayscale bars (0 .. 255).
    drm_mode_create_dumb creq{};
    creq.width = w;
    creq.height = h;
    creq.bpp = 8;
    if (drmIoctl(d.fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) != 0)
        fail("DRM_IOCTL_MODE_CREATE_DUMB (luma)");
    const uint32_t luma_handle = creq.handle, luma_stride = creq.pitch;
    drm_mode_map_dumb mreq{};
    mreq.handle = luma_handle;
    if (drmIoctl(d.fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0)
        fail("DRM_IOCTL_MODE_MAP_DUMB (luma)");
    uint8_t *lm = static_cast<uint8_t *>(mmap(nullptr, creq.size,
                                              PROT_READ | PROT_WRITE, MAP_SHARED,
                                              d.fd, mreq.offset));
    if (lm == MAP_FAILED)
        fail("mmap luma");
    const unsigned bars = 8;
    for (unsigned y = 0; y < h; ++y) {
        uint8_t *row = lm + static_cast<size_t>(y) * luma_stride;
        for (unsigned x = 0; x < w; ++x) {
            unsigned b = x * bars / w;                       // 0 .. bars-1
            row[x] = static_cast<uint8_t>(b * 255 / (bars - 1));
        }
    }

    uint32_t handles[4] = {luma_handle, d.chroma_handle, 0, 0};
    uint32_t pitches[4] = {luma_stride, d.chroma_stride, 0, 0};
    uint32_t offsets[4] = {0, 0, 0, 0};
    uint32_t fb = 0;
    if (drmModeAddFB2(d.fd, w, h, DRM_FORMAT_NV12, handles, pitches, offsets,
                      &fb, 0) != 0) {
        std::cerr << prog_name()
                  << ": drmModeAddFB2 NV12 failed: " << strerror(errno) << "\n";
        drm_dump_primary_formats(d);
        die("cannot create NV12 test framebuffer");
    }

    drm_atomic_modeset(d, fb);
    std::cout << "TEST: " << bars << " grayscale bars (Y=0..255, chroma=0x80) on "
              << conn_name << " via NV12 overlay. Ctrl-C to stop.\n";

    struct sigaction sa {};
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    while (!g_quit)
        pause(); // static image; wake on signal

    if (d.plane_id)
        drmModeSetPlane(d.fd, d.plane_id, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (d.mode_blob_id)
        drmModeDestroyPropertyBlob(d.fd, d.mode_blob_id);
    if (d.saved_crtc) {
        drmModeSetCrtc(d.fd, d.saved_crtc->crtc_id, d.saved_crtc->buffer_id,
                       d.saved_crtc->x, d.saved_crtc->y, &d.conn_id, 1,
                       &d.saved_crtc->mode);
        drmModeFreeCrtc(d.saved_crtc);
    }
    if (fb)
        drmModeRmFB(d.fd, fb);
    munmap(lm, creq.size);
    drm_mode_destroy_dumb dreq{};
    dreq.handle = luma_handle;
    drmIoctl(d.fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
    if (d.chroma_map)
        munmap(d.chroma_map, d.chroma_size);
    if (d.chroma_handle) {
        drm_mode_destroy_dumb cdr{};
        cdr.handle = d.chroma_handle;
        drmIoctl(d.fd, DRM_IOCTL_MODE_DESTROY_DUMB, &cdr);
    }
    if (d.master)
        drmDropMaster(d.fd);
    close(d.fd);
    std::cout << "Stopped.\n";
    return 0;
}
