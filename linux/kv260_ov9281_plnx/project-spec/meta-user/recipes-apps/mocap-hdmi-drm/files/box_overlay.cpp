#include "box_overlay.hpp"

#include <algorithm>
#include <cstring>

#include <sys/mman.h>

#include <mocap/ov9281_pipeline.hpp>

using namespace mocap;

BoxOverlay drm_make_box_overlay(DrmDisplay &d, unsigned w, unsigned h) {
    BoxOverlay o;
    o.w = w;
    o.h = h;
    drm_mode_create_dumb creq{};
    creq.width = w;
    creq.height = h;
    creq.bpp = 32;
    if (drmIoctl(d.fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) != 0)
        fail("DRM_IOCTL_MODE_CREATE_DUMB (box overlay)");
    o.handle = creq.handle;
    o.stride = creq.pitch;
    o.size = creq.size;
    drm_mode_map_dumb mreq{};
    mreq.handle = o.handle;
    if (drmIoctl(d.fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0)
        fail("DRM_IOCTL_MODE_MAP_DUMB (box overlay)");
    o.map = static_cast<uint8_t *>(mmap(nullptr, o.size, PROT_READ | PROT_WRITE,
                                        MAP_SHARED, d.fd, mreq.offset));
    if (o.map == MAP_FAILED)
        fail("mmap box overlay");
    std::memset(o.map, 0, o.size); // start fully transparent
    uint32_t handles[4] = {o.handle, 0, 0, 0};
    uint32_t pitches[4] = {o.stride, 0, 0, 0};
    uint32_t offsets[4] = {0, 0, 0, 0};
    if (drmModeAddFB2(d.fd, w, h, DRM_FORMAT_ARGB8888, handles, pitches, offsets,
                      &o.fb_id, 0) != 0)
        fail("drmModeAddFB2 ARGB8888 (box overlay) -- does the graphics plane "
             "support ARGB8888?");
    return o;
}

static inline void box_hline(BoxOverlay &o, int x0, int x1, int y, uint32_t c) {
    if (y < 0 || y >= static_cast<int>(o.h))
        return;
    x0 = std::max(0, x0);
    x1 = std::min(static_cast<int>(o.w) - 1, x1);
    uint32_t *row =
        reinterpret_cast<uint32_t *>(o.map + static_cast<size_t>(y) * o.stride);
    for (int x = x0; x <= x1; ++x)
        row[x] = c;
}

static inline void box_vline(BoxOverlay &o, int x, int y0, int y1, uint32_t c) {
    if (x < 0 || x >= static_cast<int>(o.w))
        return;
    y0 = std::max(0, y0);
    y1 = std::min(static_cast<int>(o.h) - 1, y1);
    for (int y = y0; y <= y1; ++y)
        reinterpret_cast<uint32_t *>(o.map +
                                     static_cast<size_t>(y) * o.stride)[x] = c;
}

void box_draw_rect(BoxOverlay &o, int x0, int y0, int x1, int y1,
                   int thickness, uint32_t c) {
    for (int t = 0; t < thickness; ++t) {
        box_hline(o, x0, x1, y0 + t, c);
        box_hline(o, x0, x1, y1 - t, c);
        box_vline(o, x0 + t, y0, y1, c);
        box_vline(o, x1 - t, y0, y1, c);
    }
}
