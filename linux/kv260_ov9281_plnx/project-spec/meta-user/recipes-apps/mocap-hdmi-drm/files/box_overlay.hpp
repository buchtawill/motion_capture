// box_overlay.hpp: the blob-box overlay -- a CPU-rendered ARGB8888 dumb buffer
// on the primary/graphics plane. Fully transparent except red box outlines
// drawn at detected-blob bounding boxes. Colors are 0xAARRGGBB packed
// (little-endian ARGB8888): opaque red = 0xFFFF0000. The capture is written
// 1:1 into the top-left of the 1920x1080 scanout buffer, so blob (sensor-space)
// coordinates map straight to overlay pixels -- no scaling.
#pragma once

#include <cstdint>
#include <cstring>

#include "drm_display.hpp"

struct BoxOverlay {
    uint32_t handle = 0, fb_id = 0;
    uint8_t *map = nullptr;
    size_t size = 0;
    uint32_t stride = 0; // bytes per row
    unsigned w = 0, h = 0;
};

BoxOverlay drm_make_box_overlay(DrmDisplay &d, unsigned w, unsigned h);

// Draw a `thickness`-pixel rectangle outline (inclusive corners).
void box_draw_rect(BoxOverlay &o, int x0, int y0, int x1, int y1,
                    int thickness, uint32_t c);

// Clear all boxes (back to fully transparent) before redrawing a new frame.
inline void box_clear(BoxOverlay &o) { std::memset(o.map, 0, o.size); }
