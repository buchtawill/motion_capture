// drm_display.hpp: DRM/KMS display state -- device/plane discovery, atomic
// property-id caching, the shared NV12 chroma buffer, per-frame luma "slot"
// dumb buffers (exported as dma-bufs for V4L2 to import), and the blocking
// atomic modeset that places the video overlay (and, optionally, the ARGB
// box-overlay primary plane) on the CRTC.
#pragma once

#include <cstdint>
#include <string>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

// Plane-property object ids (global per driver, so the same ids apply to every
// plane -- we cache them once from the chosen overlay plane).
struct PlaneProps {
    uint32_t fb_id = 0, crtc_id = 0;
    uint32_t crtc_x = 0, crtc_y = 0, crtc_w = 0, crtc_h = 0;
    uint32_t src_x = 0, src_y = 0, src_w = 0, src_h = 0;
};

struct DrmDisplay {
    int fd = -1;
    uint32_t conn_id = 0;
    uint32_t crtc_id = 0;
    int crtc_index = -1; // bit position of crtc_id in resources (for possible_crtcs)
    drmModeModeInfo mode{};
    drmModeCrtc *saved_crtc = nullptr; // to restore fbcon on exit
    bool master = false;

    // The NV12-capable scanout plane. On this DP controller NV12 lives on an
    // OVERLAY plane, not the RGB-only primary -- so we drive it via atomic KMS.
    uint32_t plane_id = 0;
    uint32_t primary_plane_id = 0; // disabled during our modeset

    // Cached atomic property ids.
    uint32_t prop_crtc_mode_id = 0, prop_crtc_active = 0;
    uint32_t prop_conn_crtc_id = 0;
    uint32_t prop_primary_alpha = 0; // global-alpha VALUE prop on the graphics plane
    uint64_t primary_alpha_max = 0xFFFF; // "opaque" value for that prop (its range
                                         // max; DPSUB uses 0..255, not 0..0xFFFF)
    uint32_t prop_primary_g_alpha_en = 0; // global-alpha ENABLE prop ("g_alpha_en").
                                          // When enabled, the DPSUB blends the
                                          // graphics layer with the global alpha
                                          // VALUE and IGNORES per-pixel alpha; when
                                          // disabled, per-pixel ARGB alpha is used.
    PlaneProps pp;
    uint32_t mode_blob_id = 0;

    // Blob-box overlay: the RGB primary/graphics plane, composited OVER the
    // video by the DPSUB blender. When a box overlay is active we repurpose this
    // plane (instead of hiding it) to carry an ARGB8888 buffer of red box
    // outlines, and switch the blend to PER-PIXEL alpha (g_alpha_en=0) so the
    // transparent regions reveal the video and only the boxes paint over it.
    // box_fb_id == 0 keeps the legacy "hide primary" path (global alpha = 0).
    PlaneProps box_pp;         // FB/CRTC/SRC prop ids on the primary plane
    uint32_t box_fb_id = 0;    // ARGB8888 overlay framebuffer (0 = disabled)

    // Shared constant-0x80 chroma buffer (one for all frames).
    uint32_t chroma_handle = 0;
    uint32_t chroma_stride = 0;
    uint8_t *chroma_map = nullptr;
    size_t chroma_size = 0;
};

#ifndef DRM_PLANE_TYPE_OVERLAY
#define DRM_PLANE_TYPE_OVERLAY 0
#define DRM_PLANE_TYPE_PRIMARY 1
#define DRM_PLANE_TYPE_CURSOR 2
#endif

// One 1920x1080 NV12 "frame slot" for the live path: a luma dumb buffer that the
// sensor VDMA writes into (exported as a dma-buf for V4L2 to import), paired with
// the shared constant-chroma buffer into an NV12 scanout framebuffer. The luma
// buffer is CPU-mapped (so AE can read it) and pre-cleared to 0 so the border
// around the smaller camera image scans out black. Fills the out-params.
struct LumaSlot {
    uint32_t handle = 0;
    uint8_t *map = nullptr;
    size_t size = 0;
    uint32_t pitch = 0;
    int dmabuf_fd = -1;
    uint32_t fb_id = 0;
};

// Look up a named property on a DRM object; returns its current value (or
// UINT64_MAX if absent) and, via out_id, its (driver-global) property id.
uint64_t drm_prop(int fd, uint32_t obj_id, uint32_t obj_type, const char *name,
                   uint32_t *out_id = nullptr);

// Find a connected connector by name (e.g. "DP-1") and a mode exactly matching
// wxh. Fills conn_id, crtc_id, mode. Aborts with context on failure.
void drm_pick_output(DrmDisplay &d, const std::string &conn_want, unsigned w,
                      unsigned h);

// Report the CRTC's primary-plane formats -- called only to produce a helpful
// message if the driver rejects an NV12 scanout (i.e. NV12 isn't supported).
void drm_dump_primary_formats(DrmDisplay &d);

// Allocate the shared constant-chroma dumb buffer (NV12 UV plane: full width,
// half height, filled with 0x80 = neutral). Returns its GEM handle/stride.
void drm_make_chroma(DrmDisplay &d, unsigned w, unsigned h);

LumaSlot drm_make_luma_slot(DrmDisplay &d, unsigned w, unsigned h);

// Pick the scanout plane. NV12 is only offered on an OVERLAY plane on this DP
// controller, so we search for the (overlay) plane usable on our CRTC whose
// format list contains NV12. Also records the primary plane so the modeset can
// disable it (its stale full-res RGB fb would otherwise reject our modeset).
void drm_pick_plane(DrmDisplay &d);

// Cache all atomic property ids we commit each frame. Aborts if any is missing.
void drm_cache_props(DrmDisplay &d);

// One blocking atomic modeset: activate the CRTC at our mode, route the
// connector to it, disable the primary plane, and place the overlay plane
// (full CRTC, unscaled 1:1) with the first frame. No page-flip event.
void drm_atomic_modeset(DrmDisplay &d, uint32_t fb);
