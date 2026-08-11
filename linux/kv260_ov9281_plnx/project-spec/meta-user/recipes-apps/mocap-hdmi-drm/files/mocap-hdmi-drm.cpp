// mocap-hdmi-drm: near-zero-copy HDMI display for the KV260 OV9281 pipeline.
//
// The PS DisplayPort video layer on this SoC is fixed at the panel's native
// 1920x1080: the driver rejects any other layer size ("Layer width:height must
// be 1920:1080") and refuses to modeset the CRTC to a smaller mode. The OV9281
// only does <=1280x800, so a 1:1 full-screen buffer is impossible. Instead we
// invert the usual V4L2->DRM buffer ownership:
//
//   * DRM allocates a ring of 1920x1080 NV12 luma buffers (dumb buffers) and
//     exports each as a dma-buf.
//   * V4L2 imports those dma-bufs (VB2_DMABUF) and the capture DMA writes each
//     sensor line straight into the TOP-LEFT of a 1080p buffer, using an
//     oversized bytesperline (= the 1920-wide luma stride) as its hardware
//     stride. The buffer is pre-cleared to 0, so everything outside the (smaller)
//     camera image scans out as a black letterbox.
//   * The sensor DMA writes the frame and the DisplayPort DMA scans the SAME
//     buffer out -- no per-frame CPU copy of pixel data.
//
// Grayscale-on-NV12 trick (unchanged): the luma plane carries the Y8 image and a
// single shared chroma buffer is filled with 0x80 (neutral). The DP hardware CSC
// turns Y + neutral chroma into grayscale RGB during scanout.
//
// The video layer is composited UNDER the graphics (primary) layer, whose global
// alpha defaults to opaque; we set the primary plane's "alpha" to 0 so the video
// shows through (see drm_cache_props / zynqmp_disp.c).
//
// Single-threaded: one poll() loop services both the V4L2 fd (new frames) and
// the DRM fd (page-flip completions), so buffer ownership is race-free. Only the
// newest dequeued buffer is ever scheduled, dropping stale frames to keep lag
// low. Auto-exposure reads the luma buffer through DMA_BUF_IOCTL_SYNC so the CPU
// sees the DMA-written pixels coherently (a cached, unsynced read reports mean
// ~0 and pins AE).

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include <linux/dma-buf.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include <mocap/argparse.hpp>
#include <mocap/auto_exposure.hpp>
#include <mocap/blob_detect.hpp>
#include <mocap/isp_stats.hpp>
#include <mocap/mocap_pipeline.hpp>
#include <mocap/ov9281_pipeline.hpp>

using namespace mocap;

// --- globals ----------------------------------------------------------------

static std::atomic<bool> g_quit{false};

static void on_signal(int) { g_quit = true; }

// Bracket a CPU read of a dma-buf so the kernel makes the DMA-written bytes
// coherent for the CPU (invalidate on START, no-op on END for a read). The
// luma buffers are written by the capture DMA; without this the CPU may read a
// stale cached view (near-zero) and auto-exposure never converges. Best-effort:
// if the driver's exporter doesn't implement sync, the ioctl fails harmlessly.
static void dmabuf_sync_read(int fd, bool start) {
    dma_buf_sync s{};
    s.flags = (start ? DMA_BUF_SYNC_START : DMA_BUF_SYNC_END) | DMA_BUF_SYNC_READ;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &s);
}

// --- DRM display state ------------------------------------------------------

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
    uint32_t prop_primary_alpha = 0; // global-alpha prop on the graphics plane
    uint64_t primary_alpha_max = 0xFFFF; // "opaque" value for that prop (its range
                                         // max; DPSUB uses 0..255, not 0..0xFFFF)
    PlaneProps pp;
    uint32_t mode_blob_id = 0;

    // Blob-box overlay: the RGB primary/graphics plane, composited OVER the
    // video by the DPSUB blender. When a box overlay is active we repurpose this
    // plane (instead of hiding it) to carry an ARGB8888 buffer of red box
    // outlines, and set its global alpha opaque so per-pixel alpha shows only the
    // boxes over the video. box_fb_id == 0 keeps the legacy "hide primary" path.
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

// ---------------------------------------------------------------------------
// Blob-box overlay: a CPU-rendered ARGB8888 dumb buffer on the primary/graphics
// plane. Fully transparent except red box outlines drawn at detected-blob
// bounding boxes. Colors are 0xAARRGGBB packed (little-endian ARGB8888): opaque
// red = 0xFFFF0000. The capture is written 1:1 into the top-left of the 1920x1080
// scanout buffer, so blob (sensor-space) coordinates map straight to overlay
// pixels -- no scaling.
struct BoxOverlay {
    uint32_t handle = 0, fb_id = 0;
    uint8_t *map = nullptr;
    size_t size = 0;
    uint32_t stride = 0; // bytes per row
    unsigned w = 0, h = 0;
};

static BoxOverlay drm_make_box_overlay(DrmDisplay &d, unsigned w, unsigned h) {
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

// Draw a `thickness`-pixel rectangle outline (inclusive corners).
static void box_draw_rect(BoxOverlay &o, int x0, int y0, int x1, int y1,
                          int thickness, uint32_t c) {
    for (int t = 0; t < thickness; ++t) {
        box_hline(o, x0, x1, y0 + t, c);
        box_hline(o, x0, x1, y1 - t, c);
        box_vline(o, x0 + t, y0, y1, c);
        box_vline(o, x1 - t, y0, y1, c);
    }
}

// Clear all boxes (back to fully transparent) before redrawing a new frame.
static inline void box_clear(BoxOverlay &o) { std::memset(o.map, 0, o.size); }

// Look up a named property on a DRM object; returns its current value (or
// UINT64_MAX if absent) and, via out_id, its (driver-global) property id.
static uint64_t drm_prop(int fd, uint32_t obj_id, uint32_t obj_type,
                         const char *name, uint32_t *out_id = nullptr) {
    drmModeObjectProperties *props =
        drmModeObjectGetProperties(fd, obj_id, obj_type);
    if (!props)
        return UINT64_MAX;
    uint64_t val = UINT64_MAX;
    for (uint32_t i = 0; i < props->count_props; ++i) {
        drmModePropertyRes *p = drmModeGetProperty(fd, props->props[i]);
        if (!p)
            continue;
        if (std::strcmp(p->name, name) == 0) {
            val = props->prop_values[i];
            if (out_id)
                *out_id = p->prop_id;
            drmModeFreeProperty(p);
            break;
        }
        drmModeFreeProperty(p);
    }
    drmModeFreeObjectProperties(props);
    return val;
}

// Find a connected connector by name (e.g. "DP-1") and a mode exactly matching
// wxh. Fills conn_id, crtc_id, mode. Aborts with context on failure.
static void drm_pick_output(DrmDisplay &d, const std::string &conn_want,
                            unsigned w, unsigned h) {
    drmModeRes *res = drmModeGetResources(d.fd);
    if (!res)
        fail("drmModeGetResources");

    drmModeConnector *conn = nullptr;
    std::string conn_name;
    for (int i = 0; i < res->count_connectors; ++i) {
        drmModeConnector *c = drmModeGetConnector(d.fd, res->connectors[i]);
        if (!c)
            continue;
        char name[64];
        const char *type = drmModeGetConnectorTypeName(c->connector_type);
        snprintf(name, sizeof(name), "%s-%u",
                 type ? type : "Unknown", c->connector_type_id);
        if (c->connection == DRM_MODE_CONNECTED &&
            conn_want == name) {
            conn = c;
            conn_name = name;
            break;
        }
        drmModeFreeConnector(c);
    }
    if (!conn)
        die("no connected connector named \"" + conn_want + "\"");

    // Exact-size mode (no scaler in this path). Prefer the first match, which
    // libdrm lists highest-refresh/preferred first.
    const drmModeModeInfo *picked = nullptr;
    for (int i = 0; i < conn->count_modes; ++i)
        if (conn->modes[i].hdisplay == w && conn->modes[i].vdisplay == h) {
            picked = &conn->modes[i];
            break;
        }
    if (!picked) {
        std::string avail;
        for (int i = 0; i < conn->count_modes; ++i) {
            avail += (avail.empty() ? "" : ", ") +
                     std::to_string(conn->modes[i].hdisplay) + "x" +
                     std::to_string(conn->modes[i].vdisplay);
        }
        drmModeFreeConnector(conn);
        drmModeFreeResources(res);
        die("no " + std::to_string(w) + "x" + std::to_string(h) +
            " mode on " + conn_name + " (this path has no scaler; pick a "
            "capture size with a matching display mode). Available: " + avail);
    }
    d.conn_id = conn->connector_id;
    d.mode = *picked;

    // Resolve a CRTC: use the connector's current encoder/crtc if set, else the
    // first CRTC allowed by any of the connector's encoders.
    uint32_t crtc_id = 0;
    if (conn->encoder_id) {
        drmModeEncoder *enc = drmModeGetEncoder(d.fd, conn->encoder_id);
        if (enc) {
            if (enc->crtc_id)
                crtc_id = enc->crtc_id;
            drmModeFreeEncoder(enc);
        }
    }
    if (!crtc_id) {
        for (int i = 0; i < conn->count_encoders && !crtc_id; ++i) {
            drmModeEncoder *enc = drmModeGetEncoder(d.fd, conn->encoders[i]);
            if (!enc)
                continue;
            for (int j = 0; j < res->count_crtcs; ++j)
                if (enc->possible_crtcs & (1u << j)) {
                    crtc_id = res->crtcs[j];
                    break;
                }
            drmModeFreeEncoder(enc);
        }
    }
    if (!crtc_id) {
        drmModeFreeConnector(conn);
        drmModeFreeResources(res);
        die("no usable CRTC for " + conn_name);
    }
    d.crtc_id = crtc_id;
    for (int j = 0; j < res->count_crtcs; ++j)
        if (res->crtcs[j] == crtc_id) {
            d.crtc_index = j;
            break;
        }

    std::cout << "Display: " << conn_name << " (conn " << d.conn_id << ", crtc "
              << d.crtc_id << ") mode " << d.mode.hdisplay << "x"
              << d.mode.vdisplay << "@" << d.mode.vrefresh << "\n";

    drmModeFreeConnector(conn);
    drmModeFreeResources(res);
}

// Report the CRTC's primary-plane formats -- called only to produce a helpful
// message if the driver rejects an NV12 scanout (i.e. NV12 isn't supported).
static void drm_dump_primary_formats(DrmDisplay &d) {
    if (drmSetClientCap(d.fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) != 0)
        return;
    drmModePlaneRes *pr = drmModeGetPlaneResources(d.fd);
    if (!pr)
        return;
    for (uint32_t i = 0; i < pr->count_planes; ++i) {
        drmModePlane *p = drmModeGetPlane(d.fd, pr->planes[i]);
        if (!p)
            continue;
        // Only planes usable on our CRTC.
        bool on_crtc = false;
        drmModeRes *res = drmModeGetResources(d.fd);
        if (res) {
            for (int j = 0; j < res->count_crtcs; ++j)
                if (res->crtcs[j] == d.crtc_id &&
                    (p->possible_crtcs & (1u << j)))
                    on_crtc = true;
            drmModeFreeResources(res);
        }
        if (on_crtc) {
            std::cerr << "  plane " << p->plane_id << " formats:";
            for (uint32_t f = 0; f < p->count_formats; ++f) {
                char c[5] = {char(p->formats[f]), char(p->formats[f] >> 8),
                             char(p->formats[f] >> 16),
                             char(p->formats[f] >> 24), 0};
                std::cerr << " " << c;
            }
            std::cerr << "\n";
        }
        drmModeFreePlane(p);
    }
    drmModeFreePlaneResources(pr);
}

// Allocate the shared constant-chroma dumb buffer (NV12 UV plane: full width,
// half height, filled with 0x80 = neutral). Returns its GEM handle/stride.
static void drm_make_chroma(DrmDisplay &d, unsigned w, unsigned h) {
    drm_mode_create_dumb creq{};
    creq.width = w;
    creq.height = h / 2; // NV12 chroma is half-height, interleaved UV
    creq.bpp = 8;
    if (drmIoctl(d.fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) != 0)
        fail("DRM_IOCTL_MODE_CREATE_DUMB (chroma)");
    d.chroma_handle = creq.handle;
    d.chroma_stride = creq.pitch;
    d.chroma_size = creq.size;

    drm_mode_map_dumb mreq{};
    mreq.handle = d.chroma_handle;
    if (drmIoctl(d.fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0)
        fail("DRM_IOCTL_MODE_MAP_DUMB (chroma)");
    void *m = mmap(nullptr, d.chroma_size, PROT_READ | PROT_WRITE, MAP_SHARED,
                   d.fd, mreq.offset);
    if (m == MAP_FAILED)
        fail("mmap chroma");
    d.chroma_map = static_cast<uint8_t *>(m);
    std::memset(d.chroma_map, 0x80, d.chroma_size); // neutral U=V=128
}

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

static LumaSlot drm_make_luma_slot(DrmDisplay &d, unsigned w, unsigned h) {
    LumaSlot s;
    drm_mode_create_dumb creq{};
    creq.width = w;
    creq.height = h;
    creq.bpp = 8;
    if (drmIoctl(d.fd, DRM_IOCTL_MODE_CREATE_DUMB, &creq) != 0)
        fail("DRM_IOCTL_MODE_CREATE_DUMB (luma slot)");
    s.handle = creq.handle;
    s.pitch = creq.pitch;
    s.size = creq.size;

    drm_mode_map_dumb mreq{};
    mreq.handle = s.handle;
    if (drmIoctl(d.fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0)
        fail("DRM_IOCTL_MODE_MAP_DUMB (luma slot)");
    void *m = mmap(nullptr, s.size, PROT_READ | PROT_WRITE, MAP_SHARED, d.fd,
                   mreq.offset);
    if (m == MAP_FAILED)
        fail("mmap luma slot");
    s.map = static_cast<uint8_t *>(m);
    std::memset(s.map, 0, s.size); // black letterbox around the camera image

    // Export for V4L2 to import and DMA into (needs read+write access).
    if (drmPrimeHandleToFD(d.fd, s.handle, DRM_CLOEXEC | DRM_RDWR,
                           &s.dmabuf_fd) != 0)
        fail("drmPrimeHandleToFD (luma slot)");

    uint32_t handles[4] = {s.handle, d.chroma_handle, 0, 0};
    uint32_t pitches[4] = {s.pitch, d.chroma_stride, 0, 0};
    uint32_t offsets[4] = {0, 0, 0, 0};
    if (drmModeAddFB2(d.fd, w, h, DRM_FORMAT_NV12, handles, pitches, offsets,
                      &s.fb_id, 0) != 0) {
        std::cerr << prog_name() << ": drmModeAddFB2 NV12 (luma slot) failed: "
                  << strerror(errno) << "\n";
        drm_dump_primary_formats(d);
        die("cannot create NV12 scanout framebuffer");
    }
    return s;
}

// Pick the scanout plane. NV12 is only offered on an OVERLAY plane on this DP
// controller, so we search for the (overlay) plane usable on our CRTC whose
// format list contains NV12. Also records the primary plane so the modeset can
// disable it (its stale full-res RGB fb would otherwise reject our modeset).
static void drm_pick_plane(DrmDisplay &d) {
    drmModePlaneRes *pr = drmModeGetPlaneResources(d.fd);
    if (!pr)
        fail("drmModeGetPlaneResources (is DRM_CLIENT_CAP_UNIVERSAL_PLANES set?)");
    for (uint32_t i = 0; i < pr->count_planes; ++i) {
        drmModePlane *p = drmModeGetPlane(d.fd, pr->planes[i]);
        if (!p)
            continue;
        if (d.crtc_index < 0 ||
            !(p->possible_crtcs & (1u << d.crtc_index))) {
            drmModeFreePlane(p);
            continue;
        }
        uint64_t type =
            drm_prop(d.fd, p->plane_id, DRM_MODE_OBJECT_PLANE, "type");
        bool has_nv12 = false;
        for (uint32_t f = 0; f < p->count_formats; ++f)
            if (p->formats[f] == DRM_FORMAT_NV12)
                has_nv12 = true;
        if (type == DRM_PLANE_TYPE_PRIMARY && !d.primary_plane_id)
            d.primary_plane_id = p->plane_id;
        if (has_nv12 && !d.plane_id &&
            (type == DRM_PLANE_TYPE_OVERLAY || type == DRM_PLANE_TYPE_PRIMARY))
            d.plane_id = p->plane_id;
        drmModeFreePlane(p);
    }
    drmModeFreePlaneResources(pr);
    if (!d.plane_id) {
        std::cerr << prog_name() << ": no NV12-capable plane on crtc "
                  << d.crtc_id << ". Plane formats:\n";
        drm_dump_primary_formats(d);
        die("cannot scan out NV12 on this display");
    }
    std::cout << "Scanout plane " << d.plane_id << " (NV12 overlay)"
              << (d.primary_plane_id
                      ? ", primary " + std::to_string(d.primary_plane_id) +
                            " (disabled during display)"
                      : "")
              << "\n";
}

// Cache all atomic property ids we commit each frame. Aborts if any is missing.
static void drm_cache_props(DrmDisplay &d) {
    auto need = [&](uint32_t obj, uint32_t type, const char *name,
                    uint32_t *dst) {
        drm_prop(d.fd, obj, type, name, dst);
        if (!*dst)
            die(std::string("atomic property '") + name + "' not found");
    };
    need(d.crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID", &d.prop_crtc_mode_id);
    need(d.crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE", &d.prop_crtc_active);
    need(d.conn_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID", &d.prop_conn_crtc_id);
    // The DPSUB blends the video (overlay) layer UNDER the graphics (primary)
    // layer; global alpha defaults to fully-opaque graphics, so the video layer
    // is invisible until we set the graphics plane's "alpha" to 0. This prop
    // lives on the PRIMARY plane, not our overlay (driver: zynqmp_disp.c).
    if (d.primary_plane_id) {
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "alpha",
             &d.prop_primary_alpha);
        // Capture the alpha prop's range max = its "opaque" value. The Zynq
        // DPSUB global-alpha is 8-bit (0..255), NOT the 16-bit DRM standard
        // (0..0xFFFF): committing 0xFFFF exceeds the range and the atomic ioctl
        // rejects it with EINVAL *before* the check phase. Use the real max.
        drmModePropertyRes *ap = drmModeGetProperty(d.fd, d.prop_primary_alpha);
        if (ap) {
            if ((ap->flags & DRM_MODE_PROP_RANGE) && ap->count_values >= 2)
                d.primary_alpha_max = ap->values[1];
            drmModeFreeProperty(ap);
        }
    }
    // When we drive the box overlay, we also commit FB/placement on the primary
    // plane -- cache those prop ids too (only if a primary plane exists).
    if (d.primary_plane_id) {
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "FB_ID", &d.box_pp.fb_id);
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_ID",
             &d.box_pp.crtc_id);
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X", &d.box_pp.crtc_x);
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y", &d.box_pp.crtc_y);
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W", &d.box_pp.crtc_w);
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H", &d.box_pp.crtc_h);
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X", &d.box_pp.src_x);
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y", &d.box_pp.src_y);
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W", &d.box_pp.src_w);
        need(d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H", &d.box_pp.src_h);
    }
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "FB_ID", &d.pp.fb_id);
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_ID", &d.pp.crtc_id);
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X", &d.pp.crtc_x);
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y", &d.pp.crtc_y);
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W", &d.pp.crtc_w);
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H", &d.pp.crtc_h);
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X", &d.pp.src_x);
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y", &d.pp.src_y);
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W", &d.pp.src_w);
    need(d.plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H", &d.pp.src_h);
}

// One blocking atomic modeset: activate the CRTC at our mode, route the
// connector to it, disable the primary plane, and place the overlay plane
// (full CRTC, unscaled 1:1) with the first frame. No page-flip event.
static void drm_atomic_modeset(DrmDisplay &d, uint32_t fb) {
    if (drmModeCreatePropertyBlob(d.fd, &d.mode, sizeof(d.mode),
                                  &d.mode_blob_id) != 0)
        fail("drmModeCreatePropertyBlob");

    drmModeAtomicReq *req = drmModeAtomicAlloc();
    drmModeAtomicAddProperty(req, d.crtc_id, d.prop_crtc_mode_id, d.mode_blob_id);
    drmModeAtomicAddProperty(req, d.crtc_id, d.prop_crtc_active, 1);
    drmModeAtomicAddProperty(req, d.conn_id, d.prop_conn_crtc_id, d.crtc_id);
    const uint64_t w = d.mode.hdisplay, h = d.mode.vdisplay;
    const bool box_active =
        d.box_fb_id != 0 && d.primary_plane_id && d.primary_plane_id != d.plane_id;
    if (box_active) {
        // Repurpose the graphics/primary plane as the ARGB8888 box overlay,
        // full-CRTC 1:1, composited OVER the video. The Zynq DPSUB planes are
        // created can_position=false, so the plane MUST span the whole CRTC at
        // (0,0) -- a sub-region (e.g. capture-sized) fails the atomic check with
        // EINVAL. So the box FB is full-screen (1920x1080 ARGB = 8 MB); size CMA
        // accordingly (cma=256M in bootargs). Global alpha opaque; the ARGB
        // per-pixel alpha (0 except the red box outlines) lets the video show
        // through between boxes. Boxes are drawn only in the top-left capture
        // region, which is where all blobs live.
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.fb_id,
                                 d.box_fb_id);
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.crtc_id,
                                 d.crtc_id);
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.crtc_x, 0);
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.crtc_y, 0);
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.crtc_w, w);
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.crtc_h, h);
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.src_x, 0);
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.src_y, 0);
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.src_w, w << 16);
        drmModeAtomicAddProperty(req, d.primary_plane_id, d.box_pp.src_h, h << 16);
        if (d.prop_primary_alpha)
            drmModeAtomicAddProperty(req, d.primary_plane_id,
                                     d.prop_primary_alpha, d.primary_alpha_max);
    } else {
        if (d.primary_plane_id && d.primary_plane_id != d.plane_id) {
            drmModeAtomicAddProperty(req, d.primary_plane_id, d.pp.fb_id, 0);
            drmModeAtomicAddProperty(req, d.primary_plane_id, d.pp.crtc_id, 0);
        }
        // Make the graphics layer fully transparent so the video overlay shows
        // through the blender (the global-alpha default hides it otherwise).
        if (d.prop_primary_alpha)
            drmModeAtomicAddProperty(req, d.primary_plane_id, d.prop_primary_alpha,
                                     0);
    }
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.fb_id, fb);
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.crtc_id, d.crtc_id);
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.crtc_x, 0);
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.crtc_y, 0);
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.crtc_w, w);
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.crtc_h, h);
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.src_x, 0);
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.src_y, 0);
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.src_w, w << 16);
    drmModeAtomicAddProperty(req, d.plane_id, d.pp.src_h, h << 16);

    int r = drmModeAtomicCommit(d.fd, req, DRM_MODE_ATOMIC_ALLOW_MODESET,
                                nullptr);
    drmModeAtomicFree(req);
    if (r != 0)
        fail("drmModeAtomicCommit (modeset)");
}

// DRM sanity path (--test): drive the exact NV12/overlay/CSC scanout pipeline
// with a locally generated luma buffer of vertical grayscale bars -- no camera,
// no V4L2, no dmabuf export. If this shows bars but the live path is black, the
// fault is isolated to the V4L2->DRM buffer import, not the DP display path.
static int run_test_pattern(const std::string &drm_dev,
                            const std::string &conn_name, unsigned w,
                            unsigned h) {
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

// --- main -------------------------------------------------------------------

int main(int argc, char *argv[]) {
    prog_name() = "mocap-hdmi-drm";
    argparse::ArgumentParser program("mocap-hdmi-drm", "1.0");
    program.add_description(
        "Zero-copy HDMI display for the KV260 OV9281 capture pipeline. "
        "Page-flips V4L2 capture buffers onto the DisplayPort via DRM/KMS "
        "(NV12, hardware CSC for mono Y8), dropping frames to minimize lag.");

    program.add_argument("-d", "--device")
        .default_value(std::string("/dev/video0"))
        .help("V4L2 capture device node");
    program.add_argument("--drm")
        .default_value(std::string("/dev/dri/card0"))
        .help("DRM device node");
    program.add_argument("--connector")
        .default_value(std::string("DP-1"))
        .help("DRM connector name (KV260 HDMI enumerates as DP)");
    program.add_argument("--mode")
        .default_value(std::string("1280x720"))
        .help("capture mode (sets width/height): " + sensor_modes_str() +
              ". Displayed top-left in a 1920x1080 black letterbox (no scaler)");
    program.add_argument("-W", "--width")
        .default_value(1280u)
        .scan<'u', unsigned>()
        .help("frame width (ignored if --mode is given)");
    program.add_argument("-H", "--height")
        .default_value(720u)
        .scan<'u', unsigned>()
        .help("frame height (ignored if --mode is given)");
    program.add_argument("-f", "--format")
        .default_value(std::string("GREY"))
        .help("4-char V4L2 pixelformat fourcc (8-bpp mono; GREY)");
    program.add_argument("-b", "--buffers")
        .default_value(5u)
        .scan<'u', unsigned>()
        .help("V4L2 streaming buffer count (>=4; display can hold ~3)");
    program.add_argument("-m", "--media")
        .default_value(std::string("/dev/media0"))
        .help("media controller node");
    program.add_argument("--sensor-entity")
        .default_value(std::string("ov9281"))
        .help("sensor entity name substring");
    program.add_argument("--csi-entity")
        .default_value(std::string("mipi_csi2_rx_subsystem"))
        .help("CSI-RX entity name substring");
    program.add_argument("--mbus-code")
        .default_value(std::string(""))
        .help("override subdev mbus code (hex, e.g. 0x3013)");
    program.add_argument("--fps")
        .default_value(std::string("max"))
        .help("target frame rate: 'max' (default, = --vblank min) or a number");
    program.add_argument("--vblank")
        .default_value(std::string("min"))
        .help("sensor vertical blanking: 'min' (max fps), 'keep', or integer");
    program.add_argument("--skip-setup")
        .default_value(false)
        .implicit_value(true)
        .help("skip media/subdev configuration (also skips --vblank/--fps)");
    program.add_argument("--ae")
        .default_value(false)
        .implicit_value(true)
        .help("enable auto-exposure control loop");
    program.add_argument("--ae-target")
        .default_value(128.0)
        .scan<'g', double>()
        .help("target mean brightness (0-255)");
    program.add_argument("--ae-speed")
        .default_value(0.3)
        .scan<'g', double>()
        .help("convergence speed (0.0-1.0; lower = slower)");
    program.add_argument("--ae-interval")
        .default_value(15)
        .scan<'d', int>()
        .help("frames between control updates");
    program.add_argument("--isp")
        .default_value(false)
        .implicit_value(true)
        .help("use ISP HW histogram for auto-exposure (auto-discover UIO)");
    program.add_argument("--quiet-stats")
        .default_value(false)
        .implicit_value(true)
        .help("suppress periodic displayed/dropped-frame stats line");
    program.add_argument("--test")
        .default_value(false)
        .implicit_value(true)
        .help("DRM sanity mode: display static grayscale bars via the NV12 "
              "overlay path, no camera/pipeline (isolates DRM from capture)");
    program.add_argument("--no-blobs")
        .default_value(false)
        .implicit_value(true)
        .help("disable the mocap blob-detector red-box overlay (on by default "
              "when the mocap UIO device is present)");
    program.add_argument("--threshold")
        .default_value(128u)
        .scan<'u', unsigned>()
        .help("blob foreground threshold (0-255; pixel >= threshold)");
    program.add_argument("--box-thickness")
        .default_value(3u)
        .scan<'u', unsigned>()
        .help("red box outline thickness in pixels");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n" << program;
        return 1;
    }

    const std::string device = program.get<std::string>("--device");
    const std::string drm_dev = program.get<std::string>("--drm");
    const std::string conn_name = program.get<std::string>("--connector");
    const std::string mode_arg = program.get<std::string>("--mode");
    unsigned width = program.get<unsigned>("--width");
    unsigned height = program.get<unsigned>("--height");
    const uint32_t pixfmt = fourcc(program.get<std::string>("--format"));
    const unsigned nbuf = program.get<unsigned>("--buffers");
    const std::string media_dev = program.get<std::string>("--media");
    const std::string sensor_name = program.get<std::string>("--sensor-entity");
    const std::string csi_name = program.get<std::string>("--csi-entity");
    const std::string mbus_arg = program.get<std::string>("--mbus-code");
    const std::string fps_arg = program.get<std::string>("--fps");
    const std::string vblank_arg = program.get<std::string>("--vblank");
    const bool skip_setup = program.get<bool>("--skip-setup");
    const bool enable_ae = program.get<bool>("--ae");
    const double ae_target = program.get<double>("--ae-target");
    const double ae_speed = program.get<double>("--ae-speed");
    const int ae_interval = program.get<int>("--ae-interval");
    const bool enable_isp = program.get<bool>("--isp");
    const bool quiet_stats = program.get<bool>("--quiet-stats");
    const bool test_mode = program.get<bool>("--test");
    const bool blobs_disabled = program.get<bool>("--no-blobs");
    const unsigned blob_threshold = program.get<unsigned>("--threshold");
    const unsigned box_thickness = program.get<unsigned>("--box-thickness");

    if (!mode_arg.empty()) {
        const SensorMode *m = find_mode(mode_arg);
        if (!m)
            die("unknown --mode '" + mode_arg + "'; valid: " +
                sensor_modes_str());
        width = m->width;
        height = m->height;
    }

    // --- DRM sanity mode (no camera) ----------------------------------------
    if (test_mode) {
        std::cout << "TEST MODE: DRM NV12 grayscale bars at native 1920x1080 "
                     "(no camera/pipeline).\n";
        return run_test_pattern(drm_dev, conn_name, width, height);
    }
    if (nbuf < 4)
        die("--buffers must be >= 4 (display holds up to 3 in flight)");
    if (pixfmt != V4L2_PIX_FMT_GREY)
        die("mocap-hdmi-drm only supports GREY (mono Y8); got '" +
            fourcc_str(pixfmt) + "'");

    // --- pipeline setup (shared with the other mocap-* apps) ----------------

    std::string sensor_path;
    if (!skip_setup) {
        uint32_t mbus_code;
        if (!mbus_arg.empty()) {
            mbus_code = static_cast<uint32_t>(std::stoul(mbus_arg, nullptr, 0));
        } else {
            mbus_code = fourcc_to_mbus(pixfmt);
            if (!mbus_code)
                die("no default mbus code for '" + fourcc_str(pixfmt) +
                    "'; pass --mbus-code");
        }
        sensor_path = configure_subdevs(media_dev, sensor_name, csi_name,
                                        mbus_code, width, height);

        if (fps_arg != "max") {
            double target;
            try {
                target = std::stod(fps_arg);
            } catch (...) {
                die("--fps must be 'max' or a number, got '" + fps_arg + "'");
            }
            apply_fps(sensor_path, target, width, height);
        } else {
            apply_vblank(sensor_path, vblank_arg);
        }
    }

    // --- DRM open + output + 1080p NV12 luma ring ---------------------------
    //
    // Open DRM first: the DP video layer is locked to the panel's native size,
    // so we allocate the 1920x1080 luma ring here and hand its dma-bufs to V4L2
    // to DMA into. DISP_W/H is the fixed scanout size; the camera writes a
    // smaller region into the top-left.
    constexpr unsigned DISP_W = 1920, DISP_H = 1080;

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
        fail("drmSetClientCap ATOMIC (driver lacks atomic modesetting?)");

    drm_pick_output(d, conn_name, DISP_W, DISP_H);
    drm_pick_plane(d);
    drm_cache_props(d);
    d.saved_crtc = drmModeGetCrtc(d.fd, d.crtc_id); // for restore on exit
    drm_make_chroma(d, DISP_W, DISP_H);

    // The frame ring: nbuf luma buffers (each pre-cleared, exported for V4L2).
    std::vector<LumaSlot> slot(nbuf);
    for (unsigned i = 0; i < nbuf; ++i)
        slot[i] = drm_make_luma_slot(d, DISP_W, DISP_H);
    const uint32_t luma_stride = slot[0].pitch; // hardware stride for V4L2

    // --- V4L2 capture (imports the DRM luma dma-bufs) -----------------------

    int vfd = open(device.c_str(), O_RDWR, 0);
    if (vfd == -1)
        fail("open " + device);

    v4l2_capability cap{};
    if (xioctl(vfd, VIDIOC_QUERYCAP, &cap) == -1)
        fail("VIDIOC_QUERYCAP");
    const bool mplane = cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE;
    const auto buf_type = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                 : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // Force the capture stride to the 1920-wide luma pitch so the DMA writes
    // each sensor line into the top-left of the 1080p buffer (the interleaved
    // DMA's inter-line gap = stride - line bytes leaves the right side black).
    v4l2_format fmt{};
    fmt.type = buf_type;
    if (mplane) {
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = pixfmt;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
        fmt.fmt.pix_mp.plane_fmt[0].bytesperline = luma_stride;
        fmt.fmt.pix_mp.plane_fmt[0].sizeimage = luma_stride * height;
    } else {
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = pixfmt;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
        fmt.fmt.pix.bytesperline = luma_stride;
        fmt.fmt.pix.sizeimage = luma_stride * height;
    }
    if (xioctl(vfd, VIDIOC_S_FMT, &fmt) == -1)
        fail("VIDIOC_S_FMT");

    const unsigned n_planes = mplane ? fmt.fmt.pix_mp.num_planes : 1;
    const unsigned gw = mplane ? fmt.fmt.pix_mp.width : fmt.fmt.pix.width;
    const unsigned gh = mplane ? fmt.fmt.pix_mp.height : fmt.fmt.pix.height;
    const unsigned bpl = mplane ? fmt.fmt.pix_mp.plane_fmt[0].bytesperline
                                : fmt.fmt.pix.bytesperline;
    const uint32_t got_pixfmt =
        mplane ? fmt.fmt.pix_mp.pixelformat : fmt.fmt.pix.pixelformat;

    if (got_pixfmt != V4L2_PIX_FMT_GREY)
        die("driver did not accept GREY (got '" + fourcc_str(got_pixfmt) + "')");
    if (gw > DISP_W || gh > DISP_H)
        die("capture " + std::to_string(gw) + "x" + std::to_string(gh) +
            " exceeds the " + std::to_string(DISP_W) + "x" +
            std::to_string(DISP_H) + " scanout buffer");
    // The DMA stride must match the luma buffer pitch exactly or the image
    // shears; the driver may round bytesperline to its own alignment.
    if (bpl != luma_stride)
        die("driver forced stride " + std::to_string(bpl) + " != luma pitch " +
            std::to_string(luma_stride) + " (stride-alignment mismatch)");

    std::cout << "Capture: " << gw << "x" << gh << " '" << fourcc_str(got_pixfmt)
              << "' into " << DISP_W << "x" << DISP_H << " top-left, stride "
              << bpl << (mplane ? " [mplane]" : "") << "\n";

    // --- request buffers (DMABUF import) + queue the luma dma-bufs ----------

    v4l2_requestbuffers req{};
    req.count = nbuf;
    req.type = buf_type;
    req.memory = V4L2_MEMORY_DMABUF;
    if (xioctl(vfd, VIDIOC_REQBUFS, &req) == -1)
        fail("VIDIOC_REQBUFS (DMABUF import; does this driver support it?)");
    if (req.count < nbuf)
        die("driver gave " + std::to_string(req.count) +
            " DMABUF slots, fewer than the " + std::to_string(nbuf) +
            " luma buffers");

    // Queue a slot's luma dma-buf back to the capture DMA (also used to requeue
    // a buffer once it is no longer on screen).
    auto qbuf = [&](int i) {
        v4l2_buffer b{};
        v4l2_plane pl[VIDEO_MAX_PLANES]{};
        b.type = buf_type;
        b.memory = V4L2_MEMORY_DMABUF;
        b.index = static_cast<uint32_t>(i);
        if (mplane) {
            b.length = n_planes;
            b.m.planes = pl;
            pl[0].m.fd = slot[i].dmabuf_fd;
            pl[0].length = slot[i].size;
        } else {
            b.m.fd = slot[i].dmabuf_fd;
            b.length = slot[i].size;
        }
        if (xioctl(vfd, VIDIOC_QBUF, &b) == -1)
            fail("VIDIOC_QBUF (DMABUF)");
    };
    for (unsigned i = 0; i < nbuf; ++i)
        qbuf(static_cast<int>(i));

    // --- mocap blob detector + red-box overlay ------------------------------
    //
    // ORDER MATTERS: arm the mocap block BEFORE VIDIOC_STREAMON. The
    // mocap_wrapper is INLINE on the CSI->VDMA stream and gates passthrough on
    // CTRL.ENABLE -- run_extractor only asserts s_ready (drains the input FIFO)
    // while the blob core is enabled and framed at the correct HRES/VRES. If the
    // camera streams first, beats pile into a disabled block: the input FIFO
    // backpressures the CSI and the first frame's SOF framing races the arm. By
    // arming first (block sits in WAIT_SOF with an empty FIFO) the very first
    // streamed frame is captured and passed through cleanly from beat 0. So we
    // MUST arm whenever the block is present; --no-blobs only suppresses the
    // overlay, it does NOT skip arming. The mocap UIO device is REQUIRED: the
    // inline block gates video passthrough on CTRL.ENABLE, so if we can't find
    // and arm it, no frames ever reach VDMA (the CSI-2 RX line buffer just
    // fills) -- a missing UIO device is a hard error, not a transparent
    // fallback. Check that uio_pdrv_genirq is bound and the DT node's
    // compatible is "generic-uio" (see mocap-pipeline-overlay.dts).
    //
    // Blob path: the block detects blobs on exactly the frames we display. We add
    // its frame-done IRQ fd to the render poll() set; on each mocap frame we read
    // the published bounding boxes, ACK the buffer, and redraw the ARGB overlay.
    // The overlay plane FB is committed once (at modeset); we update its pixels
    // in place each frame (thin red lines -- any tearing is imperceptible).
    if (blob_threshold > 255)
        die("--threshold must be 0-255");
    std::unique_ptr<MocapPipeline> mocap_pipe;
    std::unique_ptr<BlobDetector> blobdet;
    BoxOverlay box{};
    int mocap_fd = -1;
    std::vector<Blob> blob_list;
    mocap_pipe = MocapPipeline::discover();
    if (!mocap_pipe)
        die("no mocap UIO device found (searched /sys/class/uio/*/name for "
            "'mocap_wrapper'). The inline mocap block gates video on "
            "CTRL.ENABLE, so it MUST be present and armed or no frames reach "
            "VDMA. Check: (1) 'lsmod | grep uio_pdrv_genirq' is loaded; "
            "(2) the DT node compatible is \"generic-uio\" (a stale .dtbo "
            "still reading \"xlnx,mocap-wrapper-1.0\" will not bind).");
    // Start from a known-clean HW state. A previous run may have been Ctrl-C'd
    // mid-frame or while still holding a published buffer (sw_owns=1), leaving
    // the FC FSM, bank ownership, sticky error bits, FRAME_ID and DROPPED_FRAMES
    // stale across relaunches. disable() drops ENABLE so the CMD.RESET pulse
    // lands the FSM in READY (rather than re-arming mid post-reset scrub); reset()
    // then clears all dynamic state + counters + stickies. arm() below starts a
    // fresh continuous capture. (HRES/VRES/THRESHOLD are re-programmed by arm(),
    // so nothing carries over except the free-running cycle counter, by design.)
    mocap_pipe->disable();
    mocap_pipe->reset();

    // Arm for video passthrough (mandatory) at the capture resolution,
    // BEFORE streaming starts so framing is clean from the first frame.
    mocap_pipe->arm(static_cast<uint16_t>(gw), static_cast<uint16_t>(gh),
                    static_cast<uint8_t>(blob_threshold));
    std::cout << "mocap pipeline armed " << gw << "x" << gh << ", threshold "
              << blob_threshold << ", MAX_BLOBS " << mocap_pipe->max_blobs()
              << "\n";
    if (!blobs_disabled) {
        blobdet.reset(new BlobDetector(*mocap_pipe));
        // Full-screen ARGB overlay: the DPSUB graphics plane is can_position=
        // false and must span the whole CRTC (see drm_atomic_modeset), so it
        // can't be shrunk to the capture region -- needs cma=256M for the 8 MB
        // buffer. Boxes are only ever drawn in the top-left capture area.
        box = drm_make_box_overlay(d, DISP_W, DISP_H);
        d.box_fb_id = box.fb_id; // makes the modeset enable the overlay
        mocap_fd = mocap_pipe->uio_fd();
        mocap_pipe->arm_irq(); // enable the first frame-done interrupt
        std::cout << "Blob overlay ON, box thickness " << box_thickness << "\n";
    } else {
        std::cout << "Blob overlay OFF (--no-blobs); block armed for video "
                     "passthrough only\n";
    }

    // Start the camera feed now that the inline mocap block is armed and framed.
    int type_int = buf_type;
    if (xioctl(vfd, VIDIOC_STREAMON, &type_int) == -1)
        fail("VIDIOC_STREAMON");

    // --- signals ------------------------------------------------------------

    struct sigaction sa {};
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // --- ISP histogram + auto-exposure (optional) ---------------------------

    std::unique_ptr<IspStats> isp;
    if (enable_isp) {
        isp = IspStats::discover();
        if (isp)
            isp->set_resolution(static_cast<uint16_t>(gw),
                                static_cast<uint16_t>(gh));
        else
            std::cerr << "warning: --isp requested but no UIO device found; "
                         "falling back to SW histogram\n";
    }

    std::unique_ptr<AutoExposure> ae;
    if (enable_ae) {
        if (sensor_path.empty())
            die("--ae requires pipeline setup (incompatible with --skip-setup)");
        AEConfig aecfg;
        aecfg.target_mean = ae_target;
        aecfg.speed = ae_speed;
        aecfg.interval = ae_interval;
        ae = AutoExposure::create(sensor_path, aecfg, isp.get());
    }

    // --- render loop --------------------------------------------------------
    //
    // Buffer states, tracked by index:
    //   fb_ready   = newest dequeued buffer, not yet scheduled (-1 if none)
    //   fb_pending = buffer committed to an in-flight page flip (-1 if none)
    //   fb_screen  = buffer currently scanned out (-1 until first modeset)
    // A buffer is owned by V4L2 (queued) unless it is one of the three above.

    std::cout << "Displaying on " << conn_name
              << " via DRM/KMS. Ctrl-C to stop.\n";

    int fb_ready = -1, fb_pending = -1, fb_screen = -1;
    bool crtc_set = false;
    bool flip_done = false; // set by the page-flip handler on completion

    // Schedule the newest ready buffer if the display is idle. The first frame
    // does a blocking atomic modeset; subsequent frames are non-blocking atomic
    // plane flips that fire a page-flip event on scanout completion.
    auto try_schedule = [&]() {
        if (fb_pending != -1 || fb_ready == -1)
            return;
        if (!crtc_set) {
            drm_atomic_modeset(d, slot[fb_ready].fb_id);
            crtc_set = true;
            fb_screen = fb_ready; // modeset is immediate, no flip event
            fb_ready = -1;
        } else {
            drmModeAtomicReq *req = drmModeAtomicAlloc();
            drmModeAtomicAddProperty(req, d.plane_id, d.pp.fb_id,
                                     slot[fb_ready].fb_id);
            int r = drmModeAtomicCommit(
                d.fd, req,
                DRM_MODE_ATOMIC_NONBLOCK | DRM_MODE_PAGE_FLIP_EVENT,
                &flip_done);
            drmModeAtomicFree(req);
            if (r != 0)
                fail("drmModeAtomicCommit (flip)");
            fb_pending = fb_ready;
            fb_ready = -1;
        }
    };

    // Non-capturing (function-pointer-compatible) flip handler: sets the bool
    // pointed to by the user_data we passed to drmModePageFlip.
    drmEventContext evctx{};
    evctx.version = DRM_EVENT_CONTEXT_VERSION;
    evctx.page_flip_handler =
        [](int, unsigned, unsigned, unsigned, void *user) {
            *static_cast<bool *>(user) = true;
        };

    uint64_t captured = 0, displayed = 0;
    uint64_t last_captured = 0, last_displayed = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (!g_quit) {
        pollfd pfds[3] = {
            {vfd, POLLIN, 0}, {d.fd, POLLIN, 0}, {mocap_fd, POLLIN, 0}};
        const nfds_t nfds = (mocap_fd >= 0) ? 3 : 2;
        int pr = poll(pfds, nfds, 500);
        if (pr == -1) {
            if (errno == EINTR)
                continue;
            fail("poll");
        }
        if (pr == 0)
            continue;

        // New mocap blob results (frame-done IRQ). Read the published bounding
        // boxes, release the buffer, re-arm the IRQ, and redraw the overlay.
        if (mocap_fd >= 0 && (pfds[2].revents & POLLIN)) {
            mocap_pipe->drain_irq();      // consume the UIO event
            blobdet->read_all(blob_list); // published bank (held until ack)
            // Best-effort blob path: the video passthrough is never stalled, so
            // a busy frame can drop beats from the blob snoop FIFO. Warn once so
            // it's visible without spamming; video is unaffected either way.
            static bool warned_blob_fifo_ovfl = false;
            if (!warned_blob_fifo_ovfl && mocap_pipe->blob_fifo_overflow()) {
                warned_blob_fifo_ovfl = true;
                std::cerr << "WARNING: STATUS.BLOB_FIFO_OVFL set -- the blob core "
                             "fell behind on a busy frame and dropped beats; blob "
                             "results are best-effort for such frames (video is "
                             "unaffected).\n";
            }
            mocap_pipe->ack();            // release buffer back to HW
            mocap_pipe->arm_irq();        // re-enable next frame-done IRQ
            box_clear(box);
            for (const Blob &b : blob_list)
                box_draw_rect(box, b.xmin, b.ymin, b.xmax, b.ymax,
                              static_cast<int>(box_thickness), 0xFFFF0000u);
        }

        // DRM page-flip completion.
        if (pfds[1].revents & POLLIN) {
            flip_done = false;
            drmHandleEvent(d.fd, &evctx); // sets flip_done via user_data
            if (flip_done && fb_pending != -1) {
                if (fb_screen != -1)
                    qbuf(fb_screen); // previous frame now off-screen
                fb_screen = fb_pending;
                fb_pending = -1;
                ++displayed;
                try_schedule();
            }
        }

        // New captured frame.
        if (pfds[0].revents & POLLIN) {
            v4l2_buffer buf{};
            v4l2_plane planes[VIDEO_MAX_PLANES]{};
            buf.type = buf_type;
            buf.memory = V4L2_MEMORY_DMABUF;
            if (mplane) {
                buf.length = n_planes;
                buf.m.planes = planes;
            }
            if (xioctl(vfd, VIDIOC_DQBUF, &buf) == -1) {
                if (errno == EINTR)
                    continue;
                fail("VIDIOC_DQBUF");
            }
            const size_t used = mplane ? planes[0].bytesused : buf.bytesused;
            if (used == 0) {
                qbuf(buf.index);
            } else {
                ++captured;
                if (ae) {
                    // Read the DMA-written luma coherently (a cached, unsynced
                    // read reports mean ~0 and pins AE). gw x gh top-left region.
                    dmabuf_sync_read(slot[buf.index].dmabuf_fd, true);
                    ae->update(slot[buf.index].map, gw, gh, luma_stride);
                    dmabuf_sync_read(slot[buf.index].dmabuf_fd, false);
                }
                // Keep only the newest ready buffer; drop the previous one.
                if (fb_ready != -1)
                    qbuf(fb_ready);
                fb_ready = static_cast<int>(buf.index);
                try_schedule();
            }
        }

        if (!quiet_stats) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration<double>(now - last_report).count();
            if (elapsed >= 2.0) {
                const uint64_t dc = captured - last_captured;
                const uint64_t dd = displayed - last_displayed;
                const uint64_t dropped = dc > dd ? dc - dd : 0;
                std::cout << "displayed " << (dd / elapsed) << " fps, dropped "
                          << dropped << " of " << dc << " captured in "
                          << elapsed << "s";
                // Luma diagnostic on the on-screen buffer (we own it, so it is
                // not being overwritten): subsampled min/mean/max. This tells
                // "dark" (low values) from "inverted" (a normal-looking mean but
                // reversed) without a host round-trip.
                if (fb_screen >= 0) {
                    const uint8_t *base = slot[fb_screen].map;
                    dmabuf_sync_read(slot[fb_screen].dmabuf_fd, true);
                    unsigned lo = 255, hi = 0;
                    uint64_t sum = 0, cnt = 0;
                    for (unsigned y = 0; y < gh; y += 8) {
                        const uint8_t *r = base + size_t(y) * luma_stride;
                        for (unsigned x = 0; x < gw; x += 8) {
                            uint8_t v = r[x];
                            lo = v < lo ? v : lo;
                            hi = v > hi ? v : hi;
                            sum += v;
                            ++cnt;
                        }
                    }
                    dmabuf_sync_read(slot[fb_screen].dmabuf_fd, false);
                    std::cout << "  luma[min/mean/max]=" << lo << "/"
                              << (cnt ? sum / cnt : 0) << "/" << hi;
                }
                std::cout << "\n";
                last_captured = captured;
                last_displayed = displayed;
                last_report = now;
            }
        }
    }

    // --- cleanup ------------------------------------------------------------

    if (xioctl(vfd, VIDIOC_STREAMOFF, &type_int) == -1)
        std::cerr << "warning: VIDIOC_STREAMOFF: " << strerror(errno) << "\n";

    // Stop continuous blob capture (settings preserved on the block).
    if (mocap_pipe)
        mocap_pipe->disable();

    // Turn our overlay plane off so it stops covering the console.
    if (d.plane_id)
        drmModeSetPlane(d.fd, d.plane_id, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    // Turn the box overlay (primary plane) off too.
    if (d.box_fb_id && d.primary_plane_id)
        drmModeSetPlane(d.fd, d.primary_plane_id, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (d.mode_blob_id)
        drmModeDestroyPropertyBlob(d.fd, d.mode_blob_id);

    // Restore the previous CRTC config so the console (fbcon) comes back.
    if (d.saved_crtc) {
        drmModeSetCrtc(d.fd, d.saved_crtc->crtc_id, d.saved_crtc->buffer_id,
                       d.saved_crtc->x, d.saved_crtc->y, &d.conn_id, 1,
                       &d.saved_crtc->mode);
        drmModeFreeCrtc(d.saved_crtc);
    }
    for (unsigned i = 0; i < nbuf; ++i) {
        if (slot[i].fb_id)
            drmModeRmFB(d.fd, slot[i].fb_id);
        if (slot[i].dmabuf_fd >= 0)
            close(slot[i].dmabuf_fd);
        if (slot[i].map)
            munmap(slot[i].map, slot[i].size);
        if (slot[i].handle) {
            drm_mode_destroy_dumb dreq{};
            dreq.handle = slot[i].handle;
            drmIoctl(d.fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
        }
    }
    if (d.chroma_map)
        munmap(d.chroma_map, d.chroma_size);
    if (d.chroma_handle) {
        drm_mode_destroy_dumb dreq{};
        dreq.handle = d.chroma_handle;
        drmIoctl(d.fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
    }
    // Free the ARGB box overlay.
    if (box.fb_id)
        drmModeRmFB(d.fd, box.fb_id);
    if (box.map)
        munmap(box.map, box.size);
    if (box.handle) {
        drm_mode_destroy_dumb dreq{};
        dreq.handle = box.handle;
        drmIoctl(d.fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
    }
    if (d.master)
        drmDropMaster(d.fd);
    close(d.fd);

    ae.reset();
    isp.reset();
    blobdet.reset();
    mocap_pipe.reset();
    close(vfd);
    std::cout << "Stopped.\n";
    return 0;
}
