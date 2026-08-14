#include "drm_display.hpp"

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>

#include <sys/mman.h>

#include <mocap/ov9281_pipeline.hpp>

using namespace mocap;

uint64_t drm_prop(int fd, uint32_t obj_id, uint32_t obj_type, const char *name,
                   uint32_t *out_id) {
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

void drm_pick_output(DrmDisplay &d, const std::string &conn_want, unsigned w,
                      unsigned h) {
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

void drm_dump_primary_formats(DrmDisplay &d) {
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

void drm_make_chroma(DrmDisplay &d, unsigned w, unsigned h) {
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

LumaSlot drm_make_luma_slot(DrmDisplay &d, unsigned w, unsigned h) {
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

void drm_pick_plane(DrmDisplay &d) {
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

void drm_cache_props(DrmDisplay &d) {
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
        // Vendor "g_alpha_en" bool: selects global-alpha vs per-pixel-alpha blend
        // on the DPSUB graphics layer. Look it up non-fatally (drm_prop, not need)
        // so a driver without it still runs; the box overlay needs it CLEARED so
        // the ARGB per-pixel alpha (transparent except the box outlines) reveals
        // the video underneath -- setting only the alpha VALUE opaque hides video.
        drm_prop(d.fd, d.primary_plane_id, DRM_MODE_OBJECT_PLANE, "g_alpha_en",
                 &d.prop_primary_g_alpha_en);
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

void drm_atomic_modeset(DrmDisplay &d, uint32_t fb) {
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
        // accordingly (cma=256M in bootargs).
        //
        // Blend: the DPSUB graphics layer must use PER-PIXEL alpha here so the
        // transparent (A=0) regions of the ARGB buffer reveal the video and only
        // the box outlines paint. That means CLEARING g_alpha_en -- with global
        // alpha ENABLED the hardware blends the whole layer by the single global
        // value and discards per-pixel alpha, so an "opaque" global value renders
        // the full-screen buffer as solid black over the video (black screen with
        // boxes). The alpha VALUE is irrelevant once global alpha is disabled.
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
        // Per-pixel alpha: DISABLE global alpha so the ARGB alpha channel drives
        // the blend. Keep the global value opaque too (ignored while disabled, but
        // harmless and correct if a driver multiplies the two).
        if (d.prop_primary_g_alpha_en)
            drmModeAtomicAddProperty(req, d.primary_plane_id,
                                     d.prop_primary_g_alpha_en, 0);
        if (d.prop_primary_alpha)
            drmModeAtomicAddProperty(req, d.primary_plane_id,
                                     d.prop_primary_alpha, d.primary_alpha_max);
    } else {
        if (d.primary_plane_id && d.primary_plane_id != d.plane_id) {
            drmModeAtomicAddProperty(req, d.primary_plane_id, d.pp.fb_id, 0);
            drmModeAtomicAddProperty(req, d.primary_plane_id, d.pp.crtc_id, 0);
        }
        // No box overlay: hide the graphics layer entirely. Use GLOBAL alpha at
        // value 0 -- explicitly ENABLE global alpha (in case a prior state left it
        // per-pixel) so the whole layer is forced transparent regardless of the
        // buffer's own alpha, letting the video overlay show through.
        if (d.prop_primary_g_alpha_en)
            drmModeAtomicAddProperty(req, d.primary_plane_id,
                                     d.prop_primary_g_alpha_en, 1);
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
