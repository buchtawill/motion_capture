// mocap-hdmi-drm: zero-copy HDMI display for the KV260 OV9281 capture pipeline.
//
// Successor to mocap-hdmi-memcp. Instead of a CPU copy into /dev/fb0, this
// drives the same PS DisplayPort controller through DRM/KMS: each V4L2 capture
// buffer is exported as a dmabuf and imported as an NV12 DRM framebuffer, then
// page-flipped so the DisplayPort DMA scans the frame straight out of the
// capture buffer -- no per-frame pixel copy.
//
// Grayscale-on-a-YUV-plane trick: the DP scanout plane has no mono format but
// it does have a hardware CSC. So each frame is presented as semi-planar NV12
// where the luma plane IS the Y8 capture buffer (zero-copy) and the chroma
// plane is a single shared buffer filled with 0x80 (neutral chroma). The DP
// hardware converts Y + neutral chroma into grayscale RGB during scanout.
//
// Single-threaded: one poll() loop services both the V4L2 fd (new frames) and
// the DRM fd (page-flip completions), so buffer ownership is race-free. Frames
// are dropped whenever the display can't keep up -- only the newest dequeued
// buffer is ever scheduled, which is what keeps end-to-end lag low.
//
// No scaler exists in this path, so the chosen display mode must exactly match
// the capture resolution (default 1280x720 is both a sensor mode and a DP-1
// mode). A future mocap-hdmi-dma could add PL scaling / CSC.

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

#include <linux/videodev2.h>

#include <mocap/argparse.hpp>
#include <mocap/auto_exposure.hpp>
#include <mocap/isp_stats.hpp>
#include <mocap/ov9281_pipeline.hpp>

using namespace mocap;

// --- globals ----------------------------------------------------------------

static std::atomic<bool> g_quit{false};

static void on_signal(int) { g_quit = true; }

// --- DRM display state ------------------------------------------------------

struct DrmDisplay {
    int fd = -1;
    uint32_t conn_id = 0;
    uint32_t crtc_id = 0;
    drmModeModeInfo mode{};
    drmModeCrtc *saved_crtc = nullptr; // to restore fbcon on exit
    bool master = false;

    // Shared constant-0x80 chroma buffer (one for all frames).
    uint32_t chroma_handle = 0;
    uint32_t chroma_stride = 0;
    uint8_t *chroma_map = nullptr;
    size_t chroma_size = 0;
};

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
              ". Must have a matching display mode (no scaler)");
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

    if (!mode_arg.empty()) {
        const SensorMode *m = find_mode(mode_arg);
        if (!m)
            die("unknown --mode '" + mode_arg + "'; valid: " +
                sensor_modes_str());
        width = m->width;
        height = m->height;
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

    // --- V4L2 capture -------------------------------------------------------

    int vfd = open(device.c_str(), O_RDWR, 0);
    if (vfd == -1)
        fail("open " + device);

    v4l2_capability cap{};
    if (xioctl(vfd, VIDIOC_QUERYCAP, &cap) == -1)
        fail("VIDIOC_QUERYCAP");
    const bool mplane = cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE;
    const auto buf_type = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                 : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    v4l2_format fmt{};
    fmt.type = buf_type;
    if (mplane) {
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = pixfmt;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
        fmt.fmt.pix_mp.num_planes = 1;
    } else {
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = pixfmt;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
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

    std::cout << "Capture: " << gw << "x" << gh << " '" << fourcc_str(got_pixfmt)
              << "' stride " << bpl << (mplane ? " [mplane]" : "") << "\n";

    // --- request + mmap + export V4L2 buffers -------------------------------

    v4l2_requestbuffers req{};
    req.count = nbuf;
    req.type = buf_type;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(vfd, VIDIOC_REQBUFS, &req) == -1)
        fail("VIDIOC_REQBUFS");
    if (req.count < 4)
        die("got fewer than 4 V4L2 buffers");

    std::vector<MappedPlane> cpu(req.count);  // CPU map (for AE)
    std::vector<int> dmabuf_fd(req.count, -1); // exported dmabuf per buffer
    for (unsigned i = 0; i < req.count; ++i) {
        v4l2_buffer buf{};
        v4l2_plane planes[VIDEO_MAX_PLANES]{};
        buf.type = buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (mplane) {
            buf.length = n_planes;
            buf.m.planes = planes;
        }
        if (xioctl(vfd, VIDIOC_QUERYBUF, &buf) == -1)
            fail("VIDIOC_QUERYBUF");
        const size_t len = mplane ? planes[0].length : buf.length;
        const off_t off = mplane ? planes[0].m.mem_offset : buf.m.offset;
        void *m = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, vfd,
                       off);
        if (m == MAP_FAILED)
            fail("mmap v4l2 buffer");
        cpu[i] = {m, len};

        v4l2_exportbuffer expbuf{};
        expbuf.type = buf_type;
        expbuf.index = i;
        expbuf.plane = 0;
        expbuf.flags = O_RDONLY | O_CLOEXEC;
        if (xioctl(vfd, VIDIOC_EXPBUF, &expbuf) == -1)
            fail("VIDIOC_EXPBUF");
        dmabuf_fd[i] = expbuf.fd;

        if (xioctl(vfd, VIDIOC_QBUF, &buf) == -1)
            fail("VIDIOC_QBUF");
    }

    // --- DRM open + output selection ----------------------------------------

    DrmDisplay d;
    d.fd = open(drm_dev.c_str(), O_RDWR | O_CLOEXEC);
    if (d.fd == -1)
        fail("open " + drm_dev);
    if (drmSetMaster(d.fd) != 0)
        fail("drmSetMaster (is a compositor/other KMS client holding it?)");
    d.master = true;

    drm_pick_output(d, conn_name, gw, gh);
    d.saved_crtc = drmModeGetCrtc(d.fd, d.crtc_id); // for restore on exit
    drm_make_chroma(d, gw, gh);

    // Import each dmabuf and build an NV12 framebuffer: luma = capture buffer,
    // chroma = the shared constant-0x80 buffer.
    std::vector<uint32_t> fb_id(req.count, 0);
    for (unsigned i = 0; i < req.count; ++i) {
        uint32_t luma_handle = 0;
        if (drmPrimeFDToHandle(d.fd, dmabuf_fd[i], &luma_handle) != 0)
            fail("drmPrimeFDToHandle");
        uint32_t handles[4] = {luma_handle, d.chroma_handle, 0, 0};
        uint32_t pitches[4] = {bpl, d.chroma_stride, 0, 0};
        uint32_t offsets[4] = {0, 0, 0, 0};
        if (drmModeAddFB2(d.fd, gw, gh, DRM_FORMAT_NV12, handles, pitches,
                          offsets, &fb_id[i], 0) != 0) {
            std::cerr << prog_name()
                      << ": drmModeAddFB2 NV12 failed: " << strerror(errno)
                      << "\n  the DP plane likely does not support NV12. "
                         "CRTC plane formats:\n";
            drm_dump_primary_formats(d);
            die("cannot create NV12 scanout framebuffer");
        }
    }

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

    auto qbuf = [&](int idx) {
        v4l2_buffer b{};
        v4l2_plane pl[VIDEO_MAX_PLANES]{};
        b.type = buf_type;
        b.memory = V4L2_MEMORY_MMAP;
        b.index = static_cast<uint32_t>(idx);
        if (mplane) {
            b.length = n_planes;
            b.m.planes = pl;
        }
        if (xioctl(vfd, VIDIOC_QBUF, &b) == -1)
            fail("VIDIOC_QBUF (requeue)");
    };

    // Schedule the newest ready buffer if the display is idle.
    auto try_schedule = [&]() {
        if (fb_pending != -1 || fb_ready == -1)
            return;
        if (!crtc_set) {
            if (drmModeSetCrtc(d.fd, d.crtc_id, fb_id[fb_ready], 0, 0,
                               &d.conn_id, 1, &d.mode) != 0)
                fail("drmModeSetCrtc");
            crtc_set = true;
            fb_screen = fb_ready; // modeset is immediate, no flip event
            fb_ready = -1;
        } else {
            if (drmModePageFlip(d.fd, d.crtc_id, fb_id[fb_ready],
                                DRM_MODE_PAGE_FLIP_EVENT, &flip_done) != 0)
                fail("drmModePageFlip");
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
        pollfd pfds[2] = {{vfd, POLLIN, 0}, {d.fd, POLLIN, 0}};
        int pr = poll(pfds, 2, 500);
        if (pr == -1) {
            if (errno == EINTR)
                continue;
            fail("poll");
        }
        if (pr == 0)
            continue;

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
            buf.memory = V4L2_MEMORY_MMAP;
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
                if (ae)
                    ae->update(static_cast<const uint8_t *>(cpu[buf.index].start),
                               gw, gh, bpl);
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
                          << elapsed << "s\n";
                last_captured = captured;
                last_displayed = displayed;
                last_report = now;
            }
        }
    }

    // --- cleanup ------------------------------------------------------------

    if (xioctl(vfd, VIDIOC_STREAMOFF, &type_int) == -1)
        std::cerr << "warning: VIDIOC_STREAMOFF: " << strerror(errno) << "\n";

    // Restore the previous CRTC config so the console (fbcon) comes back.
    if (d.saved_crtc) {
        drmModeSetCrtc(d.fd, d.saved_crtc->crtc_id, d.saved_crtc->buffer_id,
                       d.saved_crtc->x, d.saved_crtc->y, &d.conn_id, 1,
                       &d.saved_crtc->mode);
        drmModeFreeCrtc(d.saved_crtc);
    }
    for (unsigned i = 0; i < req.count; ++i) {
        if (fb_id[i])
            drmModeRmFB(d.fd, fb_id[i]);
        if (dmabuf_fd[i] >= 0)
            close(dmabuf_fd[i]);
        if (cpu[i].start)
            munmap(cpu[i].start, cpu[i].length);
    }
    if (d.chroma_map)
        munmap(d.chroma_map, d.chroma_size);
    if (d.chroma_handle) {
        drm_mode_destroy_dumb dreq{};
        dreq.handle = d.chroma_handle;
        drmIoctl(d.fd, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
    }
    if (d.master)
        drmDropMaster(d.fd);
    close(d.fd);

    ae.reset();
    isp.reset();
    close(vfd);
    std::cout << "Stopped.\n";
    return 0;
}
