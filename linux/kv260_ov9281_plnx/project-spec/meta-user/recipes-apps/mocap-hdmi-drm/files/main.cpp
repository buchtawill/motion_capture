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
//
// This file wires together the modules that carry the actual mechanics:
//   drm_display.{hpp,cpp}  -- device/plane discovery, atomic modeset
//   box_overlay.{hpp,cpp}  -- the ARGB blob-box overlay
//   v4l2_capture.{hpp,cpp} -- V4L2 dma-buf-import capture setup
//   watchdog.{hpp,cpp}     -- the hardware-hang watchdog thread
//   test_pattern.{hpp,cpp} -- the --test DRM sanity path
//   signals.{hpp,cpp}      -- shared Ctrl-C/SIGTERM handling

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include <mocap/argparse.hpp>
#include <mocap/auto_exposure.hpp>
#include <mocap/blob_detect.hpp>
#include <mocap/isp_stats.hpp>
#include <mocap/mocap_pipeline.hpp>
#include <mocap/ov9281_pipeline.hpp>

#include "box_overlay.hpp"
#include "drm_display.hpp"
#include "signals.hpp"
#include "test_pattern.hpp"
#include "v4l2_capture.hpp"
#include "watchdog.hpp"

using namespace mocap;

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

    V4l2Capture capture = v4l2_open_capture(device, width, height, pixfmt,
                                            luma_stride, DISP_W, DISP_H);
    const int vfd = capture.fd;
    const bool mplane = capture.mplane;
    const auto buf_type = capture.buf_type;
    const unsigned n_planes = capture.n_planes;
    const unsigned gw = capture.width;
    const unsigned gh = capture.height;

    // --- request buffers (DMABUF import) + queue the luma dma-bufs ----------

    v4l2_setup_dmabuf_buffers(capture, slot, nbuf);

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

    // --- hardware-hang watchdog ---------------------------------------------
    // Runs only when the frame-done IRQ path is active (blob overlay on), since
    // FRAME_ID/IRQ progress is only meaningful once we arm + ACK per frame.
    std::thread watchdog;
    if (mocap_fd >= 0) {
        std::string irq_label = resolve_irq_label(mocap_pipe->uio_dev_path());
        long seed = read_irq_count(irq_label);
        std::cout << "Watchdog: monitoring FRAME_ID/cycle/IRQ/V4L2-seq; UIO irq "
                     "label '" << irq_label << "' ("
                  << (seed >= 0 ? "matched /proc/interrupts"
                                : "NOT found -- IRQ-lost detection disabled")
                  << ")\n";
        watchdog = std::thread(watchdog_thread, mocap_pipe.get(), irq_label);
    }

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
    unsigned wd_recoveries = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (!g_quit) {
        // Watchdog-requested recovery: soft-reset the wedged mocap block. Done
        // on this (the only) thread that writes mocap registers, so it can't
        // race the ack/arm path. Symptom-level: it unwedges the FSM (disable so
        // RESET lands in READY, then re-arm continuous capture + the IRQ) and
        // drops the stale boxes; the underlying beat-drop desync is an RTL fix.
        if (g_recover_req.exchange(false)) {
            ++wd_recoveries;
            std::cerr << "[watchdog] soft-resetting mocap block (recovery #"
                      << wd_recoveries << ")\n";
            mocap_pipe->disable();
            mocap_pipe->reset();
            mocap_pipe->arm(static_cast<uint16_t>(gw), static_cast<uint16_t>(gh),
                            static_cast<uint8_t>(blob_threshold));
            if (mocap_fd >= 0)
                mocap_pipe->arm_irq();
            box_clear(box); // clear boxes drawn from the wedged frame
        }

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
                    v4l2_qbuf(capture, slot, fb_screen); // previous frame now off-screen
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
            // Publish capture progress for the watchdog (semaphore-guarded).
            publish_capture_beat(buf.sequence);
            const size_t used = mplane ? planes[0].bytesused : buf.bytesused;
            if (used == 0) {
                v4l2_qbuf(capture, slot, buf.index);
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
                    v4l2_qbuf(capture, slot, fb_ready);
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

    // Stop the watchdog first: it reads mocap registers, so join it before we
    // tear the pipeline down (g_quit is already set by the loop exit/signal).
    if (watchdog.joinable())
        watchdog.join();

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
