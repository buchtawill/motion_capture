// mocap-hdmi-memcp: HDMI display prototype for the KV260 OV9281 capture
// pipeline. "memcp" names the strategy: frames are copied CPU-side from the
// mmap'd V4L2 buffer straight into the mmap'd /dev/fb0 framebuffer, with no
// DMA engine involved. A future mocap-hdmi-dma app can replace the copy with
// a VDMA/display-DMA path without touching the capture/AE plumbing here.
//
// Capture thread continuously dequeues V4L2 frames, runs AE, and publishes
// each one into a single-slot "latest frame" (not a queue): the render loop
// always draws the newest frame available and silently drops anything that
// arrives before the previous one finished drawing. That is what keeps
// end-to-end lag low -- there is never a backlog to catch up on.

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/fb.h>
#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <mocap/argparse.hpp>
#include <mocap/auto_exposure.hpp>
#include <mocap/isp_stats.hpp>
#include <mocap/ov9281_pipeline.hpp>

using namespace mocap;

// --- single-slot "latest frame" ---------------------------------------------
//
// Depth-1 by design: a ring would let the render loop work through a backlog
// of stale frames, which is exactly the lag this app is trying to avoid.

struct LatestFrame {
    std::mutex mu;
    std::condition_variable cv;
    std::vector<uint8_t> data;
    uint64_t gen = 0;
    bool done = false;

    void publish(const void *p, size_t n) {
        std::lock_guard<std::mutex> lk(mu);
        data.assign(static_cast<const uint8_t *>(p),
                    static_cast<const uint8_t *>(p) + n);
        ++gen;
        cv.notify_one();
    }

    // Blocks until a generation newer than last_seen is published (or done).
    // On success, copies the frame into out and returns the new generation;
    // returns last_seen unchanged if woken by stop().
    uint64_t wait_new(uint64_t last_seen, std::vector<uint8_t> &out) {
        std::unique_lock<std::mutex> lk(mu);
        cv.wait_for(lk, std::chrono::milliseconds(200), [&] {
            return gen != last_seen || done;
        });
        if (done || gen == last_seen)
            return last_seen;
        out = data;
        return gen;
    }

    void stop() {
        std::lock_guard<std::mutex> lk(mu);
        done = true;
        cv.notify_all();
    }
};

// --- globals -----------------------------------------------------------------

static std::atomic<bool> g_quit{false};
static LatestFrame g_latest;
static std::atomic<uint64_t> g_captured{0};

static void on_signal(int) {
    g_quit = true;
    g_latest.stop();
}

// --- capture thread ----------------------------------------------------------

static void capture_loop(int fd, uint32_t buf_type, bool mplane,
                          unsigned n_planes,
                          std::vector<std::vector<MappedPlane>> &buffers,
                          AutoExposure *ae, unsigned frame_w, unsigned frame_h,
                          unsigned frame_stride) {
    while (!g_quit) {
        pollfd pfd{fd, POLLIN, 0};
        int pr = poll(&pfd, 1, 500);
        if (pr == -1) {
            if (errno == EINTR)
                continue;
            fail("poll");
        }
        if (pr == 0)
            continue;

        v4l2_buffer buf{};
        v4l2_plane planes[VIDEO_MAX_PLANES]{};
        buf.type = buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        if (mplane) {
            buf.length = n_planes;
            buf.m.planes = planes;
        }
        if (xioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            if (errno == EINTR)
                continue;
            fail("VIDIOC_DQBUF");
        }

        const size_t used = mplane ? planes[0].bytesused : buf.bytesused;
        if (used > 0) {
            if (ae)
                ae->update(
                    static_cast<const uint8_t *>(buffers[buf.index][0].start),
                    frame_w, frame_h, frame_stride);
            g_latest.publish(buffers[buf.index][0].start, used);
            g_captured.fetch_add(1, std::memory_order_relaxed);
        }

        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
            fail("VIDIOC_QBUF");
    }
}

// --- framebuffer -------------------------------------------------------------

struct Framebuffer {
    int fd = -1;
    uint8_t *mem = nullptr;
    size_t mem_len = 0;
    fb_var_screeninfo var{};
    fb_fix_screeninfo fix{};
    unsigned bytespp = 0;

    static Framebuffer open_dev(const std::string &path) {
        Framebuffer fb;
        fb.fd = open(path.c_str(), O_RDWR);
        if (fb.fd == -1)
            fail("open " + path);
        if (ioctl(fb.fd, FBIOGET_VSCREENINFO, &fb.var) == -1)
            fail("FBIOGET_VSCREENINFO");
        if (ioctl(fb.fd, FBIOGET_FSCREENINFO, &fb.fix) == -1)
            fail("FBIOGET_FSCREENINFO");
        fb.bytespp = fb.var.bits_per_pixel / 8;
        if (fb.bytespp < 2 || fb.bytespp > 4)
            die(path + ": unsupported framebuffer depth (" +
                std::to_string(fb.var.bits_per_pixel) + " bpp)");

        fb.mem_len = fb.fix.smem_len;
        void *m = mmap(nullptr, fb.mem_len, PROT_READ | PROT_WRITE, MAP_SHARED,
                       fb.fd, 0);
        if (m == MAP_FAILED)
            fail("mmap " + path);
        fb.mem = static_cast<uint8_t *>(m);

        std::cout << "Framebuffer: " << fb.var.xres << "x" << fb.var.yres
                   << " @ " << fb.var.bits_per_pixel << "bpp, stride "
                   << fb.fix.line_length << " bytes\n";
        return fb;
    }

    void close_dev() {
        if (mem)
            munmap(mem, mem_len);
        if (fd != -1)
            close(fd);
    }

    // Pack an 8-bit gray value into this framebuffer's native pixel format
    // using the RGB bitfield layout the kernel reports (works for both
    // RGB565 and XRGB8888-style 32bpp panels without hardcoding either).
    inline uint32_t pack_gray(uint8_t gray) const {
        auto chan = [&](const fb_bitfield &f) -> uint32_t {
            const uint32_t v = (f.length >= 8) ? gray : (gray >> (8 - f.length));
            return v << f.offset;
        };
        return chan(var.red) | chan(var.green) | chan(var.blue);
    }

    inline void put_pixel(unsigned x, unsigned y, uint32_t val) {
        uint8_t *dst = mem + y * fix.line_length + x * bytespp;
        for (unsigned i = 0; i < bytespp; ++i)
            dst[i] = static_cast<uint8_t>(val >> (8 * i));
    }

    void clear() { std::memset(mem, 0, mem_len); }
};

// --- blit ---------------------------------------------------------------------

struct BlitRect {
    unsigned x, y, w, h;
};

// Largest rect of aspect src_w:src_h that fits inside (fb_w, fb_h), centered.
static BlitRect fit_letterbox(unsigned src_w, unsigned src_h, unsigned fb_w,
                              unsigned fb_h) {
    const double scale =
        std::min(double(fb_w) / src_w, double(fb_h) / src_h);
    const unsigned dw = std::max(1u, unsigned(src_w * scale));
    const unsigned dh = std::max(1u, unsigned(src_h * scale));
    return {(fb_w - dw) / 2, (fb_h - dh) / 2, dw, dh};
}

// Nearest-neighbor blit of an 8-bpp gray frame into a destination rect.
static void blit_gray(Framebuffer &fb, const uint8_t *src, unsigned src_w,
                      unsigned src_h, unsigned src_stride,
                      const BlitRect &dst) {
    for (unsigned y = 0; y < dst.h; ++y) {
        const unsigned sy = y * src_h / dst.h;
        const uint8_t *row = src + sy * src_stride;
        for (unsigned x = 0; x < dst.w; ++x) {
            const unsigned sx = x * src_w / dst.w;
            fb.put_pixel(dst.x + x, dst.y + y, fb.pack_gray(row[sx]));
        }
    }
}

// --- main ----------------------------------------------------------------------

int main(int argc, char *argv[]) {
    prog_name() = "mocap-hdmi-memcp";
    argparse::ArgumentParser program("mocap-hdmi-memcp", "1.0");
    program.add_description(
        "HDMI display prototype for the KV260 OV9281 capture pipeline. "
        "Copies frames CPU-side from the V4L2 buffer into /dev/fb0, "
        "dropping frames it can't keep up with to minimize lag.");

    program.add_argument("-d", "--device")
        .default_value(std::string("/dev/video0"))
        .help("V4L2 capture device node");
    program.add_argument("--fb")
        .default_value(std::string("/dev/fb0"))
        .help("framebuffer device node");
    program.add_argument("--fit")
        .default_value(std::string("letterbox"))
        .help("'letterbox' (preserve aspect ratio) or 'stretch' (fill screen)");
    program.add_argument("--mode")
        .default_value(std::string(""))
        .help("capture mode (sets width/height): " + sensor_modes_str() +
              " (overrides -W/-H)");
    program.add_argument("-W", "--width")
        .default_value(1280u)
        .scan<'u', unsigned>()
        .help("frame width (ignored if --mode is given)");
    program.add_argument("-H", "--height")
        .default_value(800u)
        .scan<'u', unsigned>()
        .help("frame height (ignored if --mode is given)");
    program.add_argument("-f", "--format")
        .default_value(std::string("GREY"))
        .help("4-char V4L2 pixelformat fourcc (8-bpp RAW; GREY for mono)");
    program.add_argument("-b", "--buffers")
        .default_value(4u)
        .scan<'u', unsigned>()
        .help("V4L2 streaming buffer count");
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
        .help("suppress periodic rendered/dropped-frame stats line");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n" << program;
        return 1;
    }

    const std::string device = program.get<std::string>("--device");
    const std::string fb_path = program.get<std::string>("--fb");
    const std::string fit_arg = program.get<std::string>("--fit");
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

    if (fit_arg != "letterbox" && fit_arg != "stretch")
        die("--fit must be 'letterbox' or 'stretch', got '" + fit_arg + "'");

    if (!mode_arg.empty()) {
        const SensorMode *m = find_mode(mode_arg);
        if (!m)
            die("unknown --mode '" + mode_arg + "'; valid: " +
                sensor_modes_str());
        width = m->width;
        height = m->height;
    }

    // --- pipeline setup (shared with mocap-sanity / mocap-perf / mocap-server) --

    std::string sensor_path;
    if (!skip_setup) {
        uint32_t mbus_code;
        if (!mbus_arg.empty()) {
            mbus_code =
                static_cast<uint32_t>(std::stoul(mbus_arg, nullptr, 0));
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
        std::cerr << "warning: '" << fourcc_str(got_pixfmt)
                  << "' is not GREY; it will be displayed as raw 8-bit "
                     "luma (no debayering in this prototype)\n";

    std::cout << "Capture: " << gw << "x" << gh << " '" << fourcc_str(got_pixfmt)
              << "'" << (mplane ? " [mplane]" : "") << "\n";

    // --- mmap V4L2 buffers ---------------------------------------------------

    v4l2_requestbuffers req{};
    req.count = nbuf;
    req.type = buf_type;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(vfd, VIDIOC_REQBUFS, &req) == -1)
        fail("VIDIOC_REQBUFS");
    if (req.count < 2)
        die("need >= 2 V4L2 buffers");

    std::vector<std::vector<MappedPlane>> buffers(req.count);
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
        buffers[i].resize(n_planes);
        for (unsigned p = 0; p < n_planes; ++p) {
            const size_t len = mplane ? planes[p].length : buf.length;
            const off_t off =
                mplane ? planes[p].m.mem_offset : buf.m.offset;
            void *m = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED,
                           vfd, off);
            if (m == MAP_FAILED)
                fail("mmap");
            buffers[i][p] = {m, len};
        }
        if (xioctl(vfd, VIDIOC_QBUF, &buf) == -1)
            fail("VIDIOC_QBUF");
    }

    int type_int = buf_type;
    if (xioctl(vfd, VIDIOC_STREAMON, &type_int) == -1)
        fail("VIDIOC_STREAMON");

    // --- framebuffer ----------------------------------------------------------

    Framebuffer fb = Framebuffer::open_dev(fb_path);
    fb.clear();
    const BlitRect dst = (fit_arg == "letterbox")
                             ? fit_letterbox(gw, gh, fb.var.xres, fb.var.yres)
                             : BlitRect{0, 0, fb.var.xres, fb.var.yres};
    std::cout << "Display rect: " << dst.w << "x" << dst.h << " at (" << dst.x
              << "," << dst.y << ") [" << fit_arg << "]\n";

    // --- signals ----------------------------------------------------------------

    struct sigaction sa {};
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // --- ISP histogram (optional) -----------------------------------------------

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

    // --- auto-exposure ------------------------------------------------------------

    std::unique_ptr<AutoExposure> ae;
    AutoExposure *ae_ptr = nullptr;
    if (enable_ae) {
        if (sensor_path.empty())
            die("--ae requires pipeline setup (incompatible with --skip-setup)");
        AEConfig aecfg;
        aecfg.target_mean = ae_target;
        aecfg.speed = ae_speed;
        aecfg.interval = ae_interval;
        ae = AutoExposure::create(sensor_path, aecfg, isp.get());
        if (ae)
            ae_ptr = ae.get();
    }

    // --- capture thread -------------------------------------------------------------

    std::thread cap_thread(capture_loop, vfd, buf_type, mplane, n_planes,
                           std::ref(buffers), ae_ptr, gw, gh, bpl);

    // --- render loop (main thread) ---------------------------------------------------

    std::cout << "Displaying on " << fb_path << ". Ctrl-C to stop.\n";

    uint64_t seen_gen = 0;
    std::vector<uint8_t> frame;
    uint64_t rendered = 0;
    uint64_t last_captured = 0, last_rendered = 0;
    auto last_report = std::chrono::steady_clock::now();

    while (!g_quit) {
        const uint64_t new_gen = g_latest.wait_new(seen_gen, frame);
        if (new_gen == seen_gen)
            continue; // timed out or stop() was called; re-check g_quit
        seen_gen = new_gen;

        blit_gray(fb, frame.data(), gw, gh, bpl, dst);
        ++rendered;

        if (!quiet_stats) {
            const auto now = std::chrono::steady_clock::now();
            const double elapsed =
                std::chrono::duration<double>(now - last_report).count();
            if (elapsed >= 2.0) {
                const uint64_t captured = g_captured.load();
                const uint64_t d_captured = captured - last_captured;
                const uint64_t d_rendered = rendered - last_rendered;
                const uint64_t dropped =
                    d_captured > d_rendered ? d_captured - d_rendered : 0;
                std::cout << "rendered " << (d_rendered / elapsed)
                          << " fps, dropped " << dropped << " frames in "
                          << elapsed << "s\n";
                last_captured = captured;
                last_rendered = rendered;
                last_report = now;
            }
        }
    }

    // --- cleanup ----------------------------------------------------------------------

    g_quit = true;
    g_latest.stop();
    cap_thread.join();

    if (xioctl(vfd, VIDIOC_STREAMOFF, &type_int) == -1)
        std::cerr << "warning: VIDIOC_STREAMOFF: " << strerror(errno) << "\n";
    for (auto &bp : buffers)
        for (auto &pl : bp)
            munmap(pl.start, pl.length);
    ae.reset();
    isp.reset();
    fb.close_dev();
    close(vfd);
    std::cout << "Stopped.\n";
    return 0;
}
