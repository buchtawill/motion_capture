// mocap-perf: measure the maximum capture frame rate of the KV260 OV9281 pipeline.
//
// Configures the CSI->DMA graph (same as mocap-sanity), then streams as fast as
// the pipeline delivers, doing zero per-frame work (no disk I/O, immediate
// requeue). FPS is computed in software from CLOCK_MONOTONIC dequeue times.
//
// By default it drives the sensor to its maximum frame rate by minimising
// vertical blanking (V4L2_CID_VBLANK): fps = pixel_rate / (HTS * (H + vblank)),
// and the mainline ov9282 ships a large default vblank that ~halves the rate.
// Use --vblank to pick a specific value (larger = slower) or 'keep' the default.
//
// Reports a per-second instantaneous rate while running and, at the end, the
// steady-state average plus inter-frame interval min/max/mean (jitter). The
// measurement window starts at the SECOND frame so it excludes STREAMON latency.

#include <cerrno>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <mocap/argparse.hpp>
#include <mocap/ov9281_pipeline.hpp>

// Pipeline config (topology walk, subdev formats, VBLANK), the fourcc helpers,
// xioctl/fail and MappedPlane all live in <mocap/ov9281_pipeline.hpp>.
using namespace mocap;

namespace {

double now_s() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Aggregate CPU jiffies from the "cpu" line of /proc/stat (summed over all
// cores). idle counts the idle + iowait fields. Idle % over an interval is
// (idle delta) / (total delta) * 100.
struct CpuTimes {
    unsigned long long total = 0;
    unsigned long long idle = 0;
};

CpuTimes read_cpu_times() {
    CpuTimes c;
    std::ifstream f("/proc/stat");
    std::string tag;
    if (f >> tag && tag == "cpu") {
        unsigned long long v, idle = 0, iowait = 0;
        for (unsigned idx = 0; f >> v; ++idx) {
            c.total += v;
            if (idx == 3) idle = v;       // idle
            else if (idx == 4) iowait = v;  // iowait
        }
        c.idle = idle + iowait;
    }
    return c;
}

// Idle percentage between two snapshots; 0 if the interval has no ticks.
double idle_pct(const CpuTimes &a, const CpuTimes &b) {
    const unsigned long long dt = b.total - a.total;
    return dt ? 100.0 * (b.idle - a.idle) / dt : 0.0;
}

}  // namespace

int main(int argc, char *argv[]) {
    prog_name() = "mocap-perf";  // prefix for shared pipeline error messages
    argparse::ArgumentParser program("mocap-perf", "1.0");
    program.add_description(
        "Stream as fast as possible and measure capture FPS in software.");

    program.add_argument("-d", "--device")
        .default_value(std::string("/dev/video0"))
        .help("V4L2 capture device node");
    program.add_argument("-W", "--width")
        .default_value(1280u).scan<'u', unsigned>().help("frame width");
    program.add_argument("-H", "--height")
        .default_value(800u).scan<'u', unsigned>().help("frame height");
    program.add_argument("-f", "--format")
        .default_value(std::string("GREY"))
        .help("4-char V4L2 pixelformat fourcc (8-bpp RAW; GREY for mono)");
    program.add_argument("-t", "--duration")
        .default_value(10.0).scan<'g', double>()
        .help("measurement duration in seconds");
    program.add_argument("-b", "--buffers")
        .default_value(6u).scan<'u', unsigned>()
        .help("number of streaming buffers");
    program.add_argument("-m", "--media")
        .default_value(std::string("/dev/media0")).help("media controller node");
    program.add_argument("--sensor-entity")
        .default_value(std::string("ov9281")).help("sensor entity name substr");
    program.add_argument("--csi-entity")
        .default_value(std::string("mipi_csi2_rx_subsystem"))
        .help("CSI-RX entity name substr");
    program.add_argument("--mbus-code")
        .default_value(std::string("")).help("override subdev mbus code (hex)");
    program.add_argument("--vblank")
        .default_value(std::string("min"))
        .help("sensor vertical blanking lines: 'min' (max fps, default), "
              "'keep' (driver default), or an integer (larger = slower)");
    program.add_argument("--skip-setup")
        .default_value(false).implicit_value(true)
        .help("skip media/subdev configuration (also skips --vblank)");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n" << program;
        return 1;
    }

    const std::string device = program.get<std::string>("--device");
    const unsigned width = program.get<unsigned>("--width");
    const unsigned height = program.get<unsigned>("--height");
    const uint32_t pixfmt = fourcc(program.get<std::string>("--format"));
    const double duration = program.get<double>("--duration");
    const unsigned nbuf = program.get<unsigned>("--buffers");
    const std::string media_dev = program.get<std::string>("--media");
    const std::string sensor_name = program.get<std::string>("--sensor-entity");
    const std::string csi_name = program.get<std::string>("--csi-entity");
    const std::string mbus_arg = program.get<std::string>("--mbus-code");
    const std::string vblank_arg = program.get<std::string>("--vblank");
    const bool skip_setup = program.get<bool>("--skip-setup");

    if (!skip_setup) {
        uint32_t mbus_code;
        if (!mbus_arg.empty()) {
            mbus_code = static_cast<uint32_t>(std::stoul(mbus_arg, nullptr, 0));
        } else {
            mbus_code = fourcc_to_mbus(pixfmt);
            if (!mbus_code) {
                std::cerr << "mocap-perf: no default mbus code for '"
                          << fourcc_str(pixfmt) << "'; pass --mbus-code 0xNNNN\n";
                return 1;
            }
        }
        const std::string sensor_path = configure_subdevs(
            media_dev, sensor_name, csi_name, mbus_code, width, height);
        // After S_FMT (which resets per-mode controls), drive the frame rate.
        apply_vblank(sensor_path, vblank_arg);
    }

    // Blocking fd: a blocking DQBUF is the lowest-overhead way to pull frames;
    // poll() guards against a stalled pipeline.
    int fd = open(device.c_str(), O_RDWR, 0);
    if (fd == -1)
        fail("open " + device);

    v4l2_capability cap{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
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
    if (xioctl(fd, VIDIOC_S_FMT, &fmt) == -1)
        fail("VIDIOC_S_FMT");
    const unsigned n_planes = mplane ? fmt.fmt.pix_mp.num_planes : 1;
    {
        const uint32_t got =
            mplane ? fmt.fmt.pix_mp.pixelformat : fmt.fmt.pix.pixelformat;
        const unsigned gw = mplane ? fmt.fmt.pix_mp.width : fmt.fmt.pix.width;
        const unsigned gh = mplane ? fmt.fmt.pix_mp.height : fmt.fmt.pix.height;
        std::cout << "Video node: " << gw << "x" << gh << " '" << fourcc_str(got)
                  << "' " << (mplane ? "[mplane]" : "") << "\n";
    }

    v4l2_requestbuffers req{};
    req.count = nbuf;
    req.type = buf_type;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1)
        fail("VIDIOC_REQBUFS");
    if (req.count < 2) {
        std::cerr << "mocap-perf: need >=2 buffers, got " << req.count << "\n";
        return 1;
    }

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
        if (xioctl(fd, VIDIOC_QUERYBUF, &buf) == -1)
            fail("VIDIOC_QUERYBUF");
        buffers[i].resize(n_planes);
        for (unsigned p = 0; p < n_planes; ++p) {
            const size_t len = mplane ? planes[p].length : buf.length;
            const off_t off = mplane ? planes[p].m.mem_offset : buf.m.offset;
            void *m = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                           off);
            if (m == MAP_FAILED)
                fail("mmap");
            buffers[i][p] = {m, len};
        }
        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
            fail("VIDIOC_QBUF");
    }

    int type = buf_type;
    if (xioctl(fd, VIDIOC_STREAMON, &type) == -1)
        fail("VIDIOC_STREAMON");

    std::cout << "Streaming for " << duration << " s with " << req.count
              << " buffers...\n";

    // Capture loop. Time from the second frame to exclude STREAMON latency.
    uint64_t frames = 0;          // frames counted in the measurement window
    double t_start = 0;           // window start (at 2nd frame)
    double t_last_frame = 0;      // previous frame dequeue time
    double t_report = 0;          // last per-second report time
    uint64_t frames_since_report = 0;
    double iv_min = 1e9, iv_max = 0, iv_sum = 0;  // inter-frame interval stats
    bool started = false;
    CpuTimes cpu_window;  // snapshot at window start (whole-run idle %)
    CpuTimes cpu_report;  // snapshot at last per-second report

    for (;;) {
        pollfd pfd{fd, POLLIN, 0};
        int pr = poll(&pfd, 1, 2000);
        if (pr == -1) {
            if (errno == EINTR)
                continue;
            fail("poll");
        }
        if (pr == 0) {
            std::cerr << "mocap-perf: 2 s stall waiting for a frame\n";
            break;
        }

        v4l2_buffer buf{};
        v4l2_plane planes[VIDEO_MAX_PLANES]{};
        buf.type = buf_type;
        buf.memory = V4L2_MEMORY_MMAP;
        if (mplane) {
            buf.length = n_planes;
            buf.m.planes = planes;
        }
        if (xioctl(fd, VIDIOC_DQBUF, &buf) == -1)
            fail("VIDIOC_DQBUF");
        const double t = now_s();

        // Requeue immediately, BEFORE any bookkeeping, so this buffer rejoins
        // the DMA's incoming queue with minimal latency. With req.count >= 2
        // the VDMA always has at least one free target buffer to write the next
        // frame into while userspace handles this one -- N-buffer ping-pong, so
        // the DMA never stalls waiting on userspace and no frame period is lost.
        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
            fail("VIDIOC_QBUF");

        if (!started) {
            // First frame: prime timers and CPU snapshots, do not count.
            started = true;
            t_start = t;
            t_last_frame = t;
            t_report = t;
            cpu_window = cpu_report = read_cpu_times();
        } else {
            ++frames;
            ++frames_since_report;
            const double iv = t - t_last_frame;
            t_last_frame = t;
            if (iv < iv_min) iv_min = iv;
            if (iv > iv_max) iv_max = iv;
            iv_sum += iv;

            if (t - t_report >= 1.0) {
                const CpuTimes cpu_now = read_cpu_times();
                std::cout << "  " << frames_since_report / (t - t_report)
                          << " fps   cpu idle " << idle_pct(cpu_report, cpu_now)
                          << " %\n";
                cpu_report = cpu_now;
                frames_since_report = 0;
                t_report = t;
            }
        }

        if (started && (t - t_start) >= duration)
            break;
    }
    const CpuTimes cpu_end = read_cpu_times();  // whole-window idle reference

    if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1)
        fail("VIDIOC_STREAMOFF");
    for (auto &bufp : buffers)
        for (auto &pl : bufp)
            munmap(pl.start, pl.length);
    close(fd);

    const double elapsed = t_last_frame - t_start;
    std::cout << "\n=== results ===\n";
    if (frames == 0 || elapsed <= 0) {
        std::cerr << "mocap-perf: too few frames to measure\n";
        return 1;
    }
    std::cout << "frames (measured): " << frames << "\n";
    std::cout << "elapsed:           " << elapsed << " s\n";
    std::cout << "average:           " << frames / elapsed << " fps\n";
    std::cout << "inter-frame ms:    min " << iv_min * 1e3 << "  mean "
              << (iv_sum / frames) * 1e3 << "  max " << iv_max * 1e3 << "\n";
    std::cout << "cpu idle (avg):    " << idle_pct(cpu_window, cpu_end)
              << " % (all cores)\n";
    return 0;
}
