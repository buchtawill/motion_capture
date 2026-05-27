// mocap-perf: measure the maximum capture frame rate of the KV260 OV9281 pipeline.
//
// Configures the CSI->DMA graph (same as mocap-sanity), then streams as fast as
// the pipeline delivers, doing zero per-frame work (no disk I/O, immediate
// requeue). FPS is computed in software from CLOCK_MONOTONIC dequeue times.
//
// Reports a per-second instantaneous rate while running and, at the end, the
// steady-state average plus inter-frame interval min/max/mean (jitter). The
// measurement window starts at the SECOND frame so it excludes STREAMON latency.

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <linux/media.h>
#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>

#include "argparse.hpp"

namespace {

// Standard pad layout for this pipeline.
constexpr unsigned kSensorSrcPad = 0;
constexpr unsigned kCsiSinkPad = 0;
constexpr unsigned kCsiSrcPad = 1;

int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

[[noreturn]] void fail(const std::string &what) {
    std::cerr << "mocap-perf: " << what << ": " << std::strerror(errno) << " ("
              << errno << ")\n";
    std::exit(1);
}

double now_s() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

uint32_t fourcc(const std::string &s) {
    char c[4] = {' ', ' ', ' ', ' '};
    for (size_t i = 0; i < s.size() && i < 4; ++i)
        c[i] = s[i];
    return v4l2_fourcc(c[0], c[1], c[2], c[3]);
}

std::string fourcc_str(uint32_t f) {
    char c[5] = {char(f), char(f >> 8), char(f >> 16), char(f >> 24), 0};
    return std::string(c);
}

uint32_t fourcc_to_mbus(uint32_t pixfmt) {
    switch (pixfmt) {
        case V4L2_PIX_FMT_GREY:   return MEDIA_BUS_FMT_Y8_1X8;
        case V4L2_PIX_FMT_SGRBG8: return MEDIA_BUS_FMT_SGRBG8_1X8;
        case V4L2_PIX_FMT_SRGGB8: return MEDIA_BUS_FMT_SRGGB8_1X8;
        case V4L2_PIX_FMT_SBGGR8: return MEDIA_BUS_FMT_SBGGR8_1X8;
        case V4L2_PIX_FMT_SGBRG8: return MEDIA_BUS_FMT_SGBRG8_1X8;
        default:                  return 0;
    }
}

struct MappedPlane {
    void *start = nullptr;
    size_t length = 0;
};

// --- media controller topology (identical approach to mocap-sanity) ------

struct Topology {
    std::vector<media_v2_entity> entities;
    std::vector<media_v2_interface> interfaces;
    std::vector<media_v2_pad> pads;
    std::vector<media_v2_link> links;
};

Topology get_topology(int mfd) {
    Topology t;
    media_v2_topology topo{};
    if (xioctl(mfd, MEDIA_IOC_G_TOPOLOGY, &topo) == -1)
        fail("MEDIA_IOC_G_TOPOLOGY (counts)");
    t.entities.resize(topo.num_entities);
    t.interfaces.resize(topo.num_interfaces);
    t.pads.resize(topo.num_pads);
    t.links.resize(topo.num_links);
    topo.ptr_entities = reinterpret_cast<uintptr_t>(t.entities.data());
    topo.ptr_interfaces = reinterpret_cast<uintptr_t>(t.interfaces.data());
    topo.ptr_pads = reinterpret_cast<uintptr_t>(t.pads.data());
    topo.ptr_links = reinterpret_cast<uintptr_t>(t.links.data());
    if (xioctl(mfd, MEDIA_IOC_G_TOPOLOGY, &topo) == -1)
        fail("MEDIA_IOC_G_TOPOLOGY (fill)");
    return t;
}

std::string subdev_path_for(const Topology &t, const std::string &name_substr) {
    uint32_t entity_id = 0;
    std::string matched;
    for (const auto &e : t.entities)
        if (std::string(e.name).find(name_substr) != std::string::npos) {
            entity_id = e.id;
            matched = e.name;
            break;
        }
    if (!entity_id) {
        std::cerr << "mocap-perf: no media entity matching \"" << name_substr
                  << "\"\n";
        std::exit(1);
    }
    uint32_t intf_id = 0;
    for (const auto &l : t.links)
        if ((l.flags & MEDIA_LNK_FL_LINK_TYPE) == MEDIA_LNK_FL_INTERFACE_LINK &&
            l.sink_id == entity_id) {
            intf_id = l.source_id;
            break;
        }
    if (!intf_id) {
        std::cerr << "mocap-perf: entity \"" << matched << "\" has no devnode\n";
        std::exit(1);
    }
    uint32_t major = 0, minor = 0;
    bool found = false;
    for (const auto &i : t.interfaces)
        if (i.id == intf_id) {
            major = i.devnode.major;
            minor = i.devnode.minor;
            found = true;
            break;
        }
    if (!found) {
        std::cerr << "mocap-perf: interface " << intf_id << " not found\n";
        std::exit(1);
    }
    const dev_t want = makedev(major, minor);
    DIR *d = opendir("/dev");
    if (!d)
        fail("opendir /dev");
    std::string path;
    for (dirent *de = readdir(d); de; de = readdir(d)) {
        std::string cand = std::string("/dev/") + de->d_name;
        struct stat st{};
        if (stat(cand.c_str(), &st) == 0 && S_ISCHR(st.st_mode) &&
            st.st_rdev == want) {
            path = cand;
            break;
        }
    }
    closedir(d);
    if (path.empty()) {
        std::cerr << "mocap-perf: no /dev node for " << matched << "\n";
        std::exit(1);
    }
    std::cout << "  " << matched << " -> " << path << "\n";
    return path;
}

void set_subdev_format(const std::string &path, unsigned pad, uint32_t code,
                       unsigned w, unsigned h, const std::string &label) {
    int fd = open(path.c_str(), O_RDWR);
    if (fd == -1)
        fail("open " + path + " (" + label + ")");
    v4l2_subdev_format sfmt{};
    sfmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
    sfmt.pad = pad;
    sfmt.format.width = w;
    sfmt.format.height = h;
    sfmt.format.code = code;
    sfmt.format.field = V4L2_FIELD_NONE;
    sfmt.format.colorspace = V4L2_COLORSPACE_RAW;
    if (xioctl(fd, VIDIOC_SUBDEV_S_FMT, &sfmt) == -1) {
        close(fd);
        fail("VIDIOC_SUBDEV_S_FMT " + label + " pad " + std::to_string(pad));
    }
    if (sfmt.format.code != code || sfmt.format.width != w ||
        sfmt.format.height != h)
        std::cerr << "mocap-perf: warning: " << label << " pad " << pad
                  << " coerced to code 0x" << std::hex << sfmt.format.code
                  << std::dec << " " << sfmt.format.width << "x"
                  << sfmt.format.height << "\n";
    close(fd);
}

void configure_subdevs(const std::string &media_dev,
                       const std::string &sensor_name,
                       const std::string &csi_name, uint32_t mbus_code,
                       unsigned w, unsigned h) {
    int mfd = open(media_dev.c_str(), O_RDWR);
    if (mfd == -1)
        fail("open " + media_dev);
    Topology t = get_topology(mfd);
    close(mfd);
    std::cout << "Resolving subdevs:\n";
    const std::string sensor_path = subdev_path_for(t, sensor_name);
    const std::string csi_path = subdev_path_for(t, csi_name);
    std::cout << "Setting pad formats (code 0x" << std::hex << mbus_code
              << std::dec << " " << w << "x" << h << "):\n";
    set_subdev_format(sensor_path, kSensorSrcPad, mbus_code, w, h,
                      sensor_name + " src");
    set_subdev_format(csi_path, kCsiSinkPad, mbus_code, w, h, csi_name + " sink");
    set_subdev_format(csi_path, kCsiSrcPad, mbus_code, w, h, csi_name + " src");
    std::cout << "Pipeline configured.\n";
}

}  // namespace

int main(int argc, char *argv[]) {
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
    program.add_argument("--skip-setup")
        .default_value(false).implicit_value(true)
        .help("skip media/subdev configuration");

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
        configure_subdevs(media_dev, sensor_name, csi_name, mbus_code, width,
                          height);
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
            // First frame: prime timers, do not count (excludes startup).
            started = true;
            t_start = t;
            t_last_frame = t;
            t_report = t;
        } else {
            ++frames;
            ++frames_since_report;
            const double iv = t - t_last_frame;
            t_last_frame = t;
            if (iv < iv_min) iv_min = iv;
            if (iv > iv_max) iv_max = iv;
            iv_sum += iv;

            if (t - t_report >= 1.0) {
                std::cout << "  " << frames_since_report / (t - t_report)
                          << " fps\n";
                frames_since_report = 0;
                t_report = t;
            }
        }

        if (started && (t - t_start) >= duration)
            break;
    }

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
    return 0;
}
