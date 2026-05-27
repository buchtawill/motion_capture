// mocap-sanity: minimal V4L2 single-frame capture for the KV260 OV9281 pipeline.
//
// Configures the whole capture graph and grabs one frame:
//   1. Walk the media controller (/dev/media0) topology.
//   2. Resolve the sensor and CSI-RX subdev device nodes by entity name.
//   3. Set the mbus format on every pad (sensor source, CSI sink, CSI source)
//      so link_validate passes.
//   4. Set the matching 8-bpp RAW pixelformat on the video (DMA) node.
//   5. MMAP stream a single frame and write the raw bytes to a file.
//
// All steps are error-checked; on failure the offending entity/pad/ioctl is
// named so a failed run is self-diagnosing.

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
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

constexpr unsigned kBufferCount = 4;

// Standard pad layout for this pipeline.
constexpr unsigned kSensorSrcPad = 0;  // ov9281 source
constexpr unsigned kCsiSinkPad = 0;    // CSI-RX sink (from sensor)
constexpr unsigned kCsiSrcPad = 1;     // CSI-RX source (to DMA)

int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

[[noreturn]] void fail(const std::string &what) {
    std::cerr << "mocap-sanity: " << what << ": " << std::strerror(errno)
              << " (" << errno << ")\n";
    std::exit(1);
}

// Decode a 4-char fourcc string ("GRBG") into a V4L2 pixelformat.
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

// Map an 8-bpp RAW pixelformat to the mbus code the subdevs expect. Returns 0
// if unknown (caller must then supply --mbus-code explicitly).
uint32_t fourcc_to_mbus(uint32_t pixfmt) {
    switch (pixfmt) {
        case V4L2_PIX_FMT_SGRBG8: return MEDIA_BUS_FMT_SGRBG8_1X8;
        case V4L2_PIX_FMT_SRGGB8: return MEDIA_BUS_FMT_SRGGB8_1X8;
        case V4L2_PIX_FMT_SBGGR8: return MEDIA_BUS_FMT_SBGGR8_1X8;
        case V4L2_PIX_FMT_SGBRG8: return MEDIA_BUS_FMT_SGBRG8_1X8;
        case V4L2_PIX_FMT_GREY:   return MEDIA_BUS_FMT_Y8_1X8;
        default:                  return 0;
    }
}

struct MappedPlane {
    void *start = nullptr;
    size_t length = 0;
};

// --- media controller topology -------------------------------------------

struct Topology {
    std::vector<media_v2_entity> entities;
    std::vector<media_v2_interface> interfaces;
    std::vector<media_v2_pad> pads;
    std::vector<media_v2_link> links;
};

Topology get_topology(int mfd) {
    Topology t;
    media_v2_topology topo{};
    // First call: ptrs are null, kernel fills the counts.
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

    // Second call: kernel fills the arrays.
    if (xioctl(mfd, MEDIA_IOC_G_TOPOLOGY, &topo) == -1)
        fail("MEDIA_IOC_G_TOPOLOGY (fill)");
    return t;
}

// Resolve the /dev/v4l-subdevN path for the entity whose name contains
// name_substr, by following its interface link to a devnode major/minor.
std::string subdev_path_for(const Topology &t, const std::string &name_substr) {
    // 1. entity by name substring.
    uint32_t entity_id = 0;
    std::string matched;
    for (const auto &e : t.entities) {
        if (std::string(e.name).find(name_substr) != std::string::npos) {
            entity_id = e.id;
            matched = e.name;
            break;
        }
    }
    if (!entity_id) {
        std::cerr << "mocap-sanity: no media entity matching \"" << name_substr
                  << "\"\n";
        std::exit(1);
    }

    // 2. interface link (source = interface, sink = entity).
    uint32_t intf_id = 0;
    for (const auto &l : t.links) {
        if ((l.flags & MEDIA_LNK_FL_LINK_TYPE) == MEDIA_LNK_FL_INTERFACE_LINK &&
            l.sink_id == entity_id) {
            intf_id = l.source_id;
            break;
        }
    }
    if (!intf_id) {
        std::cerr << "mocap-sanity: entity \"" << matched
                  << "\" has no interface (no devnode)\n";
        std::exit(1);
    }

    // 3. interface devnode major/minor.
    uint32_t major = 0, minor = 0;
    bool found = false;
    for (const auto &i : t.interfaces) {
        if (i.id == intf_id) {
            major = i.devnode.major;
            minor = i.devnode.minor;
            found = true;
            break;
        }
    }
    if (!found) {
        std::cerr << "mocap-sanity: interface " << intf_id << " for \"" << matched
                  << "\" not found\n";
        std::exit(1);
    }

    // 4. scan /dev for the char device with this rdev.
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
        std::cerr << "mocap-sanity: no /dev node for " << matched << " ("
                  << major << ":" << minor << ")\n";
        std::exit(1);
    }
    std::cout << "  " << matched << " -> " << path << "\n";
    return path;
}

// Set one subdev pad's active mbus format; aborts with context on failure.
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

    // Report what the subdev actually accepted (it may clamp/coerce).
    if (sfmt.format.code != code || sfmt.format.width != w ||
        sfmt.format.height != h) {
        std::cerr << "mocap-sanity: warning: " << label << " pad " << pad
                  << " coerced to code 0x" << std::hex << sfmt.format.code
                  << std::dec << " " << sfmt.format.width << "x"
                  << sfmt.format.height << "\n";
    }
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
    // Source-to-sink order so propagation is consistent.
    set_subdev_format(sensor_path, kSensorSrcPad, mbus_code, w, h,
                      sensor_name + " src");
    set_subdev_format(csi_path, kCsiSinkPad, mbus_code, w, h, csi_name + " sink");
    set_subdev_format(csi_path, kCsiSrcPad, mbus_code, w, h, csi_name + " src");
    std::cout << "Pipeline configured.\n";
}

}  // namespace

int main(int argc, char *argv[]) {
    argparse::ArgumentParser program("mocap-sanity", "1.0");
    program.add_description(
        "Configure the CSI->DMA graph and capture a single 8-bpp RAW frame.");

    program.add_argument("-d", "--device")
        .default_value(std::string("/dev/video0"))
        .help("V4L2 capture device node");
    program.add_argument("-o", "--output")
        .default_value(std::string("./image_capture.raw"))
        .help("output file for the raw frame");
    program.add_argument("-W", "--width")
        .default_value(1280u)
        .scan<'u', unsigned>()
        .help("frame width in pixels");
    program.add_argument("-H", "--height")
        .default_value(800u)
        .scan<'u', unsigned>()
        .help("frame height in pixels");
    program.add_argument("-f", "--format")
        .default_value(std::string("GRBG"))
        .help("4-char V4L2 pixelformat fourcc (8-bpp RAW, e.g. GRBG)");
    program.add_argument("-m", "--media")
        .default_value(std::string("/dev/media0"))
        .help("media controller device node");
    program.add_argument("--sensor-entity")
        .default_value(std::string("ov9281"))
        .help("substring matching the sensor media entity name");
    program.add_argument("--csi-entity")
        .default_value(std::string("mipi_csi2_rx_subsystem"))
        .help("substring matching the CSI-RX media entity name");
    program.add_argument("--mbus-code")
        .default_value(std::string(""))
        .help("override subdev mbus code (hex, e.g. 0x3013); default derived "
              "from --format");
    program.add_argument("--skip-setup")
        .default_value(false)
        .implicit_value(true)
        .help("skip media/subdev configuration; only touch the video node");

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n" << program;
        return 1;
    }

    const std::string device = program.get<std::string>("--device");
    const std::string output = program.get<std::string>("--output");
    const unsigned width = program.get<unsigned>("--width");
    const unsigned height = program.get<unsigned>("--height");
    const uint32_t pixfmt = fourcc(program.get<std::string>("--format"));
    const std::string media_dev = program.get<std::string>("--media");
    const std::string sensor_name = program.get<std::string>("--sensor-entity");
    const std::string csi_name = program.get<std::string>("--csi-entity");
    const std::string mbus_arg = program.get<std::string>("--mbus-code");
    const bool skip_setup = program.get<bool>("--skip-setup");

    // --- configure the subdev pipeline ------------------------------------
    if (!skip_setup) {
        uint32_t mbus_code;
        if (!mbus_arg.empty()) {
            mbus_code = static_cast<uint32_t>(std::stoul(mbus_arg, nullptr, 0));
        } else {
            mbus_code = fourcc_to_mbus(pixfmt);
            if (!mbus_code) {
                std::cerr << "mocap-sanity: no default mbus code for '"
                          << fourcc_str(pixfmt)
                          << "'; pass --mbus-code 0xNNNN\n";
                return 1;
            }
        }
        configure_subdevs(media_dev, sensor_name, csi_name, mbus_code, width,
                          height);
    }

    // --- video (DMA) node --------------------------------------------------
    int fd = open(device.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd == -1)
        fail("open " + device);

    v4l2_capability cap{};
    if (xioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
        fail("VIDIOC_QUERYCAP");
    const bool mplane = cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE;
    const auto buf_type = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                 : V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!(cap.capabilities &
          (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
        std::cerr << "mocap-sanity: " << device << " is not a capture device\n";
        return 1;
    }
    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        std::cerr << "mocap-sanity: " << device << " does not support streaming\n";
        return 1;
    }

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
                  << "' " << n_planes << " plane(s)"
                  << (mplane ? " [mplane]" : "") << "\n";
        if (got != pixfmt)
            std::cerr << "mocap-sanity: warning: driver substituted '"
                      << fourcc_str(got) << "' for requested '"
                      << fourcc_str(pixfmt) << "'\n";
    }

    v4l2_requestbuffers req{};
    req.count = kBufferCount;
    req.type = buf_type;
    req.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd, VIDIOC_REQBUFS, &req) == -1)
        fail("VIDIOC_REQBUFS");
    if (req.count < 1) {
        std::cerr << "mocap-sanity: insufficient buffer memory\n";
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

    for (;;) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
        timeval tv{};
        tv.tv_sec = 5;
        int r = select(fd + 1, &fds, nullptr, nullptr, &tv);
        if (r == -1) {
            if (errno == EINTR)
                continue;
            fail("select");
        }
        if (r == 0) {
            std::cerr << "mocap-sanity: timeout waiting for a frame\n";
            return 1;
        }
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

    std::ofstream out(output, std::ios::binary | std::ios::trunc);
    if (!out) {
        std::cerr << "mocap-sanity: cannot open " << output << " for writing\n";
        return 1;
    }
    size_t total = 0;
    for (unsigned p = 0; p < n_planes; ++p) {
        const size_t used = mplane ? planes[p].bytesused : buf.bytesused;
        out.write(static_cast<const char *>(buffers[buf.index][p].start), used);
        total += used;
    }
    out.close();
    std::cout << "Wrote " << total << " bytes to " << output << "\n";

    if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1)
        fail("VIDIOC_STREAMOFF");
    for (auto &bufp : buffers)
        for (auto &pl : bufp)
            munmap(pl.start, pl.length);
    close(fd);
    return 0;
}
