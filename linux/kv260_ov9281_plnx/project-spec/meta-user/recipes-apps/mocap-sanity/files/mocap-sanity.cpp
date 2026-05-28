// mocap-sanity: minimal V4L2 single-frame capture for the KV260 OV9281 pipeline.
//
// Configures the whole capture graph and grabs one frame:
//   1. Walk the media controller (/dev/media0) topology.
//   2. Resolve the sensor and CSI-RX subdev device nodes by entity name.
//   3. Set the mbus format on every pad (sensor source, CSI sink, CSI source)
//      so link_validate passes.
//   4. Set the matching 8-bpp RAW pixelformat on the video (DMA) node.
//   5. MMAP stream a single frame and write it to a file. A ".png" output
//      is encoded as a single-channel grayscale PNG (the OV9281 is mono);
//      any other extension writes the raw frame bytes verbatim.
//
// All steps are error-checked; on failure the offending entity/pad/ioctl is
// named so a failed run is self-diagnosing.

#include <cctype>
#include <cerrno>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <mocap/argparse.hpp>
#include <mocap/ov9281_pipeline.hpp>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <mocap/stb_image_write.h>

// Pipeline config (topology walk, subdev formats), the fourcc helpers,
// xioctl/fail and MappedPlane all live in <mocap/ov9281_pipeline.hpp>.
using namespace mocap;

namespace {

constexpr unsigned kBufferCount = 4;

// Case-insensitive ".png" suffix test on the output path.
bool wants_png(const std::string &path) {
    if (path.size() < 4)
        return false;
    std::string ext = path.substr(path.size() - 4);
    for (char &c : ext)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return ext == ".png";
}

// Write a single-channel (grayscale) 8-bpp frame as PNG. The captured buffer
// may have row padding, so the per-row stride (bytesperline) is passed through
// to stb. Returns false on encode/write failure.
bool write_png_gray(const std::string &path, const uint8_t *data, unsigned w,
                    unsigned h, unsigned stride) {
    const int row_bytes = static_cast<int>(stride ? stride : w);
    return stbi_write_png(path.c_str(), static_cast<int>(w),
                          static_cast<int>(h), 1, data, row_bytes) != 0;
}

}  // namespace

int main(int argc, char *argv[]) {
    prog_name() = "mocap-sanity";  // prefix for shared pipeline error messages
    argparse::ArgumentParser program("mocap-sanity", "1.0");
    program.add_description(
        "Configure the CSI->DMA graph and capture a single 8-bpp RAW frame.");

    program.add_argument("-d", "--device")
        .default_value(std::string("/dev/video0"))
        .help("V4L2 capture device node");
    program.add_argument("-o", "--output")
        .default_value(std::string("./image_capture.png"))
        .help("output file; a .png extension writes a grayscale PNG, "
              "otherwise raw frame bytes");
    program.add_argument("-W", "--width")
        .default_value(1280u)
        .scan<'u', unsigned>()
        .help("frame width in pixels");
    program.add_argument("-H", "--height")
        .default_value(800u)
        .scan<'u', unsigned>()
        .help("frame height in pixels");
    program.add_argument("-f", "--format")
        .default_value(std::string("GREY"))
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
    const uint32_t got =
        mplane ? fmt.fmt.pix_mp.pixelformat : fmt.fmt.pix.pixelformat;
    const unsigned gw = mplane ? fmt.fmt.pix_mp.width : fmt.fmt.pix.width;
    const unsigned gh = mplane ? fmt.fmt.pix_mp.height : fmt.fmt.pix.height;
    const unsigned bpl = mplane ? fmt.fmt.pix_mp.plane_fmt[0].bytesperline
                                : fmt.fmt.pix.bytesperline;
    {
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

    if (wants_png(output)) {
        // PNG encode: single-channel grayscale from plane 0. The OV9281 is a
        // mono sensor, so 8-bpp RAW (GREY or a cosmetic Bayer8 label) is one
        // luma byte per pixel. A Bayer mosaic is written undebayered.
        if (n_planes != 1) {
            std::cerr << "mocap-sanity: PNG output supports single-plane "
                         "formats only (got "
                      << n_planes << " planes); use a raw output path\n";
            return 1;
        }
        const auto *data =
            static_cast<const uint8_t *>(buffers[buf.index][0].start);
        if (!write_png_gray(output, data, gw, gh, bpl)) {
            std::cerr << "mocap-sanity: failed to encode PNG to " << output
                      << "\n";
            return 1;
        }
        std::cout << "Wrote " << gw << "x" << gh << " grayscale PNG to "
                  << output << "\n";
    } else {
        std::ofstream out(output, std::ios::binary | std::ios::trunc);
        if (!out) {
            std::cerr << "mocap-sanity: cannot open " << output
                      << " for writing\n";
            return 1;
        }
        size_t total = 0;
        for (unsigned p = 0; p < n_planes; ++p) {
            const size_t used = mplane ? planes[p].bytesused : buf.bytesused;
            out.write(static_cast<const char *>(buffers[buf.index][p].start),
                      used);
            total += used;
        }
        out.close();
        std::cout << "Wrote " << total << " bytes to " << output << "\n";
    }

    if (xioctl(fd, VIDIOC_STREAMOFF, &type) == -1)
        fail("VIDIOC_STREAMOFF");
    for (auto &bufp : buffers)
        for (auto &pl : bufp)
            munmap(pl.start, pl.length);
    close(fd);
    return 0;
}
