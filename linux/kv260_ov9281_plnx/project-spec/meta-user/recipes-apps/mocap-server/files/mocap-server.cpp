// mocap-server: TCP streaming server for the KV260 OV9281 capture pipeline.
//
// Capture thread continuously dequeues V4L2 frames into a 2-deep ring buffer.
// Main thread runs a TCP server: on client connect, sends a metadata header
// then streams frames. When no client is connected or the client can't keep
// up, old frames are silently dropped.

#include <atomic>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include <mocap/argparse.hpp>
#include <mocap/ov9281_pipeline.hpp>

using namespace mocap;

// --- wire protocol (native byte order = little-endian on ARM & x86) ---------

constexpr uint32_t kStreamMagic = 'M' | ('C' << 8) | ('A' << 16) | ('P' << 24);
constexpr uint32_t kFrameMagic  = 'F' | ('R' << 8) | ('A' << 16) | ('M' << 24);

struct StreamHeader {
    uint32_t magic;
    uint32_t version;
    uint32_t width;
    uint32_t height;
    uint32_t pixelformat;
    uint32_t bytesperline;
    uint32_t frame_size;
    uint32_t _pad;
};

struct FrameHeader {
    uint32_t magic;
    uint32_t sequence;
    uint32_t size;
    uint32_t timestamp_sec;
    uint32_t timestamp_usec;
    uint32_t _pad;
};

// --- 2-deep frame ring buffer -----------------------------------------------

struct FrameRing {
    static constexpr int kDepth = 2;

    struct Slot {
        std::vector<uint8_t> data;
        uint32_t seq = 0;
        uint32_t tv_sec = 0;
        uint32_t tv_usec = 0;
    };

    std::mutex mu;
    std::condition_variable cv;
    Slot slots[kDepth];
    int rd = 0, wr = 0, count = 0;
    bool done = false;

    void push(const void *p, size_t n, uint32_t seq,
              uint32_t sec, uint32_t usec) {
        std::lock_guard<std::mutex> lk(mu);
        auto &s = slots[wr];
        s.data.resize(n);
        std::memcpy(s.data.data(), p, n);
        s.seq = seq;
        s.tv_sec = sec;
        s.tv_usec = usec;
        wr = (wr + 1) % kDepth;
        if (count == kDepth)
            rd = (rd + 1) % kDepth;
        else
            ++count;
        cv.notify_one();
    }

    bool pop(Slot &out) {
        std::unique_lock<std::mutex> lk(mu);
        cv.wait(lk, [&] { return count > 0 || done; });
        if (!count)
            return false;
        std::swap(out, slots[rd]);
        rd = (rd + 1) % kDepth;
        --count;
        return true;
    }

    void flush() {
        std::lock_guard<std::mutex> lk(mu);
        rd = wr;
        count = 0;
    }

    void stop() {
        std::lock_guard<std::mutex> lk(mu);
        done = true;
        cv.notify_all();
    }
};

// --- globals ----------------------------------------------------------------

static std::atomic<bool> g_quit{false};
static FrameRing g_ring;

static void on_signal(int) {
    g_quit = true;
    g_ring.stop();
}

// --- helpers ----------------------------------------------------------------

static bool send_all(int sock, const void *data, size_t len) {
    auto *p = static_cast<const uint8_t *>(data);
    while (len > 0) {
        ssize_t n = send(sock, p, len, MSG_NOSIGNAL);
        if (n <= 0)
            return false;
        p += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

// --- capture thread ---------------------------------------------------------

static void capture_loop(int fd, uint32_t buf_type, bool mplane,
                         unsigned n_planes,
                         std::vector<std::vector<MappedPlane>> &buffers) {
    uint32_t seq = 0;
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
            g_ring.push(buffers[buf.index][0].start, used, seq++,
                        static_cast<uint32_t>(buf.timestamp.tv_sec),
                        static_cast<uint32_t>(buf.timestamp.tv_usec));
        }

        if (xioctl(fd, VIDIOC_QBUF, &buf) == -1)
            fail("VIDIOC_QBUF");
    }
}

// --- main -------------------------------------------------------------------

int main(int argc, char *argv[]) {
    prog_name() = "mocap-server";
    argparse::ArgumentParser program("mocap-server", "1.0");
    program.add_description(
        "TCP streaming server for the KV260 OV9281 capture pipeline. "
        "Streams raw frames to a connected client with a metadata header.");

    program.add_argument("-d", "--device")
        .default_value(std::string("/dev/video0"))
        .help("V4L2 capture device node");
    program.add_argument("-p", "--port")
        .default_value(5001u)
        .scan<'u', unsigned>()
        .help("TCP listen port");
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

    try {
        program.parse_args(argc, argv);
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n" << program;
        return 1;
    }

    const std::string device = program.get<std::string>("--device");
    const unsigned port = program.get<unsigned>("--port");
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

    if (!mode_arg.empty()) {
        const SensorMode *m = find_mode(mode_arg);
        if (!m)
            die("unknown --mode '" + mode_arg + "'; valid: " +
                sensor_modes_str());
        width = m->width;
        height = m->height;
    }

    // --- pipeline setup (shared with mocap-sanity / mocap-perf) --------------

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
        const std::string sensor_path = configure_subdevs(
            media_dev, sensor_name, csi_name, mbus_code, width, height);

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
    const unsigned frame_size = mplane
                                    ? fmt.fmt.pix_mp.plane_fmt[0].sizeimage
                                    : fmt.fmt.pix.sizeimage;
    const uint32_t got_pixfmt =
        mplane ? fmt.fmt.pix_mp.pixelformat : fmt.fmt.pix.pixelformat;

    std::cout << "Capture: " << gw << "x" << gh << " '" << fourcc_str(got_pixfmt)
              << "' " << frame_size << " bytes/frame"
              << (mplane ? " [mplane]" : "") << "\n";

    // --- mmap buffers -------------------------------------------------------

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

    // --- signals ------------------------------------------------------------

    struct sigaction sa {};
    sa.sa_handler = on_signal;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    signal(SIGPIPE, SIG_IGN);

    // --- capture thread -----------------------------------------------------

    std::thread cap_thread(capture_loop, vfd, buf_type, mplane, n_planes,
                           std::ref(buffers));

    // --- TCP server ---------------------------------------------------------

    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd == -1)
        fail("socket");
    int opt = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (bind(sfd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == -1)
        fail("bind");
    if (listen(sfd, 1) == -1)
        fail("listen");

    std::cout << "Listening on port " << port << "\n";

    const StreamHeader shdr{kStreamMagic, 1, gw, gh, got_pixfmt, bpl,
                            frame_size, 0};

    while (!g_quit) {
        pollfd apfd{sfd, POLLIN, 0};
        int pr = poll(&apfd, 1, 500);
        if (pr <= 0)
            continue;

        sockaddr_in client_addr{};
        socklen_t clen = sizeof(client_addr);
        int cfd =
            accept(sfd, reinterpret_cast<sockaddr *>(&client_addr), &clen);
        if (cfd == -1) {
            if (errno == EINTR)
                continue;
            fail("accept");
        }

        opt = 1;
        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));

        uint32_t cip = ntohl(client_addr.sin_addr.s_addr);
        std::cout << "Client connected: " << ((cip >> 24) & 0xFF) << "."
                  << ((cip >> 16) & 0xFF) << "." << ((cip >> 8) & 0xFF) << "."
                  << (cip & 0xFF) << ":" << ntohs(client_addr.sin_port)
                  << "\n";

        if (!send_all(cfd, &shdr, sizeof(shdr))) {
            std::cerr << "Client disconnected during header\n";
            close(cfd);
            continue;
        }

        g_ring.flush();

        FrameRing::Slot frame;
        while (!g_quit && g_ring.pop(frame)) {
            FrameHeader fhdr{kFrameMagic, frame.seq,
                             static_cast<uint32_t>(frame.data.size()),
                             frame.tv_sec, frame.tv_usec, 0};
            if (!send_all(cfd, &fhdr, sizeof(fhdr)) ||
                !send_all(cfd, frame.data.data(), frame.data.size()))
                break;
        }

        std::cout << "Client disconnected\n";
        close(cfd);
    }

    // --- cleanup ------------------------------------------------------------

    g_quit = true;
    g_ring.stop();
    cap_thread.join();

    if (xioctl(vfd, VIDIOC_STREAMOFF, &type_int) == -1)
        std::cerr << "warning: VIDIOC_STREAMOFF: " << strerror(errno) << "\n";
    for (auto &bp : buffers)
        for (auto &pl : bp)
            munmap(pl.start, pl.length);
    close(vfd);
    close(sfd);
    std::cout << "Stopped.\n";
    return 0;
}
