// mocap/ov9281_pipeline.hpp: shared CSI->DMA pipeline configuration for the
// KV260 OV9281 capture apps. OV9281-specific: the pad layout, the 8-bpp RAW
// mbus codes, and the VBLANK-based frame-rate control all assume that sensor.
//
// Walks the media controller topology, resolves the sensor and CSI-RX subdev
// device nodes by entity name, sets the mbus format on every pad so
// link_validate passes, and (optionally) drives the sensor frame rate via
// vertical blanking. All of this is identical across the capture apps, so it
// lives here and is staged into the sysroot by the mocap-common recipe.
//
// Header-only (inline) so consumers only `#include <mocap/ov9281_pipeline.hpp>`
// and add DEPENDS = "mocap-common"; no library to link. Each app sets
// mocap::prog_name() once at startup so error messages carry its name.
//
// Error handling: on any failure the offending entity/pad/ioctl is named and
// the process exits (fail() / std::exit). Suitable for these single-purpose
// command-line tools; not for long-running services.

#pragma once

#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <linux/media.h>
#include <linux/media-bus-format.h>
#include <linux/v4l2-subdev.h>
#include <linux/videodev2.h>

namespace mocap {

// Standard pad layout for this pipeline (OV9281 -> CSI-RX -> DMA).
constexpr unsigned kSensorSrcPad = 0;  // sensor source
constexpr unsigned kCsiSinkPad = 0;    // CSI-RX sink (from sensor)
constexpr unsigned kCsiSrcPad = 1;     // CSI-RX source (to DMA)

// Program name used to prefix error messages. Apps set this once in main().
inline std::string &prog_name() {
    static std::string name = "mocap";
    return name;
}

inline int xioctl(int fd, unsigned long req, void *arg) {
    int r;
    do {
        r = ioctl(fd, req, arg);
    } while (r == -1 && errno == EINTR);
    return r;
}

// Print "<prog>: <what>: <errno string>" and exit. For ioctl/syscall failures.
[[noreturn]] inline void fail(const std::string &what) {
    std::cerr << prog_name() << ": " << what << ": " << std::strerror(errno)
              << " (" << errno << ")\n";
    std::exit(1);
}

// Print "<prog>: <msg>" and exit. For logic errors with no useful errno.
[[noreturn]] inline void die(const std::string &msg) {
    std::cerr << prog_name() << ": " << msg << "\n";
    std::exit(1);
}

// One mmap'd buffer plane (filled in by the per-app capture setup).
struct MappedPlane {
    void *start = nullptr;
    size_t length = 0;
};

// Decode a 4-char fourcc string ("GRBG") into a V4L2 pixelformat.
inline uint32_t fourcc(const std::string &s) {
    char c[4] = {' ', ' ', ' ', ' '};
    for (size_t i = 0; i < s.size() && i < 4; ++i)
        c[i] = s[i];
    return v4l2_fourcc(c[0], c[1], c[2], c[3]);
}

inline std::string fourcc_str(uint32_t f) {
    char c[5] = {char(f), char(f >> 8), char(f >> 16), char(f >> 24), 0};
    return std::string(c);
}

// Map an 8-bpp RAW pixelformat to the mbus code the subdevs expect. Returns 0
// if unknown (caller must then supply an explicit mbus code).
inline uint32_t fourcc_to_mbus(uint32_t pixfmt) {
    switch (pixfmt) {
        case V4L2_PIX_FMT_GREY:   return MEDIA_BUS_FMT_Y8_1X8;
        case V4L2_PIX_FMT_SGRBG8: return MEDIA_BUS_FMT_SGRBG8_1X8;
        case V4L2_PIX_FMT_SRGGB8: return MEDIA_BUS_FMT_SRGGB8_1X8;
        case V4L2_PIX_FMT_SBGGR8: return MEDIA_BUS_FMT_SBGGR8_1X8;
        case V4L2_PIX_FMT_SGBRG8: return MEDIA_BUS_FMT_SGBRG8_1X8;
        default:                  return 0;
    }
}

// --- media controller topology -------------------------------------------

struct Topology {
    std::vector<media_v2_entity> entities;
    std::vector<media_v2_interface> interfaces;
    std::vector<media_v2_pad> pads;
    std::vector<media_v2_link> links;
};

inline Topology get_topology(int mfd) {
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
inline std::string subdev_path_for(const Topology &t,
                                   const std::string &name_substr) {
    // 1. entity by name substring.
    uint32_t entity_id = 0;
    std::string matched;
    for (const auto &e : t.entities)
        if (std::string(e.name).find(name_substr) != std::string::npos) {
            entity_id = e.id;
            matched = e.name;
            break;
        }
    if (!entity_id)
        die("no media entity matching \"" + name_substr + "\"");

    // 2. interface link (source = interface, sink = entity).
    uint32_t intf_id = 0;
    for (const auto &l : t.links)
        if ((l.flags & MEDIA_LNK_FL_LINK_TYPE) == MEDIA_LNK_FL_INTERFACE_LINK &&
            l.sink_id == entity_id) {
            intf_id = l.source_id;
            break;
        }
    if (!intf_id)
        die("entity \"" + matched + "\" has no interface (no devnode)");

    // 3. interface devnode major/minor.
    uint32_t major = 0, minor = 0;
    bool found = false;
    for (const auto &i : t.interfaces)
        if (i.id == intf_id) {
            major = i.devnode.major;
            minor = i.devnode.minor;
            found = true;
            break;
        }
    if (!found)
        die("interface " + std::to_string(intf_id) + " for \"" + matched +
            "\" not found");

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
    if (path.empty())
        die("no /dev node for " + matched + " (" + std::to_string(major) + ":" +
            std::to_string(minor) + ")");
    std::cout << "  " << matched << " -> " << path << "\n";
    return path;
}

// Set one subdev pad's active mbus format; aborts with context on failure.
inline void set_subdev_format(const std::string &path, unsigned pad,
                              uint32_t code, unsigned w, unsigned h,
                              const std::string &label) {
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
        sfmt.format.height != h)
        std::cerr << prog_name() << ": warning: " << label << " pad " << pad
                  << " coerced to code 0x" << std::hex << sfmt.format.code
                  << std::dec << " " << sfmt.format.width << "x"
                  << sfmt.format.height << "\n";
    close(fd);
}

// Configure every pad format and return the sensor subdev path (so the caller
// can drive controls like VBLANK on it afterwards -- S_FMT resets per-mode
// controls to defaults, so vblank must come after this).
inline std::string configure_subdevs(const std::string &media_dev,
                                     const std::string &sensor_name,
                                     const std::string &csi_name,
                                     uint32_t mbus_code, unsigned w,
                                     unsigned h) {
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
    return sensor_path;
}

// Set the sensor's vertical blanking (V4L2_CID_VBLANK) to raise/lower the frame
// rate. vblank_arg is "min" (max fps), "keep" (leave the driver default), or an
// integer line count (clamped to the control's queried [min,max]).
inline void apply_vblank(const std::string &sensor_path,
                         const std::string &vblank_arg) {
    if (vblank_arg == "keep")
        return;
    int fd = open(sensor_path.c_str(), O_RDWR);
    if (fd == -1)
        fail("open " + sensor_path + " (vblank)");

    v4l2_queryctrl q{};
    q.id = V4L2_CID_VBLANK;
    if (xioctl(fd, VIDIOC_QUERYCTRL, &q) == -1) {
        close(fd);
        fail("VIDIOC_QUERYCTRL VBLANK");
    }

    int want = (vblank_arg == "min") ? q.minimum : std::stoi(vblank_arg);
    if (want < q.minimum) want = q.minimum;
    if (want > q.maximum) want = q.maximum;

    v4l2_control c{};
    c.id = V4L2_CID_VBLANK;
    c.value = want;
    if (xioctl(fd, VIDIOC_S_CTRL, &c) == -1) {
        close(fd);
        fail("VIDIOC_S_CTRL VBLANK");
    }
    close(fd);
    std::cout << "Vertical blanking set to " << want << " lines (min "
              << q.minimum << ", max " << q.maximum << ", default "
              << q.default_value << ")\n";
}

}  // namespace mocap
