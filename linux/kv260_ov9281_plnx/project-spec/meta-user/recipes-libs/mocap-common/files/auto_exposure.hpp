// mocap/auto_exposure.hpp: closed-loop brightness normalisation for V4L2
// sensors that expose exposure, gain, and (optionally) black-level controls.
//
// Header-only; consumers `#include <mocap/auto_exposure.hpp>` and call
// init() once with the sensor subdev path, then update() every frame.
// Internally it skips work except every `interval` frames.
//
// Control priority: exposure (best SNR) → analog gain → black level.
// Exposure and gain use multiplicative correction; black level uses additive
// correction against the 2nd-percentile dark value.
//
// Black level is discovered at runtime by enumerating the sensor's controls
// and matching the name substring "black".  If the driver doesn't expose one,
// it's silently skipped (the mainline ov9282 doesn't; patch the driver to
// add a V4L2 control for the OV9281 BLC registers if you need it).

#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

namespace mocap {

struct AEConfig {
    double target_mean  = 128.0;
    double target_black = 5.0;
    double deadband     = 5.0;
    double speed        = 0.3;
    int    interval     = 15;
};

class AutoExposure {
public:
    bool init(const std::string &sensor_subdev_path, const AEConfig &cfg) {
        cfg_ = cfg;
        fd_ = open(sensor_subdev_path.c_str(), O_RDWR);
        if (fd_ == -1) {
            std::cerr << "AE: cannot open " << sensor_subdev_path << ": "
                      << strerror(errno) << "\n";
            return false;
        }

        query_ctrl(V4L2_CID_EXPOSURE, exposure_, "exposure");
        query_ctrl(V4L2_CID_ANALOGUE_GAIN, gain_, "analog gain");
        find_ctrl_by_substr("black", black_level_);

        if (!exposure_.available && !gain_.available) {
            std::cerr << "AE: no usable controls found\n";
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        std::cout << "Auto-exposure enabled (target mean="
                  << cfg_.target_mean << ", speed=" << cfg_.speed
                  << ", interval=" << cfg_.interval << ")\n";
        print_ctrl("  exposure", exposure_);
        print_ctrl("  analog gain", gain_);
        print_ctrl("  black level", black_level_);
        return true;
    }

    bool update(const uint8_t *data, unsigned width, unsigned height,
                unsigned stride) {
        if (fd_ == -1)
            return false;
        if (++frame_count_ < cfg_.interval)
            return false;
        frame_count_ = 0;

        FrameStats st = compute_stats(data, width, height, stride);
        bool changed = false;

        // --- exposure + gain (multiplicative) ---
        double error = cfg_.target_mean - st.mean;
        if (std::abs(error) >= cfg_.deadband) {
            double mean_safe = st.mean < 1.0 ? 1.0 : st.mean;
            double ratio = cfg_.target_mean / mean_safe;
            ratio = 1.0 + (ratio - 1.0) * cfg_.speed;
            ratio = std::clamp(ratio, 0.5, 2.0);

            if (exposure_.available) {
                int nv = static_cast<int>(
                    std::round(exposure_.cur * ratio));
                nv = std::clamp(nv, exposure_.min, exposure_.max);
                if (nv != exposure_.cur) {
                    double achieved =
                        static_cast<double>(nv) / exposure_.cur;
                    set_ctrl(exposure_, nv);
                    ratio /= achieved;
                    changed = true;
                }
            }

            if (gain_.available && std::abs(ratio - 1.0) > 0.01) {
                int nv = static_cast<int>(
                    std::round(gain_.cur * ratio));
                nv = std::clamp(nv, gain_.min, gain_.max);
                if (nv != gain_.cur) {
                    set_ctrl(gain_, nv);
                    changed = true;
                }
            }
        }

        // --- black level (additive offset) ---
        if (black_level_.available) {
            double dark_err = cfg_.target_black - st.dark_pct;
            if (std::abs(dark_err) > 2.0) {
                int step = static_cast<int>(dark_err * cfg_.speed);
                if (step == 0)
                    step = (dark_err > 0) ? 1 : -1;
                int nv = std::clamp(black_level_.cur + step,
                                    black_level_.min, black_level_.max);
                if (nv != black_level_.cur) {
                    set_ctrl(black_level_, nv);
                    changed = true;
                }
            }
        }

        if (changed) {
            std::cout << "AE: mean=" << st.mean
                      << " dark=" << static_cast<int>(st.dark_pct);
            if (exposure_.available)
                std::cout << " exp=" << exposure_.cur;
            if (gain_.available)
                std::cout << " gain=" << gain_.cur;
            if (black_level_.available)
                std::cout << " blc=" << black_level_.cur;
            std::cout << "\n";
        }

        return changed;
    }

    void close() {
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
        }
    }

private:
    struct CtrlInfo {
        uint32_t cid = 0;
        int min = 0, max = 0, cur = 0;
        bool available = false;
    };

    struct FrameStats {
        double  mean;
        uint8_t dark_pct;
    };

    AEConfig cfg_;
    int fd_          = -1;
    int frame_count_ = 0;

    CtrlInfo exposure_;
    CtrlInfo gain_;
    CtrlInfo black_level_;

    static int ae_ioctl(int fd, unsigned long req, void *arg) {
        int r;
        do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
        return r;
    }

    bool query_ctrl(uint32_t cid, CtrlInfo &out, const char *label) {
        v4l2_queryctrl q{};
        q.id = cid;
        if (ae_ioctl(fd_, VIDIOC_QUERYCTRL, &q) == -1 ||
            (q.flags & V4L2_CTRL_FLAG_DISABLED)) {
            out.available = false;
            return false;
        }
        out.cid = cid;
        out.min = q.minimum;
        out.max = q.maximum;
        out.available = true;

        v4l2_control c{};
        c.id = cid;
        out.cur = (ae_ioctl(fd_, VIDIOC_G_CTRL, &c) == 0)
                      ? c.value
                      : q.default_value;
        return true;
    }

    bool find_ctrl_by_substr(const char *substr, CtrlInfo &out) {
        v4l2_queryctrl q{};
        q.id = V4L2_CTRL_FLAG_NEXT_CTRL;
        while (ae_ioctl(fd_, VIDIOC_QUERYCTRL, &q) == 0) {
            if (!(q.flags & V4L2_CTRL_FLAG_DISABLED)) {
                std::string name(reinterpret_cast<const char *>(q.name));
                for (char &c : name)
                    c = static_cast<char>(
                        std::tolower(static_cast<unsigned char>(c)));
                if (name.find(substr) != std::string::npos) {
                    out.cid = q.id;
                    out.min = q.minimum;
                    out.max = q.maximum;
                    out.available = true;
                    v4l2_control vc{};
                    vc.id = q.id;
                    out.cur = (ae_ioctl(fd_, VIDIOC_G_CTRL, &vc) == 0)
                                  ? vc.value
                                  : q.default_value;
                    return true;
                }
            }
            q.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
        }
        out.available = false;
        return false;
    }

    void set_ctrl(CtrlInfo &ctrl, int value) {
        v4l2_control c{};
        c.id = ctrl.cid;
        c.value = value;
        if (ae_ioctl(fd_, VIDIOC_S_CTRL, &c) == 0)
            ctrl.cur = value;
    }

    static void print_ctrl(const char *label, const CtrlInfo &c) {
        if (c.available)
            std::cout << label << ": " << c.min << ".." << c.max
                      << " (current " << c.cur << ")\n";
        else
            std::cout << label << ": not available\n";
    }

    // Subsampled mean + 2nd-percentile from a histogram.
    static FrameStats compute_stats(const uint8_t *data, unsigned w,
                                    unsigned h, unsigned stride) {
        constexpr unsigned kSkip = 4;
        uint32_t hist[256] = {};
        uint64_t sum = 0;
        unsigned count = 0;

        for (unsigned y = 0; y < h; y += kSkip) {
            const uint8_t *row = data + y * stride;
            for (unsigned x = 0; x < w; x += kSkip) {
                uint8_t v = row[x];
                hist[v]++;
                sum += v;
                ++count;
            }
        }

        double mean = count ? static_cast<double>(sum) / count : 0.0;

        unsigned target = count * 2 / 100;
        if (target == 0)
            target = 1;
        unsigned cum = 0;
        uint8_t pct = 0;
        for (unsigned i = 0; i < 256; ++i) {
            cum += hist[i];
            if (cum >= target) {
                pct = static_cast<uint8_t>(i);
                break;
            }
        }

        return {mean, pct};
    }
};

}  // namespace mocap
