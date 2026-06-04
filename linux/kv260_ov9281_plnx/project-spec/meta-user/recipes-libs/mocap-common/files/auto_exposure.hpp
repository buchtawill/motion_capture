// mocap/auto_exposure.hpp: closed-loop brightness normalisation for V4L2
// sensors that expose exposure, gain, and (optionally) black-level controls.
//
// RAII — acquire via AutoExposure::create(), release in destructor.
//
// When an IspStats* is provided at construction, update() reads the HW
// histogram (zero CPU cost) instead of computing a subsampled SW histogram
// from pixel data.  When null, falls back to the SW path.

#pragma once

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include <mocap/isp_stats.hpp>

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
    static std::unique_ptr<AutoExposure> create(
        const std::string &sensor_subdev_path, const AEConfig &cfg,
        IspStats *isp = nullptr) {

        int fd = ::open(sensor_subdev_path.c_str(), O_RDWR);
        if (fd == -1) {
            std::cerr << "AE: cannot open " << sensor_subdev_path << ": "
                      << strerror(errno) << "\n";
            return nullptr;
        }

        std::unique_ptr<AutoExposure> ae(new AutoExposure(fd, cfg, isp));

        ae->query_ctrl(V4L2_CID_EXPOSURE, ae->exposure_, "exposure");
        ae->query_ctrl(V4L2_CID_ANALOGUE_GAIN, ae->gain_, "analog gain");
        ae->find_ctrl_by_substr("black", ae->black_level_);

        if (!ae->exposure_.available && !ae->gain_.available) {
            std::cerr << "AE: no usable controls found\n";
            return nullptr;
        }

        std::cout << "Auto-exposure enabled (target mean="
                  << cfg.target_mean << ", speed=" << cfg.speed
                  << ", interval=" << cfg.interval
                  << ", hw_hist=" << (isp ? "yes" : "no") << ")\n";
        print_ctrl("  exposure", ae->exposure_);
        print_ctrl("  analog gain", ae->gain_);
        print_ctrl("  black level", ae->black_level_);

        if (isp)
            isp->start_histogram();

        return ae;
    }

    ~AutoExposure() {
        if (fd_ != -1)
            ::close(fd_);
    }

    AutoExposure(const AutoExposure &) = delete;
    AutoExposure &operator=(const AutoExposure &) = delete;

    bool update(const uint8_t *data, unsigned width, unsigned height,
                unsigned stride) {
        if (++frame_count_ < cfg_.interval)
            return false;
        frame_count_ = 0;

        FrameStats st;
        if (isp_) {
            if (!isp_->is_hist_data_valid()) {
                isp_->start_histogram();
                return false;
            }
            st = read_hw_stats(width, height);
            isp_->start_histogram();
        } else {
            st = compute_stats(data, width, height, stride);
        }
        return apply(st);
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
    int fd_;
    int frame_count_ = 0;
    IspStats *isp_;

    CtrlInfo exposure_;
    CtrlInfo gain_;
    CtrlInfo black_level_;

    AutoExposure(int fd, const AEConfig &cfg, IspStats *isp)
        : cfg_(cfg), fd_(fd), isp_(isp) {}

    // --- HW histogram path --------------------------------------------------

    FrameStats read_hw_stats(unsigned w, unsigned h) {
        uint32_t npix = static_cast<uint32_t>(w) * h;
        double mean =
            npix ? static_cast<double>(isp_->read_pixel_sum()) / npix : 0.0;

        isp_hist_t hist;
        isp_->dump_histogram(hist);

        uint64_t total = 0;
        for (int i = 0; i < 256; i++)
            total += hist[i];

        uint64_t target = total * 2 / 100;
        if (target == 0)
            target = 1;
        uint64_t cum = 0;
        uint8_t pct = 0;
        for (int i = 0; i < 256; i++) {
            cum += hist[i];
            if (cum >= target) {
                pct = static_cast<uint8_t>(i);
                break;
            }
        }
        return {mean, pct};
    }

    // --- SW histogram path --------------------------------------------------

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

        unsigned thr = count * 2 / 100;
        if (thr == 0)
            thr = 1;
        unsigned cum = 0;
        uint8_t pct = 0;
        for (unsigned i = 0; i < 256; ++i) {
            cum += hist[i];
            if (cum >= thr) {
                pct = static_cast<uint8_t>(i);
                break;
            }
        }
        return {mean, pct};
    }

    // --- Control math (shared by both paths) --------------------------------

    bool apply(const FrameStats &st) {
        bool changed = false;

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

    // --- V4L2 helpers -------------------------------------------------------

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
};

}  // namespace mocap
