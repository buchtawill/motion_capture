// mocap/isp_stats.hpp: UIO-based Linux driver for the ISP histogram module.
//
// RAII — acquire via IspStats::open() or IspStats::discover(), release in
// destructor (munmap + close).  Non-copyable, movable.

#pragma once

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <unistd.h>

#include <mocap/isp_regs.h>

namespace mocap {

typedef uint32_t isp_hist_t[256];

struct IspSnapshot {
    uint64_t cycle_cnt;
    uint32_t frame_cnt;
};

class IspStats {
public:
    static constexpr uint16_t NUM_BINS = 256;

    // --- Factory methods ----------------------------------------------------

    static std::unique_ptr<IspStats> open(const std::string &uio_path) {
        int fd = ::open(uio_path.c_str(), O_RDWR | O_SYNC);
        if (fd == -1) {
            std::cerr << "IspStats: cannot open " << uio_path << ": "
                      << strerror(errno) << "\n";
            return nullptr;
        }
        void *map = mmap(nullptr, kMapSize, PROT_READ | PROT_WRITE,
                         MAP_SHARED, fd, 0);
        if (map == MAP_FAILED) {
            std::cerr << "IspStats: mmap failed: " << strerror(errno) << "\n";
            ::close(fd);
            return nullptr;
        }
        std::cout << "IspStats: opened " << uio_path << "\n";
        return std::unique_ptr<IspStats>(
            new IspStats(fd, static_cast<volatile isp_regs_t *>(map)));
    }

    static std::unique_ptr<IspStats> discover(
        const std::string &name_substr = "isp_math_wrapper") {
        std::string path = find_uio_device(name_substr);
        if (path.empty()) {
            std::cerr << "IspStats: no UIO device matching '"
                      << name_substr << "'\n";
            return nullptr;
        }
        return open(path);
    }

    // --- Lifecycle ----------------------------------------------------------

    ~IspStats() {
        if (regs_)
            munmap(const_cast<isp_regs_t *>(regs_), kMapSize);
        if (fd_ != -1)
            ::close(fd_);
    }

    IspStats(const IspStats &) = delete;
    IspStats &operator=(const IspStats &) = delete;

    IspStats(IspStats &&o) noexcept : fd_(o.fd_), regs_(o.regs_) {
        o.fd_ = -1;
        o.regs_ = nullptr;
    }
    IspStats &operator=(IspStats &&o) noexcept {
        if (this != &o) {
            if (regs_)
                munmap(const_cast<isp_regs_t *>(regs_), kMapSize);
            if (fd_ != -1)
                ::close(fd_);
            fd_ = o.fd_;
            regs_ = o.regs_;
            o.fd_ = -1;
            o.regs_ = nullptr;
        }
        return *this;
    }

    // --- Resolution ---------------------------------------------------------

    void set_resolution(uint16_t hres, uint16_t vres) {
        regs_->HRES = hres;
        regs_->VRES = vres;
    }

    void get_resolution(uint16_t &hres, uint16_t &vres) {
        hres = static_cast<uint16_t>(regs_->HRES &
                                     ISP_REGS__HRES_REG__HRES_bm);
        vres = static_cast<uint16_t>(regs_->VRES &
                                     ISP_REGS__VRES_REG__VRES_bm);
    }

    // --- Status -------------------------------------------------------------

    bool is_ready() {
        return (regs_->STATUS & ISP_REGS__STATUS_REG__READY_bm) != 0;
    }
    bool is_hist_data_valid() {
        return (regs_->STATUS & ISP_REGS__STATUS_REG__HIST_DATA_VALID_bm) != 0;
    }
    bool is_hist_fifo_err() {
        return (regs_->STATUS & ISP_REGS__STATUS_REG__HIST_FIFO_ERR_bm) != 0;
    }

    // --- Reset / control ----------------------------------------------------

    void sw_reset() { pulse_ctrl(ISP_REGS__CTRL_REG__RESET_bm); }
    void reset_frame_counter() {
        pulse_ctrl(ISP_REGS__CTRL_REG__FRAME_CNT_RESET_bm);
    }
    void reset_cycle_counter() {
        pulse_ctrl(ISP_REGS__CTRL_REG__CYCLE_CNT_RESET_bm);
    }

    void set_hist_addr_autoinc(bool enable) {
        uint32_t v =
            regs_->CTRL & ~ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm;
        if (enable)
            v |= ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm;
        regs_->CTRL = v;
    }

    // --- Histogram measurement ----------------------------------------------

    bool start_histogram() {
        uint16_t h, v;
        get_resolution(h, v);
        if (h == 0 || v == 0)
            return false;
        if (!is_ready())
            return false;
        pulse_ctrl(ISP_REGS__CTRL_REG__HISTOGRAM_START_bm);
        return true;
    }

    bool wait_histogram_valid(int timeout_ms = 1000) {
        if (is_hist_data_valid())
            return true;

        uint32_t enable = 1;
        if (::write(fd_, &enable, sizeof(enable)) ==
            static_cast<ssize_t>(sizeof(enable))) {
            pollfd pfd{fd_, POLLIN, 0};
            int ret = ::poll(&pfd, 1, timeout_ms);
            if (ret > 0) {
                uint32_t count;
                ::read(fd_, &count, sizeof(count));
                return is_hist_data_valid();
            }
            return false;
        }

        for (int i = 0; i < timeout_ms * 100; i++) {
            if (is_hist_data_valid())
                return true;
            usleep(10);
        }
        return false;
    }

    bool dump_histogram(isp_hist_t &hist) {
        if (!is_hist_data_valid())
            return false;

        bool autoinc =
            (regs_->CTRL & ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm) != 0;
        if (!autoinc)
            set_hist_addr_autoinc(true);

        regs_->HIST_ADDR = 0;
        for (int i = 0; i < NUM_BINS; i++)
            hist[i] =
                regs_->HIST_DATA & ISP_REGS__HIST_DATA_REG__HIST_DATA_bm;

        if (!autoinc)
            set_hist_addr_autoinc(false);
        return true;
    }

    bool read_hist_bin(uint8_t addr, uint32_t &data) {
        if (!is_hist_data_valid())
            return false;

        bool autoinc =
            (regs_->CTRL & ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm) != 0;
        if (autoinc)
            set_hist_addr_autoinc(false);

        regs_->HIST_ADDR = addr;
        data = regs_->HIST_DATA & ISP_REGS__HIST_DATA_REG__HIST_DATA_bm;

        if (autoinc)
            set_hist_addr_autoinc(true);
        return true;
    }

    // --- Pixel sum / brightness ---------------------------------------------

    uint32_t read_pixel_sum() { return regs_->PIXEL_SUM; }

    double compute_avg_brightness() {
        uint16_t h, v;
        get_resolution(h, v);
        uint32_t npix = static_cast<uint32_t>(h) * v;
        return npix ? static_cast<double>(read_pixel_sum()) / npix : 0.0;
    }

    // --- One-shot convenience -----------------------------------------------

    bool capture_histogram(isp_hist_t &hist, uint32_t *pixel_sum = nullptr,
                           int timeout_ms = 1000) {
        if (!start_histogram())
            return false;
        if (!wait_histogram_valid(timeout_ms))
            return false;
        if (!dump_histogram(hist))
            return false;
        if (pixel_sum)
            *pixel_sum = read_pixel_sum();
        return true;
    }

    // --- Counters / snapshot ------------------------------------------------

    IspSnapshot snapshot() {
        pulse_ctrl(ISP_REGS__CTRL_REG__SNAPSHOT_bm);
        IspSnapshot s;
        s.frame_cnt = regs_->FRAME_SNAP;
        uint32_t hi = regs_->CYCLE_SNAP_HI;
        uint32_t lo = regs_->CYCLE_SNAP_LO;
        s.cycle_cnt = (static_cast<uint64_t>(hi) << 32) | lo;
        return s;
    }

    static double compute_fps(const IspSnapshot &t1, const IspSnapshot &t2,
                              double clock_hz) {
        uint32_t fd = t2.frame_cnt - t1.frame_cnt;
        uint64_t cd = t2.cycle_cnt - t1.cycle_cnt;
        return cd ? (static_cast<double>(fd) * clock_hz) / cd : 0.0;
    }

private:
    static constexpr size_t kMapSize = 0x1000;
    int fd_ = -1;
    volatile isp_regs_t *regs_ = nullptr;

    IspStats(int fd, volatile isp_regs_t *regs) : fd_(fd), regs_(regs) {}

    static std::string find_uio_device(const std::string &name_substr) {
        DIR *d = opendir("/sys/class/uio");
        if (!d)
            return {};
        for (dirent *de = readdir(d); de; de = readdir(d)) {
            if (de->d_name[0] == '.')
                continue;
            std::string npath =
                "/sys/class/uio/" + std::string(de->d_name) + "/name";
            int nfd = ::open(npath.c_str(), O_RDONLY);
            if (nfd == -1)
                continue;
            char buf[256] = {};
            ssize_t n = ::read(nfd, buf, sizeof(buf) - 1);
            ::close(nfd);
            if (n > 0) {
                if (buf[n - 1] == '\n')
                    buf[n - 1] = '\0';
                if (std::string(buf).find(name_substr) != std::string::npos) {
                    closedir(d);
                    return "/dev/" + std::string(de->d_name);
                }
            }
        }
        closedir(d);
        return {};
    }

    void pulse_ctrl(uint32_t pulse_bm) {
        uint32_t keep =
            regs_->CTRL & ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm;
        regs_->CTRL = keep | pulse_bm;
    }
};

}  // namespace mocap
