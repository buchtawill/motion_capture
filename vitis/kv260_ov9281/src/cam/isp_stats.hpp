#ifndef ISP_STATS_HPP
#define ISP_STATS_HPP

#include "xstatus.h"
#include "isp_regs.h"

// 20-bit bins zero-extended into uint32_t.
typedef uint32_t isp_hist_t[256];

typedef struct {
    uint64_t cycle_cnt;
    uint32_t frame_cnt;
} isp_snapshot_t;

class IspStats {

public:

    static constexpr uint16_t NUM_BINS = 256;

    IspStats(uint32_t base_addr, uint32_t clock_freq_hz);

    // ---- One-shot init sanity ----------------------------------------------

    // Read default HRES/VRES to confirm the base address is wired up.
    // Should be called before set_resolution(); returns XST_SUCCESS if
    // HRES==1280 and VRES==800 (RTL reset defaults).
    XStatus probe();

    // ---- Resets / control --------------------------------------------------

    // Full SW reset: clears counters, sticky FIFO error, and kicks a scrub.
    // HRES/VRES are preserved.
    void sw_reset();

    void reset_frame_counter();
    void reset_cycle_counter();

    // Default 1 (auto-increment HIST_ADDR after each HIST_DATA read).
    void set_hist_addr_autoinc(bool enable);

    // ---- Resolution --------------------------------------------------------

    void set_resolution(uint16_t hres, uint16_t vres);
    void get_resolution(uint16_t* hres, uint16_t* vres);

    // ---- Counters / snapshot ----------------------------------------------

    // Atomically snapshot frame + cycle counters via CTRL.SNAPSHOT.
    isp_snapshot_t snapshot_frame_and_cycle_ctr();

    // FPS from two snapshots: (frame_delta * clk) / cycle_delta.
    float compute_fps(const isp_snapshot_t* t1, const isp_snapshot_t* t2);

    // ---- Status peeks (non-blocking) --------------------------------------

    bool is_ready();
    bool is_hist_data_valid();
    bool is_hist_fifo_err();   // sticky; only clears via sw_reset()

    // ---- Histogram measurement --------------------------------------------

    // Kick off a histogram + pixel-sum measurement. Returns XST_FAILURE if
    // the module is not ready, or HRES/VRES are zero.
    XStatus start_histogram();

    // Block until STATUS.READY=1 (scrub finished, ready to start).
    XStatus poll_histogram_ready();

    // Block until STATUS.HIST_DATA_VALID=1 (frame measured, sum + bins ready).
    XStatus poll_histogram_valid();

    // Read one bin (20-bit value zero-extended into *data).
    // Fails if HIST_DATA_VALID is not set.
    XStatus read_hist_bin(uint8_t addr, uint32_t* data);

    // Read all 256 bins. Uses HIST_ADDR auto-increment and restores prior
    // autoinc state.
    XStatus dump_histogram(isp_hist_t* hist_data);

    // ---- Pixel sum / brightness -------------------------------------------

    // 32-bit sum of all pixel values from the last completed measurement.
    uint32_t read_pixel_sum();

    // pixel_sum / (HRES * VRES), computed in software. Returns 0.0f if HRES
    // or VRES is zero.
    float    compute_avg_brightness();

    // ---- One-shot convenience ---------------------------------------------

    // start + poll_valid + dump + (optional) pixel_sum readback. Single call
    // suitable for menu-driven code paths.
    XStatus capture_histogram(isp_hist_t* hist_out,
                              uint32_t*   pixel_sum_out = nullptr);

    // ---- Diagnostics / config ---------------------------------------------

    void set_max_polls(uint32_t n);

    // Dump CTRL/STATUS/HRES/VRES/counters/pixel_sum via xil_printf.
    void print_status();

    // Pretty-print the histogram as a horizontal bar chart, collapsed from
    // 256 HW bins down to num_bins display bins (must divide 256 evenly:
    // 1, 2, 4, 8, 16, 32, 64, 128, 256). bar_width is the max bar length
    // in characters; the largest bin is scaled to that width.
    XStatus print_histogram(uint16_t num_bins  = 32,
                            uint16_t bar_width = 60);

private:

    volatile isp_regs_t* regs() {
        return reinterpret_cast<volatile isp_regs_t*>(_base_addr);
    }

    uint32_t _base_addr;
    uint32_t _clock_freq_hz;
    uint32_t _max_polls = 1000000;
};

#endif
