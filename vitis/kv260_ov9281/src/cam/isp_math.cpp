#include "isp_math.hpp"
#include "xil_printf.h"

namespace {

// Pulse a CTRL bit while preserving the sticky HIST_ADDR_AUTOINC bit.
inline void pulse_ctrl(volatile isp_regs_t* r, uint32_t pulse_bm) {
    uint32_t keep = r->CTRL & ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm;
    r->CTRL = keep | pulse_bm;
}

} // namespace


ISP_MATH::ISP_MATH(uint32_t base_addr, uint32_t clock_freq_hz)
    : _base_addr(base_addr),
      _clock_freq_hz(clock_freq_hz)
{
    if (probe() != XST_SUCCESS) {
        xil_printf("[ISP_MATH] WARN: probe() failed at base 0x%08x\r\n",
                   base_addr);
    }
}

XStatus ISP_MATH::probe() {
    uint16_t hres, vres;
    get_resolution(&hres, &vres);
    if (hres == 1280 && vres == 800) {
        return XST_SUCCESS;
    }
    return XST_FAILURE;
}

// ---- Resets / control ------------------------------------------------------

void ISP_MATH::sw_reset() {
    pulse_ctrl(regs(), ISP_REGS__CTRL_REG__RESET_bm);
}

void ISP_MATH::reset_frame_counter() {
    pulse_ctrl(regs(), ISP_REGS__CTRL_REG__FRAME_CNT_RESET_bm);
}

void ISP_MATH::reset_cycle_counter() {
    pulse_ctrl(regs(), ISP_REGS__CTRL_REG__CYCLE_CNT_RESET_bm);
}

void ISP_MATH::set_hist_addr_autoinc(bool enable) {
    volatile isp_regs_t* r = regs();
    uint32_t v = r->CTRL & ~ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm;
    if (enable) v |= ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm;
    r->CTRL = v;
}

// ---- Resolution ------------------------------------------------------------

void ISP_MATH::set_resolution(uint16_t hres, uint16_t vres) {
    regs()->HRES = hres;
    regs()->VRES = vres;
}

void ISP_MATH::get_resolution(uint16_t* hres, uint16_t* vres) {
    if (hres) *hres = (uint16_t)(regs()->HRES & ISP_REGS__HRES_REG__HRES_bm);
    if (vres) *vres = (uint16_t)(regs()->VRES & ISP_REGS__VRES_REG__VRES_bm);
}

// ---- Counters / snapshot ---------------------------------------------------

isp_snapshot_t ISP_MATH::snapshot_frame_and_cycle_ctr() {
    volatile isp_regs_t* r = regs();
    pulse_ctrl(r, ISP_REGS__CTRL_REG__SNAPSHOT_bm);

    isp_snapshot_t s;
    s.frame_cnt = r->FRAME_SNAP;
    uint32_t hi = r->CYCLE_SNAP_HI;
    uint32_t lo = r->CYCLE_SNAP_LO;
    s.cycle_cnt = ((uint64_t)hi << 32) | lo;
    return s;
}

float ISP_MATH::compute_fps(const isp_snapshot_t* t1, const isp_snapshot_t* t2) {
    if (!t1 || !t2) return 0.0f;
    uint32_t frame_delta = t2->frame_cnt - t1->frame_cnt;
    uint64_t cycle_delta = t2->cycle_cnt - t1->cycle_cnt;
    if (cycle_delta == 0) return 0.0f;
    return ((float)frame_delta * (float)_clock_freq_hz) / (float)cycle_delta;
}

// ---- Status peeks ----------------------------------------------------------

bool ISP_MATH::is_ready() {
    return (regs()->STATUS & ISP_REGS__STATUS_REG__READY_bm) != 0;
}

bool ISP_MATH::is_hist_data_valid() {
    return (regs()->STATUS & ISP_REGS__STATUS_REG__HIST_DATA_VALID_bm) != 0;
}

bool ISP_MATH::is_hist_fifo_err() {
    return (regs()->STATUS & ISP_REGS__STATUS_REG__HIST_FIFO_ERR_bm) != 0;
}

// ---- Histogram measurement -------------------------------------------------

XStatus ISP_MATH::start_histogram() {
    uint16_t hres, vres;
    get_resolution(&hres, &vres);
    if (hres == 0 || vres == 0) return XST_FAILURE;
    if (!is_ready())            return XST_FAILURE;
    pulse_ctrl(regs(), ISP_REGS__CTRL_REG__HISTOGRAM_START_bm);
    return XST_SUCCESS;
}

XStatus ISP_MATH::poll_histogram_ready() {
    for (uint32_t i = 0; i < _max_polls; i++) {
        if (is_ready()) return XST_SUCCESS;
    }
    return XST_FAILURE;
}

XStatus ISP_MATH::poll_histogram_valid() {
    for (uint32_t i = 0; i < _max_polls; i++) {
        if (is_hist_data_valid()) return XST_SUCCESS;
    }
    return XST_FAILURE;
}

XStatus ISP_MATH::read_hist_bin(uint8_t addr, uint32_t* data) {
    if (!data) return XST_FAILURE;
    if (!is_hist_data_valid()) return XST_FAILURE;

    volatile isp_regs_t* r = regs();
    bool autoinc_was_on =
        (r->CTRL & ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm) != 0;

    if (autoinc_was_on) set_hist_addr_autoinc(false);

    r->HIST_ADDR = addr;
    *data = r->HIST_DATA & ISP_REGS__HIST_DATA_REG__HIST_DATA_bm;

    if (autoinc_was_on) set_hist_addr_autoinc(true);
    return XST_SUCCESS;
}

XStatus ISP_MATH::dump_histogram(isp_hist_t* hist_data) {
    if (!hist_data) return XST_FAILURE;
    if (!is_hist_data_valid()) return XST_FAILURE;

    volatile isp_regs_t* r = regs();
    bool autoinc_was_on =
        (r->CTRL & ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm) != 0;
    if (!autoinc_was_on) set_hist_addr_autoinc(true);

    r->HIST_ADDR = 0;
    for (uint16_t i = 0; i < NUM_BINS; i++) {
        (*hist_data)[i] = r->HIST_DATA & ISP_REGS__HIST_DATA_REG__HIST_DATA_bm;
    }

    if (!autoinc_was_on) set_hist_addr_autoinc(false);
    return XST_SUCCESS;
}

// ---- Pixel sum / brightness ------------------------------------------------

uint32_t ISP_MATH::read_pixel_sum() {
    return regs()->PIXEL_SUM;
}

float ISP_MATH::compute_avg_brightness() {
    uint16_t hres, vres;
    get_resolution(&hres, &vres);
    uint32_t npix = (uint32_t)hres * (uint32_t)vres;
    if (npix == 0) return 0.0f;
    return (float)read_pixel_sum() / (float)npix;
}

// ---- One-shot --------------------------------------------------------------

XStatus ISP_MATH::capture_histogram(isp_hist_t* hist_out, uint32_t* pixel_sum_out) {
    if (!hist_out) return XST_FAILURE;
    if (start_histogram()      != XST_SUCCESS) return XST_FAILURE;
    if (poll_histogram_valid() != XST_SUCCESS) return XST_FAILURE;
    if (dump_histogram(hist_out) != XST_SUCCESS) return XST_FAILURE;
    if (pixel_sum_out) *pixel_sum_out = read_pixel_sum();
    return XST_SUCCESS;
}

// ---- Diagnostics -----------------------------------------------------------

void ISP_MATH::set_max_polls(uint32_t n) {
    _max_polls = n;
}

void ISP_MATH::print_status() {
    volatile isp_regs_t* r = regs();
    uint32_t ctrl   = r->CTRL;
    uint32_t status = r->STATUS;
    uint16_t hres   = (uint16_t)(r->HRES & ISP_REGS__HRES_REG__HRES_bm);
    uint16_t vres   = (uint16_t)(r->VRES & ISP_REGS__VRES_REG__VRES_bm);
    uint32_t cyc_lo = r->CYCLE_CNT_LO;
    uint32_t cyc_hi = r->CYCLE_CNT_HI;
    uint32_t fcnt   = r->FRAME_CNT;
    uint32_t psum   = r->PIXEL_SUM;

    xil_printf("ISP_MATH @ 0x%08x\r\n", _base_addr);
    xil_printf("  CTRL    : 0x%08x (autoinc=%d)\r\n",
               ctrl,
               (ctrl & ISP_REGS__CTRL_REG__HIST_ADDR_AUTOINC_bm) ? 1 : 0);
    xil_printf("  STATUS  : 0x%08x [ready=%d hist_valid=%d fifo_err=%d]\r\n",
               status,
               (status & ISP_REGS__STATUS_REG__READY_bm) ? 1 : 0,
               (status & ISP_REGS__STATUS_REG__HIST_DATA_VALID_bm) ? 1 : 0,
               (status & ISP_REGS__STATUS_REG__HIST_FIFO_ERR_bm) ? 1 : 0);
    xil_printf("  HRES    : %u\r\n", hres);
    xil_printf("  VRES    : %u\r\n", vres);
    xil_printf("  CYCLES  : 0x%08x%08x\r\n", cyc_hi, cyc_lo);
    xil_printf("  FRAMES  : %u\r\n", fcnt);
    xil_printf("  PIX_SUM : %u\r\n", psum);
}

XStatus ISP_MATH::print_histogram(uint16_t num_bins, uint16_t bar_width) {
    // num_bins must divide 256 evenly: 1,2,4,8,16,32,64,128,256.
    if (num_bins == 0 || num_bins > NUM_BINS ||
        (NUM_BINS % num_bins) != 0) {
        xil_printf("[ISP_MATH] print_histogram: num_bins=%u invalid (must divide 256)\r\n",
                   num_bins);
        return XST_FAILURE;
    }
    if (bar_width == 0) bar_width = 1;

    isp_hist_t hist;
    if (dump_histogram(&hist) != XST_SUCCESS) {
        xil_printf("[ISP_MATH] print_histogram: histogram not valid\r\n");
        return XST_FAILURE;
    }

    uint16_t group = NUM_BINS / num_bins;

    uint32_t collapsed[NUM_BINS]; // worst case num_bins == 256
    uint32_t max_count = 0;
    for (uint16_t i = 0; i < num_bins; i++) {
        uint32_t sum = 0;
        for (uint16_t j = 0; j < group; j++) {
            sum += hist[i * group + j];
        }
        collapsed[i] = sum;
        if (sum > max_count) max_count = sum;
    }

    xil_printf("Histogram (%u bins, group=%u, max=%u):\r\n",
               num_bins, group, max_count);

    for (uint16_t i = 0; i < num_bins; i++) {
        uint16_t lo = i * group;
        uint16_t hi = lo + group - 1;
        uint32_t bar = (max_count == 0) ? 0
                     : (uint32_t)(((uint64_t)collapsed[i] * bar_width) / max_count);

        xil_printf("[%3u-%3u] ", lo, hi);
        for (uint32_t b = 0; b < bar; b++)             outbyte('#');
        for (uint32_t b = bar; b < bar_width; b++)     outbyte(' ');
        xil_printf(" %u\r\n", collapsed[i]);
    }
    return XST_SUCCESS;
}
