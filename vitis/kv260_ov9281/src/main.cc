#include "xparameters.h"
#include "platform.h"
#include "xcsiss.h"
#include <xil_types.h>
#include "xil_cache.h"
#include "sleep.h"

#include "cam/AXI_VDMA.h"
#include "cam/pl_iic.hpp"
#include "cam/OV9281.h"
#include "cam/ScuGicInterruptController.h"
#include "cam/isp_stats.hpp"

#define IRPT_CTL_DEVID 		XPAR_XSCUGIC_0_BASEADDR
#define CAM_I2C_DEVID		XPAR_XIIC_0_BASEADDR
#define VDMA_DEVID          XPAR_AXI_VDMA_0_BASEADDR
#define VDMA_WRITE_INTR_ID  XPAR_FABRIC_AXI_VDMA_0_INTR
#define ISP_BASEADDR        XPAR_ISP_MATH_WRAPPER_0_BASEADDR
// Must match the AXIS clock driving isp_math_top (its cycle counter ticks here).
#define ISP_CLOCK_FREQ_HZ   200000000

#define TCA9546_ADDR        0x74
#define TCA9546_PORT2_EN    0x04    // Bit 2 = enable port 2 (0b00000100)

// CSI
XCsiSs_Config *CsiCfg;
XCsiSs         CsiInstance;

#define NUM_FRAMES 3
#define FRAME_SIZE_VERT  800
#define FRAME_SIZE_HORZ  1280
#define FB_SIZE_BYTES (NUM_FRAMES*FRAME_SIZE_VERT*FRAME_SIZE_HORZ)
static volatile uint8_t __attribute__((aligned(1024))) frame_buffer[NUM_FRAMES][FRAME_SIZE_VERT][FRAME_SIZE_HORZ];

void error_handler(const char* err){
    xil_printf("ERROR HANDLER: %s\r\n", err);
    while(1){}
}

/**
 * Routes I2C through the TCA9546A mux to reach the camera on port 2.
 * The TCA9546A has one 8-bit R/W register: bit N enables downstream port N.
 * Returns XST_SUCCESS or XST_FAILURE.
*/
static int init_iic_routing(PL_IIC& iic);

/**
 * Initializes and enables the CSI2RX 
 */
static int init_csi_subsystem();

int main() {
    xil_printf("INFO [kv260_ov9281_app] Program start\r\n");

    // digilent::ScuGicInterruptController gic(IRPT_CTL_DEVID);
    // if(gic.init() != XST_SUCCESS) error_handler("Failed to init GIC");
    // xil_printf("INFO [kv260_ov9281_app] Successful interrupt initialization\r\n");

    IspStats isp(ISP_BASEADDR, ISP_CLOCK_FREQ_HZ);
    isp.sw_reset();
    isp.set_resolution(1280, 800);

    // Initialize PL i2c and route traffic to rpi connector
    PL_IIC iic(CAM_I2C_DEVID);
    if (iic.init() != XST_SUCCESS) error_handler("Failed to init i2c");
    if (init_iic_routing(iic) != XST_SUCCESS) error_handler("Failed to route i2c to mipi connector");
    xil_printf("INFO [kv260_ov9281_app] I2C initialized and routed to camera\r\n");

    // Instantiate, reset, and init camera
    OV9281 cam(iic);
    cam.reset();
    if (cam.init() != XST_SUCCESS) error_handler("Failed to init ov9281");
    if (cam.apply_default_mode() != XST_SUCCESS) error_handler("Failed to apply camera mode");

    // Init MIPI CSI Receiver
    if(init_csi_subsystem() != XST_SUCCESS) error_handler("Failed to init CSI subsystem");

    // Initialize VDMA
    AXI_VDMA vdma(VDMA_DEVID, (UINTPTR)frame_buffer);
    if(vdma.init() != XST_SUCCESS) error_handler("Failed to init vdma");
    vdma.resetWrite();
    if(vdma.configureWrite(1280, 800) != XST_SUCCESS) error_handler("Failed to configure 1280x800 vdma");
    // if (vdma.connectWriteInterrupt(gic, VDMA_WRITE_INTR_ID) != XST_SUCCESS)                        
    //     error_handler("Failed to connect VDMA interrupt");
    vdma.enableWrite();
    cam.start_streaming();
    // gic.enableInterrupts();

    xil_printf("INFO [kv260_ov9281_app] VDMA initialized with interrupts\r\n");
    xil_printf("INFO [kv260_ov9281_app] VDMA Buffer base address: 0x%X\r\n", (UINTPTR)frame_buffer);
    xil_printf("INFO [kv260_ov9281_app] Initialization completed successfully\r\n");

    isp_snapshot_t prev_snap = isp.snapshot_frame_and_cycle_ctr();

    xil_printf("INFO [kv260_ov9281_app] Initial snapshot: frames=%u cycles=0x%08x%08x\r\n",
               prev_snap.frame_cnt,
               (uint32_t)(prev_snap.cycle_cnt >> 32),
               (uint32_t)(prev_snap.cycle_cnt));

    while(1){
        usleep(1000000);

        // ANSI clear screen + home cursor so each refresh paints over a blank screen.
        xil_printf("\x1b[2J\x1b[H");

        // FPS over the last ~1s window.
        isp_snapshot_t cur_snap = isp.snapshot_frame_and_cycle_ctr();
        float fps = isp.compute_fps(&prev_snap, &cur_snap);
        prev_snap = cur_snap;
        // xil_printf has no %f; print integer + 2 fractional digits manually.
        int fps_int  = (int)fps;
        int fps_frac = (int)((fps - (float)fps_int) * 100.0f);
        if (fps_frac < 0) fps_frac = -fps_frac;
        xil_printf("INFO [kv260_ov9281_app] FPS: %d.%02d\r\n", fps_int, fps_frac);

        // Capture one frame's histogram + pixel sum and pretty-print.
        isp_hist_t hist;
        uint32_t   psum = 0;
        if (isp.capture_histogram(&hist, &psum) == XST_SUCCESS) {
            isp.print_histogram(16, 75);
            xil_printf("INFO [kv260_ov9281_app] Pixel sum: %u\r\n", psum);
            float avg = isp.compute_avg_brightness();
            int avg_int  = (int)avg;
            int avg_frac = (int)((avg - (float)avg_int) * 100.0f);
            if (avg_frac < 0) avg_frac = -avg_frac;
            xil_printf("INFO [kv260_ov9281_app] Avg brightness: %d.%02d\r\n", avg_int, avg_frac);
        } else {
            xil_printf("WARN [kv260_ov9281_app] capture_histogram failed\r\n");
            isp.print_status();
        }
    }
    return 0;
}

static int init_csi_subsystem(){
    // Initialize the CSI-2 Rx Subsystem
    CsiCfg = XCsiSs_LookupConfig(XPAR_MIPI_CSI2_RX_SUBSYST_0_BASEADDR);
    if (CsiCfg == NULL) {
        xil_printf("ERROR [kv260_ov9281_app::init_csi_subsystem] XCsiSs_LookupConfig failed\r\n");
        return XST_FAILURE;
    }
    if (XCsiSs_CfgInitialize(&CsiInstance, CsiCfg, CsiCfg->BaseAddr) != XST_SUCCESS) {
        xil_printf("ERROR [kv260_ov9281_app::init_csi_subsystem] XCsiSs_CfgInitialize failed\r\n");
        return XST_FAILURE;
    }
    if (XCsiSs_Reset(&CsiInstance) != XST_SUCCESS) {
        xil_printf("ERROR [kv260_ov9281_app::init_csi_subsystem] XCsiSs_Reset failed\r\n");
        return XST_FAILURE;
    }
    // ActiveLanes must match the number of MIPI data lanes wired in hardware
    if (XCsiSs_Configure(&CsiInstance, CsiCfg->LanesPresent, 0) != XST_SUCCESS) {
        xil_printf("ERROR [kv260_ov9281_app::init_csi_subsystem] XCsiSs_Configure failed\r\n");
        return XST_FAILURE;
    }
    if (XCsiSs_Activate(&CsiInstance, XCSI_ENABLE) != XST_SUCCESS) {
        xil_printf("ERROR [kv260_ov9281_app::init_csi_subsystem] XCsiSs_Activate failed\r\n");
        return XST_FAILURE;
    }

    xil_printf("INFO [kv260_ov9281_app::init_csi_subsystem] Num mipi lanes: %d\r\n", CsiCfg->LanesPresent);
    return XST_SUCCESS;
}

static int init_iic_routing(PL_IIC& iic) {
    xil_printf("INFO [init_iic_routing] Enabling TCA9546A port 2 @ 0x%02X...\r\n", TCA9546_ADDR);
    u8 mux_cfg = TCA9546_PORT2_EN;
    if (iic.write(TCA9546_ADDR, &mux_cfg, 1) != XST_SUCCESS) {
        xil_printf("ERROR [init_iic_routing] TCA9546A write failed\r\n");
        return XST_FAILURE;
    }
    u8 readback = 0xAA;
    if (iic.read(TCA9546_ADDR, &readback, 1) != XST_SUCCESS) {
        xil_printf("ERROR [init_iic_routing] TCA9546A readback failed\r\n");
        return XST_FAILURE;
    }
    if (readback != TCA9546_PORT2_EN) {
        xil_printf("ERROR [init_iic_routing] TCA9546A mismatch: got 0x%02X expected 0x%02X\r\n",
                   readback, TCA9546_PORT2_EN);
        return XST_FAILURE;
    }
    xil_printf("INFO [init_iic_routing] TCA9546A routing OK (readback 0x%02X)\r\n", readback);
    return XST_SUCCESS;
}

// Xil_DCacheInvalidateRange((INTPTR)frame_buffer, FB_SIZE_BYTES);
        // xil_printf("INFO [kv260_ov9281_app] Frame buffer[0][0][0-7]: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n",
        //     frame_buffer[0][0][0], frame_buffer[0][0][1], frame_buffer[0][0][2], frame_buffer[0][0][3],
        //     frame_buffer[0][0][4], frame_buffer[0][0][5], frame_buffer[0][0][6], frame_buffer[0][0][7]
        // );