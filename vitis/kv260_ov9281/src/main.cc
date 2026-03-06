#include "xparameters.h"

#include "cam/pl_iic.hpp"

#include "platform.h"
// #include "cam/OV9281.h"
#include "cam/ScuGicInterruptController.h"
#include "cam/AXI_VDMA.h"

#include "xuartps.h"

#include <cstdint>
#include <xil_cache.h>
#include <xil_io.h>
#include <xil_types.h>

#define IRPT_CTL_DEVID 		XPAR_XSCUGIC_0_BASEADDR
#define CAM_I2C_DEVID		XPAR_XIIC_0_BASEADDR
#define VDMA_DEVID			XPAR_AXI_VDMA_0_BASEADDR
#define VDMA_S2MM_IRPT_ID	XPAR_FABRIC_AXI_VDMA_0_INTR
#define CAM_I2C_SCLK_RATE	100000

#define TCA9546_ADDR        0x74
#define TCA9546_PORT2_EN    0x04    // Bit 2 = enable port 2 (0b00000100)
#define TCA9546_DISABLE_ALL 0x00    // Disable all ports

// All code resides in DDR_0, check linker script
#define DDR_BASE_ADDR		XPAR_PSU_DDR_1_BASEADDRESS
#define MEM_BASE_ADDR		(DDR_BASE_ADDR)

#define DEBUG_EN            0x0
#define UART_BASEADDR       XPAR_UART1_BASEADDR

using namespace digilent;

enum class Resolution {
	R1920_1080_60_PP = 0,
	R1280_720_60_PP,
	R640_480_60_NN
};

typedef struct {
	enum Polarity {NEG=0, POS=1};
	Resolution res;
	uint16_t h_active, h_fp, h_sync, h_bp;
	Polarity h_pol;
	uint16_t v_active, v_fp, v_sync, v_bp;
	Polarity v_pol;
	uint32_t pclk_freq_Hz;

} timing_t;

timing_t const timing[] = {
    {Resolution::R1920_1080_60_PP, 1920, 88, 44, 148, timing_t::POS, 1080, 4, 5, 36, timing_t::POS, 148500000},
    {Resolution::R1280_720_60_PP, 1280, 110, 40, 220, timing_t::POS, 720, 5, 5, 20, timing_t::POS, 74250000},
    {Resolution::R640_480_60_NN, 640, 16, 96, 48, timing_t::NEG, 480, 10, 2, 33, timing_t::NEG, 25000000}
};

// void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
//                           OV9281& cam,
//                           Resolution res,
//                           OV9281_cfg::mode_t mode
//                           );

int main() {
    XStatus status;
    const uint8_t  cam_address   = (0xC0 >> 1); // OV9281 7-bit I2C address
    const uint16_t cam_id_reg_h  = 0x300A;       // OV9281 chip-ID high-byte register
    const uint16_t cam_id_reg_l  = 0x300B;       // OV9281 chip-ID low-byte register

    xil_printf("INFO [kv260_ov9281_app] KV260 OV9281 init program\r\n");

    PL_IIC iic(CAM_I2C_DEVID);
    if (iic.init() != XST_SUCCESS) {
        return -1;
    }

    // 4. Enable TCA9546A port 2 and read back the control register
    // The TCA9546A has a single 8-bit register: writing it sets which downstream
    // ports are enabled (bit N = port N).  Reading returns the current setting.
    xil_printf("INFO [kv260_ov9281_app] TCA9546A @ 0x%02X: enabling port 2 (0x%02X)...\r\n", TCA9546_ADDR, TCA9546_PORT2_EN);
    u8 mux_cfg = TCA9546_PORT2_EN;
    status = iic.write(TCA9546_ADDR, &mux_cfg, 1);
    if (status != XST_SUCCESS) {
        xil_printf("[FAIL] TCA9546A write failed.\r\n");
        return -1;
    }
    xil_printf("[PASS] TCA9546A write OK\r\n");

    u8 mux_readback = 0xAA;
    status = iic.read(TCA9546_ADDR, &mux_readback, 1);
    if (status != XST_SUCCESS) {
        xil_printf("[FAIL] TCA9546A read failed\r\n");
        return -1;
    }
    xil_printf("[PASS] TCA9546A readback: 0x%02X (expected 0x%02X)%s\r\n\r\n", mux_readback, TCA9546_PORT2_EN,
            (mux_readback == TCA9546_PORT2_EN) ? "" : " [MISMATCH]");

    // 5. Read OV9281 chip ID via 16-bit register addresses
    // Register map uses 16-bit addresses.  Each read is a combined transfer:
    //   START + addr+W + reg_hi + reg_lo + REPEATED_START + addr+R + byte + STOP
    xil_printf("[2] OV9281 @ 0x%02X: reading chip ID (0x%04X, 0x%04X)...\r\n", cam_address, cam_id_reg_h, cam_id_reg_l);
    u8 chip_id_h = 0, chip_id_l = 0;
    status = iic.reg16_read(cam_address, cam_id_reg_h, &chip_id_h, 1);
    if (status != XST_SUCCESS) {
        xil_printf("[FAIL] OV9281 not responding at 0x%02X\r\n", cam_address);
        return -1;
    }
    status = iic.reg16_read(cam_address, cam_id_reg_l, &chip_id_l, 1);
    if (status != XST_SUCCESS) {
        xil_printf("[FAIL] OV9281 chip ID low-byte read failed\r\n");
        return -1;
    }
    xil_printf("[PASS] OV9281 chip ID: 0x%02X%02X (expected 0x9281)\r\n\r\n", chip_id_h, chip_id_l);

    xil_printf("Init complete.\r\n");
    return 0;
}

// void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
//                           OV9281& cam,
//                           Resolution res,
//                           OV9281_cfg::mode_t mode
//                           )
// {
// 	// Bring up input pipeline back-to-front
//     vdma_driver.resetWrite();
// #if (DEBUG_EN == 0x0)
//     // cam.reset();
// #endif
//     vdma_driver.configureWrite(
//         timing[static_cast<int>(res)].h_active,
//         timing[static_cast<int>(res)].v_active
//     );
//     // TODO CSI-2, D-PHY config here
// #if (DEBUG_EN == 0x0)
//     // cam.init();
//     vdma_driver.enableWrite();
// #endif
// #if (DEBUG_EN == 0x0)
//     // cam.set_mode(mode);
//     // cam.set_awb(OV9281_cfg::awb_t::AWB_ADVANCED);
// #endif
//     vdma_driver.enableRead();
// }

/*
XUartPs uart_instance;
XUartPs_Config * uart_config;

uint8_t dGetChar(){
    //dFlushUart();
    // Wait for data on UART 
    u8 char_recv, char_old;
    while (1){
        XUartPs_Recv(&uart_instance, &char_recv, 1);
        if (char_recv == '\n'){
            break;
        }
        char_old = char_recv;
    }
    return char_old;
}

int uart_cfg(){
    XStatus status;
    uart_config = XUartPs_LookupConfig(UART_BASEADDR);
    if(uart_config == NULL){
        xil_printf("ERROR: Could not find uart config\r\n");
        return -1;
    }

    status = XUartPs_CfgInitialize(&uart_instance, uart_config, UART_BASEADDR);
    if(status != XST_SUCCESS){
        xil_printf("ERROR: Could not init uart\r\n");
        return -1;
    }

    u8 c = dGetChar();
}
*/