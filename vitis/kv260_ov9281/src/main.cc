#include "xparameters.h"
#include "xiic.h"

#include "platform.h"
#include "cam/OV9281.h"
#include "cam/ScuGicInterruptController.h"
#include "cam/PS_GPIO.h"
#include "cam/AXI_VDMA.h"
#include "cam/PS_IIC.h"

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

// ─── Helper: Detect device by attempting a 0-byte write ──────────────────────
//  XIic_Send returns the number of bytes sent. On NACK (no device), it returns 0.
static int i2c_detect(u32 BaseAddr, u8 SlaveAddr) {
    u8 dummy = 0;
    int ret = XIic_Send(BaseAddr, SlaveAddr, &dummy, 0, XIIC_STOP);
    // A successful probe returns 0 (0 bytes sent, but ACK received)
    // On NACK or error, the controller will return an error status
    // In practice with AXI IIC, check that no bus error occurred
    return (ret >= 0) ? XST_SUCCESS : XST_FAILURE;
}

XUartPs uart_instance;
XUartPs_Config * uart_config;

uint8_t dGetChar(){
    //dFlushUart();
    /* Wait for data on UART */
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

void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
                          OV9281& cam,
                          Resolution res,
                          OV9281_cfg::mode_t mode
                          );

int main() {
    XStatus status;
    uint8_t  mux_address = 0x74;
    uint8_t  cam_address = (0xC0 >> 1);
    uint16_t cam_id_l    = 0x300B;
    uint16_t cam_id_h    = 0x300A;

    u8 mux_status = 0;
    u8 rx_data[2] = {0};

    XIic_Config *pi2c_cfg = NULL;
    XIic        i2c_instance;

    xil_printf("Hello world from the KV260 OV9281\r\n");

    pi2c_cfg = XIic_LookupConfig(CAM_I2C_DEVID);
    if(pi2c_cfg == NULL){
        xil_printf("ERROR: Could not find i2c config\r\n");
        return -1;
    }
    xil_printf("Succesfully found i2c config\r\n");
    xil_printf("Config BaseAddress: 0x%08X\r\n", pi2c_cfg->BaseAddress);
    xil_printf("Reading status register...\r\n");
    u32 status_register = Xil_In32(pi2c_cfg->BaseAddress + 0x104);
    xil_printf("Status register: 0x%x\r\n", status_register);

    status = XIic_CfgInitialize(&i2c_instance, pi2c_cfg, pi2c_cfg->BaseAddress);
    if(status != XST_SUCCESS){
        xil_printf("ERROR: Unable to initialize i2c config\r\n");
        return -1;
    }
    xil_printf("Succesfully init the config\r\n");

    status = XIic_SelfTest(&i2c_instance);
    if(status != XST_SUCCESS){
        xil_printf("ERROR: i2c instance failed self test\r\n");
        return -1;
    }

    xil_printf("[1] Scanning for TCA9546A at 0x%02X...\r\n", TCA9546_ADDR);
    status = i2c_detect(CAM_I2C_DEVID, TCA9546_ADDR);
    if (status != XST_SUCCESS) {
        xil_printf("[FAIL] TCA9546A not found. Check:\r\n"
                   "       - Pull-up resistors on SCL/SDA\r\n"
                   "       - VCC to the mux\r\n"
                   "       - A2/A1/A0 pin state vs assumed addr 0x%02X\r\n",
                   TCA9546_ADDR);
        return -1;
    }
    xil_printf("[PASS] TCA9546A detected at 0x%02X\r\n\r\n", TCA9546_ADDR);

    // volatile uint8_t* fb = (uint8_t*)MEM_BASE_ADDR;
	// for(u32 i = 0; i < 1920*1080*3; i+=3){
	// 	fb[i]   = 0x00; // Green
	// 	fb[i+1] = 0xFF; // Blue
	// 	fb[i+2] = 0xFF; // Red
	// }
    // Xil_DCacheFlush();

	// ScuGicInterruptController irpt_ctl(IRPT_CTL_DEVID);

	// PS_GPIO<ScuGicInterruptController> gpio_driver(GPIO_DEVID, irpt_ctl, GPIO_IRPT_ID);
	
	// PS_IIC<ScuGicInterruptController> iic_driver(CAM_I2C_DEVID, irpt_ctl, CAM_I2C_IRPT_ID, CAM_I2C_SCLK_RATE);

	// OV9281 cam(iic_driver, gpio_driver);

	// AXI_VDMA<ScuGicInterruptController> vdma_driver(
	// 	VDMA_DEVID,
    //     MEM_BASE_ADDR,
    //     irpt_ctl,
    //     VDMA_MM2S_IRPT_ID,
    //     VDMA_S2MM_IRPT_ID
    // );


	// pipeline_mode_change(vdma_driver, cam, vid, Resolution::R1280_720_60_PP, OV9281_cfg::mode_t::MODE_720P_1280_720_60fps);	
	// xil_printf("Video init done.\r\n");


	//cleanup_platform();
    //Xil_ICacheDisable();
    //Xil_DCacheDisable();
    
	return 0;
}

void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
                          OV9281& cam,
                          Resolution res,
                          OV9281_cfg::mode_t mode
                          )
{
	// Bring up input pipeline back-to-front
    vdma_driver.resetWrite();
#if (DEBUG_EN == 0x0)
    // cam.reset();
#endif
    vdma_driver.configureWrite(
        timing[static_cast<int>(res)].h_active,
        timing[static_cast<int>(res)].v_active
    );
    // TODO CSI-2, D-PHY config here
#if (DEBUG_EN == 0x0)
    // cam.init();
    vdma_driver.enableWrite();
#endif
#if (DEBUG_EN == 0x0)
    // cam.set_mode(mode);
    // cam.set_awb(OV9281_cfg::awb_t::AWB_ADVANCED);
#endif
    vdma_driver.enableRead();
}