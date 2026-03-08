#include "xparameters.h"
#include "platform.h"
#include "xcsiss.h"

#include "cam/pl_iic.hpp"
#include "cam/OV9281.h"
#include <xil_types.h>

#define IRPT_CTL_DEVID 		XPAR_XSCUGIC_0_BASEADDR
#define CAM_I2C_DEVID		XPAR_XIIC_0_BASEADDR

#define TCA9546_ADDR        0x74
#define TCA9546_PORT2_EN    0x04    // Bit 2 = enable port 2 (0b00000100)

// Routes I2C through the TCA9546A mux to reach the camera on port 2.
// The TCA9546A has one 8-bit R/W register: bit N enables downstream port N.
// Returns XST_SUCCESS or XST_FAILURE.
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

int main() {
    xil_printf("INFO [kv260_ov9281_app] KV260 OV9281 init program\r\n");

    // Initialize the CSI-2 Rx Subsystem
    XCsiSs_Config *CsiCfg;
    XCsiSs         CsiInstance;

    CsiCfg = XCsiSs_LookupConfig(XPAR_MIPI_CSI2_RX_SUBSYST_0_BASEADDR);
    if (CsiCfg == NULL) {
        xil_printf("ERROR [main] XCsiSs_LookupConfig failed\r\n");
        return -1;
    }
    if (XCsiSs_CfgInitialize(&CsiInstance, CsiCfg, CsiCfg->BaseAddr) != XST_SUCCESS) {
        xil_printf("ERROR [main] XCsiSs_CfgInitialize failed\r\n");
        return -1;
    }
    if (XCsiSs_Reset(&CsiInstance) != XST_SUCCESS) {
        xil_printf("ERROR [main] XCsiSs_Reset failed\r\n");
        return -1;
    }
    // ActiveLanes must match the number of MIPI data lanes wired in hardware
    if (XCsiSs_Configure(&CsiInstance, CsiCfg->LanesPresent, 0) != XST_SUCCESS) {
        xil_printf("ERROR [main] XCsiSs_Configure failed\r\n");
        return -1;
    }
    if (XCsiSs_Activate(&CsiInstance, XCSI_ENABLE) != XST_SUCCESS) {
        xil_printf("ERROR [main] XCsiSs_Activate failed\r\n");
        return -1;
    }

    xil_printf("INFO [main] Num lanes from config: %d\r\n", CsiCfg->LanesPresent);



    PL_IIC iic(CAM_I2C_DEVID);
    if (iic.init() != XST_SUCCESS) return -1;
    if (init_iic_routing(iic) != XST_SUCCESS) return -1;

    digilent::OV9281 cam(iic);
    if (cam.init() != XST_SUCCESS) return -1;
    if (cam.apply_default_mode() != XST_SUCCESS) return -1;
    

    xil_printf("INFO [kv260_ov9281_app] Init complete.\r\n");
    while(1){

    }
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