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

XCsiSs_Config *CsiCfg;
XCsiSs         CsiInstance;


void error_handler(){
    print("Error handler called :( \r\n");
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
    xil_printf("INFO [kv260_ov9281_app] KV260 OV9281 init program\r\n");

    if(init_csi_subsystem() != XST_SUCCESS) error_handler();

    PL_IIC iic(CAM_I2C_DEVID);
    if (iic.init() != XST_SUCCESS) error_handler();
    if (init_iic_routing(iic) != XST_SUCCESS) error_handler();

    OV9281 cam(iic);
    if (cam.init() != XST_SUCCESS) error_handler();
    if (cam.apply_default_mode() != XST_SUCCESS) error_handler();

    xil_printf("INFO [kv260_ov9281_app] Init complete.\r\n");
    while(1){

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