#include "xparameters.h"
#include "xuartps.h"
#include "xcsiss.h"
#include <cstdint>
#include <xil_cache.h>
#include <xil_io.h>
#include <xil_types.h>
//#include "xparameters_ps.h" // for INTRs --> digilent comment, IDK (wbuchta)

#include "platform.h"

#include "cam/OV9281.h"
#include "cam/ScuGicInterruptController.h"
#include "cam/PS_GPIO.h"
#include "cam/AXI_VDMA.h"
#include "cam/PS_IIC.h"
#include "hdmi/VideoOutput.h"


#define IRPT_CTL_DEVID 		XPAR_XSCUGIC_0_BASEADDR
#define GPIO_DEVID			XPAR_GPIO0_BASEADDR
#define GPIO_IRPT_ID		XPAR_PS7_GPIO_0_INTR
#define IIC_DEV_BASEADDR    XPAR_I2C0_BASEADDR
#define CAM_I2C_IRPT_ID		XPAR_PS7_I2C_0_INTR
#define VDMA_DEVID			XPAR_AXI_VDMA_0_BASEADDR
#define VDMA_MM2S_IRPT_ID	XPAR_FABRIC_AXI_VDMA_0_INTR
#define VDMA_S2MM_IRPT_ID	XPAR_FABRIC_AXI_VDMA_0_INTR_1
#define CAM_I2C_SCLK_RATE	100000

// CSI
XCsiSs_Config *CsiCfg;
XCsiSs         CsiInstance;

#define NUM_FRAMES 1
#define FRAME_SIZE_VERT  800
#define FRAME_SIZE_HORZ  1280
#define FB_SIZE_BYTES (NUM_FRAMES*FRAME_SIZE_VERT*FRAME_SIZE_HORZ)
static volatile uint8_t __attribute__((aligned(1024))) _frame_buffer[NUM_FRAMES][FRAME_SIZE_VERT][FRAME_SIZE_HORZ];

// #define GAMMA_BASE_ADDR     XPAR_AXI_GAMMACORRECTION_0_BASEADDR

#define DEBUG_EN            0x0
#define UART_BASEADDR       XPS_UART0_BASEADDR

using namespace digilent;

void dFlushUart();
uint8_t dGetChar();

void error_handler(const char* err){
    xil_printf("ERROR HANDLER: %s\r\n", err);
    while(1){}
}
// void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
//                           OV9281& cam,
//                           VideoOutput& vid,
//                           Resolution res,
//                           OV9281_cfg::mode_t mode
//                           );

/**
 * Initializes and enables the CSI2RX 
 */
static int init_csi_subsystem();

int main()
{
	// init_platform();
    // Xil_ICacheEnable();
    // Xil_DCacheEnable();

    if(init_csi_subsystem() != XST_SUCCESS) error_handler("Failed to init CSI subsystem");

	for(u32 i = 0; i < FRAME_SIZE_VERT; i++){
		for(u32 j = 0; j < FRAME_SIZE_HORZ; j++){
			_frame_buffer[0][i][j] = 0x80; //gray
		}
	}
	xil_printf("INFO [zybo_ov9281_app] Frame buffer initialized.\r\n");

	ScuGicInterruptController irpt_ctl(IRPT_CTL_DEVID);
	
	PS_IIC iic(IIC_DEV_BASEADDR, CAM_I2C_SCLK_RATE);
    if (iic.init() != XST_SUCCESS) error_handler("Failed to init i2c");
    xil_printf("INFO [zybo_ov9281_app] I2C initialized and routed to camera\r\n");

	// Initialize VDMA
    AXI_VDMA vdma(VDMA_DEVID, (UINTPTR)_frame_buffer);
    if(vdma.init() != XST_SUCCESS) error_handler("Failed to init vdma");
    if (vdma.configureWrite(1280, 800) != XST_SUCCESS) error_handler("Failed to configure VDMA write");
    if (vdma.enableWrite() != XST_SUCCESS) error_handler("Failed to enable VDMA write");

    if (vdma.configureRead(1280, 720) != XST_SUCCESS) error_handler("Failed to configure VDMA read");
    if (vdma.enableRead() != XST_SUCCESS) error_handler("Failed to enable VDMA read");


    // Instantiate and initialize camera
    OV9281 cam(iic);
    if (cam.init() != XST_SUCCESS) error_handler("Failed to init ov9281");
    if (cam.apply_default_mode() != XST_SUCCESS) error_handler("Failed to apply camera mode");

	VideoOutput vid(XPAR_VTG_BASEADDR, XPAR_VIDEO_DYNCLK_BASEADDR);
	if (vid.init() != XST_SUCCESS) error_handler("Failed to init VideoOutput");

	// pipeline_mode_change(vdma_driver, cam, vid, Resolution::R1280_720_60_PP, OV9281_cfg::mode_t::MODE_720P_1280_720_60fps);
    vid.reset();
    vid.configure(Resolution::R1280_720_60_PP);
    vid.enable();
	
	xil_printf("Video init done.\r\n");


	//cleanup_platform();
    //Xil_ICacheDisable();
    //Xil_DCacheDisable();
    
	return 0;
}

void dFlushUart()
{
    while (XUartPs_IsReceiveData(UART_BASEADDR))
		XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
}

uint8_t dGetChar()
{
    uint8_t chRxCh = '0';
    //dFlushUart();
    /* Wait for data on UART */
    while (XUartPs_IsReceiveData(UART_BASEADDR))
    {
        chRxCh = XUartPs_ReadReg(UART_BASEADDR, XUARTPS_FIFO_OFFSET);
        if (chRxCh == '\n')
        {
            break;
        }
    }
    return chRxCh;
}

static int init_csi_subsystem(){
    // Initialize the CSI-2 Rx Subsystem
    CsiCfg = XCsiSs_LookupConfig(XPAR_MIPI_CSI2_RX_SUBSYST_0_BASEADDR);
    if (CsiCfg == NULL) {
        xil_printf("ERROR [zybo_ov9281_app::init_csi_subsystem] XCsiSs_LookupConfig failed\r\n");
        return XST_FAILURE;
    }
    if (XCsiSs_CfgInitialize(&CsiInstance, CsiCfg, CsiCfg->BaseAddr) != XST_SUCCESS) {
        xil_printf("ERROR [zybo_ov9281_app::init_csi_subsystem] XCsiSs_CfgInitialize failed\r\n");
        return XST_FAILURE;
    }
    if (XCsiSs_Reset(&CsiInstance) != XST_SUCCESS) {
        xil_printf("ERROR [zybo_ov9281_app::init_csi_subsystem] XCsiSs_Reset failed\r\n");
        return XST_FAILURE;
    }
    // ActiveLanes must match the number of MIPI data lanes wired in hardware
    if (XCsiSs_Configure(&CsiInstance, CsiCfg->LanesPresent, 0) != XST_SUCCESS) {
        xil_printf("ERROR [zybo_ov9281_app::init_csi_subsystem] XCsiSs_Configure failed\r\n");
        return XST_FAILURE;
    }
    if (XCsiSs_Activate(&CsiInstance, XCSI_ENABLE) != XST_SUCCESS) {
        xil_printf("ERROR [zybo_ov9281_app::init_csi_subsystem] XCsiSs_Activate failed\r\n");
        return XST_FAILURE;
    }

    xil_printf("INFO [zybo_ov9281_app::init_csi_subsystem] Num mipi lanes: %d\r\n", CsiCfg->LanesPresent);
    return XST_SUCCESS;
}

// void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
//                           OV9281& cam,
//                           VideoOutput& vid,
//                           Resolution res,
//                           OV9281_cfg::mode_t mode
//                           )
// {
// 	// Bring up input pipeline back-to-front
//     vdma_driver.resetWrite();
//     MIPI_CSI_2_RX_mWriteReg(
//         XPAR_MIPI_CSI_2_RX_0_BASEADDR, 
//         CR_OFFSET, 
//         (CR_RESET_MASK & ~CR_ENABLE_MASK)
//     );
//     MIPI_D_PHY_RX_mWriteReg(
//         XPAR_MIPI_D_PHY_RX_0_BASEADDR, 
//         CR_OFFSET, 
//         (CR_RESET_MASK & ~CR_ENABLE_MASK)
//     );
// #if (DEBUG_EN == 0x0)
//     // cam.reset();
// #endif
//     vdma_driver.configureWrite(
//         timing[static_cast<int>(res)].h_active,
//         timing[static_cast<int>(res)].v_active
//     );
//     // Set Gamma correction factor to 1/1.8
//     Xil_Out32(GAMMA_BASE_ADDR, 3);
//     // TODO CSI-2, D-PHY config here
// #if (DEBUG_EN == 0x0)
//     // cam.init();
//     vdma_driver.enableWrite();
// #endif
//     MIPI_CSI_2_RX_mWriteReg(
//         XPAR_MIPI_CSI_2_RX_0_BASEADDR,
//         CR_OFFSET,
//         CR_ENABLE_MASK
//     );
//     MIPI_D_PHY_RX_mWriteReg(
//         XPAR_MIPI_D_PHY_RX_0_BASEADDR,
//         CR_OFFSET,
//         CR_ENABLE_MASK
//     );
// #if (DEBUG_EN == 0x0)
//     // cam.set_mode(mode);
//     // cam.set_awb(OV9281_cfg::awb_t::AWB_ADVANCED);
// #endif
//     // Bring up output pipeline back-to-front
//     vid.reset();
//     vdma_driver.resetRead();
//     vid.configure(res);
//     // Alloc mem for frames
//     vdma_driver.configureRead(
//         timing[static_cast<int>(res)].h_active,
//         timing[static_cast<int>(res)].v_active
//     );
// #if (DEBUG_EN == 0x1)
//     // Write RBG set to test pipeline
//     // ...
//     // VDMA Frame 0 Addr: 0x0A000000
//     // VDMA Frame 1 Addr: 0x0A5EEC00
//     // VDMA Frame 2 Addr: 0x0ABDD800
//     u32 framesAddr[3] = {0x0A000000, 0x0A5EEC00, 0x0ABDD800};
//     // difference : 5E EC00
//     u32 diffAddrs = framesAddr[1] - framesAddr[0];
//     for (u32 x = 0; x < diffAddrs; x++) {
//         Xil_Out32(framesAddr[0] + x, 0x00FFFFFF);
//         Xil_Out32(framesAddr[1] + x, 0x00FFFFFF);
//         Xil_Out32(framesAddr[2] + x, 0x00FFFFFF);
//     }
// #endif
//     vid.enable();
//     vdma_driver.enableRead();
// }