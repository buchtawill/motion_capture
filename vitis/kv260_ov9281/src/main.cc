#include "xparameters.h"
//#include "xparameters_ps.h" // for INTRs

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
#define GPIO_DEVID			XPAR_GPIO0_BASEADDR
// #define GPIO_IRPT_ID		XPAR_PS7_GPIO_0_INTR
#define CAM_I2C_DEVID		XPAR_I2C1_BASEADDR
// #define CAM_I2C_IRPT_ID		XPAR_PS7_I2C_0_INTR
#define VDMA_DEVID			XPAR_AXI_VDMA_0_BASEADDR
#define VDMA_MM2S_IRPT_ID	XPAR_FABRIC_AXI_VDMA_0_INTR
#define VDMA_S2MM_IRPT_ID	XPAR_FABRIC_AXI_VDMA_0_INTR_1
#define CAM_I2C_SCLK_RATE	100000

#define DDR_BASE_ADDR		XPAR_DDR_MEM_BASEADDR
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x0A000000)

#define GAMMA_BASE_ADDR     XPAR_AXI_GAMMACORRECTION_0_BASEADDR

#define DEBUG_EN            0x0
#define UART_BASEADDR       XPS_UART0_BASEADDR

using namespace digilent;

void dFlushUart();
uint8_t dGetChar();
void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
                          OV9281& cam,
                          VideoOutput& vid,
                          Resolution res,
                          OV9281_cfg::mode_t mode
                          );

int main()
{
	// init_platform();
    // Xil_ICacheEnable();
    // Xil_DCacheEnable();
	xil_printf("Starting cam ping test\r\n");


    volatile uint8_t* fb = (uint8_t*)MEM_BASE_ADDR;
	for(u32 i = 0; i < 1920*1080*3; i+=3){
		fb[i]   = 0x00; // Green
		fb[i+1] = 0xFF; // Blue
		fb[i+2] = 0xFF; // Red
	}
    Xil_DCacheFlush();

	ScuGicInterruptController irpt_ctl(IRPT_CTL_DEVID);

	PS_GPIO<ScuGicInterruptController> gpio_driver(GPIO_DEVID, irpt_ctl, GPIO_IRPT_ID);
	
	PS_IIC<ScuGicInterruptController> iic_driver(CAM_I2C_DEVID, irpt_ctl, CAM_I2C_IRPT_ID, CAM_I2C_SCLK_RATE);

	OV9281 cam(iic_driver, gpio_driver);

	AXI_VDMA<ScuGicInterruptController> vdma_driver(
		VDMA_DEVID,
        MEM_BASE_ADDR,
        irpt_ctl,
        VDMA_MM2S_IRPT_ID,
        VDMA_S2MM_IRPT_ID
    );


	pipeline_mode_change(vdma_driver, cam, vid, Resolution::R1280_720_60_PP, OV9281_cfg::mode_t::MODE_720P_1280_720_60fps);
	
	xil_printf("Video init done.\r\n");

	uint8_t reg_value;

	while (1) {
		xil_printf("\r\n\r\n\r\nPcam 5C MAIN OPTIONS\r\n");
		xil_printf("\r\nPlease press the key corresponding to the desired option:");
		xil_printf("\r\n  a. Change Resolution");
		// xil_printf("\r\n  b. Change Liquid Lens Focus");
		xil_printf("\r\n  d. Change Image Format (Raw or RGB)");
		xil_printf("\r\n  e. Write a Register Inside the Image Sensor");
		xil_printf("\r\n  f. Read a Register Inside the Image Sensor");
		xil_printf("\r\n  g. Change Gamma Correction Factor Value");
		xil_printf("\r\n  h. Change AWB Settings\r\n\r\n");

		read_char0 = getchar(); getchar();
		xil_printf("Read: %d\r\n", read_char0);
	}

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

void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver,
                          OV9281& cam,
                          VideoOutput& vid,
                          Resolution res,
                          OV9281_cfg::mode_t mode
                          )
{
	// Bring up input pipeline back-to-front
    vdma_driver.resetWrite();
    // MIPI_CSI_2_RX_mWriteReg(
    //     XPAR_MIPI_CSI_2_RX_0_BASEADDR, 
    //     CR_OFFSET, 
    //     (CR_RESET_MASK & ~CR_ENABLE_MASK)
    // );
    // MIPI_D_PHY_RX_mWriteReg(
    //     XPAR_MIPI_D_PHY_RX_0_BASEADDR, 
    //     CR_OFFSET, 
    //     (CR_RESET_MASK & ~CR_ENABLE_MASK)
    // );
#if (DEBUG_EN == 0x0)
    // cam.reset();
#endif
    vdma_driver.configureWrite(
        timing[static_cast<int>(res)].h_active,
        timing[static_cast<int>(res)].v_active
    );
    // Set Gamma correction factor to 1/1.8
    // Xil_Out32(GAMMA_BASE_/ADDR, 3);
    // TODO CSI-2, D-PHY config here
#if (DEBUG_EN == 0x0)
    // cam.init();
    vdma_driver.enableWrite();
#endif
    // MIPI_CSI_2_RX_mWriteReg(
    //     XPAR_MIPI_CSI_2_RX_0_BASEADDR,
    //     CR_OFFSET,
    //     CR_ENABLE_MASK
    // );
    // MIPI_D_PHY_RX_mWriteReg(
    //     XPAR_MIPI_D_PHY_RX_0_BASEADDR,
    //     CR_OFFSET,
    //     CR_ENABLE_MASK
    // );
#if (DEBUG_EN == 0x0)
    // cam.set_mode(mode);
    // cam.set_awb(OV9281_cfg::awb_t::AWB_ADVANCED);
#endif
    // Bring up output pipeline back-to-front
    // vid.reset();
    // vdma_driver.resetRead();
    // // vid.configure(res);
    // // Alloc mem for frames
    // vdma_driver.configureRead(
    //     timing[static_cast<int>(res)].h_active,
    //     timing[static_cast<int>(res)].v_active
    // );
#if (DEBUG_EN == 0x1)
    // Write RBG set to test pipeline
    // ...
    // VDMA Frame 0 Addr: 0x0A000000
    // VDMA Frame 1 Addr: 0x0A5EEC00
    // VDMA Frame 2 Addr: 0x0ABDD800
    // u32 framesAddr[3] = {0x0A000000, 0x0A5EEC00, 0x0ABDD800};
    // // difference : 5E EC00
    // u32 diffAddrs = framesAddr[1] - framesAddr[0];
    // for (u32 x = 0; x < diffAddrs; x++) {
    //     Xil_Out32(framesAddr[0] + x, 0x00FFFFFF);
    //     Xil_Out32(framesAddr[1] + x, 0x00FFFFFF);
    //     Xil_Out32(framesAddr[2] + x, 0x00FFFFFF);
    // }
#endif
    // vid.enable();
    vdma_driver.enableRead();
}