/*
 * AXI_VDMA.h
 *
 *  Created on: Sep 2, 2016
 *      Author: Elod
 */

#ifndef AXI_VDMA_H_
#define AXI_VDMA_H_

#include "xil_printf.h"
#include <xil_types.h>

#include "xaxivdma.h"
#include "ScuGicInterruptController.h"


// template <typename IrptCtl>
class AXI_VDMA
{
	typedef struct {
		/* The state variable to keep track if the initialization is done */
		unsigned int init_done;
		
		/* Horizontal size of frame */
		unsigned int hsize;

		/* Vertical size of frame */
		unsigned int vsize;

		/* Buffer address from where read and write will be done by VDMA */
		unsigned int buffer_address;

		/* Flag to tell VDMA to interrupt on frame completion */
		unsigned int enable_frm_cnt_intr;

		/* The counter to tell VDMA on how many frames the interrupt should happen */
		unsigned int number_of_frame_count;

		/* The XAxiVdma_DmaSetup structure contains all the necessary information to
		 * start a frame write or read. */
		XAxiVdma_DmaSetup ReadCfg;
		XAxiVdma_DmaSetup WriteCfg;
	} vdma_context_t;
public:
	static void writeIntrHandler(void* ref, u32 mask) {
		static_cast<AXI_VDMA*>(ref)->writeHandler(mask);
	}

	AXI_VDMA(uint32_t dev_id,
             UINTPTR frame_buf_base_addr
            //  IrptCtl& irpt_ctl,
            //  uint32_t rd_irpt_id,
            //  uint32_t wr_irpt_id
             ) :
		// irpt_ctl_(irpt_ctl),
		context_{},
		dev_base_addr_(dev_id),
		frame_buf_base_addr_(frame_buf_base_addr)
	{

		//Set error interrupt error handlers, which for some reason need completion handler defined too
		// XAxiVdma_SetCallBack(
        //     &drv_inst_, 
        //     XAXIVDMA_HANDLER_GENERAL,
        //     reinterpret_cast<void*>(&MyCallback<decltype(rd_handler_)>), 
        //     &rd_handler_, 
        //     XAXIVDMA_READ
        // );
		// XAxiVdma_SetCallBack(
        //     &drv_inst_, 
        //     XAXIVDMA_HANDLER_ERROR,
        //     reinterpret_cast<void*>(&MyCallback<decltype(rd_err_handler_)>), 
        //     &rd_err_handler_, 
        //     XAXIVDMA_READ
        // );
		// XAxiVdma_SetCallBack(
        //     &drv_inst_, 
        //     XAXIVDMA_HANDLER_ERROR,
        //     reinterpret_cast<void*>(&MyCallback<decltype(wr_err_handler_)>), 
        //     &wr_err_handler_, 
        //     XAXIVDMA_WRITE
        // );

		//Register the IIC handler with the interrupt controller
		// irpt_ctl_.registerHandler(rd_irpt_id, &XAxiVdma_ReadIntrHandler, &drv_inst_);
		// irpt_ctl_.enableInterrupt(rd_irpt_id);
		// irpt_ctl_.registerHandler(wr_irpt_id, &XAxiVdma_WriteIntrHandler, &drv_inst_);
		// irpt_ctl_.enableInterrupt(wr_irpt_id);
		// irpt_ctl_.enableInterrupts();
	}

	u32 init(){
		XAxiVdma_Config* psConf;
		XStatus status;

		psConf = XAxiVdma_LookupConfig(dev_base_addr_);
        
		if (psConf == NULL){
			return XST_DEVICE_NOT_FOUND;
		}

		//Initialize driver instance and reset VDMA
		status = XAxiVdma_CfgInitialize(&drv_inst_, psConf, psConf->BaseAddress);
		if (status != XST_SUCCESS) return status;

		status = XAxiVdma_SetCallBack(
			&drv_inst_,
			XAXIVDMA_HANDLER_GENERAL,
			reinterpret_cast<void*>(writeIntrHandler),
			this,
			XAXIVDMA_WRITE
		);
		if (status != XST_SUCCESS) return status;
		return XST_SUCCESS;
	}

	XStatus resetRead(){
        //XAxiVdma_ChannelStop(&drv_inst_.ReadChannel);
        //while (XAxiVdma_ChannelIsRunning(&drv_inst_.ReadChannel));

		XAxiVdma_ChannelReset(&drv_inst_.ReadChannel);

		int polls = RESET_POLL;
		while (polls && XAxiVdma_ChannelResetNotDone(&drv_inst_.ReadChannel)) polls--;
		
		if (polls == 0) return XST_FAILURE;
		return XST_SUCCESS;
	}

	XStatus resetWrite(){
        //XAxiVdma_ChannelStop(&drv_inst_.WriteChannel);
        //while (XAxiVdma_ChannelIsRunning(&drv_inst_.WriteChannel));

		XAxiVdma_ChannelReset(&drv_inst_.WriteChannel);

		int polls = RESET_POLL;
		while (polls && XAxiVdma_ChannelResetNotDone(&drv_inst_.WriteChannel)) polls--;
	
		if (polls == 0) return XST_FAILURE;
		return XST_SUCCESS;
	}

/*
	void configureRead(uint16_t h_res, uint16_t v_res) {
		XStatus status;

		initMemAxiVdma(h_res, v_res);

        // Set again h/v attr
        context_.ReadCfg.HoriSizeInput = h_res * drv_inst_.ReadChannel.StreamWidth;
        context_.ReadCfg.VertSizeInput = v_res;
		context_.ReadCfg.Stride = context_.ReadCfg.HoriSizeInput;
        // No delays
		context_.ReadCfg.FrameDelay = 0;
		context_.ReadCfg.EnableCircularBuf = 1;
		context_.ReadCfg.EnableSync = 1;
		context_.ReadCfg.PointNum = 0;
		context_.ReadCfg.EnableFrameCounter = 0;
        // Set it on 0 until we sync
		context_.ReadCfg.FixedFrameStoreAddr = 0;

		status = XAxiVdma_DmaConfig(
            &drv_inst_,
            XAXIVDMA_READ,
            &context_.ReadCfg
        );
		
        if (XST_SUCCESS != status)
		{
			throw std::runtime_error(__FILE__ ":" LINE_STRING);
		}
		
        status = XAxiVdma_DmaSetBufferAddr(
            &drv_inst_,
            XAXIVDMA_READ,
            context_.ReadCfg.FrameStoreStartAddr
        );

		if (XST_SUCCESS != status)
		{
			throw std::runtime_error(__FILE__ ":" LINE_STRING);
		}

		//Clear errors in SR
		XAxiVdma_ClearChannelErrors(
            &drv_inst_.ReadChannel,
            XAXIVDMA_SR_ERR_ALL_MASK
        );
		//Enable read channel error and frame count interrupts
		XAxiVdma_IntrEnable(
            &drv_inst_,
            XAXIVDMA_IXR_ERROR_MASK,
            XAXIVDMA_READ
        );
	}

	void enableRead(){
		XStatus status;
		//Start read channel
		status = XAxiVdma_DmaStart(
            &drv_inst_,
            XAXIVDMA_READ
        );

		if (XST_SUCCESS != status)
		{
			throw std::runtime_error(__FILE__ ":" LINE_STRING);
		}
	}
*/	

	XStatus configureWrite(uint16_t h_res, uint16_t v_res) {
		XStatus status;
        
        initMemAxiVdma(h_res, v_res);
		
        // Set again h/v attr
        context_.WriteCfg.HoriSizeInput = h_res * bytes_pp;  // bytes per line (pixels * bytes/pixel)
        context_.WriteCfg.VertSizeInput = v_res;
        context_.WriteCfg.Stride = context_.WriteCfg.HoriSizeInput;
		context_.WriteCfg.FrameDelay = 0;
		context_.WriteCfg.EnableCircularBuf = 1;
        // Gen-Lock
		context_.WriteCfg.EnableSync = 0;
		context_.WriteCfg.PointNum = 0;
		context_.WriteCfg.EnableFrameCounter = 0;
        // Ignored, since we circle through buffers
		context_.WriteCfg.FixedFrameStoreAddr = 0;

        status = XAxiVdma_ClearDmaChannelErrors(
            &drv_inst_,
            XAXIVDMA_WRITE,
            XAXIVDMA_SR_ERR_ALL_MASK
        );
		if(status != XST_SUCCESS){
			xil_printf("Failed to clear write channel errors\r\n");
			return status;
		}

		status = XAxiVdma_DmaConfig(&drv_inst_, XAXIVDMA_WRITE, &context_.WriteCfg);

		if (XST_SUCCESS != status){
			xil_printf("Failed to configure write channel\r\n");
			return status;
		}

		status = XAxiVdma_DmaSetBufferAddr(&drv_inst_, XAXIVDMA_WRITE, context_.WriteCfg.FrameStoreStartAddr);

		if (XST_SUCCESS != status){
			xil_printf("Failed to set write channel buffer address\r\n");
			return status;
		}

		//Clear errors in SR
		XAxiVdma_ClearChannelErrors(&drv_inst_.WriteChannel, XAXIVDMA_SR_ERR_ALL_MASK);

		//Unmask error interrupts
		// XAxiVdma_MaskS2MMErrIntr(&drv_inst_,  XAXIVDMA_S2MM_IRQ_ERR_ALL_MASK, XAXIVDMA_WRITE);

		//Enable frame count interrupt
		// XAxiVdma_IntrEnable(&drv_inst_, XAXIVDMA_IXR_FRMCNT_MASK, XAXIVDMA_WRITE);
		return XST_SUCCESS;
	}

	XStatus enableWrite() {
		return XAxiVdma_DmaStart(&drv_inst_, XAXIVDMA_WRITE);
	}

	void printWriteStatus() {
		u32 sr = XAxiVdma_GetStatus(&drv_inst_, XAXIVDMA_WRITE);
		xil_printf("INFO [AXI_VDMA::printWriteStatus] VDMA write SR: 0x%08x\r\n", sr);
		if (sr & XAXIVDMA_SR_HALTED_MASK)    xil_printf("  HALTED\r\n");
		if (sr & XAXIVDMA_SR_IDLE_MASK)      xil_printf("  IDLE\r\n");
		if (sr & XAXIVDMA_SR_ERR_INTERNAL_MASK) xil_printf("  ERR: internal\r\n");
		if (sr & XAXIVDMA_SR_ERR_SLAVE_MASK)    xil_printf("  ERR: slave\r\n");
		if (sr & XAXIVDMA_SR_ERR_DECODE_MASK)   xil_printf("  ERR: decode\r\n");
	}

	void print_s2mm_cr(){
		u32 cr = XAxiVdma_ReadReg(this->dev_base_addr_ + XAXIVDMA_RX_OFFSET, XAXIVDMA_CR_OFFSET);
    	xil_printf("INFO [AXI_VDMA::print_s2mm_cr] VDMA S2MM control register: 0x%08x\r\n", cr);
	}

	// void readHandler(uint32_t irq_types)
	// {
	// 	std::cout << "VDMA:read complete - " << irq_types << std::endl;
	// }

	void writeHandler(uint32_t irq_types){
		xil_printf("INFO [AXI_VDMA::writeHandler] Write complete - %x\r\n", irq_types);
	}

	// void readErrorHandler(uint32_t mask)
	// {
	// 	std::cout << "VDMA:read error - " << mask << std::endl;
	// }

	// void writeErrorHandler(uint32_t mask)
	// {
	// 	std::cout << "VDMA:write error - " << mask << std::endl;
	// }

	XStatus connectWriteInterrupt(digilent::ScuGicInterruptController& gic, uint32_t intr_id) {        
		return gic.registerHandler(intr_id,
			(Xil_InterruptHandler)XAxiVdma_WriteIntrHandler,                                           
			&drv_inst_                                                                                 
		);
  }  
	
    void initMemAxiVdma(uint16_t hRes, uint16_t vRes) {
		int iFrm = 0;
		h_res_pix = hRes;
		v_res_pix = vRes;
		UINTPTR tmp_addr = frame_buf_base_addr_;

		//context_.ReadCfg.HoriSizeInput = prevHVSize.hRes * drv_inst_.ReadChannel.StreamWidth;
		//context_.ReadCfg.VertSizeInput = prevHVSize.vRes;
		xil_printf("INFO [AXI_VDMA::initMemAxiVdma] Max Num frames: %d\r\n", drv_inst_.MaxNumFrames);
		for (iFrm = 0; iFrm < drv_inst_.MaxNumFrames; iFrm++){
			size_t dimFrame = (hRes * bytes_pp) * vRes;  // bytes per frame (pixels * bytes/pixel * lines)

			// context_.ReadCfg.FrameStoreStartAddr[iFrm] = frame_buf_base_addr_;
			context_.WriteCfg.FrameStoreStartAddr[iFrm] = tmp_addr;
			xil_printf("VDMA Frame %d Addr: 0x%08x\r\n", iFrm, tmp_addr);
			tmp_addr += dimFrame;
		}
    }
    
    ~AXI_VDMA() {}

private:
	XAxiVdma drv_inst_;
	// std::function<void(uint32_t)> rd_handler_;
	// std::function<void(uint32_t)> rd_err_handler_;
	// std::function<void(uint32_t)> wr_err_handler_;
	// IrptCtl& irpt_ctl_;
	vdma_context_t context_;
	UINTPTR frame_buf_base_addr_;
	uint32_t dev_base_addr_;

	int const RESET_POLL = 1000;
	
	uint16_t h_res_pix = 0;
	uint16_t v_res_pix = 0;
	uint8_t  bytes_pp = 1;
};

#endif /* AXI_VDMA_H_ */