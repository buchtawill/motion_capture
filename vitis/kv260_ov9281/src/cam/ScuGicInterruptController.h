/*
 * IInterruptController.h
 *
 *  Created on: May 27, 2016
 *      Author: Elod
 */

#ifndef IINTERRUPTCONTROLLER_H_
#define IINTERRUPTCONTROLLER_H_

#include "xscugic.h"

namespace digilent {

class ScuGicInterruptController
{
public:
	typedef Xil_InterruptHandler Handler;
	typedef XStatus Errc;

	ScuGicInterruptController(uint32_t dev_id) :
		drv_inst_(),
		dev_id_(dev_id)
	{}

	Errc init()
	{
		XScuGic_Config* config = XScuGic_LookupConfig(dev_id_);
		if (config == NULL)
			return XST_DEVICE_NOT_FOUND;

		XStatus status = XScuGic_CfgInitialize(&drv_inst_, config, config->CpuBaseAddress);
		if (status != XST_SUCCESS)
			return status;

		status = XScuGic_SelfTest(&drv_inst_);
		return status;
	}

	Errc enableInterrupts()
	{
		Xil_ExceptionRegisterHandler(
			XIL_EXCEPTION_ID_INT,
			(Xil_ExceptionHandler)XScuGic_InterruptHandler,
			&drv_inst_
		);
		Xil_ExceptionEnable();
		return XST_SUCCESS;
	}

	Errc disableInterrupts()
	{
		Xil_ExceptionDisable();
		return XST_SUCCESS;
	}

	Errc registerHandler(uint32_t irpt_id, Handler handler, void* ref)
	{
		XStatus status = XScuGic_Connect(&drv_inst_, irpt_id, handler, ref);
		if (status != XST_SUCCESS)
			return status;

		XScuGic_Enable(&drv_inst_, irpt_id);
		return XST_SUCCESS;
	}

	Errc disableInterrupt(uint32_t irpt_id)
	{
		XScuGic_Disable(&drv_inst_, irpt_id);
		return XST_SUCCESS;
	}

	Errc enableInterrupt(uint32_t irpt_id)
	{
		XScuGic_Enable(&drv_inst_, irpt_id);
		return XST_SUCCESS;
	}

private:
	XScuGic  drv_inst_;
	uint32_t dev_id_;
};

} /* namespace digilent */

#endif /* IINTERRUPTCONTROLLER_H_ */
