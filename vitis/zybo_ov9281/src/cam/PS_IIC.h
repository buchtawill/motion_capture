#ifndef PS_IIC_H_
#define PS_IIC_H_

#include "xiicps.h"
#include "xil_printf.h"

class PS_IIC {
public:
    PS_IIC(UINTPTR base, u32 sclk_rate_Hz) : _base(base), _sclk_rate(sclk_rate_Hz), _instance{} {}

    // LookupConfig + CfgInitialize + SelfTest + SetSClk.
    // Returns XST_SUCCESS or XST_FAILURE.
    int init() {
        XIicPs_Config *cfg = XIicPs_LookupConfig(_base);
        if (cfg == NULL) {
            xil_printf("ERROR [PS_IIC::init()] LookupConfig failed for base 0x%08X\r\n", _base);
            return XST_FAILURE;
        }
        int status = XIicPs_CfgInitialize(&_instance, cfg, _base);
        if (status != XST_SUCCESS) {
            xil_printf("ERROR [PS_IIC::init()] CfgInitialize failed (%d)\r\n", status);
            return XST_FAILURE;
        }
        status = XIicPs_SelfTest(&_instance);
        if (status != XST_SUCCESS) {
            xil_printf("ERROR [PS_IIC::init()] SelfTest failed (%d)\r\n", status);
            return XST_FAILURE;
        }
        status = XIicPs_SetSClk(&_instance, _sclk_rate);
        if (status != XST_SUCCESS) {
            xil_printf("ERROR [PS_IIC::init()] SetSClk failed (%d)\r\n", status);
            return XST_FAILURE;
        }
        xil_printf("INFO [PS_IIC::init()] Ready at base 0x%08X @ %d Hz\r\n", _base, _sclk_rate);
        return XST_SUCCESS;
    }

    // Write `len` bytes to 7-bit slave `addr`.
    int write(u8 addr, u8 *data, unsigned len) {
        int status = XIicPs_MasterSendPolled(&_instance, data, len, addr);
        if (status != XST_SUCCESS) return XST_FAILURE;
        while (XIicPs_BusIsBusy(&_instance));
        return XST_SUCCESS;
    }

    // Read `len` bytes from 7-bit slave `addr`.
    int read(u8 addr, u8 *buf, unsigned len) {
        int status = XIicPs_MasterRecvPolled(&_instance, buf, len, addr);
        if (status != XST_SUCCESS) return XST_FAILURE;
        while (XIicPs_BusIsBusy(&_instance));
        return XST_SUCCESS;
    }

    // Combined write+read for 16-bit addressed, 8-bit value registers.
    // Wire: START|addr+W|reg_hi|reg_lo|RSTART|addr+R|data...|STOP
    int reg16_read(u8 addr, u16 reg, u8 *buf, unsigned len) {
        u8 reg_bytes[2] = { (u8)(reg >> 8), (u8)(reg & 0xFF) };
        XIicPs_SetOptions(&_instance, XIICPS_REP_START_OPTION);
        int status = XIicPs_MasterSendPolled(&_instance, reg_bytes, 2, addr);
        XIicPs_ClearOptions(&_instance, XIICPS_REP_START_OPTION);
        if (status != XST_SUCCESS) return XST_FAILURE;
        status = XIicPs_MasterRecvPolled(&_instance, buf, len, addr);
        if (status != XST_SUCCESS) return XST_FAILURE;
        while (XIicPs_BusIsBusy(&_instance));
        return XST_SUCCESS;
    }

    // Write 1 byte `val` to 16-bit addressed register.
    // Wire: START|addr+W|reg_hi|reg_lo|val|STOP
    int reg16_write(u8 addr, u16 reg, u8 val) {
        u8 buf[3] = { (u8)(reg >> 8), (u8)(reg & 0xFF), val };
        int status = XIicPs_MasterSendPolled(&_instance, buf, 3, addr);
        if (status != XST_SUCCESS) return XST_FAILURE;
        while (XIicPs_BusIsBusy(&_instance));
        return XST_SUCCESS;
    }

private:
    UINTPTR _base;
    u32     _sclk_rate;
    XIicPs  _instance;
};

#endif /* PS_IIC_H_ */
