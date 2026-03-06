#ifndef PL_IIC_H__
#define PL_IIC_H__

#include "xiic.h"
#include "xil_printf.h"

class PL_IIC {
public:
    PL_IIC(UINTPTR base) : _base(base), _instance{} {}

    // LookupConfig + CfgInitialize + SelfTest.
    // Returns XST_SUCCESS or XST_FAILURE.
    int init() {
        XIic_Config *cfg = XIic_LookupConfig(_base);
        if (cfg == NULL) {
            xil_printf("ERROR [PL_IIC::init()] LookupConfig failed for base 0x%08X\r\n", _base);
            return XST_FAILURE;
        }
        int status = XIic_CfgInitialize(&_instance, cfg, _base);
        if (status != XST_SUCCESS) {
            xil_printf("ERROR [PL_IIC::init()] CfgInitialize failed (%d)\r\n", status);
            return XST_FAILURE;
        }
        status = XIic_SelfTest(&_instance);
        if (status != XST_SUCCESS) {
            xil_printf("ERROR [PL_IIC::init()] SelfTest failed (%d)\r\n", status);
            return XST_FAILURE;
        }
        xil_printf("INFO [PL_IIC::init()] Ready at base 0x%08X\r\n", _base);
        return XST_SUCCESS;
    }

    // Write `len` bytes to 7-bit slave `addr`.
    int write(u8 addr, u8 *data, unsigned len) {
        unsigned sent = XIic_Send(_base, addr, data, len, XIIC_STOP);
        return (sent == len) ? XST_SUCCESS : XST_FAILURE;
    }

    // Read `len` bytes from 7-bit slave `addr`.
    int read(u8 addr, u8 *buf, unsigned len) {
        unsigned rcvd = XIic_Recv(_base, addr, buf, len, XIIC_STOP);
        return (rcvd == len) ? XST_SUCCESS : XST_FAILURE;
    }

    // Combined write+read for 16-bit addressed, 8-bit value registers.
    // Wire: START|addr+W|reg_hi|reg_lo|RSTART|addr+R|data...|STOP
    int reg16_read(u8 addr, u16 reg, u8 *buf, unsigned len) {
        u8 reg_bytes[2] = { (u8)(reg >> 8), (u8)(reg & 0xFF) };
        unsigned sent = XIic_Send(_base, addr, reg_bytes, 2, XIIC_REPEATED_START);
        if (sent != 2) return XST_FAILURE;
        unsigned rcvd = XIic_Recv(_base, addr, buf, len, XIIC_STOP);
        return (rcvd == len) ? XST_SUCCESS : XST_FAILURE;
    }

    // Write 1 byte `val` to 16-bit addressed register.
    // Wire: START|addr+W|reg_hi|reg_lo|val|STOP
    int reg16_write(u8 addr, u16 reg, u8 val) {
        u8 buf[3] = { (u8)(reg >> 8), (u8)(reg & 0xFF), val };
        unsigned sent = XIic_Send(_base, addr, buf, 3, XIIC_STOP);
        return (sent == 3) ? XST_SUCCESS : XST_FAILURE;
    }

private:
    UINTPTR _base;
    XIic    _instance;
};

#endif