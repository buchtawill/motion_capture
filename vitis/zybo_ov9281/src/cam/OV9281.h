/*
 * OV9281.h
 *
 *  Created on: May 26, 2016
 *      Author: Elod
 */

#ifndef OV9281_H_
#define OV9281_H_

#include <sstream>
#include <iostream>
#include <cstdio>
#include <climits>

#include "sleep.h"

#include "I2C_Client.h"
#include "GPIO_Client.h"
#include "../hdmi/VideoOutput.h"

#define SIZEOF_ARRAY(x) 			sizeof(x)/sizeof(x[0])
#define MAP_ENUM_TO_CFG(en, cfg) 	en, cfg, SIZEOF_ARRAY(cfg)
#define REG_NULL					0xFFFF

#define OV9281_REG_MODE_SELECT      0x0100
#define OV9281_MODE_STANDBY         0x00
#define OV9281_MODE_STREAMING       0x01

struct regval {
	u16 addr;
	u8 val;
};

struct ov9281_mode {
	u32 width;
	u32 height;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
};

namespace digilent 
{

typedef enum {OK=0, ERR_LOGICAL, ERR_GENERAL} Errc;

namespace OV9281_cfg 
{
	using mode_t = enum { 
        MODE_720P_1280_720_60fps = 0, MODE_1080P_1920_1080_15fps,
		MODE_1080P_1920_1080_30fps, MODE_1080P_1920_1080_30fps_336M_MIPI,
		MODE_1080P_1920_1080_30fps_336M_1LANE_MIPI, MODE_END 
    };
	using config_modes_t = struct { mode_t mode; const struct regval* cfg; size_t cfg_size; };
	using test_t = enum { TEST_DISABLED = 0, TEST_EIGHT_COLOR_BAR, TEST_END };
	using awb_t = enum { AWB_DISABLED = 0, AWB_SIMPLE, AWB_ADVANCED, AWB_END };
	using config_awb_t = struct { awb_t awb; const struct regval* cfg; size_t cfg_size; };
	using isp_format_t = enum { ISP_RAW = 0, ISP_RGB, ISP_END };
	uint16_t const OV9281_REG_PRE_ISP_TEST_SET1 = 0x503D;
	uint16_t const OV9281_FORMAT_MUX_CONTROL = 0x501f;
	
	static const struct regval cfg_advanced_awb_[] = {
		// Enable Advanced AWB
		{0x3406 ,0x00},
		{0x5192 ,0x04},
		{0x5191 ,0xf8},
		{0x518d ,0x26},
		{0x518f ,0x42},
		{0x518e ,0x2b},
		{0x5190 ,0x42},
		{0x518b ,0xd0},
		{0x518c ,0xbd},
		{0x5187 ,0x18},
		{0x5188 ,0x18},
		{0x5189 ,0x56},
		{0x518a ,0x5c},
		{0x5186 ,0x1c},
		{0x5181 ,0x50},
		{0x5184 ,0x20},
		{0x5182 ,0x11},
		{0x5183 ,0x00},
		{0x5001 ,0x03}
	};

	static const struct regval cfg_simple_awb_[] = {
		// Disable Advanced AWB
		{0x518d ,0x00},
		{0x518f ,0x20},
		{0x518e ,0x00},
		{0x5190 ,0x20},
		{0x518b ,0x00},
		{0x518c ,0x00},
		{0x5187 ,0x10},
		{0x5188 ,0x10},
		{0x5189 ,0x40},
		{0x518a ,0x40},
		{0x5186 ,0x10},
		{0x5181 ,0x58},
		{0x5184 ,0x25},
		{0x5182 ,0x11},

		// Enable simple AWB
		{0x3406 ,0x00},
		{0x5183 ,0x80},
		{0x5191 ,0xff},
		{0x5192 ,0x00},
		{0x5001 ,0x03}
	};

	static const struct regval cfg_disable_awb_[] = {
		{0x5001 ,0x02}
	};
	
	static const struct regval cfg_init_[] = {

	};

	static const struct regval rpi_cam_prog_seq[] = {
		{0x4800, 0x20},
		{0x0302, 0x32},
		{0x030e, 0x02},
		{0x3001, 0x00},
		{0x3004, 0x00},
		{0x3005, 0x00},
		{0x3006, 0x04},
		{0x3011, 0x0a},
		{0x3013, 0x18},
		{0x301c, 0xf0},
		{0x3022, 0x01},
		{0x3030, 0x10},
		{0x3039, 0x32},
		{0x303a, 0x00},
		{0x3503, 0x08},
		{0x3505, 0x8c},
		{0x3507, 0x03},
		{0x3508, 0x00},
		{0x3610, 0x80},
		{0x3611, 0xa0},
		{0x3620, 0x6e},
		{0x3632, 0x56},
		{0x3633, 0x78},
		{0x3666, 0x00},
		{0x366f, 0x5a},
		{0x3680, 0x84},
		{0x3712, 0x80},
		{0x372d, 0x22},
		{0x3731, 0x80},
		{0x3732, 0x30},
		{0x377d, 0x22},
		{0x3788, 0x02},
		{0x3789, 0xa4},
		{0x378a, 0x00},
		{0x378b, 0x4a},
		{0x3799, 0x20},
		{0x3881, 0x42},
		{0x38a8, 0x02},
		{0x38a9, 0x80},
		{0x38b1, 0x00},
		{0x38c4, 0x00},
		{0x38c5, 0xc0},
		{0x38c6, 0x04},
		{0x38c7, 0x80},
		{0x3920, 0xff},
		{0x4010, 0x40},
		{0x4043, 0x40},
		{0x4307, 0x30},
		{0x4317, 0x00},
		{0x4501, 0x00},
		{0x450a, 0x08},
		{0x4601, 0x04},
		{0x470f, 0x00},
		{0x4f07, 0x00},
		{0x5000, 0x9f},
		{0x5001, 0x00},
		{0x5e00, 0x00},
		{0x5d00, 0x07},
		{0x5d01, 0x00},
		{0x0101, 0x01},
		{0x1000, 0x03},
		{0x5a08, 0x84},
		{0x030d, 0x50},
		{0x3662, 0x05},
		{0x3778, 0x10},
		{0x3800, 0x00},
		{0x3801, 0x00},
		{0x3802, 0x00},
		{0x3803, 0x00},
		{0x3804, 0x05},
		{0x3805, 0x0f},
		{0x3806, 0x03},
		{0x3807, 0x2f},
		{0x3808, 0x02},
		{0x3809, 0x80},
		{0x380a, 0x01},
		{0x380b, 0x90},
		{0x3810, 0x00},
		{0x3811, 0x04},
		{0x3812, 0x00},
		{0x3813, 0x04},
		{0x3814, 0x31},
		{0x3815, 0x22},
		{0x3820, 0x60},
		{0x3821, 0x01},
		{0x4008, 0x02},
		{0x4009, 0x05},
		{0x400c, 0x00},
		{0x400d, 0x03},
		{0x4507, 0x03},
		{0x4509, 0x80},
		{0x3308, 0x01},
		{0x3500, 0x00},
		{0x3501, 0x06},
		{0x3502, 0x80},
		{0x3509, 0x10},
		{0x3308, 0x00},
		{0x380e, 0x0d},
		{0x380f, 0x9d},
		{0x3820, 0x60},
		{0x3821, 0x01},
		{0x380c, 0x02},
		{0x380d, 0xfd},
		{0x0100, 0x01},
		{0x3308, 0x01},
		{0x3500, 0x00},
		{0x3501, 0xd8},
		{0x3502, 0x40},
		{0x3509, 0x80},
		{0x3308, 0x00},
		{0x3308, 0x01},
		{0x3500, 0x00},
		{0x3501, 0xd8},
		{0x3502, 0x40},
		{0x3509, 0x33},
		{0x3308, 0x00},
		{0x3308, 0x01},
		{0x3500, 0x00},
		{0x3501, 0x89},
		{0x3502, 0xe0},
		{0x3509, 0x20},
		{0x3308, 0x00},
		{0x3308, 0x01},
		{0x3500, 0x00},
		{0x3501, 0x78},
		{0x3502, 0x30},
		{0x3509, 0x20},
		{0x3308, 0x00},
		{0x3308, 0x01},
		{0x3500, 0x00},
		{0x3501, 0x7a},
		{0x3502, 0xf0},
		{0x3509, 0x20},
		{0x3308, 0x00},
		{0x3308, 0x01},
		{0x3500, 0x00},
		{0x3501, 0x7c},
		{0x3502, 0x80},
		{0x3509, 0x20},
		{0x3308, 0x00},
		{REG_NULL, 0x00}
	};

	/*
	* Xclk 24Mhz
	* max_framerate 120fps for 10 bit, 144fps for 8 bit.
	* mipi_datarate per lane 800Mbps
	*/
	static const struct regval ov9281_common_regs[] = {
		// {0x0103, 0x01}, // Software reset. 1 = camera on
		{0x0302, 0x32},
		{0x030e, 0x02},
		{0x3001, 0x00},
		{0x3004, 0x00},
		{0x3005, 0x00},
		{0x3006, 0x04},
		{0x3011, 0x0a},
		{0x3013, 0x18},
		{0x3022, 0x01},
		{0x3023, 0x00},
		{0x302c, 0x00},
		{0x302f, 0x00},
		{0x3030, 0x04},
		{0x3039, 0x32},
		{0x303a, 0x00},
		{0x303f, 0x01},
		{0x3500, 0x00},
		{0x3501, 0x2a},
		{0x3502, 0x90},
		{0x3503, 0x08},
		{0x3505, 0x8c},
		{0x3507, 0x03},
		{0x3508, 0x00},
		{0x3509, 0x10},
		{0x3610, 0x80},
		{0x3611, 0xa0},
		{0x3620, 0x6f},
		{0x3632, 0x56},
		{0x3633, 0x78},
		{0x3666, 0x00},
		{0x366f, 0x5a},
		{0x3680, 0x84},
		{0x3712, 0x80},
		{0x372d, 0x22},
		{0x3731, 0x80},
		{0x3732, 0x30},
		{0x377d, 0x22},
		{0x3788, 0x02},
		{0x3789, 0xa4},
		{0x378a, 0x00},
		{0x378b, 0x4a},
		{0x3799, 0x20},
		{0x3881, 0x42},
		{0x38b1, 0x00},
		{0x3920, 0xff},
		{0x4010, 0x40},
		{0x4043, 0x40},
		{0x4307, 0x30},
		{0x4317, 0x00},
		{0x4501, 0x00},
		{0x450a, 0x08},
		{0x4601, 0x04},
		{0x470f, 0x00},
		{0x4f07, 0x00},
		{0x4800, 0x00},
		{0x5000, 0x9f},
		{0x5001, 0x00},
		{0x5e00, 0x00},
		{0x5d00, 0x07},
		{0x5d01, 0x00},
		{REG_NULL, 0x00},
	};

	static const struct regval ov9281_1280x800_regs[] = {
		{0x3778, 0x00},
		{0x3800, 0x00},
		{0x3801, 0x00},
		{0x3802, 0x00},
		{0x3803, 0x00},
		{0x3804, 0x05},
		{0x3805, 0x0f},
		{0x3806, 0x03},
		{0x3807, 0x2f},
		{0x3808, 0x05},
		{0x3809, 0x00},
		{0x380a, 0x03},
		{0x380b, 0x20},
		{0x380c, 0x02},
		{0x380d, 0xd8},
		{0x380e, 0x03},
		{0x380f, 0x8e},
		{0x3810, 0x00},
		{0x3811, 0x08},
		{0x3812, 0x00},
		{0x3813, 0x08},
		{0x3814, 0x11},
		{0x3815, 0x11},
		{0x3820, 0x40},
		{0x3821, 0x00},
		{0x4003, 0x40},
		{0x4008, 0x04},
		{0x4009, 0x0b},
		{0x400c, 0x00},
		{0x400d, 0x07},
		{0x4507, 0x00},
		{0x4509, 0x00},
		{REG_NULL, 0x00},
	};

	static const struct regval ov9281_1280x720_regs[] = {
		{0x3778, 0x00},
		{0x3800, 0x00},
		{0x3801, 0x00},
		{0x3802, 0x00},
		{0x3803, 0x28},
		{0x3804, 0x05},
		{0x3805, 0x0f},
		{0x3806, 0x03},
		{0x3807, 0x07},
		{0x3808, 0x05},
		{0x3809, 0x00},
		{0x380a, 0x02},
		{0x380b, 0xd0},
		{0x380c, 0x02},
		{0x380d, 0xd8},
		{0x380e, 0x03},
		{0x380f, 0x8e},
		{0x3810, 0x00},
		{0x3811, 0x08},
		{0x3812, 0x00},
		{0x3813, 0x08},
		{0x3814, 0x11},
		{0x3815, 0x11},
		{0x3820, 0x40},
		{0x3821, 0x00},
		{0x4003, 0x40},
		{0x4008, 0x04},
		{0x4009, 0x0b},
		{0x400c, 0x00},
		{0x400d, 0x07},
		{0x4507, 0x00},
		{0x4509, 0x00},
		{REG_NULL, 0x00},
	};

	static const struct regval ov9281_640x400_regs[] = {
		{0x3778, 0x10},
		{0x3800, 0x00},
		{0x3801, 0x00},
		{0x3802, 0x00},
		{0x3803, 0x00},
		{0x3804, 0x05},
		{0x3805, 0x0f},
		{0x3806, 0x03},
		{0x3807, 0x2f},
		{0x3808, 0x02},
		{0x3809, 0x80},
		{0x380a, 0x01},
		{0x380b, 0x90},
		{0x380c, 0x02},
		{0x380d, 0xd8},
		{0x380e, 0x02},
		{0x380f, 0x08},
		{0x3810, 0x00},
		{0x3811, 0x04},
		{0x3812, 0x00},
		{0x3813, 0x04},
		{0x3814, 0x31},
		{0x3815, 0x22},
		{0x3820, 0x60},
		{0x3821, 0x01},
		{0x4008, 0x02},
		{0x4009, 0x05},
		{0x400c, 0x00},
		{0x400d, 0x03},
		{0x4507, 0x03},
		{0x4509, 0x80},
		{REG_NULL, 0x00},
	};

	static const struct regval op_10bit[] = {
		{0x030d, 0x50},
		{0x3662, 0x05},
		{REG_NULL, 0x00},
	};

	static const struct regval op_8bit[] = {
		{0x030d, 0x60},
		{0x3662, 0x07},
		{REG_NULL, 0x00},
	};

	static const struct ov9281_mode supported_modes[] = {
		{
			.width = 1280,
			.height = 800,
			.hts_def = 0x05b0,	/* 0x2d8*2 */
			.vts_def = 0x038e,
			.exp_def = 0x0320,
			.reg_list = ov9281_1280x800_regs,
		},
		{
			.width = 1280,
			.height = 720,
			.hts_def = 0x05b0,
			.vts_def = 761,
			.exp_def = 0x0320,
			.reg_list = ov9281_1280x720_regs,
		},
		{
			.width = 640,
			.height = 400,
			.hts_def = 0x05b0,
			.vts_def = 421,
			.exp_def = 0x0320,
			.reg_list = ov9281_640x400_regs,
		},
	};
	
	// config_modes_t const modes[] =
	// {
    //     { MAP_ENUM_TO_CFG(MODE_720P_1280_720_60fps, cfg_720p_60fps_) },
    //     { MAP_ENUM_TO_CFG(MODE_1080P_1920_1080_15fps, cfg_1080p_15fps_) },
    //     { MAP_ENUM_TO_CFG(MODE_1080P_1920_1080_30fps, cfg_1080p_30fps_), },
    //     { MAP_ENUM_TO_CFG(MODE_1080P_1920_1080_30fps_336M_MIPI, cfg_1080p_30fps_336M_mipi_) },
    //     { MAP_ENUM_TO_CFG(MODE_1080P_1920_1080_30fps_336M_1LANE_MIPI, cfg_1080p_30fps_336M_1lane_mipi_) },
	// };
	// config_awb_t const awbs[] =
	// {
    //     { MAP_ENUM_TO_CFG(AWB_DISABLED, cfg_disable_awb_) },
    //     { MAP_ENUM_TO_CFG(AWB_SIMPLE, cfg_simple_awb_) },
    //     { MAP_ENUM_TO_CFG(AWB_ADVANCED, cfg_advanced_awb_) }
	// };
}

class OV9281
{
public:
	class HardwareError;

	OV9281(I2C_Client& iic, GPIO_Client& gpio) : iic_(iic), gpio_(gpio){
		// reset();
		init();
	}

	int init(){
		uint8_t id_h, id_l;
		readReg(reg_ID_h, id_h);
		readReg(reg_ID_l, id_l);

		if (id_h != dev_ID_h_ || id_l != dev_ID_l_){
			char msg[100];
			snprintf(msg, sizeof(msg), "Got %02x %02x. Expected %02x %02x\r\n", 
                     id_h, id_l, dev_ID_h_, dev_ID_l_
            );
			throw HardwareError(HardwareError::WRONG_ID, msg);
		}
		xil_printf("INFO [OV9281.h::init()] Read back correct camera device ID\r\n");

		// usleep(1000000);

		// readReg(OV9281_REG_MODE_SELECT, stream_val);
		// xil_printf("INFO [OV9281.h::init()] Mode select pre-write:  0x%x\r\n", stream_val);
		uint8_t stream_val = 0;
		int ret; 
		ret = write_reg_array(OV9281_cfg::ov9281_common_regs);
		ret = write_reg_array(OV9281_cfg::ov9281_1280x720_regs);
		ret = write_reg_array(OV9281_cfg::op_10bit);
		writeReg(OV9281_REG_MODE_SELECT, OV9281_MODE_STREAMING);
		xil_printf("INFO [OV9281.h::init()] Configs programmed to camera, reading back...\r\n");
		ret = validate_reg_array(OV9281_cfg::ov9281_common_regs);
		ret = validate_reg_array(OV9281_cfg::ov9281_1280x720_regs);
		ret = validate_reg_array(OV9281_cfg::op_10bit);
        return ret;
	}

	Errc reset(){
		//Power cycle
		gpio_.clearBit(gpio_.Bits::CAM_GPIO0);
		usleep(1000000);
		gpio_.setBit(gpio_.Bits::CAM_GPIO0);
		usleep(1000000);

		return OK;
	}

	Errc set_mode(OV9281_cfg::mode_t mode){
		// if (mode >= OV9281_cfg::mode_t::MODE_END)
		// 	return ERR_LOGICAL;
		//
		// //[7]=0 Software reset; [6]=1 Software power down; Default=0x02
		// writeReg(0x3008, 0x42);
		//
		// auto cfg_mode = &OV9281_cfg::modes[mode];
		// writeConfig(cfg_mode->cfg, cfg_mode->cfg_size);
		//
		// //[7]=0 Software reset; [6]=0 Software power down; Default=0x02
		// writeReg(0x3008, 0x02);
	
        return OK;
	}

	Errc set_awb(OV9281_cfg::awb_t awb){
		// if (awb >= OV9281_cfg::awb_t::AWB_END)
		// 	return ERR_LOGICAL;
		// //[7]=0 Software reset; [6]=1 Software power down; Default=0x02
		// writeReg(0x3008, 0x42);
	    //
		// auto cfg_mode = &OV9281_cfg::awbs[awb];
		// writeConfig(cfg_mode->cfg, cfg_mode->cfg_size);
	    //
		// //[7]=0 Software reset; [6]=0 Software power down; Default=0x02
		// writeReg(0x3008, 0x02);
    
		return OK;
	}

	Errc set_isp_format(OV9281_cfg::isp_format_t isp){
		if (isp >= OV9281_cfg::isp_format_t::ISP_END)
			return ERR_LOGICAL;
		//[7]=0 Software reset; [6]=1 Software power down; Default=0x02
		writeReg(0x3008, 0x42);

		switch (isp)
		{
			case OV9281_cfg::isp_format_t::ISP_RGB:
				writeReg(
                    OV9281_cfg::OV9281_FORMAT_MUX_CONTROL, 
                    0x01
                );
				break;
			case OV9281_cfg::isp_format_t::ISP_RAW:
				writeReg(
                    OV9281_cfg::OV9281_FORMAT_MUX_CONTROL, 
                    0x03
                );
				break;
			default:
				break;
		}

		//[7]=0 Software reset; [6]=0 Software power down; Default=0x02
		writeReg(0x3008, 0x02);

        return OK;
	}

	~OV9281() {}

	void set_test(OV9281_cfg::test_t test){
		switch (test)
		{
			case OV9281_cfg::test_t::TEST_DISABLED:
				writeReg(OV9281_cfg::OV9281_REG_PRE_ISP_TEST_SET1, 0x00);
				break;
			case OV9281_cfg::test_t::TEST_EIGHT_COLOR_BAR:
				writeReg(
                    OV9281_cfg::OV9281_REG_PRE_ISP_TEST_SET1, 
                    0x80
                );
				break;
			default:
				break;
		}
	}

	void readReg(uint16_t reg_addr, uint8_t& buf){
		for (auto retry_count = retry_count_; retry_count > 0; --retry_count){
			try
			{
				auto buf_addr = std::vector<uint8_t>{(uint8_t)(reg_addr>>8), (uint8_t)reg_addr};
				iic_.write(dev_address_, buf_addr.data(), buf_addr.size());
				iic_.read(dev_address_, &buf, 1);
				break; //If no exceptions, no more retries
			}
			catch (I2C_Client::TransmitError const& e)
			{
				if (retry_count > 0)
				{
					continue;
				}
				else
				{
					throw HardwareError(HardwareError::IIC_NACK, e.what());
				}
			}
		}
	}

    // TODO: Change to return int value
	int writeReg(uint16_t reg_addr, uint8_t const reg_data){
		for (auto retry_count = retry_count_; retry_count > 0; --retry_count){
			try{
				auto buf = std::vector<uint8_t>{(uint8_t)(reg_addr>>8), (uint8_t)reg_addr, reg_data};
				iic_.write(dev_address_, buf.data(), buf.size());
				break; //If no exceptions, no mo retries
			}
			catch (I2C_Client::TransmitError const& e){
				if (retry_count > 0) continue;
				else throw HardwareError(HardwareError::IIC_NACK, e.what());
			}
		}
        return 0;
	}

	class HardwareError : public std::runtime_error{
	public:
		using Errc = enum {WRONG_ID = 1, IIC_NACK};
		HardwareError(Errc errc, char const* msg) : std::runtime_error(msg), errc_(errc) {}
		Errc errc() const { return errc_; }
	private:
		Errc errc_;
	};

private:

	int write_reg_array(const struct regval *regs){
		u32 i;
		int ret = 0;

		for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
			ret = writeReg(regs[i].addr, regs[i].val);

		return ret;
	}

	int validate_reg_array(const struct regval *regs){
		u32 i;
		int ret = 0;

		for(i = 0; ret == 0 && regs[i].addr != REG_NULL; i++){
			u8 data_recvd;
			readReg(regs[i].addr, data_recvd);
			if(data_recvd != regs[i].val){
				xil_printf("ERROR [OV9281::validate_reg_array] Mismatch @ 0x%X: got 0x%X expected 0x%X\r\n", regs[i].addr, data_recvd, regs[i].val);
				ret = 1;
			}
		}
		xil_printf("INFO [OV9281::validate_reg_array] Registers properly written\r\n");
		return ret;
	}

private:
	I2C_Client& iic_;
	GPIO_Client& gpio_;
	uint8_t  const dev_address_ = (0xC0 >> 1); // Address is 0xC0 8b / 0x60 7b
	uint8_t  const dev_ID_h_ = 0x92;
	uint8_t  const dev_ID_l_ = 0x81;
	uint16_t const reg_ID_h = 0x300A;
	uint16_t const reg_ID_l = 0x300B;
	uint16_t       retry_count_ = 10;
};

} /* namespace digilent */

#endif /* OV9281_H_ */