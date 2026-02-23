/*
 * OV9281 Camera Capture Driver for NXP i.MX8MP
 * Based on NXP's ov5640_mipi_v2.c
 * 
 * Copyright (C) 2024
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define OV9281_VOLTAGE_ANALOG               2800000
#define OV9281_VOLTAGE_DIGITAL_CORE         1200000
#define OV9281_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 120
#define DEFAULT_FPS 30

#define OV9281_XCLK_MIN 6000000
#define OV9281_XCLK_MAX 27000000

#define OV9281_CHIP_ID_HIGH_BYTE    0x300A
#define OV9281_CHIP_ID_LOW_BYTE     0x300B
#define OV9281_CHIP_ID              0x9281

#define OV9281_REG_MODE_SELECT       0x0100
#define OV9281_MODE_STANDBY         0x00
#define OV9281_MODE_STREAMING       0x01

/* Resolution modes */
enum ov9281_mode {
	ov9281_mode_MIN = 0,
	ov9281_mode_640_400 = 0,
	ov9281_mode_1280_720 = 1,
	ov9281_mode_1280_800 = 2,
	ov9281_mode_MAX = 2,
	ov9281_mode_INIT = 0xff,
};

enum ov9281_frame_rate {
	ov9281_30_fps,
	ov9281_60_fps,
	ov9281_120_fps
};

static int ov9281_framerates[] = {
	[ov9281_30_fps] = 30,
	[ov9281_60_fps] = 60,
	[ov9281_120_fps] = 120,
};

struct ov9281_datafmt {
	u32	code;
	enum v4l2_colorspace colorspace;
};

struct reg_value {
	u16 reg_addr;
	u8 val;
	u8 mask;
	u32 delay_ms;
};

struct ov9281_mode_info {
	enum ov9281_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

/* OV9281 initial common registers */
static struct reg_value ov9281_common_regs[] = {
	/* PLL Configuration (incomplete - missing other PLL1/PLL2 registers) */
	{0x0302, 0x32, 0, 0},  /* PLL1 multiplier[7:0] = 50 (incomplete PLL config) */
	{0x030e, 0x02, 0, 0},  /* PLL2 sys_div = 2 */
	
	/* System Control */
	{0x3001, 0x00, 0, 0},  /* pd_data_en/pd_pk control (0x00 not default 0x02) */
	{0x3004, 0x00, 0, 0},  /* IO pad out enable[17:16] = disabled */
	{0x3005, 0x00, 0, 0},  /* IO pad out enable[15:8] = disabled */
	{0x3006, 0x04, 0, 0},  /* IO pad out enable[7:0] = bit 2 enabled (STROBE?) */
	{0x3011, 0x0a, 0, 0},  /* MIPI PHY: bit[3]=1 mipi_pad, bit[1]=1 */
	{0x3013, 0x10, 0, 0},  /* MIPI PHY: bit[4]=1 pgm_lptx drive strength */
	{0x3022, 0x51, 0, 0},  /* MIPI bit mode: 0001 = 10-bit (but bits[7:4] should be 0101) */
	{0x3023, 0x00, 0, 0},  /* Power down control: all power saving disabled */
	
	/* Low Power Mode Configuration */
	{0x302c, 0x00, 0, 0},  /* Sleep period[31:24] = 0 */
	{0x302f, 0x00, 0, 0},  /* Sleep period[7:0] = 0 */
	/*{0x3030, 0x04, 0, 0},  /* Low power: bit[2]=1 external trigger snapshot mode */
	/*{0x303f, 0x01, 0, 0},  /* Frame on number = 1 frame */
	
	/* MIPI Configuration */
	{0x3039, 0x32, 0, 0},  /* MIPI: bits[7:5]=001 (2-lane), bit[4]=1 (MIPI en), bit[1]=1 */
	{0x303a, 0x00, 0, 0},  /* MIPI lane disable = none */
	
	/* Exposure and Gain Control */
	{0x3500, 0x00, 0, 0},  /* Exposure[19:16] = 0 */
	{0x3501, 0x5f, 0, 0},  /* Exposure[15:8] = 0x5f */
	{0x3502, 0x1e, 0, 0},  /* Exposure[7:0] = 0x1e (total = 0x5f1e = 382.1 lines) */
	{0x3503, 0x08, 0, 0},  /* AEC manual: bit[3]=1 gain_prec16_en */
	{0x3505, 0x8c, 0, 0},  /* Gain conversion: bits[3:2]=11 (required), bit[7]=1 */
	{0x3507, 0x03, 0, 0},  /* Gain shift: left shift 3 bits (8x digital gain) */
	{0x3508, 0x00, 0, 0},  /* Debug mode register */
	{0x3509, 0x10, 0, 0},  /* Gain = 0x10 (1x in real gain format) */
	
	/* Analog Control (mostly undocumented internal registers) */
	{0x3666, 0x00, 0, 0},  /* FSIN/VSYNC input select: 0x00 = from FSIN pin */
	
	
	/* PWM and BLC Control */
	{0x3920, 0xff, 0, 0},  /* PWM strobe pattern = 0xff */
	{0x4010, 0x40, 0, 0},  /* BLC: bit[6]=1 gain_chg_trig_en */
	{0x4043, 0x40, 0, 0},  /* BLC: bit[6]=1 bot_blk_ln_en */
	
	/* Format and Output Control */
	{0x4307, 0x30, 0, 0},  /* Embedded data: bits[7:4]=0011 embed_st */
	{0x4317, 0x00, 0, 0},  /* DVP disabled (MIPI mode) */
	
	
	/* VFIFO and DVP */
	{0x4601, 0x04, 0, 0},  /* VFIFO read start low = 4 */
	{0x470f, 0x00, 0, 0},  /* DVP bypass select = 0 */
	
	/* Power Save Mode */
	{0x4f07, 0x00, 0, 0},  /* PCHG_ST_OFFS = 0 (power charge start offset) */
	
	/* ISP Control */
	{0x5000, 0x9f, 0, 0},  /* ISP enable: BLC, WC, DPC buffer, AWB gain, BLC all ON */
	{0x5001, 0x00, 0, 0},  /* ISP control: all functions in normal mode */
	{0x5e00, 0x00, 0, 0},  /* Test pattern disabled */
	
	/* Unknown Registers (0x5dxx not in datasheet) */
	/*{0x5d00, 0x07, 0, 0},  /* Unknown - not in datasheet */
	/*{0x5d01, 0x00, 0, 0},  /* Unknown - not in datasheet */
	
	/* Incorrect/Unknown Usage */
	/*{0x0101, 0x01, 0, 0},  /* WARNING: Reserved register - should not be used for mirror */
	/*{0x1000, 0x03, 0, 0},  /* Unknown - not in datasheet */
	/*{0x5a08, 0x84, 0, 0},  /* Window control: bit[7]=1 out_size_sel, bit[2]=1 emb_flag_sel */
};

/* 640x400 @ 60fps - Properly centered crop */
static struct reg_value ov9281_mode_640_400_regs[] = {
	/* Windowing - Center crop from 1296x816 sensor array */
	{0x3800, 0x01, 0, 0},  /* X start high */
	{0x3801, 0x48, 0, 0},  /* X start low = 328 ((1296-640)/2) */
	{0x3802, 0x00, 0, 0},  /* Y start high */
	{0x3803, 0xd0, 0, 0},  /* Y start low = 208 ((816-400)/2) */
	{0x3804, 0x03, 0, 0},  /* X end high */
	{0x3805, 0xc7, 0, 0},  /* X end low = 967 (328+640-1) */
	{0x3806, 0x02, 0, 0},  /* Y end high */
	{0x3807, 0x5f, 0, 0},  /* Y end low = 607 (208+400-1) */
	
	/* Output Size - 640x400 */
	{0x3808, 0x02, 0, 0},  /* ISP output width high */
	{0x3809, 0x80, 0, 0},  /* ISP output width low = 640 */
	{0x380a, 0x01, 0, 0},  /* ISP output height high */
	{0x380b, 0x90, 0, 0},  /* ISP output height low = 400 */
	
	/* Timing for 60fps @ 80MHz PCLK */
	{0x380c, 0x05, 0, 0},  /* HTS high */
	{0x380d, 0xdc, 0, 0},  /* HTS low = 1500 */
	{0x380e, 0x03, 0, 0},  /* VTS high */
	{0x380f, 0x71, 0, 0},  /* VTS low = 881 */
	/* FPS = 80MHz / (1500 * 881) = 60.5fps */
	
	/* ISP Windowing Offset */
	{0x3810, 0x00, 0, 0},  /* X offset high */
	{0x3811, 0x04, 0, 0},  /* X offset low = 4 */
	{0x3812, 0x00, 0, 0},  /* Y offset high */
	{0x3813, 0x04, 0, 0},  /* Y offset low = 4 */
	
	/* Subsampling - No binning/skipping */
	{0x3814, 0x11, 0, 0},  /* X increment = 1 (no skip) */
	{0x3815, 0x11, 0, 0},  /* Y increment = 1 (no skip) */
	
	/* Format Control */
	{0x3820, 0x40, 0, 0},  /* Vertical format */
	{0x3821, 0x04, 0, 0},  /* Horizontal format - mirror enabled */
};

/* 1280x800 @ 30fps - Full resolution */
static struct reg_value ov9281_mode_1280_800_regs[] = {
	/* Windowing - Active pixel area (not full array) */
	{0x3800, 0x00, 0, 0},  /* X start high */
	{0x3801, 0x00, 0, 0},  /* X start low = 8 */
	{0x3802, 0x00, 0, 0},  /* Y start high */
	{0x3803, 0x00, 0, 0},  /* Y start low = 8 */
	{0x3804, 0x05, 0, 0},  /* X end high */
	{0x3805, 0x0F, 0, 0},  /* X end low = 1287 (8+1280-1) */
	{0x3806, 0x03, 0, 0},  /* Y end high */
	{0x3807, 0x2F, 0, 0},  /* Y end low = 807 (8+800-1) */
	
	/* Output Size - 1280x800 */
	{0x3808, 0x05, 0, 0},  /* ISP output width high */
	{0x3809, 0x00, 0, 0},  /* ISP output width low = 1280 */
	{0x380a, 0x03, 0, 0},  /* ISP output height high */
	{0x380b, 0x20, 0, 0},  /* ISP output height low = 800 */
	
	/* Timing for 30fps @ 80MHz PCLK */
	{0x380c, 0x02, 0, 0},  /* HTS high */
	{0x380d, 0xD8, 0, 0},  /* HTS low = 1580 */
	{0x380e, 0x03, 0, 0},  /* VTS high */
	{0x380f, 0x8E, 0, 0},  /* VTS low = 1675 */
	/* FPS = 80MHz / (1580 * 1675) = 30.2fps */
	
	/* ISP Windowing Offset */
	{0x3810, 0x00, 0, 0},  /* X offset high */
	{0x3811, 0x08, 0, 0},  /* X offset low = 8 */
	{0x3812, 0x00, 0, 0},  /* Y offset high */
	{0x3813, 0x08, 0, 0},  /* Y offset low = 8 */
	
	/* Subsampling - No binning/skipping */
	{0x3814, 0x11, 0, 0},  /* X increment = 1 (no skip) */
	{0x3815, 0x11, 0, 0},  /* Y increment = 1 (no skip) */
	
	/* Format Control */
	{0x3820, 0x40, 0, 0},  /* Vertical format */
	{0x3821, 0x00, 0, 0},  /* Horizontal format - mirror enabled */
};

static struct ov9281_mode_info ov9281_mode_info_data[ov9281_mode_MAX + 1] = {
	{
		ov9281_mode_640_400,
		640, 400,
		ov9281_mode_640_400_regs,
		ARRAY_SIZE(ov9281_mode_640_400_regs),
	},
	{
		ov9281_mode_1280_720,
		1280, 720,
		ov9281_mode_1280_800_regs,
		ARRAY_SIZE(ov9281_mode_1280_800_regs),
	},
	{
		ov9281_mode_1280_800,
		1280, 800,
		ov9281_mode_1280_800_regs,
		ARRAY_SIZE(ov9281_mode_1280_800_regs),
	},
};

static struct ov9281_datafmt ov9281_colour_fmts[] = {
	/* RAW10 output */
	{MEDIA_BUS_FMT_Y10_1X10, V4L2_COLORSPACE_RAW},
};

struct ov9281 {
	struct v4l2_subdev subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	const struct ov9281_datafmt *fmt;
	struct v4l2_captureparm streamcap;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	
	/* NXP specific */
	struct clk *sensor_clk;
	int csi;
	u32 mclk;
	u32 mclk_source;
	
	/* GPIOs */
	int pwn_gpio;
	int rst_gpio;
	
	/* Status */
	bool on;
	bool streaming;
	
	/* Current mode */
	int mode;
	struct ov9281_mode_info *pmodinfo;
	
	struct mutex lock;
	int power_count;
};

static inline struct ov9281 *to_ov9281(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov9281, subdev);
}

/* Read a register */
static int ov9281_read_reg(struct ov9281 *sensor, u16 reg, u8 *val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg[2];
	u8 buf[2];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = val;
	msg[1].len = 1;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=0x%04x\n", __func__, reg);
		return ret;
	}

	return 0;
}

/* Write a register */
static int ov9281_write_reg(struct ov9281 *sensor, u16 reg, u8 val)
{
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msg;
	u8 buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "%s: error: reg=0x%04x, val=0x%02x\n",
			__func__, reg, val);
		return ret;
	}

	return 0;
}

/* Write a list of registers */
static int ov9281_write_array(struct ov9281 *sensor, struct reg_value *regs, int array_size)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < array_size; i++) {
		ret = ov9281_write_reg(sensor, regs[i].reg_addr, regs[i].val);
		if (ret < 0)
			return ret;
		
		if (regs[i].delay_ms)
			msleep(regs[i].delay_ms);
	}

	return 0;
}

static void ov9281_reset(struct ov9281 *sensor)
{
	if (gpio_is_valid(sensor->rst_gpio)) {
		pr_info("OV9281: Reset using GPIO %d (expected: gpio1.IO[19] = GPIO_%d)\n", 
			sensor->rst_gpio, sensor->rst_gpio);
		gpio_set_value_cansleep(sensor->rst_gpio, 0);
		msleep(20);
		gpio_set_value_cansleep(sensor->rst_gpio, 1);
		msleep(20);
		pr_info("OV9281: Reset sequence completed on GPIO %d\n", sensor->rst_gpio);
	} else {
		pr_info("OV9281: No valid reset GPIO configured (rst_gpio = %d)\n", sensor->rst_gpio);
	}
}

static void ov9281_power_down(struct ov9281 *sensor, int enable)
{
	if (gpio_is_valid(sensor->pwn_gpio)) {
		pr_info("OV9281: Power %s using GPIO %d (expected: gpio2.IO[11] = GPIO_%d)\n", 
			enable ? "DOWN" : "UP", sensor->pwn_gpio, sensor->pwn_gpio);
		gpio_set_value_cansleep(sensor->pwn_gpio, enable);
	} else {
		pr_info("OV9281: No valid power GPIO configured (pwn_gpio = %d), power %s skipped\n", 
			sensor->pwn_gpio, enable ? "DOWN" : "UP");
	}
	
	msleep(2);
}

static int ov9281_init_mode(struct ov9281 *sensor)
{
	struct ov9281_mode_info *pmodinfo;
	struct i2c_client *client = sensor->i2c_client;
	int ret;

	pmodinfo = sensor->pmodinfo;
	
	dev_info(&client->dev, "OV9281_INIT_MODE: Initializing mode %dx%d\n", 
		 pmodinfo->width, pmodinfo->height);

	/* Write common registers */
	dev_info(&client->dev, "OV9281_INIT_MODE: Writing %zu common registers\n", 
		 ARRAY_SIZE(ov9281_common_regs));
	ret = ov9281_write_array(sensor, ov9281_common_regs,
				 ARRAY_SIZE(ov9281_common_regs));
	if (ret < 0) {
		dev_err(&client->dev, "OV9281_INIT_MODE: Failed to write common registers: %d\n", ret);
		return ret;
	}

	/* Write mode specific registers */
	dev_info(&client->dev, "OV9281_INIT_MODE: Writing %u mode-specific registers\n", 
		 pmodinfo->init_data_size);
	ret = ov9281_write_array(sensor, pmodinfo->init_data_ptr,
				 pmodinfo->init_data_size);
	if (ret < 0) {
		dev_err(&client->dev, "OV9281_INIT_MODE: Failed to write mode-specific registers: %d\n", ret);
		return ret;
	}

	dev_info(&client->dev, "OV9281_INIT_MODE: Mode initialization completed successfully\n");
	return 0;
}

static int ov9281_stream_on(struct ov9281 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;
	
	dev_info(&client->dev, "OV9281_STREAM_ON: Starting streaming mode\n");
	ret = ov9281_write_reg(sensor, OV9281_REG_MODE_SELECT, OV9281_MODE_STREAMING);
	if (ret < 0) {
		dev_err(&client->dev, "OV9281_STREAM_ON: Failed to write streaming mode register: %d\n", ret);
	} else {
		dev_info(&client->dev, "OV9281_STREAM_ON: Successfully enabled streaming mode\n");
	}
	return ret;
}

static int ov9281_stream_off(struct ov9281 *sensor)
{
	struct i2c_client *client = sensor->i2c_client;
	int ret;
	
	dev_info(&client->dev, "OV9281_STREAM_OFF: Stopping streaming mode\n");
	ret = ov9281_write_reg(sensor, OV9281_REG_MODE_SELECT, OV9281_MODE_STANDBY);
	if (ret < 0) {
		dev_err(&client->dev, "OV9281_STREAM_OFF: Failed to write standby mode register: %d\n", ret);
	} else {
		dev_info(&client->dev, "OV9281_STREAM_OFF: Successfully enabled standby mode\n");
	}
	return ret;
}

/* V4L2 subdev video operations */
static int ov9281_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov9281 *sensor = to_ov9281(sd);
	struct i2c_client *client = sensor->i2c_client;
	int ret = 0;

	dev_info(&client->dev, "OV9281_S_STREAM: Called with enable=%d (current streaming=%d)\n", 
		 enable, sensor->streaming);

	mutex_lock(&sensor->lock);

	if (sensor->streaming == enable) {
		dev_info(&client->dev, "OV9281_S_STREAM: Already in requested state, returning\n");
		mutex_unlock(&sensor->lock);
		return 0;
	}

	if (enable) {
		dev_info(&client->dev, "OV9281_S_STREAM: Enabling stream - initializing mode\n");
		ret = ov9281_init_mode(sensor);
		if (ret < 0) {
			dev_err(&client->dev, "OV9281_S_STREAM: Failed to initialize mode: %d\n", ret);
			goto out;
		}
		
		dev_info(&client->dev, "OV9281_S_STREAM: Mode initialized, starting stream\n");
		ret = ov9281_stream_on(sensor);
		if (ret < 0) {
			dev_err(&client->dev, "OV9281_S_STREAM: Failed to start stream: %d\n", ret);
			goto out;
		}
		
		dev_info(&client->dev, "OV9281_S_STREAM: Stream started successfully\n");
	} else {
		dev_info(&client->dev, "OV9281_S_STREAM: Disabling stream\n");
		ret = ov9281_stream_off(sensor);
		if (ret < 0) {
			dev_err(&client->dev, "OV9281_S_STREAM: Failed to stop stream: %d\n", ret);
		} else {
			dev_info(&client->dev, "OV9281_S_STREAM: Stream stopped successfully\n");
		}
	}

	sensor->streaming = enable;
	dev_info(&client->dev, "OV9281_S_STREAM: Updated streaming state to %d\n", sensor->streaming);

out:
	mutex_unlock(&sensor->lock);
	dev_info(&client->dev, "OV9281_S_STREAM: Returning %d\n", ret);
	return ret;
}

static int ov9281_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov9281 *sensor = to_ov9281(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (sensor->power_count == !on) {
		if (on) {
			pr_info("OV9281: Sensor power on");
			clk_prepare_enable(sensor->sensor_clk);
			ov9281_power_down(sensor, 0);
			ov9281_reset(sensor);
		} else {
			pr_info("OV9281: Sensor power off");
			ov9281_power_down(sensor, 1);
			clk_disable_unprepare(sensor->sensor_clk);
		}
	}

	sensor->power_count += on ? 1 : -1;
	WARN_ON(sensor->power_count < 0);

	mutex_unlock(&sensor->lock);
	return ret;
}


static int ov9281_enum_mbus_code(struct v4l2_subdev *sd,
                                 struct v4l2_subdev_state *sd_state,
                                 struct v4l2_subdev_mbus_code_enum *code)
{
    if (code->pad || code->index > 0)
        return -EINVAL;
    
    code->code = MEDIA_BUS_FMT_Y10_1X10;
    return 0;
}

static int ov9281_enum_frame_size(struct v4l2_subdev *sd,
                                  struct v4l2_subdev_state *sd_state,
                                  struct v4l2_subdev_frame_size_enum *fse)
{
    if (fse->index >= ARRAY_SIZE(ov9281_mode_info_data))
        return -EINVAL;
    
    if (fse->code != MEDIA_BUS_FMT_Y10_1X10)
        return -EINVAL;
    
    fse->min_width = ov9281_mode_info_data[fse->index].width;
    fse->max_width = fse->min_width;
    fse->min_height = ov9281_mode_info_data[fse->index].height;
    fse->max_height = fse->min_height;
    
    return 0;
}

static int ov9281_get_fmt(struct v4l2_subdev *sd,
                         struct v4l2_subdev_state *sd_state,
                         struct v4l2_subdev_format *format)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct ov9281 *sensor = to_ov9281(sd);
    struct v4l2_mbus_framefmt *mf = &format->format;
    
    if (format->pad)
        return -EINVAL;
    
    // OV9281 only supports one format
    mf->code = MEDIA_BUS_FMT_Y10_1X10;
    mf->colorspace = V4L2_COLORSPACE_RAW;
    mf->field = V4L2_FIELD_NONE;
    mf->width = sensor->pix.width;
    mf->height = sensor->pix.height;
    
    dev_info(&client->dev, "%s: returning %dx%d, code=0x%04x\n",
            __func__, mf->width, mf->height, mf->code);
    
    return 0;
}

static int ov9281_get_capturemode(int width, int height)
{
    int i;
    
    for (i = 0; i <= ov9281_mode_MAX; i++) {
        if (ov9281_mode_info_data[i].width == width &&
            ov9281_mode_info_data[i].height == height)
            return i;
    }
    return -1;
}

static int ov9281_set_fmt(struct v4l2_subdev *sd,
                         struct v4l2_subdev_state *sd_state,
                         struct v4l2_subdev_format *format)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct ov9281 *sensor = to_ov9281(sd);
    struct v4l2_mbus_framefmt *mf = &format->format;
    int capturemode;
    
    dev_info(&client->dev, "%s: requested %dx%d, code=0x%04x, which=%d\n",
            __func__, mf->width, mf->height, mf->code, format->which);
    
    if (format->pad)
        return -EINVAL;
    
    // OV9281 only supports Y10 format
    if (mf->code != MEDIA_BUS_FMT_Y10_1X10) {
        mf->code = MEDIA_BUS_FMT_Y10_1X10;
        dev_dbg(&client->dev, "%s: code corrected to 0x%04x\n", 
                __func__, mf->code);
    }
    
    mf->colorspace = V4L2_COLORSPACE_RAW;
    mf->field = V4L2_FIELD_NONE;
    
    // Find matching resolution mode
    capturemode = ov9281_get_capturemode(mf->width, mf->height);
    if (capturemode < 0) {
        // Default to 1280x800 if no match
        capturemode = ov9281_mode_1280_800;
        mf->width = 1280;
        mf->height = 800;
        dev_warn(&client->dev, "%s: no match, defaulting to %dx%d\n",
                 __func__, mf->width, mf->height);
    }
    
    // Just return for TRY format
    if (format->which == V4L2_SUBDEV_FORMAT_TRY)
        return 0;
    
    // Actually apply the format
    sensor->mode = capturemode;
    sensor->pmodinfo = &ov9281_mode_info_data[capturemode];
    sensor->pix.width = mf->width;
    sensor->pix.height = mf->height;
    sensor->pix.pixelformat = V4L2_PIX_FMT_Y10;
    
    dev_info(&client->dev, "%s: set to %dx%d, mode=%d\n",
             __func__, mf->width, mf->height, capturemode);
    
    return 0;
}

static struct v4l2_subdev_video_ops ov9281_subdev_video_ops = {
    .s_stream = ov9281_s_stream,
    // g_parm and s_parm are optional for basic operation
};

static const struct v4l2_subdev_pad_ops ov9281_subdev_pad_ops = {
    .enum_mbus_code = ov9281_enum_mbus_code,
    .enum_frame_size = ov9281_enum_frame_size, 
    .get_fmt = ov9281_get_fmt,
    .set_fmt = ov9281_set_fmt,
    // enum_frame_interval is optional
};

static struct v4l2_subdev_core_ops ov9281_subdev_core_ops = {
    .s_power = ov9281_s_power,
    // Register access is optional/debug only
};

static struct v4l2_subdev_ops ov9281_subdev_ops = {
    .core = &ov9281_subdev_core_ops,
    .video = &ov9281_subdev_video_ops,
    .pad = &ov9281_subdev_pad_ops,
};

/* V4L2 Control handlers */
static int ov9281_set_exposure(struct ov9281 *sensor, int val)
{
    struct i2c_client *client = sensor->i2c_client;
    int ret;
    
    dev_info(&client->dev, "OV9281_SET_EXPOSURE: Setting exposure to %d\n", val);
    
    /* OV9281 exposure is stored in registers 0x3500-0x3502 (20-bit) */
    ret = ov9281_write_reg(sensor, 0x3500, (val >> 12) & 0xff);
    if (ret < 0) return ret;
    
    ret = ov9281_write_reg(sensor, 0x3501, (val >> 4) & 0xff);
    if (ret < 0) return ret;
    
    ret = ov9281_write_reg(sensor, 0x3502, (val & 0x0f) << 4);
    if (ret < 0) return ret;
    
    dev_info(&client->dev, "OV9281_SET_EXPOSURE: Exposure set successfully\n");
    return 0;
}

static int ov9281_set_gain(struct ov9281 *sensor, int val)
{
    struct i2c_client *client = sensor->i2c_client;
    int ret;
    
    dev_info(&client->dev, "OV9281_SET_GAIN: Setting gain to %d\n", val);
    
    /* OV9281 gain is stored in registers 0x3508-0x3509 (10-bit) */
    ret = ov9281_write_reg(sensor, 0x3508, (val >> 8) & 0x03);
    if (ret < 0) return ret;
    
    ret = ov9281_write_reg(sensor, 0x3509, val & 0xff);
    if (ret < 0) return ret;
    
    dev_info(&client->dev, "OV9281_SET_GAIN: Gain set successfully\n");
    return 0;
}

static int ov9281_set_test_pattern(struct ov9281 *sensor, int val)
{
    struct i2c_client *client = sensor->i2c_client;
    int ret = 0;
    
    dev_info(&client->dev, "OV9281_SET_TEST_PATTERN: Setting test pattern to %d\n", val);
    
    /* OV9281 test pattern registers from datasheet */
    switch (val) {
    case 0: /* Normal mode */
        /* Disable all test patterns */
        ret = ov9281_write_reg(sensor, 0x5E00, 0x00); /* Disable general test pattern bar */
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4320, 0x80); /* Disable solid test pattern (keep bit[7]=1, bit[1]=0) */
        break;
        
    case 1: /* Color bars */
        /* Enable general test pattern bar */
        ret = ov9281_write_reg(sensor, 0x4320, 0x80); /* Disable solid pattern first */
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x5E00, 0x80); /* Enable general test pattern bar (bit[7]=1) */
        break;
        
    case 2: /* Solid color pattern */
        /* Disable bar pattern first */
        ret = ov9281_write_reg(sensor, 0x5E00, 0x00); /* Disable general test pattern bar */
        if (ret < 0) break;
        /* Enable solid test pattern */
        ret = ov9281_write_reg(sensor, 0x4320, 0x82); /* Enable solid test pattern (bit[7]=1, bit[1]=1) */
        if (ret < 0) break;
        /* Set solid pattern colors - using datasheet registers */
        ret = ov9281_write_reg(sensor, 0x4322, 0x03); /* solid_testpattern_P1[9:8] */
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4323, 0xFF); /* solid_testpattern_P1[7:0] */
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4324, 0x01); /* solid_testpattern_P2[9:8] */
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4325, 0x00); /* solid_testpattern_P2[7:0] */
        break;
        
    case 3: /* Different solid color */
        ret = ov9281_write_reg(sensor, 0x5E00, 0x00);
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4320, 0x82);
        if (ret < 0) break;
        /* Different solid color values */
        ret = ov9281_write_reg(sensor, 0x4322, 0x02); /* P1 = 0x200 */
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4323, 0x00);
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4324, 0x00); /* P2 = 0x080 */
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4325, 0x80);
        break;
        
    case 4: /* Another solid color */
        ret = ov9281_write_reg(sensor, 0x5E00, 0x00);
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4320, 0x82);
        if (ret < 0) break;
        /* Another solid color */
        ret = ov9281_write_reg(sensor, 0x4322, 0x01); /* P1 = 0x180 */
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4323, 0x80);
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4324, 0x03); /* P2 = 0x300 */
        if (ret < 0) break;
        ret = ov9281_write_reg(sensor, 0x4325, 0x00);
        break;
        
    default:
        /* Default to normal mode */
        ret = ov9281_write_reg(sensor, 0x5E00, 0x00);
        if (ret == 0)
            ret = ov9281_write_reg(sensor, 0x4320, 0x80);
        break;
    }
    
    if (ret < 0) {
        dev_err(&client->dev, "OV9281_SET_TEST_PATTERN: Failed to set test pattern: %d\n", ret);
    } else {
        dev_info(&client->dev, "OV9281_SET_TEST_PATTERN: Test pattern set successfully\n");
    }
    
    return ret;
}

static int ov9281_s_ctrl(struct v4l2_ctrl *ctrl)
{
    struct ov9281 *sensor = container_of(ctrl->handler, struct ov9281, ctrl_handler);
    int ret = 0;
    
    mutex_lock(&sensor->lock);
    
    switch (ctrl->id) {
    case V4L2_CID_EXPOSURE:
        ret = ov9281_set_exposure(sensor, ctrl->val);
        break;
    case V4L2_CID_GAIN:
        ret = ov9281_set_gain(sensor, ctrl->val);
        break;
    case V4L2_CID_TEST_PATTERN:
        ret = ov9281_set_test_pattern(sensor, ctrl->val);
        break;
    default:
        ret = -EINVAL;
        break;
    }
    
    mutex_unlock(&sensor->lock);
    return ret;
}

static const struct v4l2_ctrl_ops ov9281_ctrl_ops = {
    .s_ctrl = ov9281_s_ctrl,
};

/* Entity link_setup callback */
static int ov9281_link_setup(struct media_entity *entity,
                             const struct media_pad *local,
                             const struct media_pad *remote, u32 flags)
{
    struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    
    dev_info(&client->dev, "OV9281 LINK_SETUP: Called with flags=0x%x\n", flags);
    dev_info(&client->dev, "OV9281 LINK_SETUP: Linking pad %d to %s pad %d\n", 
             local->index, remote->entity->name, remote->index);
    
    /* For a simple sensor, we don't need to do anything special for link setup.
     * Just return success to allow the media framework to establish the link. */
    dev_info(&client->dev, "OV9281 LINK_SETUP: Link setup successful\n");
    return 0;
}

static const struct media_entity_operations ov9281_entity_ops = {
    .link_setup = ov9281_link_setup,
};

static int ov9281_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct ov9281 *sensor;
	struct v4l2_subdev *sd;
	int ret = 0;
	u8 chip_id_high, chip_id_low;
	u16 chip_id;

	dev_info(dev, "OV9281 probe starting: I2C addr=0x%02x\n", client->addr);

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor) {
		dev_err(dev, "OV9281 probe: Failed to allocate sensor memory\n");
		return -ENOMEM;
	}
	
	dev_info(dev, "OV9281 probe: Memory allocated successfully\n");

	sensor->i2c_client = client;
	
	/* Get CSI index */
	ret = of_property_read_u32(np, "csi_id", &sensor->csi);
	if (ret) {
		dev_err(dev, "OV9281 probe: csi_id missing or invalid\n");
		return ret;
	}
	dev_info(dev, "OV9281 probe: CSI ID = %d\n", sensor->csi);

	/* Get mclk */
	ret = of_property_read_u32(np, "mclk", &sensor->mclk);
	if (ret) {
		dev_err(dev, "OV9281 probe: mclk missing or invalid\n");
		return ret;
	}
	dev_info(dev, "OV9281 probe: MCLK = %d Hz\n", sensor->mclk);

	/* Get mclk source */
	ret = of_property_read_u32(np, "mclk_source", 
				   (u32 *)&sensor->mclk_source);
	if (ret) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return ret;
	}

	/* Get system clock */
	dev_info(dev, "OV9281 probe: Getting system clock\n");
	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		dev_info(dev, "OV9281 probe: 'csi_mclk' not found, trying default clock\n");
		sensor->sensor_clk = devm_clk_get(dev, NULL);
		if (IS_ERR(sensor->sensor_clk)) {
			dev_err(dev, "OV9281 probe: Failed to get mclk\n");
			return PTR_ERR(sensor->sensor_clk);
		}
	}
	
	dev_info(dev, "OV9281 probe: Clock obtained, setting rate to %d Hz\n", sensor->mclk);
	clk_set_rate(sensor->sensor_clk, sensor->mclk);

	/* Request power down pin */
	dev_info(dev, "OV9281 probe: Getting power down GPIO\n");
	sensor->pwn_gpio = of_get_named_gpio(np, "pwn-gpios", 0);
	if (gpio_is_valid(sensor->pwn_gpio)) {
		dev_info(dev, "OV9281 probe: Power down GPIO = %d\n", sensor->pwn_gpio);
		ret = devm_gpio_request_one(dev, sensor->pwn_gpio,
					    GPIOF_OUT_INIT_HIGH,
					    "ov9281_pwdn");
		if (ret < 0) {
			dev_warn(dev, "OV9281 probe: Failed to set power pin\n");
			sensor->pwn_gpio = -1;
		}
	} else {
		dev_info(dev, "OV9281 probe: No power down GPIO specified\n");
	}

	/* Request reset pin */
	dev_info(dev, "OV9281 probe: Getting reset GPIO\n");
	sensor->rst_gpio = of_get_named_gpio(np, "rst-gpios", 0);
	if (!gpio_is_valid(sensor->rst_gpio))
		sensor->rst_gpio = of_get_named_gpio(np, "reset-gpios", 0);
	
	if (gpio_is_valid(sensor->rst_gpio)) {
		dev_info(dev, "OV9281 probe: Reset GPIO = %d\n", sensor->rst_gpio);
		ret = devm_gpio_request_one(dev, sensor->rst_gpio,
					    GPIOF_OUT_INIT_HIGH,
					    "ov9281_reset");
		if (ret < 0) {
			dev_warn(dev, "OV9281 probe: Failed to set reset pin\n");
			sensor->rst_gpio = -1;
		}
	} else {
		dev_info(dev, "OV9281 probe: No reset GPIO specified\n");
	}

	/* Power on and identify the sensor */
	dev_info(dev, "OV9281 probe: Powering on sensor\n");
	clk_prepare_enable(sensor->sensor_clk);
	dev_info(dev, "OV9281 probe: Clock enabled\n");
	ov9281_power_down(sensor, 0);
	dev_info(dev, "OV9281 probe: Power down released\n");
	ov9281_reset(sensor);
	dev_info(dev, "OV9281 probe: Reset sequence completed\n");

	/* Read chip ID */
	dev_info(dev, "OV9281 probe: Reading chip ID\n");
	ret = ov9281_read_reg(sensor, OV9281_CHIP_ID_HIGH_BYTE, &chip_id_high);
	if (ret < 0) {
		dev_err(dev, "OV9281 probe: Failed to read chip ID high byte (reg 0x%04x)\n", OV9281_CHIP_ID_HIGH_BYTE);
		goto error;
	}
	dev_info(dev, "OV9281 probe: Chip ID high byte: 0x%02x\n", chip_id_high);

	ret = ov9281_read_reg(sensor, OV9281_CHIP_ID_LOW_BYTE, &chip_id_low);
	if (ret < 0) {
		dev_err(dev, "OV9281 probe: Failed to read chip ID low byte (reg 0x%04x)\n", OV9281_CHIP_ID_LOW_BYTE);
		goto error;
	}
	dev_info(dev, "OV9281 probe: Chip ID low byte: 0x%02x\n", chip_id_low);

	chip_id = (chip_id_high << 8) | chip_id_low;
	dev_info(dev, "OV9281 probe: Combined chip ID: 0x%04x (expected 0x%04x)\n", chip_id, OV9281_CHIP_ID);
	if (chip_id != OV9281_CHIP_ID) {
		dev_err(dev, "OV9281 probe: Chip ID mismatch: expected 0x%04x, got 0x%04x\n",
			OV9281_CHIP_ID, chip_id);
		ret = -ENODEV;
		goto error;
	}

	dev_info(dev, "OV9281 probe: *** CHIP ID VERIFIED *** Sensor detected at I2C address 0x%02x\n", client->addr);

	/* Initialize subdev */
	dev_info(dev, "OV9281 probe: Initializing V4L2 subdev\n");
	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov9281_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	dev_info(dev, "OV9281 probe: V4L2 subdev initialized\n");
	
	/* Initialize source pad */
	dev_info(dev, "OV9281 probe: Initializing media entity pads\n");
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, 1, &sensor->pad);
	if (ret < 0) {
		dev_err(dev, "OV9281 probe: Failed to initialize media entity pads\n");
		goto error;
	}
	
	/* Assign entity operations for debugging */
	sd->entity.ops = &ov9281_entity_ops;
	
	dev_info(dev, "OV9281 probe: Media entity pads initialized\n");

	/* Set default mode */
	sensor->pmodinfo = &ov9281_mode_info_data[ov9281_mode_1280_800];
	sensor->mode = ov9281_mode_1280_800;
	sensor->fmt = &ov9281_colour_fmts[0];
	sensor->pix.width = sensor->pmodinfo->width;
	sensor->pix.height = sensor->pmodinfo->height;
	sensor->pix.pixelformat = V4L2_PIX_FMT_Y10;
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;

	mutex_init(&sensor->lock);

	/* Initialize V4L2 controls */
	dev_info(dev, "OV9281 probe: Initializing V4L2 controls\n");
	v4l2_ctrl_handler_init(&sensor->ctrl_handler, 3);
	
	/* Add exposure control (20-bit, default based on current register values) */
	v4l2_ctrl_new_std(&sensor->ctrl_handler, &ov9281_ctrl_ops,
			  V4L2_CID_EXPOSURE, 1, 0x7ffff, 1, 0x5f1e);
	
	/* Add gain control (10-bit, default based on current register values) */
	v4l2_ctrl_new_std(&sensor->ctrl_handler, &ov9281_ctrl_ops,
			  V4L2_CID_GAIN, 1, 1023, 1, 16);
	
	/* Add test pattern control */
	static const char * const test_pattern_menu[] = {
		"Normal",
		"Color bars",
		"Solid color (bright)",
		"Solid color (medium)",
		"Solid color (high)"
	};
	v4l2_ctrl_new_std_menu_items(&sensor->ctrl_handler, &ov9281_ctrl_ops,
				     V4L2_CID_TEST_PATTERN, ARRAY_SIZE(test_pattern_menu) - 1,
				     0, 0, test_pattern_menu);
	
	if (sensor->ctrl_handler.error) {
		dev_err(dev, "OV9281 probe: Control handler error: %d\n", sensor->ctrl_handler.error);
		ret = sensor->ctrl_handler.error;
		goto error_controls;
	}
	
	sd->ctrl_handler = &sensor->ctrl_handler;
	dev_info(dev, "OV9281 probe: V4L2 controls initialized successfully\n");

	/* Register subdev */
	dev_info(dev, "OV9281 probe: Registering V4L2 async subdev\n");
	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		dev_err(dev, "OV9281 probe: *** FAILED TO REGISTER ASYNC SUBDEV *** error=%d\n", ret);
		goto error_media;
	}
	dev_info(dev, "OV9281 probe: *** V4L2 ASYNC SUBDEV REGISTERED SUCCESSFULLY ***\n");

	/* Power off */
	dev_info(dev, "OV9281 probe: Powering down sensor\n");
	ov9281_power_down(sensor, 1);
	clk_disable_unprepare(sensor->sensor_clk);

	dev_info(dev, "OV9281 probe: *** PROBE COMPLETED SUCCESSFULLY *** Driver ready at CSI%d\n", sensor->csi);
	return 0;

error_media:
	media_entity_cleanup(&sd->entity);
error_controls:
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
error:
	ov9281_power_down(sensor, 1);
	clk_disable_unprepare(sensor->sensor_clk);
	return ret;
}

static void ov9281_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov9281 *sensor = to_ov9281(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&sensor->ctrl_handler);
	mutex_destroy(&sensor->lock);
}

static const struct i2c_device_id ov9281_id[] = {
	{ "ov9281", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ov9281_id);

static const struct of_device_id ov9281_dt_ids[] = {
	{ .compatible = "ovti,ov9281" },
	{ .compatible = "ovti,ov9282" },
	{ }
};
MODULE_DEVICE_TABLE(of, ov9281_dt_ids);

static struct i2c_driver ov9281_driver = {
	.driver = {
		.name = "ov9281_mipi",
		.of_match_table = ov9281_dt_ids,
	},
	.probe = ov9281_probe,
	.remove = ov9281_remove,
	.id_table = ov9281_id,
};

module_i2c_driver(ov9281_driver);

MODULE_DESCRIPTION("OV9281 MIPI Camera Driver for NXP i.MX");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");