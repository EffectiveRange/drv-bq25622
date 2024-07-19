// SPDX-License-Identifier: GPL-2.0
// BQ2562X driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/usb/phy.h>

#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#include "bq2562x_charger.h"

#define BQ2562X_NUM_WD_VAL 4

// #define BQ_WARN_DEBUG

#ifdef BQ_WARN_DEBUG
#define BQ2562X_DEBUG(dev, ...) dev_warn(dev, __VA_ARGS__)
#else
#define BQ2562X_DEBUG(dev, ...)
#endif

#define RET_NZ_W_VAL(val, fn, ...)                                          \
	({                                                                  \
		ret = fn(__VA_ARGS__);                                      \
		if (ret) {                                                  \
			dev_err(bq->dev,                                    \
				"%s@%d:" #fn " failed with return code %d", \
				__func__, __LINE__, ret);                   \
			return val;                                         \
		}                                                           \
		ret;                                                        \
	})

#define RET_NZ(fn, ...) RET_NZ_W_VAL(ret, fn, __VA_ARGS__)

#define RET_FAIL(fn, ...)                                                   \
	({                                                                  \
		ret = fn(__VA_ARGS__);                                      \
		if (ret < 0) {                                              \
			dev_err(bq->dev,                                    \
				"%s@%d:" #fn " failed with return code %d", \
				__func__, __LINE__, ret);                   \
			return ret;                                         \
		}                                                           \
		ret;                                                        \
	})

struct bq2562x_init_data {
	u32 ichg;
	u32 ilim;
	u32 vreg;
	u32 iterm;
	u32 iprechg;
	u32 vlim;
	u32 ichg_max;
	u32 vreg_max;
};

struct bq2562x_state {
	bool online;
	u8 chrg_status;
	u8 ce_status;
	u8 chrg_type;
	u8 health;
	u8 chrg_fault;
	u8 vsys_status;
	u8 vbus_status;
	u8 fault_0;
	u32 vbat_adc;
	u32 vbus_adc;
	s32 ibat_adc;
	s32 ibus_adc;
	// ICHG value is halved by Watchdog reset
	// we'll use this is as two way variable, from the set_property
	// site and also resetting it to this value on each WD reset event
	u32 ichg_curr;
};

enum bq2562x_id {
	BQ25620,
	BQ25622,
};

struct bq2562x_device {
	struct i2c_client *client;
	struct device *dev;
	struct power_supply *charger;
	struct power_supply *battery;
	struct mutex lock;

	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];
	int device_id;

	struct gpio_desc *ac_detect_gpio;
	struct gpio_desc *ce_gpio;

	struct bq2562x_init_data init_data;
	struct bq2562x_state state;
	// to signal initial status read
	// to not only react to flags, but
	// read all status regs on first read
	bool initial;
	int watchdog_timer;
	int watchdog_timer_reg;
};

/* clang-format off */
static struct reg_default bq25620_reg_defs[] = {
	{BQ2562X_CHRG_I_LIM_MSB, 0x03},
	{BQ2562X_CHRG_I_LIM_LSB, 0x40},
	{BQ2562X_CHRG_V_LIM_MSB, 0x0D},
	{BQ2562X_CHRG_V_LIM_LSB, 0x20},
	{BQ2562X_INPUT_I_LIM_MSB, 0x0A},
	{BQ2562X_INPUT_I_LIM_LSB, 0x00},
	{BQ2562X_INPUT_V_LIM_MSB, 0x0E},
	{BQ2562X_INPUT_V_LIM_LSB, 0x60},
	{BQ2562X_IOTG_MSB, 0x03},
	{BQ2562X_IOTG_LSB, 0x20},
	{BQ2562X_VOTG_MSB, 0x0F},
	{BQ2562X_VOTG_LSB, 0xC0},
	{BQ2562X_MIN_SYS_V_MSB, 0x0B},
	{BQ2562X_MIN_SYS_V_LSB, 0x00},
	{BQ2562X_PRECHRG_CTRL_MSB, 0x00},
	{BQ2562X_PRECHRG_CTRL_LSB, 0x50},
	{BQ2562X_TERM_CTRL_MSB, 0x00},
	{BQ2562X_TERM_CTRL_LSB, 0x30},
	{BQ2562X_CHRG_CTRL, 0x06},
	{BQ2562X_TIMER_CTRL, 0x5C},
	{BQ2562X_CHRG_CTRL_1, 0xA1},
	{BQ2562X_CHRG_CTRL_2, 0x4E},
	{BQ2562X_CHRG_CTRL_3, 0x04},
	{BQ2562X_CHRG_CTRL_4, 0xC4},
	{BQ2562X_NTC_CTRL_0, 0x3D},
	{BQ2562X_NTC_CTRL_1, 0x25},
	{BQ2562X_NTC_CTRL_2, 0x3F},
	{BQ2562X_CHRG_STAT_0, 0x00},
	{BQ2562X_CHRG_STAT_1, 0x00},
	{BQ2562X_FAULT_STAT_0, 0x00},
	{BQ2562X_CHRG_FLAG_0, 0x00},
	{BQ2562X_CHRG_FLAG_1, 0x00},
	{BQ2562X_FAULT_FLAG_0, 0x00},
	{BQ2562X_CHRG_MSK_0, 0x00},
	{BQ2562X_CHRG_MSK_1, 0x00},
	{BQ2562X_FAULT_MSK_0, 0x00},
	{BQ2562X_ADC_CTRL, 0x30},
	{BQ2562X_FN_DISABE_0, 0x00},
	{BQ2562X_ADC_IBUS_MSB, 0x00},
	{BQ2562X_ADC_IBUS_LSB, 0x00},
	{BQ2562X_ADC_IBAT_MSB, 0x00},
	{BQ2562X_ADC_IBAT_LSB, 0x00},
	{BQ2562X_ADC_VBUS_MSB, 0x00},
	{BQ2562X_ADC_VBUS_LSB, 0x00},
	{BQ2562X_ADC_VPMID_MSB, 0x00},
	{BQ2562X_ADC_VPMID_LSB, 0x00},
	{BQ2562X_ADC_VBAT_MSB, 0x00},
	{BQ2562X_ADC_VBAT_LSB, 0x00},
	{BQ2562X_ADC_VSYS_MSB, 0x00},
	{BQ2562X_ADC_VSYS_LSB, 0x00},
	{BQ2562X_ADC_TS_MSB, 0x00},
	{BQ2562X_ADC_TS_LSB, 0x00},
	{BQ2562X_ADC_TDIE_MSB, 0x00},
	{BQ2562X_ADC_TDIE_LSB, 0x00},
	{BQ2562X_PART_INFO, 0x00},
};

static struct reg_default bq25622_reg_defs[] = {
	{BQ2562X_CHRG_I_LIM_MSB, 0x03},
	{BQ2562X_CHRG_I_LIM_LSB, 0x40},
	{BQ2562X_CHRG_V_LIM_MSB, 0x0D},
	{BQ2562X_CHRG_V_LIM_LSB, 0x20},
	{BQ2562X_INPUT_I_LIM_MSB, 0x0A},
	{BQ2562X_INPUT_I_LIM_LSB, 0x00},
	{BQ2562X_INPUT_V_LIM_MSB, 0x0E},
	{BQ2562X_INPUT_V_LIM_LSB, 0x60},
	{BQ2562X_IOTG_MSB, 0x03},
	{BQ2562X_IOTG_LSB, 0x20},
	{BQ2562X_VOTG_MSB, 0x0F},
	{BQ2562X_VOTG_LSB, 0xC0},
	{BQ2562X_MIN_SYS_V_MSB, 0x0B},
	{BQ2562X_MIN_SYS_V_LSB, 0x00},
	{BQ2562X_PRECHRG_CTRL_MSB, 0x00},
	{BQ2562X_PRECHRG_CTRL_LSB, 0x50},
	{BQ2562X_TERM_CTRL_MSB, 0x00},
	{BQ2562X_TERM_CTRL_LSB, 0x30},
	{BQ2562X_CHRG_CTRL, 0x06},
	{BQ2562X_TIMER_CTRL, 0x5C},
	{BQ2562X_CHRG_CTRL_1, 0xA1},
	{BQ2562X_CHRG_CTRL_2, 0x4E},
	{BQ2562X_CHRG_CTRL_3, 0x04},
	{BQ2562X_CHRG_CTRL_4, 0xC4},
	{BQ2562X_NTC_CTRL_0, 0x3D},
	{BQ2562X_NTC_CTRL_1, 0x25},
	{BQ2562X_NTC_CTRL_2, 0x3F},
	{BQ2562X_CHRG_STAT_0, 0x00},
	{BQ2562X_CHRG_STAT_1, 0x00},
	{BQ2562X_FAULT_STAT_0, 0x00},
	{BQ2562X_CHRG_FLAG_0, 0x00},
	{BQ2562X_CHRG_FLAG_1, 0x00},
	{BQ2562X_FAULT_FLAG_0, 0x00},
	{BQ2562X_CHRG_MSK_0, 0x00},
	{BQ2562X_CHRG_MSK_1, 0x00},
	{BQ2562X_FAULT_MSK_0, 0x00},
	{BQ2562X_ADC_CTRL, 0x30},
	{BQ2562X_FN_DISABE_0, 0x00},
	{BQ2562X_ADC_IBUS_MSB, 0x00},
	{BQ2562X_ADC_IBUS_LSB, 0x00},
	{BQ2562X_ADC_IBAT_MSB, 0x00},
	{BQ2562X_ADC_IBAT_LSB, 0x00},
	{BQ2562X_ADC_VBUS_MSB, 0x00},
	{BQ2562X_ADC_VBUS_LSB, 0x00},
	{BQ2562X_ADC_VPMID_MSB, 0x00},
	{BQ2562X_ADC_VPMID_LSB, 0x00},
	{BQ2562X_ADC_VBAT_MSB, 0x00},
	{BQ2562X_ADC_VBAT_LSB, 0x00},
	{BQ2562X_ADC_VSYS_MSB, 0x00},
	{BQ2562X_ADC_VSYS_LSB, 0x00},
	{BQ2562X_ADC_TS_MSB, 0x00},
	{BQ2562X_ADC_TS_LSB, 0x00},
	{BQ2562X_ADC_TDIE_MSB, 0x00},
	{BQ2562X_ADC_TDIE_LSB, 0x00},
	{BQ2562X_PART_INFO, 0x00},
};

static int bq2562x_watchdog_time[BQ2562X_NUM_WD_VAL] = {0, 40000, 80000, 160000};

static enum power_supply_usb_type bq2562x_usb_type[] = {
	POWER_SUPPLY_USB_TYPE_ACA,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_DCP,
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
};
/* clang-format on */

static bool bq2562x_get_charge_enable(struct bq2562x_device *bq)
{
	int ret = 0;
	unsigned int ce_pin = 0;
	int charger_enable = 0;
	unsigned int chrg_ctrl_0 = 0;

	if (bq->ce_gpio) {
		ce_pin = RET_FAIL(gpiod_get_value_cansleep, bq->ce_gpio);
		BQ2562X_DEBUG(bq->dev, "read CE pin: %u", ce_pin);
	}

	RET_NZ(regmap_read, bq->regmap, BQ2562X_CHRG_CTRL_1, &chrg_ctrl_0);

	charger_enable = chrg_ctrl_0 & BQ2562X_CHRG_EN;

	BQ2562X_DEBUG(bq->dev, "read charge enable: %d.\n", charger_enable);

	if (bq->ce_gpio) {
		return charger_enable && ce_pin;
	}
	return charger_enable;
}

static int bq2562x_set_charge_enable(struct bq2562x_device *bq, int val)
{
	int ret;
	BQ2562X_DEBUG(bq->dev, "setting charge enable to %d", val);
	if (bq->ce_gpio) {
		BQ2562X_DEBUG(bq->dev, "setting charge enable gpio pin to %d",
			      val);
		gpiod_set_value_cansleep(bq->ce_gpio, val);
	}

	RET_NZ(regmap_update_bits, bq->regmap, BQ2562X_CHRG_CTRL_1,
	       BQ2562X_CHRG_EN, val ? BQ2562X_CHRG_EN : 0);
	return 0;
}

static int bq2562x_get_word_signed(struct bq2562x_device *bq,
				   unsigned int baseaddr, const char *const key,
				   unsigned step, int scale, int *res)
{
	int ret = 0;
	s16 read_res = 0;
	u8 rd_buff[2];
	(void)key;

	RET_FAIL(regmap_bulk_read, bq->regmap, baseaddr, rd_buff, 2);

	read_res = (s16)(((rd_buff[1] << 8) | rd_buff[0]) >> step);

	*res = read_res * scale;

	BQ2562X_DEBUG(
		bq->dev,
		"read %s addr:%02x msb:0x%02x lsb:0x%02x read_result:%d result:%d",
		key, baseaddr, rd_buff[1], rd_buff[0], read_res, *res);

	return 0;
}

static int bq2562x_get_word_unsigned(struct bq2562x_device *bq,
				     unsigned int baseaddr,
				     const char *const key, unsigned step,
				     int scale)
{
	int ret = 0;
	u16 read_res = 0;
	int res = 0;
	u8 rd_buff[2];
	(void)key;

	RET_FAIL(regmap_bulk_read, bq->regmap, baseaddr, rd_buff, 2);

	read_res = ((rd_buff[1] << 8) | rd_buff[0]) >> step;

	res = read_res * scale;

	BQ2562X_DEBUG(
		bq->dev,
		"read %s addr:%02x msb:0x%02x lsb:0x%02x read_result:%u result:%u",
		key, baseaddr, rd_buff[1], rd_buff[0], read_res, res);

	return res;
}

static int bq2562x_set_word_unsigned(struct bq2562x_device *bq,
				     unsigned baseaddr, unsigned val,
				     unsigned step, unsigned scale,
				     unsigned minval, unsigned maxval)
{
	int ret;
	int reg_val;
	u8 wr_buff[2];

	val = clamp(val, minval, maxval);

	reg_val = (val / scale) << step;
	wr_buff[0] = reg_val & 0xff;
	wr_buff[1] = (reg_val >> 8) & 0xff;
	RET_NZ(regmap_bulk_write, bq->regmap, baseaddr, wr_buff, 2);

	return 0;
}

static int bq2562x_get_vbat_adc(struct bq2562x_device *bq)
{
	return bq2562x_get_word_unsigned(bq, BQ2562X_ADC_VBAT_LSB, "vbat adc",
					 BQ2562X_ADC_VBAT_MOVE_STEP,
					 BQ2562X_ADC_VBAT_STEP_uV);
}

static int bq2562x_get_vbus_adc(struct bq2562x_device *bq)
{
	return bq2562x_get_word_unsigned(bq, BQ2562X_ADC_VBUS_LSB, "vbus adc",
					 BQ2562X_ADC_VBUS_MOVE_STEP,
					 BQ2562X_ADC_VBUS_STEP_uV);
}

static int bq2562x_get_ibat_adc(struct bq2562x_device *bq)
{
	int res = 0;
	bq2562x_get_word_signed(bq, BQ2562X_ADC_IBAT_LSB, "ibat adc", 0,
				BQ2562X_ADC_CURR_STEP_uA, &res);
	return 0;
}

static int bq2562x_get_ibus_adc(struct bq2562x_device *bq)
{
	int ret = 0;
	RET_NZ_W_VAL(0, bq2562x_get_word_signed, bq, BQ2562X_ADC_IBUS_LSB,
		     "ibus adc", 0, BQ2562X_ADC_CURR_STEP_uA, &ret);
	return ret;
}

static int bq2562x_get_term_curr(struct bq2562x_device *bq)
{
	return bq2562x_get_word_unsigned(bq, BQ2562X_TERM_CTRL_LSB,
					 "iterm curr", BQ2562X_ITERM_MOVE_STEP,
					 BQ2562X_TERMCHRG_CURRENT_STEP_uA);
}

static int bq2562x_get_prechrg_curr(struct bq2562x_device *bq)
{
	return bq2562x_get_word_unsigned(bq, BQ2562X_PRECHRG_CTRL_LSB,
					 "precharge curr",
					 BQ2562X_PRECHRG_MOVE_STEP,
					 BQ2562X_PRECHRG_CURRENT_STEP_uA);
}

static int bq2562x_get_ichrg_curr(struct bq2562x_device *bq)
{
	int ret;
	RET_NZ_W_VAL(0, bq2562x_get_word_signed, bq, BQ2562X_CHRG_I_LIM_LSB,
		     "ichrg_curr", BQ2562X_ICHG_MOVE_STEP,
		     BQ2562X_ICHRG_CURRENT_STEP_uA, &ret);
	return ret;
}

static int bq2562x_set_term_curr(struct bq2562x_device *bq, int term_current)
{
	int ret;
	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_TERM_CTRL_LSB,
		 term_current, BQ2562X_ITERM_MOVE_STEP,
		 BQ2562X_TERMCHRG_CURRENT_STEP_uA, BQ2562X_TERMCHRG_I_MIN_uA,
		 BQ2562X_TERMCHRG_I_MAX_uA);
	return 0;
}

static int bq2562x_set_prechrg_curr(struct bq2562x_device *bq, int pre_current)
{
	int ret;
	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_PRECHRG_CTRL_LSB,
		 pre_current, BQ2562X_PRECHRG_MOVE_STEP,
		 BQ2562X_PRECHRG_CURRENT_STEP_uA, BQ2562X_PRECHRG_I_MIN_uA,
		 BQ2562X_PRECHRG_I_MAX_uA);
	return 0;
}

static int bq2562x_set_ichrg_curr(struct bq2562x_device *bq, int chrg_curr)
{
	int ret;
	int chrg_curr_max = bq->init_data.ichg_max;

	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_CHRG_I_LIM_LSB,
		 chrg_curr, BQ2562X_ICHG_MOVE_STEP,
		 BQ2562X_ICHRG_CURRENT_STEP_uA, BQ2562X_ICHRG_I_MIN_uA,
		 chrg_curr_max);

	return 0;
}

static int bq2562x_set_chrg_volt(struct bq2562x_device *bq, int chrg_volt)
{
	int ret;
	int chrg_volt_max = bq->init_data.vreg_max;

	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_CHRG_V_LIM_LSB,
		 chrg_volt, BQ2562X_CHRG_V_LIM_MOVE_STEP,
		 BQ2562X_VREG_V_STEP_uV, BQ2562X_VREG_V_MIN_uV, chrg_volt_max);

	return 0;
}

static int bq2562x_get_chrg_volt(struct bq2562x_device *bq)
{
	int ret;

	RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_CHRG_V_LIM_LSB,
		 "chrg volt", BQ2562X_CHRG_V_LIM_MOVE_STEP,
		 BQ2562X_VREG_V_STEP_uV);

	return 0;
}

static int bq2562x_set_input_volt_lim(struct bq2562x_device *bq, int vindpm)
{
	int ret;
	int vlim_lsb, vlim_msb;
	int vlim;

	vindpm =
		clamp(vindpm, BQ2562X_VINDPM_V_MIN_uV, BQ2562X_VINDPM_V_MAX_uV);

	vlim = (vindpm / BQ2562X_VINDPM_STEP_uV)
	       << BQ2562X_INPUT_V_LIM_MOVE_STEP;

	vlim_msb = (vlim >> 8) & 0xff;

	RET_NZ(regmap_write, bq->regmap, BQ2562X_INPUT_V_LIM_MSB, vlim_msb);

	vlim_lsb = vlim & 0xff;

	return regmap_write(bq->regmap, BQ2562X_INPUT_V_LIM_LSB, vlim_lsb);
}

static int bq2562x_get_input_volt_lim(struct bq2562x_device *bq)
{
	int ret;
	int vlim;
	int vlim_lsb, vlim_msb;

	RET_NZ_W_VAL(-1, regmap_read, bq->regmap, BQ2562X_INPUT_V_LIM_LSB,
		     &vlim_lsb);

	RET_NZ_W_VAL(-1, regmap_read, bq->regmap, BQ2562X_INPUT_V_LIM_MSB,
		     &vlim_msb);

	vlim = ((vlim_msb << 8) | vlim_lsb) >> BQ2562X_INPUT_V_LIM_MOVE_STEP;

	return vlim * BQ2562X_VINDPM_STEP_uV;
}

static int bq2562x_set_input_curr_lim(struct bq2562x_device *bq, int iindpm)
{
	int ret;
	int ilim, ilim_lsb, ilim_msb;

	iindpm =
		clamp(iindpm, BQ2562X_IINDPM_I_MIN_uA, BQ2562X_IINDPM_I_MAX_uA);

	ilim = (iindpm / BQ2562X_IINDPM_STEP_uA)
	       << BQ2562X_INPUT_I_LIM__MOVE_STEP;

	ilim_lsb = ilim & 0xff;
	RET_NZ(regmap_write, bq->regmap, BQ2562X_INPUT_I_LIM_LSB, ilim_lsb);

	ilim_msb = (ilim >> 8) & 0xff;
	RET_NZ(regmap_write, bq->regmap, BQ2562X_INPUT_I_LIM_MSB, ilim_msb);

	return ret;
}

static int bq2562x_get_input_curr_lim(struct bq2562x_device *bq)
{
	int ret;
	int ilim_msb, ilim_lsb;
	u16 ilim;

	RET_NZ_W_VAL(-1, regmap_read, bq->regmap, BQ2562X_INPUT_I_LIM_LSB,
		     &ilim_lsb);

	RET_NZ_W_VAL(-1, regmap_read, bq->regmap, BQ2562X_INPUT_I_LIM_MSB,
		     &ilim_msb);

	ilim = ((ilim_msb << 8) | ilim_lsb) >> BQ2562X_INPUT_I_LIM__MOVE_STEP;

	return ilim * BQ2562X_IINDPM_STEP_uA;
}

static int bq2562x_get_online_status(int chrg_stat_1)
{
	int online_status;

	online_status = chrg_stat_1 & BQ2562X_VBUS_STAT_MSK;

	if ((online_status == BQ2562X_USB_SDP) ||
	    (online_status == BQ2562X_OTG_MODE))
		online_status = 0;
	else
		online_status = 1;

	return online_status;
}

// start a one-shot ADC measuerement
static int bq2562x_start_adc(struct bq2562x_device *bq)
{
	return regmap_update_bits(bq->regmap, BQ2562X_ADC_CTRL,
				  BQ2562X_ADC_RATE | BQ2562X_ADC_EN,
				  BQ2562X_ADC_RATE | BQ2562X_ADC_EN);
}

static int bq2562x_reset_watchdog(struct bq2562x_device *bq)
{
	int ret;
	BQ2562X_DEBUG(
		bq->dev,
		"resetting and setting watchdog timer register to %d based on %d matching %d",
		bq->watchdog_timer_reg, bq->watchdog_timer,
		bq2562x_watchdog_time[bq->watchdog_timer_reg]);

	RET_NZ(regmap_update_bits, bq->regmap, BQ2562X_CHRG_CTRL_1,
	       BQ2562X_WATCHDOG_MASK | BQ2562X_WD_RST,
	       (bq->watchdog_timer_reg << BQ2562X_WATCHDOG_LSB) |
		       BQ2562X_WD_RST);
	return 0;
}

static int bq2562x_update_ichrg_curr(struct bq2562x_device *bq, int *ichg)
{
	int ret;
	RET_NZ(bq2562x_set_ichrg_curr, bq, *ichg);
	*ichg = bq2562x_get_ichrg_curr(bq);
	return 0;
}

static int bq2562x_update_state(struct bq2562x_device *bq,
				struct bq2562x_state *state)
{
	int chrg_stat_0 = 0, chrg_stat_1 = 0, chrg_flag_0 = 0, chrg_flag_1 = 0;
	int fault_flag_0 = 0, fault_status_0 = 0;
	int ret = 0;
	int val = 0;

	RET_NZ(regmap_read, bq->regmap, BQ2562X_CHRG_FLAG_0, &chrg_flag_0);

	RET_NZ(regmap_read, bq->regmap, BQ2562X_CHRG_FLAG_1, &chrg_flag_1);

	RET_NZ(regmap_read, bq->regmap, BQ2562X_FAULT_FLAG_0, &fault_flag_0);
	BQ2562X_DEBUG(
		bq->dev,
		"read interrupt flags chrg_flag_0:0x%02x chrg_flag_1:0x%02x fault_flag_0:0x%02x",
		chrg_flag_0, chrg_flag_1, fault_flag_0);

	if (bq->initial || chrg_flag_0) {
		RET_NZ(regmap_read, bq->regmap, BQ2562X_CHRG_STAT_0,
		       &chrg_stat_0);
		BQ2562X_DEBUG(bq->dev, "read BQ2562X_CHRG_STAT_0 as 0x%02x",
			      chrg_stat_0);

		if (chrg_stat_0 & BQ2562X_WD_STAT) {
			// Watchdog expire halves the ichg
			// set to original setpoint, and readback verify
			bq2562x_update_ichrg_curr(bq, &state->ichg_curr);
			bq2562x_reset_watchdog(bq);
			// start oneshot adc on WD expire
			bq2562x_start_adc(bq);
		}
		if ((chrg_flag_0 & BQ2562X_ADC_DONE) &&
		    (chrg_stat_0 & BQ2562X_ADC_DONE)) {
			BQ2562X_DEBUG(bq->dev, "ADC ready");

			state->vbat_adc = bq2562x_get_vbat_adc(bq);

			state->vbus_adc = bq2562x_get_vbus_adc(bq);

			state->ibat_adc = bq2562x_get_ibat_adc(bq);

			state->ibus_adc = bq2562x_get_ibus_adc(bq);
		}
	}

	if (bq->initial || chrg_flag_1) {
		RET_NZ(regmap_read, bq->regmap, BQ2562X_CHRG_STAT_1,
		       &chrg_stat_1);

		BQ2562X_DEBUG(bq->dev, "read BQ2562X_CHRG_STAT_1 pin as 0x%02x",
			      chrg_stat_1);

		state->chrg_status = chrg_stat_1 & BQ2562X_CHG_STAT_MSK;
		state->ce_status = bq2562x_get_charge_enable(bq);
		state->chrg_type = chrg_stat_1 & BQ2562X_VBUS_STAT_MSK;
		state->online = bq2562x_get_online_status(chrg_stat_1);
		// start adc if charge status changed and not already started
		if (!(chrg_stat_0 & BQ2562X_WD_STAT)) {
			bq2562x_start_adc(bq);
		}
	}

	if (bq->initial || fault_flag_0) {
		RET_NZ(regmap_read, bq->regmap, BQ2562X_FAULT_STAT_0,
		       &fault_status_0);
		state->health = fault_status_0 & BQ2562X_TEMP_MASK;

		state->fault_0 = fault_status_0;
	}

	if (bq->ac_detect_gpio) {
		val = gpiod_get_value_cansleep(bq->ac_detect_gpio);
		BQ2562X_DEBUG(bq->dev, "read ac_detect pin as %d", val);
		state->vbus_status = val;
	}
	bq->initial = false;
	return 0;
}

static void bq2562x_charger_reset(void *data)
{
	// TODO: figure out if this is needed or not
}

static int bq2562x_set_property(struct power_supply *psy,
				enum power_supply_property prop,
				const union power_supply_propval *val)
{
	struct bq2562x_device *bq = power_supply_get_drvdata(psy);
	int ret = -EINVAL;
	int tmp = 0;
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = RET_NZ(bq2562x_set_input_curr_lim, bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = RET_NZ(bq2562x_set_chrg_volt, bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		tmp = val->intval;
		ret = RET_NZ(bq2562x_update_ichrg_curr, bq, &tmp);
		mutex_lock(&bq->lock);
		bq->state.ichg_curr = tmp;
		mutex_unlock(&bq->lock);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = RET_NZ(bq2562x_set_prechrg_curr, bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = RET_NZ(bq2562x_set_term_curr, bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = RET_NZ(bq2562x_set_input_volt_lim, bq, val->intval);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bq2562x_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq2562x_device *bq = power_supply_get_drvdata(psy);
	struct bq2562x_state state;
	int ret = 0;
	BQ2562X_DEBUG(bq->dev, "bq2562x_get_property prop: 0x%08x", psp);

	mutex_lock(&bq->lock);
	state = bq->state;
	mutex_unlock(&bq->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		BQ2562X_DEBUG(
			bq->dev,
			"POWER_SUPPLY_PROP_STATUS chrg_type 0x%02x charge status 0x%02x ibat %d",
			state.chrg_type, state.chrg_status, state.ibat_adc);

		if (!state.chrg_type || (state.chrg_type == BQ2562X_OTG_MODE))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!state.chrg_status) {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			if (state.ibat_adc == 0)
				val->intval = POWER_SUPPLY_STATUS_FULL;
		} else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		BQ2562X_DEBUG(
			bq->dev,
			"POWER_SUPPLY_PROP_CHARGE_TYPE charge status 0x%02x",
			state.chrg_status);
		switch (state.chrg_status) {
		case BQ2562X_TRICKLE_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case BQ2562X_TAPER_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
			break;
		case BQ2562X_TOP_OFF_CHRG:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case BQ2562X_NOT_CHRGING:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = BQ2562X_MANUFACTURER;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = bq->model_name;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = state.online;
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		switch (state.chrg_type) {
		case BQ2562X_USB_SDP:
			val->intval = POWER_SUPPLY_USB_TYPE_SDP;
			break;
		case BQ2562X_USB_CDP:
			val->intval = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		case BQ2562X_USB_DCP:
			val->intval = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case BQ2562X_OTG_MODE:
			val->intval = POWER_SUPPLY_USB_TYPE_ACA;
			break;
		default:
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_HEALTH:
		if (state.fault_0 &
		    (BQ2562X_OTG_FAULT_STAT | BQ2562X_SYS_FAULT_STAT))
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			val->intval = POWER_SUPPLY_HEALTH_GOOD;

		switch (state.health) {
		case BQ2562X_TEMP_TS_NORMAL:
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case BQ2562X_TEMP_HOT:
			val->intval = POWER_SUPPLY_HEALTH_HOT;
			break;
		case BQ2562X_TEMP_WARM:
		case BQ2562X_TEMP_PREWARM:
			val->intval = POWER_SUPPLY_HEALTH_WARM;
			break;
		case BQ2562X_TEMP_COOL:
		case BQ2562X_TEMP_PRECOOL:
			val->intval = POWER_SUPPLY_HEALTH_COOL;
			break;
		case BQ2562X_TEMP_COLD:
			val->intval = POWER_SUPPLY_HEALTH_COLD;
			break;
		case BQ2562X_TEMP_PIN_BIAS_REFER_FAULT:
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
			break;
		}
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = state.ichg_curr;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = RET_FAIL(bq2562x_get_chrg_volt, bq);
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = RET_FAIL(bq2562x_get_prechrg_curr, bq);
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = RET_FAIL(bq2562x_get_term_curr, bq);
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vbus_adc;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state.ibus_adc;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = RET_FAIL(bq2562x_get_input_volt_lim, bq);
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = RET_FAIL(bq2562x_get_input_curr_lim, bq);
		val->intval = ret;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int bq2562x_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct bq2562x_device *bq = power_supply_get_drvdata(psy);
	struct bq2562x_state state;
	int ret = 0;
	BQ2562X_DEBUG(bq->dev, "bq2562x_battery_get_property prop: 0x%08x",
		      psp);

	mutex_lock(&bq->lock);
	state = bq->state;
	mutex_unlock(&bq->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = bq->init_data.ichg_max;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		val->intval = bq->init_data.vreg_max;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vbat_adc;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state.ibat_adc;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static bool bq2562x_state_changed(struct bq2562x_device *bq,
				  struct bq2562x_state *new_state)
{
	struct bq2562x_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->state;
	mutex_unlock(&bq->lock);

	return memcmp(&old_state, new_state, sizeof(struct bq2562x_state)) != 0;
}

static irqreturn_t bq2562x_irq_handler_thread(int irq, void *private)
{
	struct bq2562x_device *bq = private;
	struct bq2562x_state state;
	int ret;
	BQ2562X_DEBUG(bq->dev, "bq2562x_irq_handler_thread irq: 0x%08x", irq);
	// starting off with the previous state
	mutex_lock(&bq->lock);
	state = bq->state;
	mutex_unlock(&bq->lock);

	ret = bq2562x_update_state(bq, &state);
	if (ret)
		goto irq_out;

	if (!bq2562x_state_changed(bq, &state))
		goto irq_out;
	BQ2562X_DEBUG(bq->dev, "bq2562x_irq_handler_thread state changed");

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);
	power_supply_changed(bq->charger);

irq_out:
	return IRQ_HANDLED;
}

// TODO: add charge enable property
/*
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_BATTERY_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
*/
static enum power_supply_property bq2562x_power_supply_props[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_NOW,

};

/* TODO: add battery temperature reading */
static enum power_supply_property bq2562x_battery_props[] = {
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static int bq2562x_property_is_writeable(struct power_supply *psy,
					 enum power_supply_property prop)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		return true;
	default:
		return false;
	}
}

static const struct power_supply_desc bq2562x_power_supply_desc = {
	.name = "bq2562x-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = bq2562x_usb_type,
	.num_usb_types = ARRAY_SIZE(bq2562x_usb_type),
	.properties = bq2562x_power_supply_props,
	.num_properties = ARRAY_SIZE(bq2562x_power_supply_props),
	.get_property = bq2562x_get_property,
	.set_property = bq2562x_set_property,
	.property_is_writeable = bq2562x_property_is_writeable,
};

static const struct power_supply_desc bq2562x_battery_desc = {
	.name = "bq2562x-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.get_property = bq2562x_battery_get_property,
	.properties = bq2562x_battery_props,
	.num_properties = ARRAY_SIZE(bq2562x_battery_props),
};

static bool bq2562x_is_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case BQ2562X_CHRG_STAT_0 ... BQ2562X_FAULT_FLAG_0:
	case BQ2562X_ADC_IBUS_LSB ... BQ2562X_ADC_TDIE_MSB:
	case BQ2562X_CHRG_CTRL_1:
	case BQ2562X_CHRG_I_LIM_LSB:
	case BQ2562X_CHRG_I_LIM_MSB:
	case BQ2562X_ADC_CTRL:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config bq25620_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ2562X_PART_INFO,
	.reg_defaults = bq25620_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25620_reg_defs),
	.cache_type = REGCACHE_FLAT,
	.volatile_reg = bq2562x_is_volatile_reg,
};

static const struct regmap_config bq25622_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ2562X_PART_INFO,
	.reg_defaults = bq25622_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25622_reg_defs),
	.cache_type = REGCACHE_FLAT,
	.volatile_reg = bq2562x_is_volatile_reg,
};

static int bq2562x_power_supply_init(struct bq2562x_device *bq,
				     struct power_supply_config *psy_cfg,
				     struct device *dev)
{
	bq->charger = devm_power_supply_register(
		bq->dev, &bq2562x_power_supply_desc, psy_cfg);
	if (IS_ERR(bq->charger))
		return -EINVAL;

	bq->battery = devm_power_supply_register(bq->dev, &bq2562x_battery_desc,
						 psy_cfg);
	if (IS_ERR(bq->battery))
		return -EINVAL;
	return 0;
}

static int bq2562x_hw_init(struct bq2562x_device *bq)
{
	struct power_supply_battery_info *bat_info;
	struct power_supply_battery_info default_bat_info = {};
	int wd_reg_val = BQ2562X_WATCHDOG_DIS;
	int wd_max_val = BQ2562X_NUM_WD_VAL - 1;
	int ret = 0;
	int i;
	bq->initial = true;

	if (bq->watchdog_timer) {
		if (bq->watchdog_timer >= bq2562x_watchdog_time[wd_max_val])
			wd_reg_val = wd_max_val;
		else {
			for (i = 0; i < wd_max_val; i++) {
				if (bq->watchdog_timer >=
					    bq2562x_watchdog_time[i] &&
				    bq->watchdog_timer <
					    bq2562x_watchdog_time[i + 1]) {
					wd_reg_val = i;
					break;
				}
			}
		}
	}
	bq->watchdog_timer_reg = wd_reg_val;

	RET_NZ(bq2562x_reset_watchdog, bq);

	ret = power_supply_get_battery_info(bq->charger, &bat_info);
	if (ret) {
		dev_warn(
			bq->dev,
			"battery info missing, default values will be applied\n");
		bat_info = &default_bat_info;
		default_bat_info.constant_charge_current_max_ua =
			BQ2562X_ICHRG_I_DEF_uA;

		default_bat_info.constant_charge_voltage_max_uv =
			BQ2562X_VREG_V_DEF_uV;

		default_bat_info.precharge_current_ua =
			BQ2562X_PRECHRG_I_DEF_uA;
		default_bat_info.charge_term_current_ua =
			BQ2562X_TERMCHRG_I_DEF_uA;
		bq->init_data.ichg_max = BQ2562X_ICHRG_I_MAX_uA;
		bq->init_data.vreg_max = BQ2562X_VREG_V_MAX_uV;
	} else {
		bq->init_data.ichg_max =
			bat_info->constant_charge_current_max_ua;

		bq->init_data.vreg_max =
			bat_info->constant_charge_voltage_max_uv;
	}

	// initialize charge current
	bq->state.ichg_curr = bq->init_data.ichg_max;
	RET_NZ(bq2562x_set_ichrg_curr, bq, bq->init_data.ichg_max);
	RET_NZ(bq2562x_set_prechrg_curr, bq, bat_info->precharge_current_ua);
	RET_NZ(bq2562x_set_chrg_volt, bq,
	       bat_info->constant_charge_voltage_max_uv);

	RET_NZ(bq2562x_set_term_curr, bq, bat_info->charge_term_current_ua);

	RET_NZ(bq2562x_set_input_volt_lim, bq, bq->init_data.vlim);

	RET_NZ(bq2562x_set_input_curr_lim, bq, bq->init_data.ilim);

	power_supply_put_battery_info(bq->charger, bat_info);

	// TODO: add battery detection with Force IBAT discharge
	// Enable chargin by default
	RET_NZ(bq2562x_set_charge_enable, bq, 1);

	RET_NZ(bq2562x_start_adc, bq);

	return 0;
}

static int bq2562x_parse_dt(struct bq2562x_device *bq,
			    struct power_supply_config *psy_cfg,
			    struct device *dev)
{
	int ret = 0;

	psy_cfg->drv_data = bq;
	psy_cfg->of_node = dev->of_node;

	ret = device_property_read_u32(bq->dev, "ti,watchdog-timeout-ms",
				       &bq->watchdog_timer);
	if (ret)
		bq->watchdog_timer = BQ2562X_WATCHDOG_DIS;

	if (bq->watchdog_timer > BQ2562X_WATCHDOG_MAX ||
	    bq->watchdog_timer < BQ2562X_WATCHDOG_DIS)
		return -EINVAL;

	ret = device_property_read_u32(bq->dev, "input-voltage-limit-microvolt",
				       &bq->init_data.vlim);
	if (ret)
		bq->init_data.vlim = BQ2562X_VINDPM_DEF_uV;

	if (bq->init_data.vlim > BQ2562X_VINDPM_V_MAX_uV ||
	    bq->init_data.vlim < BQ2562X_VINDPM_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(bq->dev, "input-current-limit-microamp",
				       &bq->init_data.ilim);
	if (ret)
		bq->init_data.ilim = BQ2562X_IINDPM_DEF_uA;

	if (bq->init_data.ilim > BQ2562X_IINDPM_I_MAX_uA ||
	    bq->init_data.ilim < BQ2562X_IINDPM_I_MIN_uA)
		return -EINVAL;

	bq->ac_detect_gpio =
		devm_gpiod_get_optional(bq->dev, "ac-detect", GPIOD_IN);
	if (IS_ERR(bq->ac_detect_gpio)) {
		ret = PTR_ERR(bq->ac_detect_gpio);
		dev_err(bq->dev, "Failed to get ac detect");
		return ret;
	}
	// startup with charging disable
	bq->ce_gpio = devm_gpiod_get_optional(bq->dev, "charge-enable",
					      GPIOD_OUT_LOW);
	if (IS_ERR(bq->ce_gpio)) {
		ret = PTR_ERR(bq->ce_gpio);
		dev_err(bq->dev, "Failed to get ce");
		return ret;
	}

	return 0;
}

static int bq2562x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct bq2562x_device *bq;
	struct power_supply_config psy_cfg = {};
	int ret;

	bq = devm_kzalloc(dev, sizeof(*bq), GFP_KERNEL);
	if (!bq)
		return -ENOMEM;

	bq->client = client;
	bq->dev = dev;

	mutex_init(&bq->lock);

	strncpy(bq->model_name, id->name, I2C_NAME_SIZE);

	bq->device_id = id->driver_data;

	switch (bq->device_id) {
	case BQ25620:
		bq->regmap =
			devm_regmap_init_i2c(client, &bq25620_regmap_config);
		break;
	case BQ25622:
		bq->regmap =
			devm_regmap_init_i2c(client, &bq25622_regmap_config);
		break;
	}

	if (IS_ERR(bq->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		return PTR_ERR(bq->regmap);
	}

	i2c_set_clientdata(client, bq);

	RET_NZ(bq2562x_parse_dt, bq, &psy_cfg, dev);

	RET_NZ(devm_add_action_or_reset, dev, bq2562x_charger_reset, bq);

	// need to allocate power supply before registering interrupt
	RET_NZ(bq2562x_power_supply_init, bq, &psy_cfg, dev);

	RET_NZ(bq2562x_hw_init, bq);

	// last step to setup IRQ, as it can be called as soon as
	// this returns, resulting in uninitialized state
	if (client->irq) {
		ret = devm_request_threaded_irq(
			dev, client->irq, NULL, bq2562x_irq_handler_thread,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			dev_name(&client->dev), bq);
		if (ret < 0) {
			dev_err(dev, "get irq fail: %d\n", ret);
			return ret;
		}
	}

	return ret;
}

static const struct i2c_device_id bq2562x_i2c_ids[] = {
	{ "bq25620", BQ25620 },
	{ "bq25622", BQ25622 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq2562x_i2c_ids);

static const struct of_device_id bq2562x_of_match[] = {
	{
		.compatible = "ti,bq25620",
	},
	{
		.compatible = "ti,bq25622",
	},
	{},
};
MODULE_DEVICE_TABLE(of, bq2562x_of_match);

static const struct acpi_device_id bq2562x_acpi_match[] = {
	{ "bq25620", BQ25620 },
	{ "bq25622", BQ25622 },
	{},
};
MODULE_DEVICE_TABLE(acpi, bq2562x_acpi_match);

static struct i2c_driver bq2562x_driver = {
    .driver =
        {
            .name = "bq2562x-charger",
            .of_match_table = bq2562x_of_match,
            .acpi_match_table = bq2562x_acpi_match,
        },
    .probe = bq2562x_probe,
    .id_table = bq2562x_i2c_ids,
};
module_i2c_driver(bq2562x_driver);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("bq2562x charger driver");
MODULE_LICENSE("GPL v2");
