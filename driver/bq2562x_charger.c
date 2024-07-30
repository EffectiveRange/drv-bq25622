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
	u32 ext_ilim;
	u32 bat_energy;
	u32 bat_cap;
	u32 bat_low_v;
};

struct bq2562x_state {
	bool online;
	u8 chrg_status;
	u8 ce_status;
	u8 chrg_type;
	u8 chrg_fault;
	u8 vsys_status;
	u8 vbus_status;
	u8 fault_0;
	u32 vbus_adc;
	s32 ibus_adc;
	s32 tdie_adc;

	// ICHG value is halved by Watchdog reset
	// we'll use this is as two way variable, from the set_property
	// site and also resetting it to this value on each WD reset event
	u32 ichg_curr;
	u32 ilim_curr;
	// derived attributes
	int charging_state;
};

struct bq2562x_battery_state {
	s32 ts_adc;
	u32 vbat_adc;
	s32 ibat_adc;
	// these ADC measurements are performed with averaging
	u32 vbat_adc_avg;
	s32 ibat_adc_avg;
	// battery SoC section
	u32 curr_percent;
	u32 curr_energy;

	int charging_state;
	u8 health;
	u8 chrg_status;
	bool online;
	bool overvoltage;
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
	struct bq2562x_battery_state bat_state;
	// to signal initial status read
	// to not only react to flags, but
	// read all status regs on first read
	bool initial;
	int watchdog_timer;
	int watchdog_timer_reg;
	// TS ADC reading -> Celsius := (ADC * - ts_coeff_a +  ts_coeff_b)/ts_coeff_scale
	int ts_coeff_a;
	int ts_coeff_b;
	int ts_coeff_scale;

	// don't use ce pin even if specified
	bool ce_pin_override;
	// ce pin is active low
	bool ce_pin_negate;
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
	bool gpio_ce = false;
	int charger_enable = 0;
	unsigned int chrg_ctrl_0 = 0;

	if (bq->ce_gpio) {
		ce_pin = RET_FAIL(gpiod_get_value_cansleep, bq->ce_gpio);
		BQ2562X_DEBUG(bq->dev, "read CE pin: %u (negate:%d)", ce_pin,
			      bq->ce_pin_negate);
		gpio_ce = ((bool)ce_pin) != bq->ce_pin_negate;
	}

	RET_NZ(regmap_read, bq->regmap, BQ2562X_CHRG_CTRL_1, &chrg_ctrl_0);

	charger_enable = chrg_ctrl_0 & BQ2562X_CHRG_EN;

	BQ2562X_DEBUG(bq->dev, "read charge enable: %d.\n", charger_enable);

	if (bq->ce_gpio) {
		return charger_enable && gpio_ce;
	}
	return charger_enable;
}

static int bq2562x_set_charge_enable(struct bq2562x_device *bq, int val)
{
	int ret;
	int gpioval;
	BQ2562X_DEBUG(bq->dev, "setting charge enable to %d (negate:%d)", val,
		      bq->ce_pin_negate);
	if (bq->ce_gpio) {
		gpioval = ((bool)val) != bq->ce_pin_negate ? 1 : 0;
		BQ2562X_DEBUG(bq->dev, "setting charge enable gpio pin to %d",
			      gpioval);
		gpiod_set_value_cansleep(bq->ce_gpio, gpioval);
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
	u8 rd_buff[2] = { 0, 0 };
	(void)key;

	RET_FAIL(regmap_bulk_read, bq->regmap, baseaddr, rd_buff, 2);

	read_res = ((s16)((rd_buff[1] << 8) | rd_buff[0])) >> step;

	*res = read_res * scale;

	BQ2562X_DEBUG(
		bq->dev,
		"read %s addr:0x%02x msb:0x%02x lsb:0x%02x read_result:%d result:%d",
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
	u8 rd_buff[2] = { 0, 0 };
	(void)key;

	RET_FAIL(regmap_bulk_read, bq->regmap, baseaddr, rd_buff, 2);

	read_res = ((rd_buff[1] << 8) | rd_buff[0]) >> step;

	res = read_res * scale;

	BQ2562X_DEBUG(
		bq->dev,
		"read %s addr:0x%02x msb:0x%02x lsb:0x%02x read_result:%u result:%u",
		key, baseaddr, rd_buff[1], rd_buff[0], read_res, res);

	return res;
}

static int bq2562x_set_word_unsigned(struct bq2562x_device *bq,
				     unsigned baseaddr, const char *const key,
				     unsigned val, unsigned step,
				     unsigned scale, unsigned minval,
				     unsigned maxval)
{
	int ret = 0;
	int reg_val = 0;
	u8 wr_buff[2] = { 0, 0 };

	val = clamp(val, minval, maxval);

	reg_val = (val / scale) << step;
	wr_buff[0] = reg_val & 0xff;
	wr_buff[1] = (reg_val >> 8) & 0xff;
	RET_NZ(regmap_bulk_write, bq->regmap, baseaddr, wr_buff, 2);

	BQ2562X_DEBUG(bq->dev,
		      "write %s addr:0x%02x val:%u msb:0x%02x lsb:0x%02x ", key,
		      baseaddr, val, wr_buff[1], wr_buff[0]);

	return 0;
}

static int bq2562x_get_vbat_adc(struct bq2562x_device *bq)
{
	int ret = 0;
	return RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_ADC_VBAT_LSB,
			"vbat adc", BQ2562X_ADC_VBAT_MOVE_STEP,
			BQ2562X_ADC_VBAT_STEP_uV);
}

static int bq2562x_get_vbus_adc(struct bq2562x_device *bq)
{
	int ret = 0;
	return RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_ADC_VBUS_LSB,
			"vbus adc", BQ2562X_ADC_VBUS_MOVE_STEP,
			BQ2562X_ADC_VBUS_STEP_uV);
}

static int bq2562x_get_ibat_adc(struct bq2562x_device *bq)
{
	int ret = 0;
	int res = 0;
	RET_NZ_W_VAL(0, bq2562x_get_word_signed, bq, BQ2562X_ADC_IBAT_LSB,
		     "ibat adc", 0, BQ2562X_ADC_CURR_STEP_uA, &res);
	return res;
}

static int bq2562x_get_ibus_adc(struct bq2562x_device *bq)
{
	int ret = 0;
	int res = 0;
	RET_NZ_W_VAL(0, bq2562x_get_word_signed, bq, BQ2562X_ADC_IBUS_LSB,
		     "ibus adc", 0, BQ2562X_ADC_CURR_STEP_uA, &res);
	return res;
}

static int bq2562x_get_tdie_adc(struct bq2562x_device *bq)
{
	int ret = 0;
	int res = 0;
	u8 sign = 0;
	s16 read_res = 0;
	u8 rd_buff[2] = { 0, 0 };

	RET_NZ_W_VAL(0, regmap_bulk_read, bq->regmap, BQ2562X_ADC_TDIE_LSB,
		     rd_buff, 2);

	sign = rd_buff[1] & BQ2562X_ADC_TDIE_MSB_MSBIT;
	if (sign) {
		rd_buff[1] |= ~BQ2562X_ADC_TDIE_MSB_MASK;
	}
	read_res = ((s16)((rd_buff[1]) | rd_buff[0]));

	res = read_res * BQ2562X_ADC_TDIE_TEMP_STEP_01C;

	BQ2562X_DEBUG(
		bq->dev,
		"read tdie adc addr:0x%02x msb:0x%02x lsb:0x%02x read_result:%d result:%d",
		BQ2562X_ADC_TDIE_LSB, rd_buff[1], rd_buff[0], read_res, res);

	return res;
}

static int bq2562x_get_ts_adc(struct bq2562x_device *bq)
{
	int ret = 0;
	u16 tsadc = 0;
	int res = 0;
	tsadc = RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_ADC_TS_LSB,
			 "ts adc", 0, 1);
	res = ((int)tsadc * (-bq->ts_coeff_a) + bq->ts_coeff_b) /
	      bq->ts_coeff_scale;
	BQ2562X_DEBUG(bq->dev, "ts adc result is %d [0.1 Celsius]", res);
	return res;
}

static int bq2562x_get_term_curr(struct bq2562x_device *bq)
{
	int ret;
	return RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_TERM_CTRL_LSB,
			"iterm curr", BQ2562X_ITERM_MOVE_STEP,
			BQ2562X_TERMCHRG_CURRENT_STEP_uA);
}

static int bq2562x_get_prechrg_curr(struct bq2562x_device *bq)
{
	int ret;
	return RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_PRECHRG_CTRL_LSB,
			"precharge curr", BQ2562X_PRECHRG_MOVE_STEP,
			BQ2562X_PRECHRG_CURRENT_STEP_uA);
}

static int bq2562x_get_ichrg_curr(struct bq2562x_device *bq)
{
	int ret;
	return RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_CHRG_I_LIM_LSB,
			"ichrg_curr", BQ2562X_ICHG_MOVE_STEP,
			BQ2562X_ICHRG_CURRENT_STEP_uA);
}

static int bq2562x_set_term_curr(struct bq2562x_device *bq, int term_current)
{
	int ret;
	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_TERM_CTRL_LSB,
		 "term curr", term_current, BQ2562X_ITERM_MOVE_STEP,
		 BQ2562X_TERMCHRG_CURRENT_STEP_uA, BQ2562X_TERMCHRG_I_MIN_uA,
		 BQ2562X_TERMCHRG_I_MAX_uA);
	return 0;
}

static int bq2562x_set_prechrg_curr(struct bq2562x_device *bq, int pre_current)
{
	int ret;
	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_PRECHRG_CTRL_LSB,
		 "prechrg curr", pre_current, BQ2562X_PRECHRG_MOVE_STEP,
		 BQ2562X_PRECHRG_CURRENT_STEP_uA, BQ2562X_PRECHRG_I_MIN_uA,
		 BQ2562X_PRECHRG_I_MAX_uA);
	return 0;
}

static int bq2562x_set_ichrg_curr(struct bq2562x_device *bq, int chrg_curr)
{
	int ret;
	int chrg_curr_max = bq->init_data.ichg_max;
	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_CHRG_I_LIM_LSB,
		 "ichrg curr", chrg_curr, BQ2562X_ICHG_MOVE_STEP,
		 BQ2562X_ICHRG_CURRENT_STEP_uA, BQ2562X_ICHRG_I_MIN_uA,
		 chrg_curr_max);
	return 0;
}

static int bq2562x_set_chrg_volt(struct bq2562x_device *bq, int chrg_volt)
{
	int ret;
	int chrg_volt_max = bq->init_data.vreg_max;
	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_CHRG_V_LIM_LSB,
		 "chrg volt", chrg_volt, BQ2562X_CHRG_V_LIM_MOVE_STEP,
		 BQ2562X_VREG_V_STEP_uV, BQ2562X_VREG_V_MIN_uV, chrg_volt_max);
	return 0;
}

static int bq2562x_get_chrg_volt(struct bq2562x_device *bq)
{
	int ret;
	return RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_CHRG_V_LIM_LSB,
			"chrg volt", BQ2562X_CHRG_V_LIM_MOVE_STEP,
			BQ2562X_VREG_V_STEP_uV);
}

static int bq2562x_set_input_volt_lim(struct bq2562x_device *bq, int vindpm)
{
	int ret;
	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_INPUT_V_LIM_LSB,
		 "input volt", vindpm, BQ2562X_INPUT_V_LIM_MOVE_STEP,
		 BQ2562X_VINDPM_STEP_uV, BQ2562X_VINDPM_V_MIN_uV,
		 BQ2562X_VINDPM_V_MAX_uV);
	return 0;
}

static int bq2562x_get_input_volt_lim(struct bq2562x_device *bq)
{
	int ret;
	return RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_INPUT_V_LIM_LSB,
			"input volt", BQ2562X_INPUT_V_LIM_MOVE_STEP,
			BQ2562X_VINDPM_STEP_uV);
}

static int bq25622_disable_ext_ilim(struct bq2562x_device *bq, int iindpm)
{
	int ret = 0;
	if (bq->device_id == BQ25622 && iindpm > bq->init_data.ext_ilim) {
		RET_FAIL(regmap_update_bits, bq->regmap, BQ2562X_CHRG_CTRL_4,
			 BQ2562X_EN_EXT_ILIM, 0);
	}
	return 0;
}

static int bq2562x_set_input_curr_lim(struct bq2562x_device *bq, int iindpm)
{
	int ret;
	RET_FAIL(bq2562x_set_word_unsigned, bq, BQ2562X_INPUT_I_LIM_LSB,
		 "input curr", iindpm, BQ2562X_INPUT_I_LIM__MOVE_STEP,
		 BQ2562X_IINDPM_STEP_uA, BQ2562X_IINDPM_I_MIN_uA,
		 BQ2562X_IINDPM_I_MAX_uA);
	RET_FAIL(bq25622_disable_ext_ilim, bq, iindpm);

	return 0;
}

static int bq2562x_get_input_curr_lim(struct bq2562x_device *bq)
{
	int ret;

	return RET_FAIL(bq2562x_get_word_unsigned, bq, BQ2562X_INPUT_I_LIM_LSB,
			"input curr", BQ2562X_INPUT_I_LIM__MOVE_STEP,
			BQ2562X_IINDPM_STEP_uA);
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
static int bq2562x_start_adc_oneshot(struct bq2562x_device *bq)
{
	int ret = 0;
	RET_FAIL(regmap_write, bq->regmap, BQ2562X_FN_DISABE_0,
		 BQ2562X_VPMID_ADC_DIS);

	RET_FAIL(regmap_update_bits, bq->regmap, BQ2562X_ADC_CTRL,
		 BQ2562X_ADC_AVG, 0);

	return regmap_update_bits(bq->regmap, BQ2562X_ADC_CTRL,
				  BQ2562X_ADC_RATE | BQ2562X_ADC_EN,
				  BQ2562X_ADC_RATE | BQ2562X_ADC_EN);
}

static int bq2562x_start_adc_avg(struct bq2562x_device *bq)
{
	int ret = 0;
	int adc_mask = BQ2562X_IBUS_ADC_DIS | BQ2562X_VBUS_ADC_DIS |
		       BQ2562X_VSYS_ADC_DIS | BQ2562X_TS_ADC_DIS |
		       BQ2562X_TDIE_ADC_DIS | BQ2562X_VPMID_ADC_DIS;

	RET_FAIL(regmap_write, bq->regmap, BQ2562X_FN_DISABE_0, adc_mask);

	RET_FAIL(regmap_update_bits, bq->regmap, BQ2562X_ADC_CTRL,
		 BQ2562X_ADC_RATE, 0);

	return regmap_update_bits(
		bq->regmap, BQ2562X_ADC_CTRL,
		BQ2562X_ADC_AVG | BQ2562X_ADC_AVG_INIT | BQ2562X_ADC_EN,
		BQ2562X_ADC_AVG | BQ2562X_ADC_AVG_INIT | BQ2562X_ADC_EN);
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

static int bq2562x_update_adc_now(struct bq2562x_device *bq,
				  struct bq2562x_state *state,
				  struct bq2562x_battery_state *bat_state)
{
	bat_state->vbat_adc = bq2562x_get_vbat_adc(bq);

	state->vbus_adc = bq2562x_get_vbus_adc(bq);

	bat_state->ibat_adc = bq2562x_get_ibat_adc(bq);

	state->ibus_adc = bq2562x_get_ibus_adc(bq);

	state->tdie_adc = bq2562x_get_tdie_adc(bq);

	bat_state->ts_adc = bq2562x_get_ts_adc(bq);
	return 0;
}

static int bq2562x_update_adc_avg(struct bq2562x_device *bq,
				  struct bq2562x_battery_state *state)
{
	state->vbat_adc_avg = bq2562x_get_vbat_adc(bq);

	state->ibat_adc_avg = bq2562x_get_ibat_adc(bq);

	return 0;
}

int get_power_supply_charging_state(struct bq2562x_device *bq,
				    struct bq2562x_state *state,
				    struct bq2562x_battery_state *bat_state)
{
	if (!state->chrg_type || (state->chrg_type == BQ2562X_OTG_MODE))
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (!state->chrg_status) {
		if (bat_state->ibat_adc == 0 && bq2562x_get_charge_enable(bq))
			return POWER_SUPPLY_STATUS_FULL;
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	return POWER_SUPPLY_STATUS_CHARGING;
}

static int bq2562x_update_battery_state(struct bq2562x_device *bq,
					struct bq2562x_battery_state *bat_state)
{
	s32 calc_energy = 0;
	s32 ibat = 0;
	// minimum battery voltage reached when discharging, report 1% fixed
	if ((bat_state->charging_state == POWER_SUPPLY_STATUS_DISCHARGING ||
	     bat_state->charging_state == POWER_SUPPLY_STATUS_NOT_CHARGING) &&
	    bat_state->vbat_adc_avg > 0 &&
	    bat_state->vbat_adc_avg < bq->init_data.bat_low_v) {
		bat_state->curr_percent = 1;
		bat_state->curr_energy = bq->init_data.bat_energy / 100;
		return 0;
	}

	switch (bat_state->charging_state) {
	case POWER_SUPPLY_STATUS_FULL:
		bat_state->curr_percent = 100;
		bat_state->curr_energy = bq->init_data.bat_energy;
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		switch (bat_state->chrg_status) {
		case BQ2562X_TRICKLE_CHRG:
			bat_state->curr_percent =
				max(bat_state->curr_percent, (u32)1);
			bat_state->curr_energy = bq->init_data.bat_energy *
						 bat_state->curr_percent / 100;
			break;
		case BQ2562X_TOP_OFF_CHRG:
			bat_state->curr_percent = 99;
			bat_state->curr_energy = bq->init_data.bat_energy *
						 bat_state->curr_percent / 100;
			break;
		case BQ2562X_TAPER_CHRG:
			// FIXME: estimate SoC when chargin, using 50% for now
			// TODO:
			// - in standard CC report based on vbat between 1-CC_limit
			// - in standard CV report based on Ibat between CC_limit-99%
			bat_state->curr_percent =
				max(bat_state->curr_percent, (u32)50);
			bat_state->curr_energy = bq->init_data.bat_energy *
						 bat_state->curr_percent / 100;
			break;
		}
		break;
	case POWER_SUPPLY_STATUS_DISCHARGING:
		ibat = max(-bat_state->ibat_adc_avg, (s32)0);
		calc_energy = bat_state->curr_energy -
			      (ibat / 1000) * (bat_state->vbat_adc_avg / 1000) *
				      (bq->watchdog_timer / 1000) / 3600;

		BQ2562X_DEBUG(
			bq->dev,
			"SoC calc init energy: %d curr energy:%d ibat:%d vbat:%d wdt:%d calc energy:%d",
			bq->init_data.bat_energy, bat_state->curr_energy, ibat,
			bat_state->vbat_adc_avg, bq->watchdog_timer,
			calc_energy);

		bat_state->curr_energy =
			clamp(calc_energy, 0, (s32)bq->init_data.bat_energy);
		bat_state->curr_percent =
			bat_state->curr_energy * 100 / bq->init_data.bat_energy;
		break;
	}
	return 0;
}

static int bq2562x_update_state(struct bq2562x_device *bq,
				struct bq2562x_state *state,
				struct bq2562x_battery_state *bat_state)
{
	int chrg_stat_0 = 0, chrg_stat_1 = 0, chrg_flag_0 = 0, chrg_flag_1 = 0;
	int fault_flag_0 = 0, fault_status_0 = 0;
	int ret = 0;
	int val = 0;
	u8 flags[3] = { 0, 0, 0 };
	u8 stats[3] = { 0, 0, 0 };

	RET_NZ(regmap_bulk_read, bq->regmap, BQ2562X_CHRG_FLAG_0, flags, 3);
	chrg_flag_0 = flags[0];
	chrg_flag_1 = flags[1];
	fault_flag_0 = flags[2];

	BQ2562X_DEBUG(
		bq->dev,
		"read interrupt flags chrg_flag_0:0x%02x chrg_flag_1:0x%02x fault_flag_0:0x%02x",
		chrg_flag_0, chrg_flag_1, fault_flag_0);

	RET_NZ(regmap_bulk_read, bq->regmap, BQ2562X_CHRG_STAT_0, stats, 3);
	chrg_stat_0 = stats[0];
	chrg_stat_1 = stats[1];
	fault_status_0 = stats[2];

	BQ2562X_DEBUG(
		bq->dev,
		"read status regs chrg_stat_0:0x%02x chrg_stat_1:0x%02x fault_status_0:0x%02x",
		chrg_stat_0, chrg_stat_1, fault_status_0);

	if (chrg_stat_0 & BQ2562X_WD_STAT) {
		// process ADC averaging results
		if (!bq->initial) {
			bq2562x_update_adc_avg(bq, bat_state);
		}
		// Watchdog expire halves the ichg
		// set to original setpoint, and readback verify
		bq2562x_update_ichrg_curr(bq, &state->ichg_curr);
		bq25622_disable_ext_ilim(bq, state->ilim_curr);
		bq2562x_reset_watchdog(bq);
		// start oneshot adc on WD expire
		bq2562x_start_adc_oneshot(bq);
	}
	if ((chrg_flag_0 & BQ2562X_ADC_DONE) &&
	    (chrg_stat_0 & BQ2562X_ADC_DONE)) {
		BQ2562X_DEBUG(bq->dev, "ADC ready");
		bq2562x_update_adc_now(bq, state, bat_state);
		// Start averaging until the next WD expiry
		bq2562x_start_adc_avg(bq);
	}

	bat_state->chrg_status = state->chrg_status = chrg_stat_1 &
						      BQ2562X_CHG_STAT_MSK;
	state->ce_status = bq2562x_get_charge_enable(bq);
	state->chrg_type = chrg_stat_1 & BQ2562X_VBUS_STAT_MSK;
	state->online = bq2562x_get_online_status(chrg_stat_1);
	bat_state->charging_state = state->charging_state =
		get_power_supply_charging_state(bq, state, bat_state);
	// start adc if charge status changed and not already started
	if (!(chrg_stat_0 & BQ2562X_WD_STAT)) {
		bq2562x_start_adc_oneshot(bq);
	}

	bat_state->health = fault_status_0 & BQ2562X_TEMP_MASK;
	bat_state->overvoltage = fault_status_0 & BQ2562X_BAT_FAULT_STAT;
	state->fault_0 = fault_status_0;

	if (bq->ac_detect_gpio) {
		val = gpiod_get_value_cansleep(bq->ac_detect_gpio);
		BQ2562X_DEBUG(bq->dev, "read ac_detect pin as %d", val);
		state->vbus_status = val;
	}
	bq2562x_update_battery_state(bq, bat_state);
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
		mutex_lock(&bq->lock);
		bq->state.ilim_curr = val->intval;
		mutex_unlock(&bq->lock);
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
		val->intval = state.charging_state;
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
		if (state.fault_0 & BQ2562X_TSHUT_FAULT_STAT) {
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else if (state.fault_0 &
			   (BQ2562X_OTG_FAULT_STAT | BQ2562X_SYS_FAULT_STAT |
			    BQ2562X_VSYS_FAULT_STAT)) {
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		} else {
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
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
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = state.tdie_adc;
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
		val->intval = BQ2562X_TEMP_MIN_01C;
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		val->intval = BQ2562X_TEMP_MAX_01C;
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
	struct bq2562x_battery_state state;
	int ret = 0;
	BQ2562X_DEBUG(bq->dev, "bq2562x_battery_get_property prop: 0x%08x",
		      psp);

	mutex_lock(&bq->lock);
	state = bq->bat_state;
	mutex_unlock(&bq->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = state.charging_state;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (state.overvoltage) {
			// TODO: differentiate between overvoltage and
			// overcurrent if possible
			val->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		} else {
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
			default:
				val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
			}
		}
		break;
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
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		val->intval = state.vbat_adc_avg;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		val->intval = state.ibat_adc_avg;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = state.ts_adc;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = state.curr_percent;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq->init_data.bat_cap;
		break;
	case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
		val->intval = bq->init_data.bat_energy;
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

static bool bq2562x_bat_state_changed(struct bq2562x_device *bq,
				      struct bq2562x_battery_state *new_state)
{
	struct bq2562x_battery_state old_state;

	mutex_lock(&bq->lock);
	old_state = bq->bat_state;
	mutex_unlock(&bq->lock);

	return memcmp(&old_state, new_state,
		      sizeof(struct bq2562x_battery_state)) != 0;
}

static irqreturn_t bq2562x_irq_handler_thread(int irq, void *private)
{
	struct bq2562x_device *bq = private;
	struct bq2562x_state state;
	struct bq2562x_battery_state bat_state;
	int ret;
	BQ2562X_DEBUG(bq->dev, "bq2562x_irq_handler_thread irq: 0x%08x", irq);
	// starting off with the previous state
	mutex_lock(&bq->lock);
	state = bq->state;
	bat_state = bq->bat_state;
	mutex_unlock(&bq->lock);

	ret = bq2562x_update_state(bq, &state, &bat_state);
	if (ret) {
		dev_err(bq->dev, "error updating battery state");
		goto irq_out;
	}

	if (bq2562x_state_changed(bq, &state)) {
		BQ2562X_DEBUG(bq->dev,
			      "bq2562x_irq_handler_thread state changed");
		mutex_lock(&bq->lock);
		bq->state = state;
		mutex_unlock(&bq->lock);
		power_supply_changed(bq->charger);
	}

	if (bq2562x_bat_state_changed(bq, &bat_state)) {
		BQ2562X_DEBUG(
			bq->dev,
			"bq2562x_irq_handler_thread battery state changed");
		mutex_lock(&bq->lock);
		bq->bat_state = bat_state;
		mutex_unlock(&bq->lock);
		power_supply_changed(bq->battery);
	}

irq_out:
	return IRQ_HANDLED;
}

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
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
};

static enum power_supply_property bq2562x_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
	POWER_SUPPLY_PROP_TEMP,
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
	case BQ2562X_CHRG_CTRL_4:
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
	dev_info(bq->dev, "Intializing BQ25620/22 HW ...");

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
		bq->init_data.bat_cap = 0;
		bq->init_data.bat_energy = 0;
	} else {
		bq->init_data.ichg_max =
			bat_info->constant_charge_current_max_ua;

		bq->init_data.vreg_max =
			bat_info->constant_charge_voltage_max_uv;
		bq->init_data.bat_cap = bat_info->charge_full_design_uah;
		bq->init_data.bat_energy = bat_info->energy_full_design_uwh;
	}

	// TODO: determine initial percent
	bq->bat_state.curr_percent = 50;
	bq->bat_state.curr_energy =
		bq->init_data.bat_energy * bq->bat_state.curr_percent / 100;
	// initialize charge current
	bq->state.ichg_curr = bq->init_data.ichg_max;
	RET_NZ(bq2562x_set_ichrg_curr, bq, bq->init_data.ichg_max);
	RET_NZ(bq2562x_set_prechrg_curr, bq, bat_info->precharge_current_ua);
	RET_NZ(bq2562x_set_chrg_volt, bq,
	       bat_info->constant_charge_voltage_max_uv);

	RET_NZ(bq2562x_set_term_curr, bq, bat_info->charge_term_current_ua);

	RET_NZ(bq2562x_set_input_volt_lim, bq, bq->init_data.vlim);

	RET_NZ(bq2562x_set_input_curr_lim, bq, bq->init_data.ilim);
	bq->state.ilim_curr = bq->init_data.ilim;

	power_supply_put_battery_info(bq->charger, bat_info);

	// Enable chargin by default
	RET_NZ(bq2562x_set_charge_enable, bq, 1);

	RET_NZ(bq2562x_start_adc_oneshot, bq);

	bq->state.charging_state = POWER_SUPPLY_STATUS_UNKNOWN;

	return 0;
}

static int bq2562x_parse_dt(struct bq2562x_device *bq,
			    struct power_supply_config *psy_cfg,
			    struct device *dev)
{
	int ret = 0;
	u32 ext_ilim_r = 0;

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

	if (bq->device_id == BQ25622) {
		ret = device_property_read_u32(bq->dev, "ext-ilim-resistor",
					       &ext_ilim_r);
		// using a uA value here requires 64bit math
		// keeping in mA, then scaling it up to uA
		bq->init_data.ext_ilim =
			BQ25622_K_ILIM_mA_MAX / ext_ilim_r * 1000;
		if (ret || ext_ilim_r == 0 || bq->init_data.ext_ilim == 0) {
			return -EINVAL;
		}
	}
	// TODO: add overflow check and return -EINVAL
	ret = device_property_read_u32(bq->dev, "ti,ts-adc-coeff-a",
				       &bq->ts_coeff_a);
	if (ret)
		bq->ts_coeff_a = BQ2562X_TS_ADC_COEFF_A_DEF;

	ret = device_property_read_u32(bq->dev, "ti,ts-adc-coeff-b",
				       &bq->ts_coeff_b);
	if (ret)
		bq->ts_coeff_b = BQ2562X_TS_ADC_COEFF_B_DEF;

	ret = device_property_read_u32(bq->dev, "ti,ts-adc-coeff-scale",
				       &bq->ts_coeff_scale);
	if (ret)
		bq->ts_coeff_scale = BQ2562X_TS_ADC_COEFF_SCALE_DEF;

	dev_info(bq->dev, "TS ADC coefficients a=%u b=%u scale=%u",
		 bq->ts_coeff_a, bq->ts_coeff_b, bq->ts_coeff_scale);

	ret = device_property_read_u32(bq->dev, "bat-low-voltage-microvolt",
				       &bq->init_data.bat_low_v);
	if (ret)
		bq->init_data.bat_low_v = BQ25622_VBAT_UVLOZ_uV;

	bq->ac_detect_gpio =
		devm_gpiod_get_optional(bq->dev, "ac-detect", GPIOD_IN);
	if (IS_ERR(bq->ac_detect_gpio)) {
		ret = PTR_ERR(bq->ac_detect_gpio);
		dev_err(bq->dev, "Failed to get ac detect");
		return ret;
	}

	bq->ce_pin_override =
		device_property_read_bool(bq->dev, "charge-enable-override");
	if (bq->ce_pin_override) {
		dev_info(bq->dev,
			 "ce pin override specified, won't use ce gpio");
		bq->ce_gpio = NULL;
	} else {
		// startup with charging disable
		bq->ce_gpio = devm_gpiod_get_optional(bq->dev, "charge-enable",
						      GPIOD_OUT_LOW);
		if (IS_ERR(bq->ce_gpio)) {
			ret = PTR_ERR(bq->ce_gpio);
			dev_err(bq->dev, "Failed to get ce");
			return ret;
		}
	}

	bq->ce_pin_negate =
		device_property_read_bool(bq->dev, "charge-enable-negate");

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

static void bq2562x_remove(struct i2c_client *client)
{
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
	.remove = bq2562x_remove,
    .id_table = bq2562x_i2c_ids,
};
module_i2c_driver(bq2562x_driver);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("bq2562x charger driver");
MODULE_LICENSE("GPL v2");
