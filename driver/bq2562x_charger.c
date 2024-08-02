// SPDX-License-Identifier: GPL-2.0
// BQ2562X driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/err.h>
#include <linux/reboot.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/usb/phy.h>
#include <linux/workqueue.h>
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
	u32 bat_cap;
};
enum bq2562x_state_enum {
	BQ2562X_STATE_INIT = 0,
	BQ2562X_STATE_IBAT_DIS_ADC1,
	BQ2562X_STATE_IBAT_DIS_ADC2,
	BQ2562X_STATE_OPERATIONAL
};

enum bq2562x_shutdown_type {
	BQ2562X_SHUT_NOOP = 0,
	BQ2562X_SHUT_SHIP,
	BQ2562X_SHUT_SHUTDOWN,
	BQ2562X_SHUT_MAX_VAL = BQ2562X_SHUT_SHUTDOWN
};

struct bq2562x_state {
	enum bq2562x_state_enum state;
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
	int bat_present;
};

struct bq2562x_battery_state {
	int present;
	s32 ts_adc;
	u32 vbat_adc;
	s32 ibat_adc;
	// these ADC measurements are performed with averaging
	u32 vbat_adc_avg;
	s32 ibat_adc_avg;
	// battery SoC section
	int curr_percent;

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
	struct power_supply_battery_info *bat_info;
	struct mutex lock;

	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];
	int device_id;

	struct gpio_desc *ac_detect_gpio;
	struct gpio_desc *ce_gpio;

	// in case we missed a WD interrupt
	// we'll have a safety timer, that triggers a watchdog reset
	struct timer_list manage_timer;
	struct work_struct manage_work;
	struct workqueue_struct *wq;

	struct bq2562x_init_data init_data;
	struct bq2562x_state state;
	struct bq2562x_battery_state bat_state;

	struct dev_ext_attribute shut_type_attr;
	struct dev_ext_attribute force_ce_attr;
	struct dev_ext_attribute force_cd_attr;

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
	bool emit_battery_diag;

	enum bq2562x_shutdown_type shutdown_type;
	bool force_ce;
	bool force_cd;
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
	{BQ2562X_CHRG_CTRL_2, 0x4F},
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
	{BQ2562X_CHRG_CTRL_2, 0x4F},
	{BQ2562X_CHRG_CTRL_3, 0x04},
	{BQ2562X_CHRG_CTRL_4, 0xC0},
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

	// setup managing timer to WD/2
	mod_timer(
		&bq->manage_timer,
		jiffies +
			msecs_to_jiffies(
				bq2562x_watchdog_time[bq->watchdog_timer_reg] /
				2));

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

static int get_power_supply_charging_state(
	struct bq2562x_device *bq, struct bq2562x_state *state,
	struct bq2562x_battery_state *bat_state, bool charge_enabled)
{
	if (!state->chrg_type || (state->chrg_type == BQ2562X_OTG_MODE))
		return POWER_SUPPLY_STATUS_DISCHARGING;
	else if (!state->chrg_status) {
		if (bat_state->ibat_adc == 0 && charge_enabled)
			return POWER_SUPPLY_STATUS_FULL;
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	return POWER_SUPPLY_STATUS_CHARGING;
}

static int bq2562x_update_battery_state(struct bq2562x_device *bq,
					struct bq2562x_state *state,
					struct bq2562x_battery_state *bat_state)
{
	int ri = 0, ri_comp = 0, charging = 0;
	int ri_temp_comp = 100;
	int vocv = 0;

	// update battery presence from charger result
	bat_state->present = state->bat_present;

	if (!bat_state->present && bat_state->ibat_adc_avg != 0) {
		dev_info(bq->dev, "detected battery");
		bat_state->present = state->bat_present = 1;
	}

	switch (bat_state->charging_state) {
	case POWER_SUPPLY_STATUS_FULL:
		bat_state->curr_percent = 100;
		break;
	default:
		charging = bat_state->charging_state ==
					   POWER_SUPPLY_STATUS_CHARGING ?
				   1 :
				   0;
		ri = power_supply_vbat2ri(bq->bat_info, bat_state->vbat_adc_avg,
					  charging);

		if (bq->bat_info->resist_table) {
			ri_temp_comp = power_supply_temp2resist_simple(
				bq->bat_info->resist_table,
				bq->bat_info->resist_table_size,
				bat_state->ts_adc);
		}
		ri_comp = ri * ri_temp_comp / 100;

		// NOTE: ibat is negative when discharging!
		// mA * mOhm = uV
		vocv = bat_state->vbat_adc_avg -
		       bat_state->ibat_adc_avg / 1000 * ri_comp / 1000;

		// this might return -EINVAL if ocv2cap tables are not initialized
		bat_state->curr_percent = power_supply_batinfo_ocv2cap(
			bq->bat_info, vocv, bat_state->ts_adc);
		// don't report 100% while we are actively charging
		// using charger state to report correct value
		// on initial evaluation as well
		if (state->charging_state == POWER_SUPPLY_STATUS_CHARGING) {
			bat_state->curr_percent =
				min(bat_state->curr_percent, 99);
		}
		if (bq->emit_battery_diag) {
			dev_info(
				bq->dev,
				"bat percent from ocv vbat_adc_avg:%u ibat_adc_avg:%d charging:%d ri:%d ts_adc:%u ri_temp_comp:%d ri_comp:%d vocv:%d percent:%d",
				bat_state->vbat_adc_avg,
				bat_state->ibat_adc_avg, charging, ri,
				bat_state->ts_adc, ri_temp_comp, ri_comp, vocv,
				bat_state->curr_percent);
		}
		break;
	}
	return 0;
}

enum BQ2562X_ADC_START_TYPE {
	BQ2562X_ADC_OFF = 0,
	BQ2562X_ADC_START_ONESHOT,
	BQ2562X_ADC_START_AVG,
};

static int bq2562x_update_state(struct bq2562x_device *bq,
				struct bq2562x_state *state,
				struct bq2562x_battery_state *bat_state,
				bool timed,
				enum BQ2562X_ADC_START_TYPE *adc_start)
{
	int chrg_stat_0 = 0, chrg_stat_1 = 0, chrg_flag_0 = 0, chrg_flag_1 = 0;
	int fault_flag_0 = 0, fault_status_0 = 0;
	int ret = 0;
	int val = 0;
	u8 flags[3] = { 0, 0, 0 };
	u8 stats[3] = { 0, 0, 0 };
	bool adc_avg_updated = false;
	bool adc_now_updated = false;
	bool process_wd = false;
	bool charge_enabled = false;

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

	process_wd = (chrg_stat_0 & BQ2562X_WD_STAT) || timed;
	if (process_wd) {
		// process ADC averaging results
		bq2562x_update_adc_avg(bq, bat_state);
		adc_avg_updated = true;
		// Watchdog expire halves the ichg
		// set to original setpoint, and readback verify
		bq2562x_update_ichrg_curr(bq, &state->ichg_curr);
		bq25622_disable_ext_ilim(bq, state->ilim_curr);
		bq2562x_reset_watchdog(bq);
		// start oneshot adc on WD expire
		*adc_start = BQ2562X_ADC_START_ONESHOT;
	}
	if ((chrg_flag_0 & BQ2562X_ADC_DONE) &&
	    (chrg_stat_0 & BQ2562X_ADC_DONE)) {
		BQ2562X_DEBUG(bq->dev, "ADC ready");
		bq2562x_update_adc_now(bq, state, bat_state);
		adc_now_updated = true;
		// Start averaging until the next WD expiry
		*adc_start = BQ2562X_ADC_START_AVG;
	}
	state->ce_status = bq2562x_get_charge_enable(bq);
	state->chrg_type = chrg_stat_1 & BQ2562X_VBUS_STAT_MSK;
	state->online = bq2562x_get_online_status(chrg_stat_1);
	state->chrg_status = chrg_stat_1 & BQ2562X_CHG_STAT_MSK;
	charge_enabled = bq2562x_get_charge_enable(bq);
	state->charging_state = get_power_supply_charging_state(
		bq, state, bat_state, charge_enabled);
	// only update battery state if any measurement has been done
	// this prevents reporting an initial potentially 0% capacity
	if (bat_state->vbat_adc_avg != 0 || bat_state->ibat_adc_avg != 0) {
		bat_state->chrg_status = state->chrg_status;
		bat_state->charging_state = state->charging_state;
	}

	// start adc if charge status changed and not already started
	// on WD expiry
	if (chrg_flag_1 && !process_wd) {
		*adc_start = BQ2562X_ADC_START_ONESHOT;
	}

	// this is safe to update here on initial update
	// as if there's no fault,then all reads 0 (i.e. no change)
	bat_state->health = fault_status_0 & BQ2562X_TEMP_MASK;
	bat_state->overvoltage = fault_status_0 & BQ2562X_BAT_FAULT_STAT;
	state->fault_0 = fault_status_0;

	if (bq->ac_detect_gpio) {
		val = gpiod_get_value_cansleep(bq->ac_detect_gpio);
		BQ2562X_DEBUG(bq->dev, "read ac_detect pin as %d", val);
		state->vbus_status = val;
	}
	// for the initial measurement use the momentary values
	// to not report a bogus 0 percent capacity but still report
	// battery capacity as soon as possible
	if (bat_state->vbat_adc_avg == 0 && bat_state->ibat_adc_avg == 0 &&
	    adc_now_updated) {
		bat_state->vbat_adc_avg = bat_state->vbat_adc;
		bat_state->ibat_adc_avg = bat_state->ibat_adc;
		adc_avg_updated = true;
	}
	// only update battery state if we have up-to-date measurements
	if (adc_avg_updated) {
		bq2562x_update_battery_state(bq, state, bat_state);
	}
	// forcing charge enable/disable with disabling having higher priority
	if (charge_enabled && bq->force_cd) {
		bq2562x_set_charge_enable(bq, 0);
	} else if (!charge_enabled && bq->force_ce) {
		bq2562x_set_charge_enable(bq, 1);
	}
	return 0;
}

static void bq2562x_charger_reset(void *data)
{
	struct bq2562x_device *bq = data;
	// do a REG reset on on driver unload
	regmap_update_bits(bq->regmap, BQ2562X_CHRG_CTRL_2,
			   BQ2562X_CHRG_CTRL2_REG_RST,
			   BQ2562X_CHRG_CTRL2_REG_RST);
}

static void bq2562x_force_ibat_dis(struct bq2562x_device *bq, u8 val)
{
	regmap_update_bits(bq->regmap, BQ2562X_CHRG_CTRL_1,
			   BQ2562X_CHRG_CTRL1_FORCE_IBATDIS,
			   BQ2562X_CHRG_CTRL1_FORCE_IBATDIS * (val ? 1 : 0));
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

	mutex_lock(&bq->lock);
	state = bq->state;
	mutex_unlock(&bq->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = state.charging_state;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
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

	mutex_lock(&bq->lock);
	state = bq->bat_state;
	mutex_unlock(&bq->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = bq->bat_info->technology;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		BQ2562X_DEBUG(bq->dev, "bat get property present:%d",
			      state.present);
		val->intval = state.present;
		break;
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
		BQ2562X_DEBUG(bq->dev, "bat get property capacity:%d",
			      state.curr_percent);
		val->intval = state.curr_percent;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = bq->init_data.bat_cap;
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

static int
bq2562x_initial_charge_enable(struct bq2562x_device *bq,
			      struct bq2562x_state *state,
			      struct bq2562x_battery_state *bat_state)
{
	u8 flags[3] = { 0, 0, 0 };
	int bat_present = 0;
	regmap_bulk_read(bq->regmap, BQ2562X_CHRG_FLAG_0, flags, 3);

	BQ2562X_DEBUG(
		bq->dev,
		"read interrupt flags for init state:%d chrg_flag_0:0x%02x chrg_flag_1:0x%02x fault_flag_0:0x%02x",
		state->state, flags[0], flags[1], flags[2]);

	bq2562x_update_adc_now(bq, state, bat_state);

	bq2562x_force_ibat_dis(bq, state->state != BQ2562X_STATE_IBAT_DIS_ADC2);

	bat_present = state->state == BQ2562X_STATE_IBAT_DIS_ADC2 &&
				      bat_state->vbat_adc >=
					      BQ25622_VBAT_UVLO_uV ?
			      1 :
			      0;
	bq2562x_set_charge_enable(bq, bat_present);

	state->bat_present = bat_present;
	state->state += 1;

	bq2562x_reset_watchdog(bq);
	return 0;
}

static void bq2562x_manage(struct bq2562x_device *bq, bool timed)
{
	int ret = 0;
	struct bq2562x_state state;
	struct bq2562x_battery_state bat_state;
	enum BQ2562X_ADC_START_TYPE adc_type = BQ2562X_ADC_OFF;
	// starting off with the previous state
	mutex_lock(&bq->lock);
	state = bq->state;
	bat_state = bq->bat_state;
	mutex_unlock(&bq->lock);

	// Battery detection on initial startup
	// to determine charge enable and battery presence
	// no need for handling concurrency between invocation from the WD thread an IRQ thread
	// as this MUST finish way before the first WD interrupt
	if (state.state < BQ2562X_STATE_OPERATIONAL) {
		bq2562x_initial_charge_enable(bq, &state, &bat_state);
		mutex_lock(&bq->lock);
		bq->state = state;
		bq->bat_state = bat_state;
		mutex_unlock(&bq->lock);
		bq2562x_start_adc_oneshot(bq);
		return;
	}
	///////////////////////////
	// Operational state below
	ret = bq2562x_update_state(bq, &state, &bat_state, timed, &adc_type);
	if (ret) {
		dev_err(bq->dev, "error updating battery state");
		return;
	}

	if (bq2562x_state_changed(bq, &state)) {
		BQ2562X_DEBUG(bq->dev, "state changed");
		mutex_lock(&bq->lock);
		bq->state = state;
		mutex_unlock(&bq->lock);
		power_supply_changed(bq->charger);
	}
	if (bq2562x_bat_state_changed(bq, &bat_state)) {
		BQ2562X_DEBUG(bq->dev, "battery state changed");
		mutex_lock(&bq->lock);
		bq->bat_state = bat_state;
		mutex_unlock(&bq->lock);
		power_supply_changed(bq->battery);
	}
	// Don't eagerly start ADCs in the interrupt handler, but keep as a last step
	// as it  might result in overlapping IRQs
	if (adc_type == BQ2562X_ADC_START_ONESHOT) {
		bq2562x_start_adc_oneshot(bq);
	} else if (adc_type == BQ2562X_ADC_AVG) {
		bq2562x_start_adc_avg(bq);
	}
}

static irqreturn_t bq2562x_irq_handler_thread(int irq, void *private)
{
	struct bq2562x_device *bq = private;
	BQ2562X_DEBUG(bq->dev, "bq2562x_irq_handler_thread irq:0x%08x pid:%d",
		      irq, current->pid);
	queue_work(bq->wq, &bq->manage_work);
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
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
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
	case BQ2562X_PART_INFO:
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
		return PTR_ERR(bq->charger);

	bq->battery = devm_power_supply_register(bq->dev, &bq2562x_battery_desc,
						 psy_cfg);
	if (IS_ERR(bq->battery))
		return PTR_ERR(bq->battery);
	return 0;
}

static void bq2562x_wd_safety_timer(struct timer_list *timer)
{
	struct bq2562x_device *bq =
		container_of(timer, struct bq2562x_device, manage_timer);
	queue_work(bq->wq, &bq->manage_work);
}

static void bq2562x_wd_safety_timer_work(struct work_struct *work)
{
	struct bq2562x_device *bq =
		container_of(work, struct bq2562x_device, manage_work);
	BQ2562X_DEBUG(bq->dev, "managing device pid:%d", current->pid);
	bq2562x_manage(bq, timer_pending(&bq->manage_timer) == 0);
}

static int bq2562x_map_wd_to_reg(struct bq2562x_device *bq)
{
	int i = 0;
	int wd_reg_val = BQ2562X_WATCHDOG_DIS;
	int wd_max_val = BQ2562X_NUM_WD_VAL - 1;
	if (bq->watchdog_timer >= bq2562x_watchdog_time[wd_max_val])
		wd_reg_val = wd_max_val;
	else {
		for (i = 0; i < wd_max_val; i++) {
			if (bq->watchdog_timer >= bq2562x_watchdog_time[i] &&
			    bq->watchdog_timer < bq2562x_watchdog_time[i + 1]) {
				wd_reg_val = i;
				break;
			}
		}
	}
	if (wd_reg_val == BQ2562X_WATCHDOG_DIS) {
		dev_err(bq->dev,
			"failed to map watchdog timer %d to watchdog reg value",
			bq->watchdog_timer);
		return -EINVAL;
	}
	return wd_reg_val;
}

static int
bq2562x_parse_battery_dt_vbat_to_ri(struct bq2562x_device *bq,
				    const char *const key,
				    struct device_node *battery_np, int *size,
				    struct power_supply_vbat_ri_table **table)
{
	int len, index;
	const __be32 *list = of_get_property(battery_np, key, &len);
	if (list && len) {
		BQ2562X_DEBUG(bq->dev, "parsing table %s", key);
		*size = len / (2 * sizeof(__be32));

		*table = devm_kcalloc(&bq->charger->dev, *size,
				      sizeof(struct power_supply_vbat_ri_table),
				      GFP_KERNEL);

		if (!*table) {
			return -ENOMEM;
		}

		for (index = 0; index < *size; index++) {
			(*table)[index].vbat_uv = be32_to_cpu(*list++);
			(*table)[index].ri_uohm = be32_to_cpu(*list++);
		}
	}
	return 0;
}

// power_supply_core.c doesn't parse crucial attributes
// for the simple battery. this function parses those attributes
static int bq2562x_fixup_battery_info(struct bq2562x_device *bq)
{
	struct device_node *battery_np = NULL;
	struct fwnode_handle *fwnode = NULL;
	int err = 0;
	struct fwnode_reference_args args;

	if (bq->charger->of_node) {
		battery_np = of_parse_phandle(bq->charger->of_node,
					      "monitored-battery", 0);
		if (!battery_np)
			return -ENODEV;

		fwnode = fwnode_handle_get(of_fwnode_handle(battery_np));
	} else if (bq->charger->dev.parent) {
		err = fwnode_property_get_reference_args(
			dev_fwnode(bq->charger->dev.parent),
			"monitored-battery", NULL, 0, 0, &args);
		if (err)
			return err;

		fwnode = args.fwnode;
	}
	if (!fwnode)
		return -ENOENT;

	// internal resistance charging
	fwnode_property_read_u32(
		fwnode, "factory-internal-resistance-charging-micro-ohms",
		&bq->bat_info->factory_internal_resistance_uohm);

	err = bq2562x_parse_battery_dt_vbat_to_ri(
		bq, "vbat-to-internal-resistance-charging-table", battery_np,
		&bq->bat_info->vbat2ri_charging_size,
		&bq->bat_info->vbat2ri_charging);
	if (err)
		goto out_put_node;

	err = bq2562x_parse_battery_dt_vbat_to_ri(
		bq, "vbat-to-internal-resistance-discharging-table", battery_np,
		&bq->bat_info->vbat2ri_discharging_size,
		&bq->bat_info->vbat2ri_discharging);

out_put_node:
	fwnode_handle_put(fwnode);
	of_node_put(battery_np);
	return err;
}

static void bq2562x_cleanup_battery_info(void *data)
{
	struct bq2562x_device *bq = data;
	power_supply_put_battery_info(bq->battery, bq->bat_info);
}

static int bq2562x_hw_init(struct bq2562x_device *bq)
{
	int ret = 0;

	dev_info(bq->dev, "Initializing BQ25620/22 HW ...");

	// do a REG reset first, just be sure that we start from
	// the well knwon state
	RET_NZ(regmap_update_bits, bq->regmap, BQ2562X_CHRG_CTRL_2,
	       BQ2562X_CHRG_CTRL2_REG_RST, BQ2562X_CHRG_CTRL2_REG_RST);

	// enable batfet control with vbus for being able force reset
	// if something goes wrong
	RET_NZ(regmap_update_bits, bq->regmap, BQ2562X_CHRG_CTRL_3,
	       BQ2562X_CHRG_CTRL3_BATFET_CTRL_WVBUS,
	       BQ2562X_CHRG_CTRL3_BATFET_CTRL_WVBUS);

	bq->watchdog_timer_reg = RET_FAIL(bq2562x_map_wd_to_reg, bq);

	RET_NZ(power_supply_get_battery_info, bq->charger, &bq->bat_info);
	RET_NZ(devm_add_action_or_reset, bq->dev, bq2562x_cleanup_battery_info,
	       bq);

	RET_NZ(bq2562x_fixup_battery_info, bq);

	switch (bq->bat_info->technology) {
	case POWER_SUPPLY_TECHNOLOGY_LION:
	case POWER_SUPPLY_TECHNOLOGY_LIPO:
	case POWER_SUPPLY_TECHNOLOGY_LiFe:
	case POWER_SUPPLY_TECHNOLOGY_LiMn:
		dev_info(bq->dev, "battery technology is:%d",
			 bq->bat_info->technology);
		break;
	default:
		dev_err(bq->dev, "unsupported battery technology:%d",
			bq->bat_info->technology);
		return -EINVAL;
	}

	bq->init_data.ichg_max = bq->bat_info->constant_charge_current_max_ua;
	bq->init_data.vreg_max = bq->bat_info->constant_charge_voltage_max_uv;
	bq->init_data.bat_cap = bq->bat_info->charge_full_design_uah;

	bq->bat_state.curr_percent = 0;
	// initialize charge current
	bq->state.ichg_curr = bq->init_data.ichg_max;
	RET_NZ(bq2562x_set_ichrg_curr, bq, bq->init_data.ichg_max);
	RET_NZ(bq2562x_set_prechrg_curr, bq,
	       bq->bat_info->precharge_current_ua);
	RET_NZ(bq2562x_set_chrg_volt, bq,
	       bq->bat_info->constant_charge_voltage_max_uv);

	RET_NZ(bq2562x_set_term_curr, bq, bq->bat_info->charge_term_current_ua);

	RET_NZ(bq2562x_set_input_volt_lim, bq, bq->init_data.vlim);

	RET_NZ(bq2562x_set_input_curr_lim, bq, bq->init_data.ilim);
	bq->state.ilim_curr = bq->init_data.ilim;

	bq->bat_state.charging_state = bq->state.charging_state =
		POWER_SUPPLY_STATUS_UNKNOWN;

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
	if (ret) {
		dev_err(bq->dev, "failed to read watchdog dt property");
		return -EINVAL;
	}

	ret = device_property_read_u32(bq->dev, "ti,shutdown-type",
				       &bq->shutdown_type);
	if (ret) {
		dev_err(bq->dev, "failed to read shutdown type dt property");
		return -EINVAL;
	}
	if (bq->shutdown_type > BQ2562X_SHUT_MAX_VAL) {
		dev_err(bq->dev,
			"invalid shutdown type read from dt property: %u",
			bq->shutdown_type);
		return -EINVAL;
	}

	if (bq->watchdog_timer > BQ2562X_WATCHDOG_MAX ||
	    bq->watchdog_timer < BQ2562X_WATCHDOG_DIS) {
		dev_err(bq->dev, "invalid watchdog timer setting: %d",
			bq->watchdog_timer);
		return -EINVAL;
	}

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

	bq->ac_detect_gpio =
		devm_gpiod_get_optional(bq->dev, "ac-detect", GPIOD_IN);
	if (IS_ERR(bq->ac_detect_gpio)) {
		ret = PTR_ERR(bq->ac_detect_gpio);
		dev_err(bq->dev, "Failed to get ac detect");
		return ret;
	}

	// can be used to emit battery percentage calculation diagnostics
	// that can be used to calibrate a battery
	bq->emit_battery_diag =
		device_property_read_bool(bq->dev, "emit-battery-diag");

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

#define BQ2562X_SYSFS_STR(name)                                     \
	static const char *const bq2562x_sysfs_str_##name = #name;  \
	static const size_t const bq2562x_sysfs_str_##name##_size = \
		sizeof(#name) - 1

BQ2562X_SYSFS_STR(noop);
BQ2562X_SYSFS_STR(ship);
BQ2562X_SYSFS_STR(shutdown);
BQ2562X_SYSFS_STR(true);
BQ2562X_SYSFS_STR(false);

static int bq2562x_power_off_handler(struct sys_off_data *data)
{
	struct bq2562x_device *bq = data->cb_data;
	int ret = 0;
	u8 mode = 0;
	regcache_cache_bypass(bq->regmap, true);
	switch (bq->shutdown_type) {
	case BQ2562X_SHUT_SHIP:
		mode = BQ2562X_CHRG_CTRL3_BATFET_CTRL_SHIP;
		break;
	case BQ2562X_SHUT_SHUTDOWN:
		mode = BQ2562X_CHRG_CTRL3_BATFET_CTRL_SHUT;
		break;
	default:
		return NOTIFY_OK;
	}
	dev_info(bq->dev, "shutting down with shutdown type: %s",
		 mode == BQ2562X_SHUT_SHIP ? bq2562x_sysfs_str_ship :
					     bq2562x_sysfs_str_shutdown);

	ret = regmap_update_bits(bq->regmap, BQ2562X_CHRG_CTRL_3,
				 BQ2562X_CHRG_CTRL3_BATFET_CTRL_WVBUS |
					 BQ2562X_CHRG_CTRL3_BATFET_CTRL,
				 BQ2562X_CHRG_CTRL3_BATFET_CTRL_WVBUS | mode);
	if (ret < 0) {
		dev_err(bq->dev,
			"failed to set BATFET_CTRL register with error:%d",
			ret);
		return NOTIFY_BAD;
	}
	return NOTIFY_OK;
}

static ssize_t bq2562x_sysfs_shutdown_type_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct bq2562x_device *bq = dev_get_drvdata(dev);
	const char *msg = "unknown";
	switch (bq->shutdown_type) {
	case BQ2562X_SHUT_NOOP:
		msg = bq2562x_sysfs_str_noop;
		break;
	case BQ2562X_SHUT_SHIP:
		msg = bq2562x_sysfs_str_ship;
		break;
	case BQ2562X_SHUT_SHUTDOWN:
		msg = bq2562x_sysfs_str_shutdown;
		break;
	}
	return sprintf(buf, "%s\n", msg);
}

static ssize_t bq2562x_sysfs_shutdown_type_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	struct bq2562x_device *bq = dev_get_drvdata(dev);
	ssize_t ret = count;
	enum bq2562x_shutdown_type shut;
	if (strncmp(buf, bq2562x_sysfs_str_noop,
		    min(bq2562x_sysfs_str_noop_size, count)) == 0) {
		shut = BQ2562X_SHUT_NOOP;
	} else if (strncmp(buf, bq2562x_sysfs_str_ship,
			   min(bq2562x_sysfs_str_ship_size, count)) == 0) {
		shut = BQ2562X_SHUT_SHIP;
	} else if (strncmp(buf, bq2562x_sysfs_str_shutdown,
			   min(bq2562x_sysfs_str_shutdown_size, count)) == 0) {
		shut = BQ2562X_SHUT_SHUTDOWN;
	} else {
		dev_warn(bq->dev,
			 "invalid value received for shutdown type from sysfs");
		ret = -EINVAL;
	}
	if (ret == count) {
		dev_info(bq->dev, "setting shutdown type to %d", shut);
		bq->shutdown_type = shut;
	}
	return ret;
}

static ssize_t bq2562x_sysfs_boolean_show(char *buf, bool *val)
{
	return sprintf(buf, "%s\n", *val ? "true" : "false");
}

static ssize_t bq2562x_sysfs_bool_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count,
					bool *val)
{
	struct bq2562x_device *bq = dev_get_drvdata(dev);
	ssize_t ret = count;
	bool curr_val;
	bool notify = false;
	if (strncmp(buf, bq2562x_sysfs_str_true,
		    min(bq2562x_sysfs_str_true_size, count)) == 0) {
		curr_val = true;
	} else if (strncmp(buf, bq2562x_sysfs_str_false,
			   min(bq2562x_sysfs_str_false_size, count)) == 0) {
		curr_val = false;
	} else {
		dev_warn(bq->dev,
			 "invalid value received for boolean value from sysfs");
		ret = -EINVAL;
	}
	if (ret == count) {
		notify = *val != curr_val;
		*val = curr_val;
	}
	if (notify) {
		queue_work(bq->wq, &bq->manage_work);
	}
	return ret;
}

static ssize_t bq2562x_sysfs_force_ce_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct bq2562x_device *bq = dev_get_drvdata(dev);
	return bq2562x_sysfs_boolean_show(buf, &bq->force_ce);
}

static ssize_t bq2562x_sysfs_force_ce_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct bq2562x_device *bq = dev_get_drvdata(dev);
	return bq2562x_sysfs_bool_store(dev, attr, buf, count, &bq->force_ce);
}

static ssize_t bq2562x_sysfs_force_cd_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct bq2562x_device *bq = dev_get_drvdata(dev);
	return bq2562x_sysfs_boolean_show(buf, &bq->force_cd);
}

static ssize_t bq2562x_sysfs_force_cd_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct bq2562x_device *bq = dev_get_drvdata(dev);
	return bq2562x_sysfs_bool_store(dev, attr, buf, count, &bq->force_cd);
}

static void bq2562x_cleanup_attr(void *data)
{
	struct dev_ext_attribute *attr = data;
	struct bq2562x_device *bq = attr->var;
	device_remove_file(bq->dev, &attr->attr);
}

static int bq2562x_register_attr(struct bq2562x_device *bq,
				 struct dev_ext_attribute *attr)
{
	int ret = device_create_file(bq->dev, &attr->attr);
	if (ret < 0) {
		dev_err(bq->dev,
			"failed to register sysfs entry %s with code:%d",
			attr->attr.attr.name, ret);
		return ret;
	}
	attr->var = bq;
	RET_NZ(devm_add_action_or_reset, bq->dev, bq2562x_cleanup_attr, attr);
	return ret;
}

static void bq2562x_timer_cleanup(void *data)
{
	struct bq2562x_device *bq = data;
	del_timer_sync(&bq->manage_timer);
}

static void bq2562x_cleanup_workqueue(void *data)
{
	struct bq2562x_device *bq = data;
	destroy_workqueue(bq->wq);
}

static void bq2562x_cleanup_work(void *data)
{
	struct bq2562x_device *bq = data;
	cancel_work_sync(&bq->manage_work);
}

DEVICE_ATTR(shutdown_type, 0644, bq2562x_sysfs_shutdown_type_show,
	    bq2562x_sysfs_shutdown_type_store);
DEVICE_ATTR(force_charge_enable, 0644, bq2562x_sysfs_force_ce_show,
	    bq2562x_sysfs_force_ce_store);
DEVICE_ATTR(force_charge_disable, 0644, bq2562x_sysfs_force_cd_show,
	    bq2562x_sysfs_force_cd_store);

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

	bq->regmap = devm_regmap_init_i2c(
		client, bq->device_id == BQ25620 ? &bq25620_regmap_config :
						   &bq25622_regmap_config);
	if (IS_ERR(bq->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		return PTR_ERR(bq->regmap);
	}

	i2c_set_clientdata(client, bq);
	dev_set_drvdata(dev, bq);

	RET_NZ(devm_add_action_or_reset, dev, bq2562x_charger_reset, bq);

	RET_FAIL(devm_register_sys_off_handler, dev,
		 SYS_OFF_MODE_POWER_OFF_PREPARE, SYS_OFF_PRIO_FIRMWARE,
		 bq2562x_power_off_handler, bq);

	bq->wq = create_singlethread_workqueue("manage_wq");
	if (IS_ERR(bq->wq)) {
		ret = PTR_ERR(bq->wq);
		dev_err(bq->dev, "failed to create workqueue:%d", ret);
		return ret;
	}
	RET_FAIL(devm_add_action_or_reset, bq->dev, bq2562x_cleanup_workqueue,
		 bq);

	INIT_WORK(&bq->manage_work, bq2562x_wd_safety_timer_work);

	RET_FAIL(devm_add_action_or_reset, bq->dev, bq2562x_cleanup_work, bq);

	timer_setup(&bq->manage_timer, bq2562x_wd_safety_timer, 0);

	RET_FAIL(devm_add_action_or_reset, bq->dev, bq2562x_timer_cleanup, bq);

	// need to allocate power supply before registering interrupt
	RET_NZ(bq2562x_parse_dt, bq, &psy_cfg, dev);

	RET_NZ(bq2562x_power_supply_init, bq, &psy_cfg, dev);

	RET_NZ(bq2562x_hw_init, bq);
	// last step to setup IRQ, as it can be called as soon as
	// this returns, resulting in uninitialized state
	if (client->irq) {
		RET_FAIL(devm_request_threaded_irq, dev, client->irq, NULL,
			 bq2562x_irq_handler_thread,
			 IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			 dev_name(&client->dev), bq);
	}

	bq->shut_type_attr.attr = dev_attr_shutdown_type;
	bq->force_ce_attr.attr = dev_attr_force_charge_enable;
	bq->force_cd_attr.attr = dev_attr_force_charge_disable;
	RET_NZ(bq2562x_register_attr, bq, &bq->shut_type_attr);
	RET_NZ(bq2562x_register_attr, bq, &bq->force_ce_attr);
	RET_NZ(bq2562x_register_attr, bq, &bq->force_cd_attr);

	RET_NZ(bq2562x_reset_watchdog, bq);
	RET_NZ(bq2562x_start_adc_oneshot, bq);
	return 0;
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
