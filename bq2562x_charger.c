// SPDX-License-Identifier: GPL-2.0
// BQ2562X driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/usb/phy.h>

#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include "bq2562x_charger.h"

#define BQ2562X_NUM_WD_VAL	4

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
	u8 chrg_type;
	u8 health;
	u8 chrg_fault;
	u8 vsys_status;
	u8 vbus_status;
	u8 fault_0;
	u8 fault_1;
	u32 vbat_adc;
	u32 vbus_adc;
	u32 ibat_adc;
	u32 ibus_adc;
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

	struct usb_phy *usb2_phy;
	struct usb_phy *usb3_phy;
	struct notifier_block usb_nb;
	struct work_struct usb_work;
	unsigned long usb_event;
	struct regmap *regmap;

	char model_name[I2C_NAME_SIZE];
	int device_id;

	struct gpio_desc *ac_detect_gpio;
	struct gpio_desc *ce_gpio;

	struct bq2562x_init_data init_data;
	struct bq2562x_state state;
	int watchdog_timer;
};

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
	{BQ2562X_CHRG_CTRL_0, 0xA1},
	{BQ2562X_CHRG_CTRL_1, 0x4E},
	{BQ2562X_CHRG_CTRL_2, 0x04},
	{BQ2562X_CHRG_CTRL_3, 0xC4},
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
	{BQ2562X_CHRG_CTRL_0, 0xA1},
	{BQ2562X_CHRG_CTRL_1, 0x4E},
	{BQ2562X_CHRG_CTRL_2, 0x04},
	{BQ2562X_CHRG_CTRL_3, 0xC4},
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

static int bq2562x_usb_notifier(struct notifier_block *nb, unsigned long val,
				void *priv)
{
	struct bq2562x_device *bq =
			container_of(nb, struct bq2562x_device, usb_nb);

	bq->usb_event = val;
	queue_work(system_power_efficient_wq, &bq->usb_work);

	return NOTIFY_OK;
}

static void bq2562x_usb_work(struct work_struct *data)
{
	struct bq2562x_device *bq =
			container_of(data, struct bq2562x_device, usb_work);

	switch (bq->usb_event) {
	case USB_EVENT_ID:
		break;

	case USB_EVENT_NONE:
		power_supply_changed(bq->charger);
		break;
	}
}

static bool bq2562x_get_charge_enable(struct bq2562x_device *bq)
{
	int ret;
	int ce_pin;
	int charger_enable;
	int chrg_ctrl_0;

	if (bq->ce_gpio)
		ce_pin = gpiod_get_value_cansleep(bq->ce_gpio);

	ret = regmap_read(bq->regmap, BQ2562X_CHRG_CTRL_0, &chrg_ctrl_0);
	if (ret)
		return ret;


	charger_enable = chrg_ctrl_0 & BQ2562X_CHRG_EN;

	if (charger_enable) {
		if (bq->ce_gpio && !ce_pin)
			return true;
	}
	return false;
}

static int bq2562x_set_charge_enable(struct bq2562x_device *bq, int val)
{
	if (bq->ce_gpio){
		gpiod_set_value_cansleep(bq->ce_gpio, !val);
	}

	return regmap_update_bits(bq->regmap, BQ2562X_CHRG_CTRL_0,
					BQ2562X_CHRG_EN, val);
}

static int bq2562x_get_vbat_adc(struct bq2562x_device *bq)
{
	int ret;
	int vbat_adc_lsb, vbat_adc_msb;
	u16 vbat_adc;

	ret = regmap_read(bq->regmap, BQ2562X_ADC_VBAT_LSB, &vbat_adc_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_ADC_VBAT_MSB, &vbat_adc_msb);
	if (ret)
		return ret;

	vbat_adc = ((vbat_adc_msb << 8) | vbat_adc_lsb) >> BQ2562X_ADC_VBAT_MOVE_STEP;

	return vbat_adc * BQ2562X_ADC_VBAT_STEP_uV;
}

static int bq2562x_get_vbus_adc(struct bq2562x_device *bq)
{
	int ret;
	int vbus_adc_lsb, vbus_adc_msb;
	u16 vbus_adc;

	ret = regmap_read(bq->regmap, BQ2562X_ADC_VBUS_LSB, &vbus_adc_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_ADC_VBUS_MSB, &vbus_adc_msb);
	if (ret)
		return ret;



	vbus_adc = ((vbus_adc_msb << 8) | vbus_adc_lsb) >> BQ2562X_ADC_VBUS_MOVE_STEP;
	return vbus_adc * BQ2562X_ADC_VBUS_STEP_uV;
}

static int bq2562x_get_ibat_adc(struct bq2562x_device *bq)
{
	int ret;
	int ibat_adc_lsb, ibat_adc_msb;
	u16 ibat_adc;

	ret = regmap_read(bq->regmap, BQ2562X_ADC_IBAT_LSB, &ibat_adc_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_ADC_IBAT_MSB, &ibat_adc_msb);
	if (ret)
		return ret;

	ibat_adc = ((ibat_adc_msb << 8) | ibat_adc_lsb) >> BQ2562X_ADC_IBAT_MOVE_STEP;

	return ibat_adc * BQ2562X_ADC_IBAT_STEP_uV;
}

static int bq2562x_get_ibus_adc(struct bq2562x_device *bq)
{
	int ret;
	int ibus_adc_lsb, ibus_adc_msb;
	int ibus_adc;

	ret = regmap_read(bq->regmap, BQ2562X_ADC_IBUS_LSB, &ibus_adc_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_ADC_IBUS_MSB, &ibus_adc_msb);
	if (ret)
		return ret;

	ibus_adc = ((ibus_adc_msb << 8) | ibus_adc_lsb) >> BQ2562X_ADC_IBUS_MOVE_STEP;

	return ibus_adc * BQ2562X_ADC_CURR_STEP_uA;
}

static int bq2562x_get_term_curr(struct bq2562x_device *bq)
{
	int ret;
	int iterm_lsb, iterm_msb;
	u16 iterm;

	ret = regmap_read(bq->regmap, BQ2562X_TERM_CTRL_LSB, &iterm_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_TERM_CTRL_MSB, &iterm_msb);
	if (ret)
		return ret;

	iterm = ((iterm_msb << 8) | iterm_lsb) >> BQ2562X_ITERM_MOVE_STEP;

	return iterm * BQ2562X_TERMCHRG_CURRENT_STEP_uA;
}

static int bq2562x_get_prechrg_curr(struct bq2562x_device *bq)
{
	int ret;
	int prechrg_curr_lsb, prechrg_curr_msb;
	u16 prechrg_curr;

	ret = regmap_read(bq->regmap, BQ2562X_PRECHRG_CTRL_LSB, &prechrg_curr_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_PRECHRG_CTRL_MSB, &prechrg_curr_msb);
	if (ret)
		return ret;

	prechrg_curr = ((prechrg_curr_msb << 8) | prechrg_curr_lsb) >> BQ2562X_PRECHRG_MOVE_STEP;

	return prechrg_curr * BQ2562X_PRECHRG_CURRENT_STEP_uA;
}

static int bq2562x_get_ichg_curr(struct bq2562x_device *bq)
{
	int ret;
	int ichg, ichg_lsb, ichg_msb;

	ret = regmap_read(bq->regmap, BQ2562X_CHRG_I_LIM_MSB, &ichg_msb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_CHRG_I_LIM_LSB, &ichg_lsb);
	if (ret)
		return ret;

	ichg = ((ichg_msb << 8) | ichg_lsb) >> BQ2562X_ICHG_MOVE_STEP;

	return ichg * BQ2562X_ICHRG_CURRENT_STEP_uA;
}

static int bq2562x_set_term_curr(struct bq2562x_device *bq, int term_current)
{
	int reg_val;
	int term_curr_msb, term_curr_lsb;
	int ret;

	term_current = clamp(term_current, BQ2562X_TERMCHRG_I_MIN_uA,
					BQ2562X_TERMCHRG_I_MAX_uA);

	reg_val = (term_current / BQ2562X_TERMCHRG_CURRENT_STEP_uA) << BQ2562X_ITERM_MOVE_STEP;

	term_curr_msb = (reg_val >> 8) & 0xff;
	ret = regmap_write(bq->regmap, BQ2562X_TERM_CTRL_MSB, term_curr_msb);
	if (ret)
		return ret;

	term_curr_lsb = reg_val & 0xff;

	return regmap_write(bq->regmap, BQ2562X_TERM_CTRL_LSB, term_curr_lsb);
}

static int bq2562x_set_prechrg_curr(struct bq2562x_device *bq, int pre_current)
{
	int reg_val;
	int prechrg_curr_msb, prechrg_curr_lsb;
	int ret;

	pre_current = clamp(pre_current, BQ2562X_PRECHRG_I_MIN_uA,
					BQ2562X_PRECHRG_I_MAX_uA);

	bq2562x_set_charge_enable(bq, 0);

	reg_val = (pre_current / BQ2562X_PRECHRG_CURRENT_STEP_uA) << BQ2562X_PRECHRG_MOVE_STEP;

	prechrg_curr_msb = (reg_val >> 8) & 0xff;
	ret = regmap_write(bq->regmap, BQ2562X_PRECHRG_CTRL_MSB, prechrg_curr_msb);
	if (ret)
		return ret;

	prechrg_curr_lsb = reg_val & 0xff;

	ret = regmap_write(bq->regmap, BQ2562X_PRECHRG_CTRL_LSB, prechrg_curr_lsb);
	if (ret)
		return ret;

	return bq2562x_set_charge_enable(bq, 1);
}


static int bq2562x_set_ichrg_curr(struct bq2562x_device *bq, int chrg_curr)
{
	int chrg_curr_max = bq->init_data.ichg_max;
	int ichg, ichg_msb, ichg_lsb;
	int ret;

	chrg_curr = clamp(chrg_curr, BQ2562X_ICHRG_I_MIN_uA,
					chrg_curr_max);

	bq2562x_set_charge_enable(bq, 0);

	ichg = ((chrg_curr / BQ2562X_ICHRG_CURRENT_STEP_uA) << BQ2562X_ICHG_MOVE_STEP);
	ichg_msb = (ichg >> 8) & 0xff;
	ret = regmap_write(bq->regmap, BQ2562X_CHRG_I_LIM_MSB, ichg_msb);
	if (ret)
		return ret;

	ichg_lsb = ichg & 0xff;

	ret = regmap_write(bq->regmap, BQ2562X_CHRG_I_LIM_LSB, ichg_lsb);
	if (ret)
		return ret;

	return bq2562x_set_charge_enable(bq, 1);
}

static int bq2562x_set_chrg_volt(struct bq2562x_device *bq, int chrg_volt)
{
	int chrg_volt_max = bq->init_data.vreg_max;
	int chrg_volt_lsb, chrg_volt_msb, vlim;
	int ret;

	chrg_volt = clamp(chrg_volt, BQ2562X_VREG_V_MIN_uV, chrg_volt_max);

	vlim = (chrg_volt / BQ2562X_VREG_V_STEP_uV) << BQ2562X_CHRG_V_LIM_MOVE_STEP;
	chrg_volt_msb = (vlim >> 8) & 0xff;
	ret = regmap_write(bq->regmap, BQ2562X_CHRG_V_LIM_MSB, chrg_volt_msb);
	if (ret)
		return ret;

	chrg_volt_lsb = vlim & 0xff;

	return regmap_write(bq->regmap, BQ2562X_CHRG_V_LIM_LSB, chrg_volt_lsb);
}

static int bq2562x_get_chrg_volt(struct bq2562x_device *bq)
{
	int ret;
	int chrg_volt_lsb, chrg_volt_msb, chrg_volt;

	ret = regmap_read(bq->regmap, BQ2562X_CHRG_V_LIM_LSB, &chrg_volt_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_CHRG_V_LIM_MSB, &chrg_volt_msb);
	if (ret)
		return ret;

	chrg_volt = ((chrg_volt_msb << 8) | chrg_volt_lsb) >> BQ2562X_CHRG_V_LIM_MOVE_STEP;

	return chrg_volt * BQ2562X_VREG_V_STEP_uV;
}

static int bq2562x_set_input_volt_lim(struct bq2562x_device *bq, int vindpm)
{
	int ret;
	int vlim_lsb, vlim_msb;
	int vlim;

	vindpm = clamp(vindpm, BQ2562X_VINDPM_V_MIN_uV,
						BQ2562X_VINDPM_V_MAX_uV);

	vlim = (vindpm / BQ2562X_VINDPM_STEP_uV) << BQ2562X_INPUT_V_LIM_MOVE_STEP;

	vlim_msb = (vlim >> 8) & 0xff;

	ret = regmap_write(bq->regmap, BQ2562X_INPUT_V_LIM_MSB, vlim_msb);
	if (ret)
		return ret;

	vlim_lsb = vlim & 0xff;

	return regmap_write(bq->regmap, BQ2562X_INPUT_V_LIM_LSB, vlim_lsb);
}

static int bq2562x_get_input_volt_lim(struct bq2562x_device *bq)
{
	int ret;
	int vlim;
	int vlim_lsb, vlim_msb;

	ret = regmap_read(bq->regmap, BQ2562X_INPUT_V_LIM_LSB, &vlim_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_INPUT_V_LIM_MSB, &vlim_msb);
	if (ret)
		return ret;

	vlim = ((vlim_msb << 8) | vlim_lsb) >> BQ2562X_INPUT_V_LIM_MOVE_STEP;

	return vlim * BQ2562X_VINDPM_STEP_uV;
}

static int bq2562x_set_input_curr_lim(struct bq2562x_device *bq, int iindpm)
{
	int ret;
	int ilim, ilim_lsb, ilim_msb;

	iindpm = clamp(iindpm, BQ2562X_IINDPM_I_MIN_uA,
						BQ2562X_IINDPM_I_MAX_uA);

	ilim = (iindpm / BQ2562X_IINDPM_STEP_uA) << BQ2562X_INPUT_I_LIM__MOVE_STEP;

	ilim_lsb = ilim & 0xff;
	ret = regmap_write(bq->regmap, BQ2562X_INPUT_I_LIM_LSB, ilim_lsb);
	if (ret)
		return ret;

	ilim_msb = (ilim >> 8) & 0xff;
	ret = regmap_write(bq->regmap, BQ2562X_INPUT_I_LIM_MSB, ilim_msb);

	return ret;
}

static int bq2562x_get_input_curr_lim(struct bq2562x_device *bq)
{
	int ret;
	int ilim_msb, ilim_lsb;
	u16 ilim;

	ret = regmap_read(bq->regmap, BQ2562X_INPUT_I_LIM_LSB, &ilim_lsb);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_INPUT_I_LIM_MSB, &ilim_msb);
	if (ret)
		return ret;

	ilim = ((ilim_msb << 8) | ilim_lsb) >> BQ2562X_INPUT_I_LIM__MOVE_STEP;

	return ilim * BQ2562X_IINDPM_STEP_uA;
}

static int bq2562x_get_online_status(struct bq2562x_device *bq)
{
	int ret;
	int chrg_stat_1;
	int online_status;

	ret = regmap_update_bits(bq->regmap, BQ2562X_CHRG_STAT_1,
				BQ2562X_VBUS_STAT_MSK, BQ2562X_UNKNOWN_500MA);
	if (ret)
		return ret;

	ret = regmap_read(bq->regmap, BQ2562X_CHRG_STAT_1, &chrg_stat_1);
	if (ret)
		return ret;

	online_status = chrg_stat_1 & BQ2562X_VBUS_STAT_MSK;

	if ((online_status == BQ2562X_USB_SDP) || (online_status == BQ2562X_OTG_MODE))
		online_status = 0;
	else
		online_status = 1;

	return online_status;
}

static int bq2562x_get_state(struct bq2562x_device *bq,
			     struct bq2562x_state *state)
{
	int chrg_stat_0, chrg_stat_1;
	int fault_0, fault_1;
	int ret;
	int val = 0;

	ret = regmap_update_bits(bq->regmap, BQ2562X_ADC_CTRL,
				 BQ2562X_ADC_EN, BQ2562X_ADC_EN);
	if (ret)
		return ret;

	if (bq->ac_detect_gpio) {
		val = gpiod_get_value_cansleep(bq->ac_detect_gpio);
		state->vbus_status = val;
	}

	ret = regmap_read(bq->regmap, BQ2562X_CHRG_STAT_1, &chrg_stat_1);
	if (ret)
		return ret;

	if (bq2562x_get_charge_enable(bq))
		state->chrg_status = chrg_stat_1 & BQ2562X_CHG_STAT_MSK;
	else
		state->chrg_status = BQ2562X_NOT_CHRGING;

	state->chrg_type = chrg_stat_1 & BQ2562X_VBUS_STAT_MSK;

	ret = regmap_read(bq->regmap, BQ2562X_FAULT_STAT_0, &fault_0);
	if (ret)
		return ret;

	state->health = fault_0 & BQ2562X_TEMP_MASK;

	state->fault_0 = fault_0;

	ret = regmap_read(bq->regmap, BQ2562X_FAULT_STAT_0, &fault_1);
	if (ret)
		return ret;

	state->fault_1 = fault_1;

	ret = regmap_read(bq->regmap, BQ2562X_CHRG_STAT_0, &chrg_stat_0);
	if (ret)
		return ret;

	state->online = bq2562x_get_online_status(bq);

	state->vbat_adc = bq2562x_get_vbat_adc(bq);

	state->vbus_adc = bq2562x_get_vbus_adc(bq);

	state->ibat_adc = bq2562x_get_ibat_adc(bq);

	state->ibus_adc = bq2562x_get_ibus_adc(bq);

	return 0;
}

static void bq2562x_charger_reset(void *data)
{
	struct bq2562x_device *bq = data;

	if (!IS_ERR_OR_NULL(bq->usb2_phy))
		usb_unregister_notifier(bq->usb2_phy, &bq->usb_nb);

	if (!IS_ERR_OR_NULL(bq->usb3_phy))
		usb_unregister_notifier(bq->usb3_phy, &bq->usb_nb);
}

static int bq2562x_set_property(struct power_supply *psy,
		enum power_supply_property prop,
		const union power_supply_propval *val)
{
	struct bq2562x_device *bq = power_supply_get_drvdata(psy);
	int ret = -EINVAL;

	switch (prop) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq2562x_set_input_curr_lim(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq2562x_set_chrg_volt(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = bq2562x_set_ichrg_curr(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = bq2562x_set_prechrg_curr(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = bq2562x_set_term_curr(bq, val->intval);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq2562x_set_input_volt_lim(bq, val->intval);
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

	ret = bq2562x_get_state(bq, &state);
	if (ret)
		return ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (!state.chrg_type || (state.chrg_type == BQ2562X_OTG_MODE))
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (!state.chrg_status) {
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			if (bq2562x_get_chrg_volt(bq) >=
				(state.vbat_adc - BQ2562X_ADC_VBAT_STEP_uV))
				val->intval = POWER_SUPPLY_STATUS_FULL;
		}
		//else if (state.chrg_status == BQ2562X_TERM_CHRG)
		//	val->intval = POWER_SUPPLY_STATUS_FULL;
		else
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
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
		if (!state.chrg_type) {
			val->intval = POWER_SUPPLY_USB_TYPE_UNKNOWN;
			break;
		}
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
		if (state.fault_1 & (BQ2562X_OTG_FAULT_STAT | BQ2562X_SYS_FAULT_STAT))
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
		ret = bq2562x_get_ichg_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = bq2562x_get_chrg_volt(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = bq2562x_get_prechrg_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = bq2562x_get_term_curr(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = state.vbus_adc;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = state.ibus_adc;
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = bq2562x_get_input_volt_lim(bq);
		if (ret < 0)
			return ret;

		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = bq2562x_get_input_curr_lim(bq);
		if (ret < 0)
			return ret;

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

	ret = bq2562x_get_state(bq, &state);
	if (ret)
		return ret;

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

	return memcmp(&old_state, new_state,
				sizeof(struct bq2562x_state)) != 0;
}

static irqreturn_t bq2562x_irq_handler_thread(int irq, void *private)
{
	struct bq2562x_device *bq = private;
	struct bq2562x_state state;
	int ret;

	ret = bq2562x_get_state(bq, &state);
	if (ret < 0)
		goto irq_out;

	if (!bq2562x_state_changed(bq, &state))
		goto irq_out;

	mutex_lock(&bq->lock);
	bq->state = state;
	mutex_unlock(&bq->lock);

	power_supply_changed(bq->charger);

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

};

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
	case BQ2562X_CHRG_STAT_0...BQ2562X_FAULT_FLAG_0:
	case BQ2562X_ADC_IBUS_MSB...BQ2562X_ADC_TDIE_LSB:
	case BQ2562X_CHRG_CTRL_0:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config bq25620_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ2562X_PART_INFO,
	.reg_defaults	= bq25620_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25620_reg_defs),
	.cache_type = REGCACHE_FLAT,
	.volatile_reg = bq2562x_is_volatile_reg,
};

static const struct regmap_config bq25622_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = BQ2562X_PART_INFO,
	.reg_defaults	= bq25622_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(bq25622_reg_defs),
	.cache_type = REGCACHE_FLAT,
	.volatile_reg = bq2562x_is_volatile_reg,
};


static int bq2562x_power_supply_init(struct bq2562x_device *bq,
		struct power_supply_config *psy_cfg, struct device *dev)
{
	bq->charger = devm_power_supply_register(bq->dev,
						 &bq2562x_power_supply_desc,
						 psy_cfg);
	if (IS_ERR(bq->charger))
		return -EINVAL;

	bq->battery = devm_power_supply_register(bq->dev,
						      &bq2562x_battery_desc,
						      psy_cfg);
	if (IS_ERR(bq->battery))
		return -EINVAL;
	return 0;
}

static int bq2562x_hw_init(struct bq2562x_device *bq)
{
	struct power_supply_battery_info * bat_info;
	struct power_supply_battery_info  default_bat_info = {};
	int wd_reg_val = BQ2562X_WATCHDOG_DIS;
	int wd_max_val = BQ2562X_NUM_WD_VAL - 1;
	int ret = 0;
	int i;

	if (bq->watchdog_timer) {
		if (bq->watchdog_timer >= bq2562x_watchdog_time[wd_max_val])
			wd_reg_val = wd_max_val;
		else {
			for (i = 0; i < wd_max_val; i++) {
				if (bq->watchdog_timer > bq2562x_watchdog_time[i] &&
				    bq->watchdog_timer < bq2562x_watchdog_time[i + 1]) {
					wd_reg_val = i;
					break;
				}
			}
		}
	}

	ret = regmap_update_bits(bq->regmap, BQ2562X_CHRG_CTRL_0,
				 BQ2562X_WATCHDOG_MASK, wd_reg_val);
	if (ret)
		return ret;

	ret = power_supply_get_battery_info(bq->charger, &bat_info); 
	if (ret) {
		dev_warn(bq->dev, "battery info missing, default values will be applied\n");
		bat_info = &default_bat_info;
		default_bat_info.constant_charge_current_max_ua =
							BQ2562X_ICHRG_I_DEF_uA;

		default_bat_info.constant_charge_voltage_max_uv =
							BQ2562X_VREG_V_DEF_uV;

		default_bat_info.precharge_current_ua = BQ2562X_PRECHRG_I_DEF_uA;
		default_bat_info.charge_term_current_ua = BQ2562X_TERMCHRG_I_DEF_uA;
		bq->init_data.ichg_max = BQ2562X_ICHRG_I_MAX_uA;
		bq->init_data.vreg_max = BQ2562X_VREG_V_MAX_uV;
	} else {
		bq->init_data.ichg_max =
				bat_info->constant_charge_current_max_ua;

		bq->init_data.vreg_max =
				bat_info->constant_charge_voltage_max_uv;
	}

	ret = bq2562x_set_ichrg_curr(bq,
				bat_info->constant_charge_current_max_ua);
	if (ret)
		return ret;

	ret = bq2562x_set_prechrg_curr(bq, bat_info->precharge_current_ua);
	if (ret)
		return ret;

	ret = bq2562x_set_chrg_volt(bq,
				bat_info->constant_charge_voltage_max_uv);
	if (ret)
		return ret;

	ret = bq2562x_set_term_curr(bq, bat_info->charge_term_current_ua);
	if (ret)
		return ret;

	ret = bq2562x_set_input_volt_lim(bq, bq->init_data.vlim);
	if (ret)
		return ret;

	ret = bq2562x_set_input_curr_lim(bq, bq->init_data.ilim);
	if (ret)
		return ret;

	power_supply_put_battery_info(bq->charger, bat_info);

	return 0;
}

static int bq2562x_parse_dt(struct bq2562x_device *bq,
		struct power_supply_config *psy_cfg, struct device *dev)
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

	ret = device_property_read_u32(bq->dev,
				       "input-voltage-limit-microvolt",
				       &bq->init_data.vlim);
	if (ret)
		bq->init_data.vlim = BQ2562X_VINDPM_DEF_uV;

	if (bq->init_data.vlim > BQ2562X_VINDPM_V_MAX_uV ||
	    bq->init_data.vlim < BQ2562X_VINDPM_V_MIN_uV)
		return -EINVAL;

	ret = device_property_read_u32(bq->dev,
				       "input-current-limit-microamp",
				       &bq->init_data.ilim);
	if (ret)
		bq->init_data.ilim = BQ2562X_IINDPM_DEF_uA;

	if (bq->init_data.ilim > BQ2562X_IINDPM_I_MAX_uA ||
	    bq->init_data.ilim < BQ2562X_IINDPM_I_MIN_uA)
		return -EINVAL;

	bq->ac_detect_gpio = devm_gpiod_get_optional(bq->dev,
						   "ac-detect", GPIOD_IN);
	if (IS_ERR(bq->ac_detect_gpio)) {
		ret = PTR_ERR(bq->ac_detect_gpio);
		dev_err(bq->dev, "Failed to get ac detect");
		return ret;
	}

	bq->ce_gpio = devm_gpiod_get_optional(bq->dev,
						   "charge-enable",
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
	struct power_supply_config psy_cfg = { };

	int ret;
#ifdef CONFIG_INTERRUPT_AS_GPIO
	int irq_gpio, irqn;
#endif

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
		bq->regmap = devm_regmap_init_i2c(client,
						&bq25620_regmap_config);
		break;
	case BQ25622:
		bq->regmap = devm_regmap_init_i2c(client,
						&bq25622_regmap_config);
		break;
	}

	if (IS_ERR(bq->regmap)) {
		dev_err(dev, "Failed to allocate register map\n");
		return PTR_ERR(bq->regmap);
	}

	i2c_set_clientdata(client, bq);

	ret = bq2562x_parse_dt(bq, &psy_cfg, dev);
	if (ret) {
		dev_err(dev, "Failed to read device tree properties%d\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, bq2562x_charger_reset, bq);
	if (ret)
		return ret;

	/* OTG reporting */
	bq->usb2_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB2);
	if (!IS_ERR_OR_NULL(bq->usb2_phy)) {
		INIT_WORK(&bq->usb_work, bq2562x_usb_work);
		bq->usb_nb.notifier_call = bq2562x_usb_notifier;
		usb_register_notifier(bq->usb2_phy, &bq->usb_nb);
	}

	bq->usb3_phy = devm_usb_get_phy(dev, USB_PHY_TYPE_USB3);
	if (!IS_ERR_OR_NULL(bq->usb3_phy)) {
		INIT_WORK(&bq->usb_work, bq2562x_usb_work);
		bq->usb_nb.notifier_call = bq2562x_usb_notifier;
		usb_register_notifier(bq->usb3_phy, &bq->usb_nb);
	}
#ifdef CONFIG_INTERRUPT_AS_GPIO
	irq_gpio = of_get_named_gpio(client->dev.of_node, "ti,irq-gpio", 0);
	if (!gpio_is_valid(irq_gpio)) {
		dev_err(bq->dev, "%s: %d gpio get failed\n", __func__, irq_gpio);
		return -EINVAL;
	}

	ret = gpio_request(irq_gpio, "bq2562x irq pin");
	if (ret) {
		dev_err(bq->dev, "%s: %d gpio request failed\n", __func__, irq_gpio);
		return ret;
	}

	gpio_direction_input(irq_gpio);
	irqn = gpio_to_irq(irq_gpio);
	if (irqn < 0) {
		dev_err(bq->dev, "%s:%d gpio_to_irq failed\n", __func__, irqn);
		return irqn;
	}
	client->irq = irqn;
#endif
	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						bq2562x_irq_handler_thread,
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
						dev_name(&client->dev), bq);
		if (ret < 0) {
			dev_err(dev, "get irq fail: %d\n", ret);
			return ret;
		}
	}

	ret = bq2562x_power_supply_init(bq, &psy_cfg, dev);
	if (ret) {
		dev_err(dev, "Failed to register power supply\n");
		return ret;
	}

	ret = bq2562x_hw_init(bq);
	if (ret) {
		dev_err(dev, "Cannot initialize the chip.\n");
		return ret;
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
	{ .compatible = "ti,bq25620", },
	{ .compatible = "ti,bq25622", },
	{ },
};
MODULE_DEVICE_TABLE(of, bq2562x_of_match);

static const struct acpi_device_id bq2562x_acpi_match[] = {
	{ "bq25620", BQ25620 },
	{ "bq25622", BQ25622 },
	{},
};
MODULE_DEVICE_TABLE(acpi, bq2562x_acpi_match);

static struct i2c_driver bq2562x_driver = {
	.driver = {
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
