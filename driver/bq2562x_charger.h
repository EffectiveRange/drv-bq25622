/* SPDX-License-Identifier: GPL-2.0-only */
// BQ2562X Charger Driver
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/

#ifndef _BQ2562X_CHARGER_H
#define _BQ2562X_CHARGER_H

/* clang-format off */
#define BQ2562X_MANUFACTURER	"Texas Instruments"

#define BQ2562X_RESERVED_LSB	0x00
#define BQ2562X_RESERVED_MSB	0x01
#define BQ2562X_CHRG_I_LIM_LSB	0x02
#define BQ2562X_CHRG_I_LIM_MSB	0x03
#define BQ2562X_CHRG_V_LIM_LSB	0x04
#define BQ2562X_CHRG_V_LIM_MSB	0x05
#define BQ2562X_INPUT_I_LIM_LSB	0x06
#define BQ2562X_INPUT_I_LIM_MSB	0x07
#define BQ2562X_INPUT_V_LIM_LSB	0x08
#define BQ2562X_INPUT_V_LIM_MSB	0x09
#define BQ2562X_IOTG_LSB		0x0a
#define BQ2562X_IOTG_MSB		0x0b
#define BQ2562X_VOTG_LSB		0x0c
#define BQ2562X_VOTG_MSB		0x0d
#define BQ2562X_MIN_SYS_V_LSB	0x0e
#define BQ2562X_MIN_SYS_V_MSB	0x0f
#define BQ2562X_PRECHRG_CTRL_LSB	0x10
#define BQ2562X_PRECHRG_CTRL_MSB	0x11
#define BQ2562X_TERM_CTRL_LSB	0x12
#define BQ2562X_TERM_CTRL_MSB	0x13
#define BQ2562X_CHRG_CTRL		0x14
#define BQ2562X_TIMER_CTRL		0x15
#define BQ2562X_CHRG_CTRL_1		0x16
#define BQ2562X_CHRG_CTRL_2		0x17
#define BQ2562X_CHRG_CTRL_3		0x18
#define BQ2562X_CHRG_CTRL_4		0x19
#define BQ2562X_NTC_CTRL_0		0x1a
#define BQ2562X_NTC_CTRL_1		0x1b
#define BQ2562X_NTC_CTRL_2		0x1c
#define BQ2562X_CHRG_STAT_0		0x1d
#define BQ2562X_CHRG_STAT_1		0x1e
#define BQ2562X_FAULT_STAT_0	0x1f
#define BQ2562X_CHRG_FLAG_0		0x20
#define BQ2562X_CHRG_FLAG_1		0x21
#define BQ2562X_FAULT_FLAG_0	0x22
#define BQ2562X_CHRG_MSK_0		0x23
#define BQ2562X_CHRG_MSK_1		0x24
#define BQ2562X_FAULT_MSK_0		0x25
#define BQ2562X_ADC_CTRL		0x26
#define BQ2562X_FN_DISABE_0		0x27
#define BQ2562X_ADC_IBUS_LSB	0x28
#define BQ2562X_ADC_IBUS_MSB	0x29
#define BQ2562X_ADC_IBAT_LSB	0x2a
#define BQ2562X_ADC_IBAT_MSB	0x2b
#define BQ2562X_ADC_VBUS_LSB	0x2c
#define BQ2562X_ADC_VBUS_MSB	0x2d
#define BQ2562X_ADC_VPMID_LSB	0x2e
#define BQ2562X_ADC_VPMID_MSB	0x2f
#define BQ2562X_ADC_VBAT_LSB	0x30
#define BQ2562X_ADC_VBAT_MSB	0x31
#define BQ2562X_ADC_VSYS_LSB	0x32
#define BQ2562X_ADC_VSYS_MSB	0x33
#define BQ2562X_ADC_TS_LSB		0x34
#define BQ2562X_ADC_TS_MSB		0x35
#define BQ2562X_ADC_TDIE_LSB	0x36
#define BQ2562X_ADC_TDIE_MSB	0x37
#define BQ2562X_PART_INFO		0x38

#define BQ2562X_VIRT_CTRL0		0x80
#define BQ2562X_VIRT_CTRL1		0x81

#define BQ2562X_CHRG_CTRL1_FORCE_IBATDIS    BIT(6)

#define BQ2562X_CHRG_CTRL2_REG_RST          BIT(7)

#define BQ2562X_CHRG_EN		    BIT(5)
#define BQ2562X_ADC_EN		    BIT(7)
#define BQ2562X_ADC_RATE	    BIT(6)
#define BQ2562X_ADC_AVG	        BIT(3)
#define BQ2562X_ADC_AVG_INIT    BIT(2)

#define BQ2562X_EN_EXT_ILIM BIT(2)

#define BQ2562X_CHG_STAT_MSK	GENMASK(4, 3)
#define BQ2562X_NOT_CHRGING		0
#define BQ2562X_TRICKLE_CHRG	BIT(3)
#define BQ2562X_TAPER_CHRG		BIT(4)
#define BQ2562X_TOP_OFF_CHRG	(BIT(4) | BIT(3))

#define BQ2562X_VBUS_PRESENT	BIT(0)

#define BQ2562X_VBUS_STAT_MSK	GENMASK(2, 0)
#define BQ2562X_USB_SDP		BIT(0)
#define BQ2562X_USB_CDP		BIT(1)
#define BQ2562X_USB_DCP		(BIT(1) | BIT(0))
#define BQ2562X_UNKNOWN_500MA	BIT(2)
#define BQ2562X_NON_STANDARD	(BIT(2) | BIT(0))
#define BQ2562X_HVDCP		(BIT(2) | BIT(1))
#define BQ2562X_OTG_MODE	(BIT(2) | BIT(1) | BIT(0))

#define BQ2562X_TEMP_TS_NORMAL	0x00
#define BQ2562X_TEMP_COLD		BIT(0)
#define BQ2562X_TEMP_HOT		BIT(1)
#define BQ2562X_TEMP_COOL		(BIT(1) | BIT(0))
#define BQ2562X_TEMP_WARM		BIT(2)
#define BQ2562X_TEMP_PRECOOL	(BIT(2) | BIT(0))
#define BQ2562X_TEMP_PREWARM	(BIT(2) | BIT(1))
#define BQ2562X_TEMP_PIN_BIAS_REFER_FAULT	(BIT(2) | BIT(1) | BIT(0))
#define BQ2562X_TEMP_MASK		GENMASK(2, 0)


#define BQ2562X_TSHUT_FAULT_STAT    BIT(3)
#define BQ2562X_OTG_FAULT_STAT	    BIT(4)
#define BQ2562X_SYS_FAULT_STAT	    BIT(5)
#define BQ2562X_BAT_FAULT_STAT	    BIT(6)
#define BQ2562X_VSYS_FAULT_STAT     BIT(7)

#define BQ2562X_PRECHRG_CUR_MASK		GENMASK(8, 4)
#define BQ2562X_PRECHRG_CURRENT_STEP_uA		20000
#define BQ2562X_PRECHRG_I_MIN_uA		20000
#define BQ2562X_PRECHRG_I_MAX_uA		620000
#define BQ2562X_PRECHRG_I_DEF_uA		100000

#define BQ2562X_TERMCHRG_CUR_MASK		GENMASK(8, 3)
#define BQ2562X_TERMCHRG_CURRENT_STEP_uA	10000
#define BQ2562X_TERMCHRG_I_MIN_uA		10000
#define BQ2562X_TERMCHRG_I_MAX_uA		620000
#define BQ2562X_TERMCHRG_I_DEF_uA		60000

#define BQ2562X_ICHRG_CURRENT_STEP_uA		80000
#define BQ2562X_ICHRG_I_MIN_uA			80000
#define BQ2562X_ICHRG_I_MAX_uA			3520000
#define BQ2562X_ICHRG_I_DEF_uA			1040000

#define BQ2562X_VREG_V_MAX_uV	4800000
#define BQ2562X_VREG_V_MIN_uV	3500000
#define BQ2562X_VREG_V_DEF_uV	4200000
#define BQ2562X_VREG_V_STEP_uV	10000

#define BQ2562X_IINDPM_I_MIN_uA	100000
#define BQ2562X_IINDPM_I_MAX_uA	3200000
#define BQ2562X_IINDPM_STEP_uA	20000
#define BQ2562X_IINDPM_DEF_uA	3200000

#define BQ2562X_VINDPM_V_MIN_uV 3800000
#define BQ2562X_VINDPM_V_MAX_uV 16800000
#define BQ2562X_VINDPM_STEP_uV	40000
#define BQ2562X_VINDPM_DEF_uV	4600000

#define BQ2562X_ADC_VBUS_STEP_uV	3970
#define BQ2562X_ADC_VBAT_STEP_uV	1990
#define BQ2562X_ADC_IBAT_STEP_uA	4000

#define BQ2562X_ADC_VBAT_MASK	GENMASK(12, 1)
#define BQ2562X_ADC_VBAT_MOVE_STEP	1

#define BQ2562X_ADC_VBUS_MASK	GENMASK(14, 2)
#define BQ2562X_ADC_VBUS_MOVE_STEP	2

#define BQ2562X_ADC_IBAT_MASK	GENMASK(15, 2)
#define BQ2562X_ADC_IBAT_MOVE_STEP	2

#define BQ2562X_ADC_TDIE_MSB_MSBIT      BIT(3)
#define BQ2562X_ADC_TDIE_MSB_MASK       GENMASK(3,0)
#define BQ2562X_ADC_TDIE_TEMP_STEP_01C  5

#define BQ2562X_ITERM_MASK		GENMASK(8, 2)
#define BQ2562X_ITERM_MOVE_STEP		2

#define BQ2562X_ICHG_MASK		GENMASK(11, 6)
#define BQ2562X_ICHG_MOVE_STEP		6

#define BQ2562X_PRECHRG_MASK		GENMASK(8, 4)
#define BQ2562X_PRECHRG_MOVE_STEP		4

#define BQ2562X_CHRG_V_LIM_MOVE_STEP	3

#define BQ2562X_INPUT_V_LIM_MOVE_STEP	5

#define BQ2562X_INPUT_I_LIM__MOVE_STEP	4

#define BQ2562X_ADC_IBUS_MOVE_STEP	1
#define BQ2562X_ADC_CURR_STEP_uA	1000
#define BQ2562X_ADC_VOLTAGE_STEP_uV	1000

#define BQ2562X_HIZ_EN BIT(4)

#define BQ2562X_WATCHDOG_MASK	GENMASK(1, 0)
#define BQ2562X_WATCHDOG_LSB	BIT(0)
#define BQ2562X_WATCHDOG_DIS	0
#define BQ2562X_WATCHDOG_MAX	160000
#define BQ2562X_WD_RST     BIT(2)

#define BQ2562X_ADC_DONE        BIT(6)
#define BQ2562X_WD_STAT         BIT(0)

#define BQ2562X_TS_ADC_COEFF_A_DEF      14409
#define BQ2562X_TS_ADC_COEFF_B_DEF      11248000
#define BQ2562X_TS_ADC_COEFF_SCALE_DEF  10000

#define BQ2562X_TEMP_MIN_01C  -400
#define BQ2562X_TEMP_MAX_01C  1250

#define BQ25622_K_ILIM_mA_MIN 2250000
#define BQ25622_K_ILIM_mA_TYP 2500000
#define BQ25622_K_ILIM_mA_MAX 2750000

#define BQ2562X_IBUS_ADC_DIS		BIT(7)
#define BQ2562X_IBAT_ADC_DIS		BIT(6)
#define BQ2562X_VBUS_ADC_DIS		BIT(5)
#define BQ2562X_VBAT_ADC_DIS		BIT(4)
#define BQ2562X_VSYS_ADC_DIS		BIT(3)
#define BQ2562X_TS_ADC_DIS		    BIT(2)
#define BQ2562X_TDIE_ADC_DIS		BIT(1)
#define BQ2562X_VPMID_ADC_DIS		BIT(0)

#define BQ25622_VBAT_UVLOZ_uV   2500000
#define BQ25622_VBAT_UVLO_uV    2100000

/* clang-format on */
#endif /* _BQ2562X_CHARGER_H */
