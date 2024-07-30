# Device drive and DT overlay for TI BQ25622

On our Raspberry PI extension HAT called MrHAT, we use the TI BQ25622 BMS IC for power supply and battery management ([see product page](https://effective-range.com/hardware/mrhat/) and [github repo](https://github.com/EffectiveRange/pcb-mrhat)). 
The original driver had been received by email from TI, but that contained numerous bugs. This was almost entirely rewritten.

# Pre-requisites for usage

A battery definition must be available in the system, the BQ25622 DT overlay file expects it to have the name 'BAT1'. ( [reference battery definition](https://github.com/EffectiveRange/drv-battery))

# Operation

The driver after HW initialization tries to detect if a battery is attached to the charger, by forcing a battery discharge current with charging disabled, in one-shot ADC mode. The measurements are taken 3 times, if VBAT is less than VBAT_UVLO then there's no battery attached, charging is disabled, and battery present is set to false.

After this the operational state commences:
- The irq handler processes all flag changes, but if no other event occurs, then WD expire event is driving the irq handler
- A single ADC one-shot measurement is taken for each parameters (Except VPMID), then the ADC is operated with averaging mode only for VBAT and IBAT, others are disabled.
- At the beginning of the phase, the averaged VBAT and IBAT values are captured used for determining the battery state of charge