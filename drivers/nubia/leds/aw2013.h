/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __AW2013_H__
#define __AW2013_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
//#include "../../arch/arm/mach-msm/include/mach/board.h"
//#include <linux/mfd/pm8xxx/pm8921-charger.h>

#define AW_DEVICE_NAME "aw2013"


struct aw2013_control_data {
	struct i2c_client *i2c_client;
	struct i2c_driver *i2c_driver;
	struct work_struct work;
	struct hrtimer timer;
	struct led_classdev	cdev;

	int Rise_Fall_time; // intergrated as a ingrater.
	int Hold_time;
	int Off_time;

	int Delay_time;
	int Repeat_time;

	int min_grade;
	int max_grade;
	int outn;
	int brightness;
	int blink_mode;

	int middle_grade_percentage;
	int side_grade_percentage;
	int grade_scale_offset;
	int blink_max_grade;
	int onceblink_fade_time;
	int onceblink_fullon_time;
	int onceblink_fulloff_time;
	int autoblink_fade_time;
	int autoblink_fullon_time;
	int autoblink_fulloff_time;
	int max_current;  //0x03 = 15mA, 0x02= 10mA, 0x01 =5mA

	int debug_mark;
	int left_light_which_channel;
	int middle_light_which_channel;
	int right_light_which_channel;
	bool i2c_suspend;
};


struct aw2013_regs_data {
	int soft_reset;     			 //RSTR: when write 0x55 to it, all the circule reset, and all registers' value restore to default value.
	int enable_led;     			 //GCR:  gloable control register, use to open led function(bit 0 is 1) and enable interrupts.
	int irq_state;      		     //ISR: interrupt state register, use for read the interrupt state.
	int enable_ledx;        			 //LCTR: led control register, use for enable or disable single led output.

	int led0_mode_conf; 		 	 //LCFG0: use for config led0's fade_on/off, led work mode(pwm/autonomous),led max current.
	int led1_mode_conf;  		 	 //LCFG1: use for config led1's fade_on/off, led work mode(pwm/autonomous),led max current.
	int led2_mode_conf; 		 	 //LCFG2: use for config led2's fade_on/off, led work mode(pwm/autonomous),led max current.

	int pwm0_max_lightness;     	 //PWM0: use for config the led0's max lightness in blink mode,its range is 0-255.
	int pwm1_max_lightness;	 	     //PWM1: use for config the led1's max lightness in blink mode,its range is 0-255.
	int pwm2_max_lightness;       	 //PWM2: use for config the led2's max lightness in blink mode,its range is 0-255.

	int led0_rise_and_hold_time; 	 //LEDiT0:use for config led0's rise time(T1) and max brighness hold time(T2).
	int led0_fall_and_off_time; 	 //LEDiT1:use for config led0's fall time(T3) and off hold time(T4).
	int led0_delay_and_repeat_time;  //LEDiT2:use for config led0's delay time(T0) and repeate times(REPEAT) in autonomous mode.

	int led1_rise_and_hold_time; 	 //LEDiT0:use for config led1's rise time(T1) and max brighness hold time(T2).
	int led1_fall_and_off_time; 	 //LEDiT1:use for config led1's fall time(T3) and off hold time(T4).
	int led1_delay_and_repeat_time;  //LEDiT2:use for config led1's delay time(T0) and repeate times(REPEAT) in autonomous mode.REPEAT=0 will never stop.

	int led2_rise_and_hold_time; 	 //LEDiT0:use for config led2's rise time(T1) and max brighness hold time(T2).
	int led2_fall_and_off_time; 	 //LEDiT1:use for config led2's fall time(T3) and off hold time(T4).
	int led2_delay_and_repeat_time;  //LEDiT2:use for config led2's delay time(T0) and repeate times(REPEAT) in autonomous mode.
};


#endif /* __AW2013_H__ */


