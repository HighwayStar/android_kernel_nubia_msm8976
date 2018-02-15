
/*
 *  apds9922_ps.h - Linux kernel modules for proximity sensor
 *
 *  Copyright (c) 2013, All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 and
 *  only version 2 as published by the Free Software Foundation.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __APDS9922_PS_H__
#define __APDS9922_PS_H__

#define INPUT_NAME_PS       "proximity"
#define PS_CAL_FILE_PATH    "/persist/sensors/xtalk_cal"

/*apds9922 als/ps Default*/
#define APDS9922_PS_POLL_DELAY		             50
//#define APDS9922_PS_ENABLE_DELAY	                    20


#define APDS9921_PS_CAL_LOOP			             5
#define APDS9921_PS_CAL_CROSSTALK_LOW	      0
#define APDS9921_PS_CAL_CROSSTALK_HIGH	      150
//#define APDS9921_PS_CROSSTALK_DELTA		      20

#define APDS9922_PS_TH_MIN		                          0	// Minimun value
#define APDS9922_PS_TH_MAX		                   255     // 8 bit MAX

#define APDS9922_PS_UNCOVER_DATA_MIN            0
#define APDS9922_PS_UNCOVER_DATA_MAX           150

#define APDS9922_PS_THRESH_DATA_MIN                15
#define APDS9922_PS_THRESH_DATA_MAX               200

#define APDS9922_PS_NEAR_DISTANCE	                    3       //Near distance 0 cm
#define APDS9922_PS_FAR_DISTANCE	                    10       //Far distance 1 cm
#define APDS9922_PS_UNKOWN_DISTANCE	             0       //Unkown distance 1 cm


#define PS_POLLING		                                        0	// 0:INT Mode 1:Polling Mode
#define APDS9921_DD_PRX_INT_EN				0x01  /* 1: PS Interrupt enabled */
#define APDS9922_PS_ACTIVE                                  0x01


/* Register Value define : PS_LED */
#define APDS9921_DD_LED_CURRENT_2_5_MA		0x00  /* 2.5 mA */
#define APDS9921_DD_LED_CURRENT_5_MA		0x01  /* 5 mA */
#define APDS9921_DD_LED_CURRENT_10_MA		0x02  /* 10 mA */
#define APDS9921_DD_LED_CURRENT_25_MA		0x03  /* 25 mA */
#define APDS9921_DD_LED_CURRENT_50_MA		0x04  /* 50 mA */
#define APDS9921_DD_LED_CURRENT_75_MA		0x05  /* 75 mA */
#define APDS9921_DD_LED_CURRENT_100_MA		0x06  /* 100 mA */
#define APDS9921_DD_LED_CURRENT_125_MA		0x07  /* 125 mA */

#define APDS9921_DD_LED_CURRENT_PEAK_ON		0x08

#define APDS9921_DD_LED_FREQ_60_KHZ			0x30  /* LED Pulse frequency = 60KHz */
#define APDS9921_DD_LED_FREQ_70_KHZ			0x40  /* LED Pulse frequency = 70KHz */
#define APDS9921_DD_LED_FREQ_80_KHZ			0x50  /* LED Pulse frequency = 80KHz */
#define APDS9921_DD_LED_FREQ_90_KHZ			0x60  /* LED Pulse frequency = 90KHz */
#define APDS9921_DD_LED_FREQ_100_KHZ		      0x70  /* LED Pulse frequency = 100KHz */

/* Register Value define : PS_MEAS_RATE */
#define APDS9921_DD_PRX_MEAS_RATE_6_25_MS	0x01  /* PS Measurement rate = 6.25 ms */
#define APDS9921_DD_PRX_MEAS_RATE_12_5_MS	0x02  /* PS Measurement rate = 12.5 ms */
#define APDS9921_DD_PRX_MEAS_RATE_25_MS		0x03  /* PS Measurement rate = 25 ms */
#define APDS9921_DD_PRX_MEAS_RATE_50_MS		0x04  /* PS Measurement rate = 50 ms */
#define APDS9921_DD_PRX_MEAS_RATE_100_MS	0x05  /* PS Measurement rate = 100 ms */
#define APDS9921_DD_PRX_MEAS_RATE_200_MS	0x06  /* PS Measurement rate = 200 ms */
#define APDS9921_DD_PRX_MEAS_RATE_400_MS	0x07  /* PS Measurement rate = 400 ms */

#define APDS9921_DD_PRX_MEAS_RES_8_BIT		0x00  /* PS resolution 8 bit (full range : 0 ~ 1023) */
#define APDS9921_DD_PRX_MEAS_RES_9_BIT		0x08  /* PS resolution 9 bit (full range : 0 ~ 2047) */
#define APDS9921_DD_PRX_MEAS_RES_10_BIT		0x10  /* PS resolution 10 bit (full range : 0 ~ 3071) */
#define APDS9921_DD_PRX_MEAS_RES_11_BIT		0x18  /* PS resolution 11 bit (full range : 0 ~ 4095) */


#define	APDS9921_DD_PRX_DEFAULT_PULSE			32	// drop to 16 if crosstalk is too high
#define APDS9921_DD_PRX_DEFAULT_LED_CURRENT	APDS9921_DD_LED_CURRENT_100_MA
#define APDS9921_DD_PRX_DEFAULT_LED_FREQ		APDS9921_DD_LED_FREQ_60_KHZ
#define APDS9921_DD_PRX_DEFAULT_RES				APDS9921_DD_PRX_MEAS_RES_8_BIT
#define APDS9921_DD_PRX_DEFAULT_MEAS_RATE		APDS9921_DD_PRX_MEAS_RATE_50_MS



/* Register Addresses define */
#define APDS9921_DD_PRX_LED_ADDR			       0x01
#define APDS9921_DD_PRX_PULSES_ADDR			0x02
#define APDS9921_DD_PRX_MEAS_RATE_ADDR		0x03
#define APDS9921_DD_PRX_DATA_ADDR			0x08
#define APDS9921_DD_PRX_DATA_0_ADDR			0x08
#define APDS9921_DD_PRX_DATA_1_ADDR			0x09
#define APDS9921_DD_COMP_DATA_ADDR			0x16
#define APDS9921_DD_COMP_DATA_0_ADDR		0x16
#define APDS9921_DD_COMP_DATA_1_ADDR		0x17
#define APDS9921_DD_COMP_DATA_2_ADDR		0x18
#define APDS9921_DD_PRX_THRES_UP_ADDR		0x1B
#define APDS9921_DD_PRX_THRES_UP_0_ADDR		0x1B
#define APDS9921_DD_PRX_THRES_UP_1_ADDR		0x1C
#define APDS9921_DD_PRX_THRES_LOW_ADDR		0x1D
#define APDS9921_DD_PRX_THRES_LOW_0_ADDR	0x1D
#define APDS9921_DD_PRX_THRES_LOW_1_ADDR	0x1E
#define APDS9921_DD_PRX_CAN_ADDR			       0x1F
#define APDS9921_DD_PRX_CAN_0_ADDR			0x1F
#define APDS9921_DD_PRX_CAN_1_ADDR			0x20
#define	APDS9921_DD_DEVICE_CONFIG_ADDR		0x2F

static void apds9922_report_event(struct apds9922_ps_data *data);
static void apds9922_irq_enable(struct apds9922_ps_data *data, bool enable, bool flag_sync);
static int apds9922_init_client(struct apds9922_ps_data *data);
static int apds9922_ps_unregister(struct apds9922_platform_device *pdev_node);

#endif
