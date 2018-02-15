
/*
 *  apds9922_als.h - Linux kernel modules for proximity sensor
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

#ifndef __PA22_ALS_H__
#define __PA22_ALS_H__

#define DEV_NAME_ALS	"light"	//ALS input name
//#define CHIP_NAME_ALS "apds9922"

/* Calibration */
#define LUX_CAL_PATH		 "/persist/sensors/rgbc_cfg"
#define SCALE_FACTOR(x, y) (x)/(y)

//rare chip calibration
#define APDS9921_ALS_THRESHOLD_HSYTERESIS	1	/* 1 = 1% */

typedef enum
{
	APDS9921_DD_ALS_RES_16BIT = 0,  /* 25ms integration time */
	APDS9921_DD_ALS_RES_17BIT = 1,  /* 50ms integration time */
	APDS9921_DD_ALS_RES_18BIT = 2,  /* 100ms integration time */
	APDS9921_DD_ALS_RES_19BIT = 3,  /* 200ms integration time */
	APDS9921_DD_ALS_RES_20BIT = 4   /* 400ms integration time */
} apds9921_dd_als_res_e;


typedef enum
{
	APDS9921_DD_ALS_GAIN_1X = 0,    /* 1x ALS GAIN */
	APDS9921_DD_ALS_GAIN_3X = 1,    /* 3x ALS GAIN */
	APDS9921_DD_ALS_GAIN_6X = 2,    /* 6x ALS GAIN */
	APDS9921_DD_ALS_GAIN_9X = 3,    /* 9x ALS GAIN */
	APDS9921_DD_ALS_GAIN_18X = 4    /* 18x ALS GAIN */
} apds9921_dd_als_gain_e;


/* Register Value define : ALS_MEAS_RATE */
#define APDS9921_DD_ALS_MEAS_RATE_25_MS		0x00  /* ALS Measurement rate = 25 ms */
#define APDS9921_DD_ALS_MEAS_RATE_50_MS		0x01  /* ALS Measurement rate = 50 ms */
#define APDS9921_DD_ALS_MEAS_RATE_100_MS	0x02  /* ALS Measurement rate = 100 ms */
#define APDS9921_DD_ALS_MEAS_RATE_200_MS	0x03  /* ALS Measurement rate = 200 ms */
#define APDS9921_DD_ALS_MEAS_RATE_500_MS	0x04  /* ALS Measurement rate = 500 ms */
#define APDS9921_DD_ALS_MEAS_RATE_1000_MS	0x05  /* ALS Measurement rate = 1000 ms */
#define APDS9921_DD_ALS_MEAS_RATE_2000_MS	0x06  /* ALS Measurement rate = 2000 ms */

#define APDS9921_DD_ALS_MEAS_RES_20_BIT		0x00  /* ALS resolution 20 bit (full range : 0 ~ 1048575) [ADC conversion time = 400ms] */
#define APDS9921_DD_ALS_MEAS_RES_19_BIT		0x10  /* ALS resolution 19 bit (full range : 0 ~ 524287) [ADC conversion time = 200ms]  */
#define APDS9921_DD_ALS_MEAS_RES_18_BIT		0x20  /* ALS resolution 18 bit (full range : 0 ~ 262143) [ADC conversion time = 100ms]  */
#define APDS9921_DD_ALS_MEAS_RES_17_BIT		0x30  /* ALS resolution 17 bit (full range : 0 ~ 131071) [ADC conversion time = 50ms]  */
#define APDS9921_DD_ALS_MEAS_RES_16_BIT		0x40  /* ALS resolution 16 bit (full range : 0 ~ 65535) [ADC conversion time = 25ms]  */

/* Register Value define : ALS_GAIN */
#define APDS9921_DD_ALS_GAIN_1				0x00  /* ALS Gain 1 */
#define APDS9921_DD_ALS_GAIN_3				0x01  /* ALS Gain 3 */
#define APDS9921_DD_ALS_GAIN_6				0x02  /* ALS Gain 6 */
#define APDS9921_DD_ALS_GAIN_9				0x03  /* ALS Gain 9 */
#define APDS9921_DD_ALS_GAIN_18				0x04  /* ALS Gain 18 */

/*als default setting*/
#define APDS9921_DD_ALS_DEFAULT_RES			APDS9921_DD_ALS_MEAS_RES_20_BIT
#define APDS9921_DD_ALS_DEFAULT_MEAS_RATE	APDS9921_DD_ALS_MEAS_RATE_500_MS
#define APDS9921_DD_ALS_DEFAULT_GAIN			APDS9921_DD_ALS_GAIN_18X

/* Driver Parameters  */
#define APDS9922_ALS_POLL_DELAY		              500
#define APDS9922_ALS_POLLING		                     1    // 0:INT Mode 1:Polling Mode
#define APDS9922_ALS_ACTIVE                                  0x02
#define APDS9921_DD_ALS_DATA_STATUS	               0x08  /* 1: New data, not read yet (cleared after read) */


/* ALS REG ADDR*/
#define APDS9921_DD_ALS_DATA_ADDR			0x0D
#define APDS9921_DD_ALS_DATA_0_ADDR			0x0D
#define APDS9921_DD_ALS_DATA_1_ADDR			0x0E
#define APDS9921_DD_ALS_DATA_2_ADDR			0x0F
#define APDS9921_DD_ALS_MEAS_RATE_ADDR		0x04
#define APDS9921_DD_ALS_GAIN_ADDR			0x05


static unsigned short apds9921_als_meas_rate_tb[] = {25, 50, 100, 200, 400};
static unsigned int    apds9921_als_res_tb[] = { 65535, 131071, 262143, 524287, 1048575 };
static unsigned char  apds9921_als_gain_tb[] = { 1, 3, 6, 9, 18 };
static unsigned char  apds9921_als_gain_bit_tb[] = { 0x00, 0x01, 0x02, 0x03, 0x04 };

static int    apds9922_als_init_client(struct apds9922_als_data * pdata);
static void apds9922_als_calidata_init(struct apds9922_als_data *data);
static int apds9922_als_cal_init(struct apds9922_als_data *data);
static int   apds9922_als_unregister(struct apds9922_platform_device *pdev_node);

#endif
