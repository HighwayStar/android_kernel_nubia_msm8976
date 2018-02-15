
/*
 *  pa22_als.h - Linux kernel modules for proximity sensor
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
#define CHIP_NAME_ALS "pa22a"

/* Calibration */
#define LUX_CAL_PATH		 "/persist/sensors/rgbc_cfg"
#define SCALE_FACTOR(x, y) (x)/(y)

/* Driver Parameters  */
#define ALS_AVG_ENABLE          0
#define PA22_ALS_POLL_DELAY		640
#define PA22_ALS_ENABLE_DELAY	200
#define PA2A_AFSR		3	/*0:375 lux | 1:300 lux | 2:6000 lux | 3:30000*/
#define ALS_POLLING		1	// 0:INT Mode 1:Polling Mode
#define ALS_ACTIVE              0x01

/* ALS REG */
#define REG_ALS_DATA_LSB        0X0B    // ALS DATA
#define REG_ALS_DATA_MSB        0X0C    // ALS DATA
#define REG_CFG0 			    0X00  	// PS_ON(D1)

static int pa22_als_init_client(struct pa22_als_data * pdata);
static void pa22_als_calidata_init(struct pa22_als_data *data);
static int pa22_als_unregister(struct pa22_platform_device *pdev_node);

#endif