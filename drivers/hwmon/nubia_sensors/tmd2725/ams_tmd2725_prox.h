/*
* This file is part of the tmd2725 sensor driver.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
*Reversion
*
when         	who         		Remark : what, where, why          		version
-----------   	------------     	-------------------------           	--------
201x/x/xx       AMS															 V1.0
2016/7/11       Bao QI              Tmd2725 code refcatoring                 V2.0
====================================================================================
*/

#ifndef __AMS_TMD2725_PROX_H
#define __AMS_TMD2725_PROX_H

#include <linux/i2c.h>
#include "ams_tmd2725.h"

#define DEV_PS_NAME "proximity"
#define PS_FAR_DISTANCE_EV              10
#define PS_NEAR_DISTANCE_EV             3

#define PS_UNCOVER_DATA_MIN             1
#define PS_UNCOVER_DATA_MAX             150
#define PS_THRESH_DATA_MIN              30
#define PS_THRESH_DATA_MAX              200
#define PS_DATA_MAX                     255
#define PS_CAL_FILE_PATH                "/persist/sensors/xtalk_cal"
#define PS_AVG_TIME                     6
#define PS_THRES_FAR                    50
#define PS_THRES_NEAR                   80
#define PS_THRES_OIL_NEAR               254
#define PS_THRES_OIL_FAR                150
#define PS_OFFSET_CAL_THRESH            240
#define PS_DEFAULT_THRES                PS_OFFSET_CAL_THRESH

int tmd2725_ps_device_register(struct tmd2725_chip *chip, struct i2c_driver *driver);
void tmd2725_ps_device_unregister(struct tmd2725_chip *chip);

#endif /*__AMS_TMD2725_PROX_H */
