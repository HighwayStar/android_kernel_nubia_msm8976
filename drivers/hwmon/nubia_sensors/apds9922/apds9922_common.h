/*
 *  apds9922.h - Linux kernel modules for proximity sensor
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

#ifndef __APDS9922_H__
#define __APDS9922_H__

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/ioctl.h>
#include <asm/atomic.h>
#include <linux/wakelock.h>


#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>

#define LOG_TAG "APDS9922"

extern int sensor_debug;

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)

#define SENSOR_LOG_DEBUG(fmt, args...) if(1 == sensor_debug)printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,LOG_TAG, __FUNCTION__, __LINE__, ##args)

#define APDS9922_DRV_NAME "apds9922"
#define APDS9922_CHIP_NAME "pa224"
#define DRIVER_VERSION "1.0.0"
/* I2C */
#define I2C_RETRY_TIMES			3
#define I2C_RETRY_DELAY			10	//10ms

/* POWER SUPPLY VOLTAGE RANGE */
#define APDS9922_VDD_MIN_UV  2850000
#define APDS9922_VDD_MAX_UV  3300000
#define APDS9922_VIO_MIN_UV  1750000
#define APDS9922_VIO_MAX_UV  1950000

/* Analog voltage @2.7 V */
#define AVDD_VTG_MIN_UV		3000000
#define AVDD_VTG_MAX_UV		3000000
#define AVDD_ACTIVE_LOAD_UA	15000

/* Digital voltage @1.8 V */
#define VDDIO_VTG_DIG_MIN_UV	1800000
#define VDDIO_VTG_DIG_MAX_UV	1800000
#define VDDIO_ACTIVE_LOAD_DIG_UA	10000

#define VDDIO_I2C_VTG_MIN_UV		1800000
#define VDDIO_I2C_VTG_MAX_UV		1800000
#define VDDIO_I2C_LOAD_UA		10000

/* Register Value define : INT_PERSISTENCE */
#define APDS9921_DD_PRX_PERS_1				0x00  /* Every PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_2				0x01  /* 2 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_3				0x02  /* 3 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_4				0x03  /* 4 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_5				0x04  /* 5 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_6				0x05  /* 6 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_7				0x06  /* 7 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_8				0x07  /* 8 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_9				0x08  /* 9 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_10				0x09  /* 10 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_11				0x0A  /* 11 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_12				0x0B  /* 12 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_13				0x0C  /* 13 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_14				0x0D  /* 14 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_15				0x0E  /* 15 consecutive PS value out of threshold range */
#define APDS9921_DD_PRX_PERS_16				0x0F  /* 16 consecutive PS value out of threshold range */


#define APDS9921_DD_ALS_PERS_1				0x00  /* Every ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_2				0x10  /* 2 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_3				0x20  /* 3 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_4				0x30  /* 4 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_5				0x40  /* 5 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_6				0x50  /* 6 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_7				0x60  /* 7 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_8				0x70  /* 8 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_9				0x80  /* 9 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_10				0x90  /* 10 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_11				0xA0  /* 11 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_12				0xB0  /* 12 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_13				0xC0  /* 13 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_14				0xD0  /* 14 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_15				0xE0  /* 15 consecutive ALS value out of threshold range */
#define APDS9921_DD_ALS_PERS_16				0xF0  /* 16 consecutive ALS value out of threshold range */

/* Register Value define : MAIN_CTRL */
#define APDS9921_DD_PRX_EN					0x01

/* Device id*/
#define APDS9921_DD_PART_ID_ADDR				0X06         //APDS9921_DD_PART_ID_ADDR
#define APDS9921_DD_MAIN_CTRL_ADDR                 0x00         //0:PS active 1: als active 4 : reset   1->triggered
#define APDS9921_DD_INT_PERSISTENCE_ADDR	0x1A
#define APDS9921_DD_INT_CFG_ADDR			       0x19
#define APDS9921_DD_MAIN_STATUS_ADDR		0x07
#define MAX_NAME_LEN    12
enum {
	ALS = 1,
	PS,
};
enum {
	ERR_NAKED_CAL = 1,
	ERR_THRES_CAL,
	ERR_FILE_OPS,
	ERR_DEV_OPS,
	ERR_OTHER,
};

struct apds9922_platform_data {
	struct i2c_client *client;
	struct mutex lock;
	struct mutex reg_lock;
	struct regulator *vdd;
	struct regulator *vio;
	struct list_head hw_device_list;
	u8 vdd_always_on;
	u8 enable_mask;
	int debug_enable;
};

struct apds9922_platform_device {
	char *name;
	int handle;
	struct list_head node;
	struct apds9922_platform_ops *ops;
};

struct apds9922_platform_ops {
	int (*init)(struct apds9922_platform_data *p);
	void (*exit)(struct apds9922_platform_data *p);
	int (*suspend)(struct apds9922_platform_device *d);
	int (*resume)(struct apds9922_platform_device *d);
	int (*unregister)(struct apds9922_platform_device *d);
};

struct light_parameter {
	int lux;
};
struct light_cal_parameter{
	struct light_parameter base;
	struct light_parameter cur;
	int flag;
};

struct apds9922_als_data {
	struct apds9922_platform_device device_node;
	struct apds9922_platform_data *platform_data;
	struct device *als_dev;
	struct mutex als_lock;
	struct delayed_work als_dwork;
	struct input_dev *als_input_dev;

   /* calibration parameter*/
	struct light_cal_parameter lux_cal_data;
	int als_lux_factor;

    /* enable/poll parameter*/
	int als_enable;
	int als_poll_delay;
	int als_res_index;		/* storage for als integratiion time */
	int als_gain_index;	/* storage for als GAIN */
    /* als value */
	int als_cal_lux;
	int als_lux;

};

struct apds9922_ps_data {
	struct apds9922_platform_device device_node;
	struct apds9922_platform_data *platform_data;
	struct device *proximity_dev;
	struct mutex ps_lock;
	struct input_dev *ps_input_dev;
	struct delayed_work ps_dwork;
	struct workqueue_struct *irq_work_queue;
	struct work_struct irq_dwork;
	struct wake_lock apds9922_wake_lock;
	struct list_head node;

	/* irq gpio cfg */
	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	/* irq status */
	int irq;
	bool irq_enabled;
	int irq_gpio;  /* proximity/light-sensor- external irq*/

    /* Ps ctrl*/
	int ps_poll_delay;
	int ps_enable;

	/* PS Threshold */
	int ps_thrd_low;
	int ps_thrd_high;
	int ps_offset;
	int ps_pulse;
	int ps_data;
	int ps_detection;
	int ps_oil_occurred;
	int ps_oil_value;
	/* Object status, near = 3, far = 10 */
	u8 ps_status;
	u8 ps_last_status;

	/* debug flag */
	bool prox_debug;
	int ps_cal_low;
	int ps_cal_high;
	/* PS Calibration */
	int near_diff_cnt;
	int far_diff_cnt;
	int crosstalk;
};

int apds9922_i2c_read_reg(struct i2c_client *client, u8 reg, u8 *buf);
int apds9922_i2c_write_reg(struct i2c_client *client, u8 reg, u8 buf);
int apds9922_create_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
void apds9922_remove_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
int apds9922_write_file(char *file_path, const char *write_buf, int count);
int apds9922_read_file(char *file_path, char *read_buf ,int count);
int apds9922_sensor_platform_hw_power_on(struct apds9922_platform_data *pdata, bool on);
int apds9922_sensor_regulator_configure(struct apds9922_platform_data *pdata, bool on);
int apds9922_ps_register(struct apds9922_platform_data *data, struct i2c_driver *driver);
int apds9922_als_register(struct apds9922_platform_data *data, struct i2c_driver *driver);

#endif
