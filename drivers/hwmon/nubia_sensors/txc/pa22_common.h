/*
 *  pa224.h - Linux kernel modules for proximity sensor
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

#ifndef __PA224_H__
#define __PA224_H__

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

#define LOG_TAG "TXC-PA22X"
#define DEBUG_ON //DEBUG SWITCH

#define SENSOR_LOG_FILE__ strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1) : __FILE__

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

#define PA22_DRV_NAME		"pa224"

/* I2C */
#define I2C_RETRY_TIMES			3
#define I2C_RETRY_DELAY			10	//10ms

/* POWER SUPPLY VOLTAGE RANGE */
#define PA224_VDD_MIN_UV  2000000
#define PA224_VDD_MAX_UV  3300000
#define PA224_VIO_MIN_UV  1750000
#define PA224_VIO_MAX_UV  1950000

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

/* Device id*/
#define REG_ID				0x7f


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

struct pa22_platform_data {
	struct i2c_client *client;
	struct mutex lock;
	struct mutex reg_lock;
	struct regulator *vdd;
	struct list_head hw_device_list;
	u8 vdd_always_on;
	u8 enable_mask;
};

struct pa22_platform_device {
    char *name;
    int handle;
    struct list_head node;
    struct pa22_platform_ops *ops;
};

struct pa22_platform_ops {
	int (*init)(struct pa22_platform_data *p);
	void (*exit)(struct pa22_platform_data *p);
    int (*suspend)(struct pa22_platform_device *d);
    int (*resume)(struct pa22_platform_device *d);
    int (*unregister)(struct pa22_platform_device *d);
};

struct light_parameter {
	int lux;
};
struct light_cal_parameter{
	struct light_parameter base;
	struct light_parameter cur;
	int flag;
};

struct pa22_als_data {
    struct pa22_platform_device device_node;
	struct pa22_platform_data *platform_data;
	struct device *als_dev;
	struct mutex als_lock;
	struct delayed_work als_dwork;
	struct input_dev *als_input_dev;
    struct light_cal_parameter lux_cal_data;

    /* enable/poll parameter*/
	int als_enable;
	int als_enable_delay;
	int als_poll_delay;

    /* als value */
	int als_prev_lux;
	int als_cal_lux;
	int als_lux;

    /* debug flag */
	bool als_debug;
};

struct pa22_ps_data {
    struct pa22_platform_device device_node;
	struct pa22_platform_data *platform_data;
	struct device *proximity_dev;
	struct mutex ps_lock;
	struct input_dev *ps_input_dev;
	struct delayed_work ps_dwork;
	struct workqueue_struct *irq_work_queue;
	struct work_struct irq_dwork;
	struct wake_lock pa22_wake_lock;
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
	unsigned int ps_poll_delay;
	unsigned int ps_enable_delay;
	int ps_enable;

	/* PS Calibration */
	int near_diff_cnt;
	int far_diff_cnt;
	u8 crosstalk;
	u8 crosstalk_base;

	/* PS Threshold */
	u8 ps_thrd_low;
	u8 ps_thrd_high;

	/* PS status */
	/* Object status, near = 3, far = 10 */
	u8 ps_status;
	u8 ps_last_status;

	/* debug flag */
	bool prox_debug;
};

int i2c_read_reg(struct i2c_client *client, u8 reg, u8 *buf);
int i2c_write_reg(struct i2c_client *client, u8 reg, u8 buf);
int create_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
void remove_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count);
int pa22_write_file(char *file_path, const char *write_buf, int count);
int pa22_read_file(char *file_path, char *read_buf ,int count);
int sensor_platform_hw_power_on(struct pa22_platform_data *pdata, bool on);
int sensor_regulator_configure(struct pa22_platform_data *pdata, bool on);
int pa22_ps_register(struct pa22_platform_data *data, struct i2c_driver *driver);
int pa22_als_register(struct pa22_platform_data *data, struct i2c_driver *driver);

#endif
