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
-----------   	------------     	-------------------------           	----------
201x/x/xx       AMS															 V1.0
2016/7/11       Bao QI              Tmd2725 code refcatoring                 V2.0
==========================================================================================
*/

#ifndef __TMD2725_H
#define __TMD2725_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/wait.h>
#include <linux/wakelock.h>
//#define ABI_SET_GET_REGISTERS

#define LOG_TAG "TMD2725"

#define DEBUG_ON //DEBUG SWITCH

#define SENSOR_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG_DEBUG_IF(en, fmt, args...) \
do { \
    if (en) { \
        printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args); \
    }; \
} while (0)
#ifdef  DEBUG_ON
#define SENSOR_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define SENSOR_LOG_DEBUG(fmt, args...)
#endif

#ifdef AMS_MUTEX_DEBUG
#define AMS_MUTEX_LOCK(m) { \
		printk(KERN_INFO "%s: Mutex Lock\n", __func__); \
		mutex_lock(m); \
	}
#define AMS_MUTEX_UNLOCK(m) { \
		printk(KERN_INFO "%s: Mutex Unlock\n", __func__); \
		mutex_unlock(m); \
	}
#else
#define AMS_MUTEX_LOCK(m) { \
		mutex_lock(m); \
	}
#define AMS_MUTEX_UNLOCK(m) { \
		mutex_unlock(m); \
	}
#endif

#define INPUT_ALS_NAME "light"
#define INPUT_PS_NAME "proximity"
#define DEVICE_CHIP_NAME "pa224:tmd2725"

// pldrive
#define PDRIVE_MA(p)   (((u8)((p + 3) / 6) - 1) & 0x3f)
#define P_TIME_US(p)   ((((p) / 88) - 1.0) + 0.5)
#define PRX_PERSIST(p) (((p) & 0xf) << 4)

#define INTEGRATION_CYCLE 2816
#define AW_TIME_MS(p)  ((((p) * 1000) + (INTEGRATION_CYCLE - 1)) / INTEGRATION_CYCLE)
#define WAIT_TIME_MS(p)  ((((p) * 10) + 14) / 28 - 1)
#define ALS_PERSIST(p) (((p) & 0xf) << 0)

// lux
#define INDOOR_LUX_TRIGGER	6000
#define OUTDOOR_LUX_TRIGGER	10000
#define TMD2725_MAX_LUX		0xffff
#define TMD2725_MAX_ALS_VALUE	0xffff
#define TMD2725_MIN_ALS_VALUE	10
#define MAX_REGS 256

enum tmd2725_prox_distance {
    PROX_NONE              = 0,
    PROX_NEAR              = 3,
    PROX_FAR               = 10,
};

enum tmd2725_regs {
	TMD2725_REG_ENABLE     = 0x80,
	TMD2725_REG_ATIME      = 0x81,
	TMD2725_REG_PTIME      = 0x82,
	TMD2725_REG_WTIME      = 0x83,
	TMD2725_REG_AILT       = 0x84,
	TMD2725_REG_AILT_HI    = 0x85,
	TMD2725_REG_AIHT       = 0x86,
	TMD2725_REG_AIHT_HI    = 0x87,
	TMD2725_REG_PILT       = 0x88,
	TMD2725_REG_PIHT       = 0x8A,
	TMD2725_REG_PERS       = 0x8C,
	TMD2725_REG_CFG0       = 0x8D,
	TMD2725_REG_PGCFG0     = 0x8E,
	TMD2725_REG_PGCFG1     = 0x8F,

	TMD2725_REG_CFG1       = 0x90,
	TMD2725_REG_REVID      = 0x91,
	TMD2725_REG_ID         = 0x92,
	TMD2725_REG_STATUS     = 0x93,
	TMD2725_REG_CH0DATA    = 0x94,
	TMD2725_REG_CH0DATA_HI = 0x95,
	TMD2725_REG_CH1DATA    = 0x96,
	TMD2725_REG_CH1DATA_HI = 0x97,
	TMD2725_REG_PDATA      = 0x9C,

	TMD2725_REG_ADCDATA_L  = 0x9D,
	TMD2725_REG_AUXID      = 0x9E,
	TMD2725_REG_CFG2       = 0x9F,

	TMD2725_REG_CFG3       = 0xAB,
	TMD2725_REG_CFG4       = 0xAC,
	TMD2725_REG_CFG5       = 0xAD,

	TMD2725_REG_POFFSET_L  = 0xC0,
	TMD2725_REG_POFFSET_H  = 0xC1,

	TMD2725_REG_AZ_CONFIG  = 0xD6,
	TMD2725_REG_CALIB      = 0xD7,
	TMD2725_REG_CALIBCFG   = 0xD9,
	TMD2725_REG_CALIBSTAT  = 0xDC,
	TMD2725_REG_INTENAB    = 0xDD,
};

enum tmd2725__reg {
	TMD2725_MASK_BINSRCH_TARGET = 0x70,
	TMD2725_SHIFT_BINSRCH_TARGET = 4,

	TMD2725_MASK_START_OFFSET_CALIB = 0x01,
	TMD2725_SHIFT_START_OFFSET_CALIB = 0,

	TMD2725_MASK_PROX_PERS = 0xf0,
	TMD2725_SHIFT_PROX_PERS = 4,

	TMD2725_MASK_PDRIVE = 0x1f,
	TMD2725_SHIFT_PDRIVE = 0,

	TMD2725_MASK_PGAIN = 0xC0,
	TMD2725_SHIFT_PGAIN = 6,

	TMD2725_MASK_AGAIN = 0x03,
	TMD2725_SHIFT_AGAIN = 0,

	TMD2725_MASK_APERS = 0x0f,
	TMD2725_SHIFT_APERS = 0,

	TMD2725_MASK_WLONG = 0x04,
	TMD2725_SHIFT_WLONG = 2,

	TMD2725_MASK_POFFSET_H = 0x01,
	TMD2725_SHIFT_POFFSET_H = 0,

	TMD2725_MASK_PROX_DATA_AVG = 0x07,
	TMD2725_SHIFT_PROX_DATA_AVG = 0,

	TMD2725_MASK_PROX_AUTO_OFFSET_ADJUST = 0x08,
	TMD2725_SHIFT_PROX_AUTO_OFFSET_ADJUST = 3,
};

enum tmd2725_en_reg {
	TMD2725_PON  = (1 << 0),
	TMD2725_AEN  = (1 << 1),
	TMD2725_PEN  = (1 << 2),
	TMD2725_WEN  = (1 << 3),
	TMD2725_EN_ALL = (TMD2725_AEN |
			  TMD2725_PEN |
			  TMD2725_WEN),
};

enum tmd2725_status {
	TMD2725_ST_PGSAT_AMBIENT  = (1 << 0),
	TMD2725_ST_PGSAT_RELFLECT = (1 << 1),
	TMD2725_ST_ZERODET    = (1 << 1),
	TMD2725_ST_CAL_IRQ    = (1 << 3),
	TMD2725_ST_ALS_IRQ    = (1 << 4),
	TMD2725_ST_PRX_IRQ    = (1 << 5),
	TMD2725_ST_PRX_SAT    = (1 << 6),
	TMD2725_ST_ALS_SAT    = (1 << 7),
};

enum tmd2725_intenab_reg {
	TMD2725_ZIEN = (1 << 2),
	TMD2725_CIEN = (1 << 3),
	TMD2725_AIEN = (1 << 4),
	TMD2725_PIEN = (1 << 5),
	TMD2725_PSIEN = (1 << 6),
	TMD2725_ASIEN = (1 << 7),
};

enum tmd2725_oil_state {
	OIL_NONE = 0,
	OIL_STATE = 1,
};

enum tmd2725_pwr_state {
	POWER_OFF       = (0 << 0),
	POWER_PS_ON     = (1 << 0),
	POWER_ALS_ON    = (2 << 0),
	POWER_ON        = (3 << 0),
};

enum tmd2725_prox_state {
	PROX_STATE_NONE = 0,
	PROX_STATE_INIT,
	PROX_STATE_CALIB,
	PROX_STATE_WAIT_AND_CALIB
};

enum tmd2725_ctrl_reg {
	AGAIN_1        = (0 << 0),
	AGAIN_4        = (1 << 0),
	AGAIN_16       = (2 << 0),
	AGAIN_64       = (3 << 0),
	PGAIN_1        = (0 << TMD2725_SHIFT_PGAIN),
	PGAIN_2        = (1 << TMD2725_SHIFT_PGAIN),
	PGAIN_4        = (2 << TMD2725_SHIFT_PGAIN),
	PGAIN_8        = (3 << TMD2725_SHIFT_PGAIN),
	PG_PULSE_4US   = (0 << 6),
	PG_PULSE_8US   = (1 << 6),
	PG_PULSE_16US  = (2 << 6),
	PG_PULSE_32US  = (3 << 6),
};

struct tmd2725_lux_segment {
	int64_t ch0_coef;
	int64_t ch1_coef;
};

struct tmd2725_als_parameters {
    long which_seg;
	long d_factor;
	long seg0_ch0_coef;
	long seg0_ch1_coef;
	long seg1_ch0_coef;
	long seg1_ch1_coef;
};
struct tp_als_parameter{
	long tp_module_id;
	struct tmd2725_als_parameters  tp_als_param;
};

struct tmd2725_prox_parameters {
	u8 prox_thres_near;
	u8 prox_thres_far;
	u8 prox_thres_oil_near;
	u8 prox_thres_oil_far;
	u16 prox_raw;
	u16 prox_th_min;
	u16 prox_th_max;
	s16 poffset;
	u8 prox_gain;
	u8 prox_drive;
	u8 persist;
	s16 prox_offset;
	u8 prox_pulse_cnt;
	u8 prox_pulse_len;
};
struct tmd2725_light_parameters {
	u8  als_time;
	u16 als_deltaP;
	u8  als_gain;
	u32 d_factor;
	u8  which_seg;
	u8  panel_id;
	struct tmd2725_lux_segment lux_segment[2];
};
struct tmd2725_parameters {
	u8 prox_thres_near;
	u8 prox_thres_far;
	u8 prox_thres_oil_near;
	u8 prox_thres_oil_far;
	u16 prox_raw;
	u16 prox_th_min;
	u16 prox_th_max;
	s16 poffset;
	u8 prox_gain;
	u8 prox_drive;
	u8 persist;
	s16 prox_offset;
	u8 prox_pulse_cnt;
	u8 prox_pulse_len;
	u8 prox_oil_state;

	u8  als_time;
	u8  wait_time;
	u16 als_deltaP;
	u8  als_gain;
	int64_t d_factor;
	u16 als_th_low;
	u16 als_th_high;
	u8  which_seg;
	u8  panel_id;
	u8 cfg1;
	u8 cfg3;
	struct tmd2725_lux_segment lux_segment[2];
};

struct tmd2725_als_info {
	u16 als_ch0; // photopic channel
	u16 als_ch1; // ir channel
	u32 cpl;
	int64_t lux1_ch0_coef;
	int64_t lux1_ch1_coef;
	int64_t lux2_ch0_coef;
	int64_t lux2_ch1_coef;
	u32 saturation;
	u16 lux;
	u16 cal_lux;
	u16 prev_lux;
};

struct tmd2725_prox_info {
	u16 raw;
    u8  prox_raw_debug;
    int last_detected;
	int detected;
};

struct light_parameter {
	int lux;
};
struct tmd2725_als_cal_data{
	struct light_parameter base;
	struct light_parameter cur;
	int flag;
};

struct tmd2725_chip {
	struct device *als_dev;
	struct device *ps_dev;
	struct workqueue_struct *ps_workqueue;
	struct work_struct  ps_work;
	struct work_struct  ps_irq_work;
	struct delayed_work als_work;
	struct input_dev *p_idev;
	struct input_dev *a_idev;
	struct mutex ps_lock;
	struct mutex als_lock;
	struct i2c_client *client;
	struct wake_lock ps_wlock;

	struct pinctrl *pinctrl;
	struct pinctrl_state *pin_default;
	struct pinctrl_state *pin_sleep;

	struct tmd2725_platform_data *pdata;
	struct tmd2725_parameters params;
	struct tmd2725_als_parameters als_params;
	struct tmd2725_prox_info prx_inf;
	struct tmd2725_als_info als_inf;
	struct tmd2725_als_cal_data als_cal_data;

	u8 shadow[MAX_REGS];
	u8 device_index;

	int irq_gpio;
	int irq;
	bool irq_enabled;
	bool wake_irq;

	int  als_poll_delay;
	int  alsps_poll_delay;
	bool als_enabled;
	bool als_gain_auto;
	bool amsIndoorMode;

	u8   ps_status;
	bool prox_enabled;
	bool amsCalComplete;
	bool amsFirstProx;
	bool wakeup_from_suspend;

	enum tmd2725_prox_state prox_state;
};

struct tmd2725_platfotm_ops {
	int (*platform_power)(struct tmd2725_chip *chip, bool on);
	int (*platform_init)(struct tmd2725_chip *chip);
	int (*platform_exit)(struct tmd2725_chip *chip);
};
struct tmd2725_platform_data {
	struct tmd2725_platfotm_ops *ops;
	struct mutex lock;
	struct mutex i2c_lock;
	struct regulator *vdd;
	struct regulator *vio;
	u8    power_state;
	u8    power_always_on;
	u8    debug_level;
	char  *als_name;
	char  *prox_name;
	bool  has_als;
	bool  has_ps;
};
#endif /* __TMD2725_H */
