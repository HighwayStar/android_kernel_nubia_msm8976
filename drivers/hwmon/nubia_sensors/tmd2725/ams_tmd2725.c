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

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>

#include "ams_tmd2725.h"
#include "ams_i2c.h"
#include "ams_common.h"
#include "ams_tmd2725_prox.h"
#include "ams_tmd2725_als.h"

static struct i2c_driver tmd2725_driver;
/* TMD2725 Identifiers */
static u8 const tmd2725_ids[] = {
//	ID,	AUXID,	REV
	0xE4,	0x00,	0x20
};

/* TMD2725 Device Names */
static char const *tmd2725_names[] = {
	DEVICE_CHIP_NAME
};

/* Registers to restore */
static u8 const restorable_regs[] = {
	TMD2725_REG_PILT,
	TMD2725_REG_PIHT,
	TMD2725_REG_PERS,
	TMD2725_REG_PGCFG0,
	TMD2725_REG_PGCFG1,
	TMD2725_REG_CFG1,
	TMD2725_REG_PTIME,
	TMD2725_REG_ATIME,
	TMD2725_REG_WTIME,
	TMD2725_REG_AILT,
	TMD2725_REG_AILT_HI,
	TMD2725_REG_AIHT,
	TMD2725_REG_AIHT_HI,
	TMD2725_REG_CFG3,
};

static int tmd2725_flush_regs(struct tmd2725_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg, chip->shadow[reg]);
		if (rc < 0) {
			SENSOR_LOG_ERROR("err on reg 0x%02x\n",reg);
			break;
		}
	}

	return rc;
}

static int tmd2725_update_enable_reg(struct tmd2725_chip *chip)
{
	return ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_ENABLE,
			chip->shadow[TMD2725_REG_ENABLE]);
}

static void tmd2725_set_defaults(struct tmd2725_chip *chip)
{
	u8 *sh = chip->shadow;
	// Clear the register shadow area
	memset(chip->shadow, 0x00, sizeof(chip->shadow));

	SENSOR_LOG_INFO("use defaults\n");
	chip->params.prox_th_min = PS_THRES_FAR;
	chip->params.prox_th_max = PS_THRES_NEAR;
	chip->params.persist = PRX_PERSIST(1) | ALS_PERSIST(2);
	chip->params.prox_pulse_cnt = 16;
	chip->params.prox_gain = PGAIN_2;
	chip->params.prox_drive = PDRIVE_MA(100);
	chip->params.prox_offset = 0;
	chip->params.prox_pulse_len = PG_PULSE_8US;
	chip->params.als_gain = AGAIN_16;
	chip->params.als_deltaP = 10;
	chip->params.als_time = AW_TIME_MS(100);
	chip->params.als_th_high = 0xFFFF;
	chip->params.als_th_low = 0;
	chip->params.cfg3 = 0x4C;
	chip->params.d_factor = 41;
	chip->params.lux_segment[0].ch0_coef = 1000;
	chip->params.lux_segment[0].ch1_coef = 260;
	chip->params.lux_segment[1].ch0_coef = 800;
	chip->params.lux_segment[1].ch1_coef = 270;
	chip->params.wait_time = WAIT_TIME_MS(10);

	chip->als_gain_auto = true;

	// Copy the default values into the register shadow area
	sh[TMD2725_REG_PILT]    = (chip->params.prox_th_min & 0xff);
	sh[TMD2725_REG_PIHT]    = (chip->params.prox_th_max & 0xff);
	sh[TMD2725_REG_PERS]    = chip->params.persist;
	sh[TMD2725_REG_PGCFG0]  = (chip->params.prox_pulse_cnt - 1) | chip->params.prox_pulse_len;
	sh[TMD2725_REG_ATIME]   = chip->params.als_time;
	sh[TMD2725_REG_CFG1]    = chip->params.als_gain;
	sh[TMD2725_REG_WTIME]    = chip->params.wait_time;
	sh[TMD2725_REG_PTIME]   = P_TIME_US(2816);
	sh[TMD2725_REG_PGCFG1] = (chip->params.prox_gain & TMD2725_MASK_PGAIN) |
		chip->params.prox_drive;
	sh[TMD2725_REG_AILT] = chip->params.als_th_low&0xFF;
	sh[TMD2725_REG_AILT_HI] = (chip->params.als_th_low&0xFF00)>>8;
	sh[TMD2725_REG_AIHT] = chip->params.als_th_high&0xFF;
	sh[TMD2725_REG_AIHT_HI] = (chip->params.als_th_high&0xFF00)>>8;
	sh[TMD2725_REG_CFG3] = chip->params.cfg3;
}

#ifdef ABI_SET_GET_REGISTERS
/* bitmap of registers that are in use */
static u8 reginuse[MAX_REGS / 8] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* 0x00 - 0x3f */
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,	/* 0x40 - 0x7f */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,	/* 0x80 - 0xbf */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,	/* 0xc0 - 0xff */
};

static ssize_t tmd2725_regs_get(struct tmd2725_chip *chip, char *buf, int bufsiz)
{
	u8 regval[16];
	int i, j, cnt;

	// find first
	for (i = 0; i < sizeof(reginuse) / sizeof(reginuse[0]); i++) {
		if (reginuse[i] != 0)
			break;
	}

	i &= ~1;  // round down to the start of a group of 16
	i *= 8;  // set to actual register id

	cnt = 0;
	for (; i < MAX_REGS; i += 16) {
		cnt += snprintf(buf + cnt, bufsiz - cnt, "%02x  ", i);

		ams_i2c_blk_read(chip->client, i, &regval[0], 16);

		for (j = 0; j < 16; j++) {

			if (reginuse[(i >> 3) + (j >> 3)] & (1 << (j & 7))) {
				cnt += snprintf(buf + cnt, bufsiz - cnt, " %02x", regval[j]);
			} else {
				cnt += snprintf(buf + cnt, bufsiz - cnt, " --");
			}

			if (j == 7)
				cnt += snprintf(buf + cnt, bufsiz - cnt, "  ");
		}

		cnt += snprintf(buf + cnt, bufsiz - cnt, "\n");
	}

	cnt += snprintf(buf + cnt, bufsiz - cnt, "\n");
	return cnt;

}

void tmd2725_reg_log(struct tmd2725_chip *chip)
{
	char *buf;

	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (buf) {
		tmd2725_regs_get(chip, &buf[0], PAGE_SIZE);
		SENSOR_LOG_ERROR("%s\n", buf);
		kfree(buf);
	}
	else {
		SENSOR_LOG_ERROR("out of memory!\n");
	}
}

static ssize_t tmd2725_regs_dump(
	struct device *dev, struct device_attribute *attr, char *buf)
{
	return tmd2725_regs_get(dev_get_drvdata(dev), buf, PAGE_SIZE);
}

static ssize_t tmd2725_reg_set(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int preg;
	int pval;
	int pmask = -1;
	int numparams;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	numparams = sscanf(buf, "0x%x:0x%x:0x%x", &preg, &pval, &pmask);
	if (numparams == 0) {
		// try decimal
		numparams = sscanf(buf, "%d:%d:%d", &preg, &pval, &pmask);
	}

	if ( (numparams < 2) || (numparams > 3) )
		return -EINVAL;
	if ( (numparams >= 1) && ((preg < 0) || ((reginuse[(preg >> 3)] & (1 << (preg & 7))) == 0)) )
		return -EINVAL;
	if ( (numparams >= 2) && (preg < 0 || preg > 0xff) )
		return -EINVAL;
	if ( (numparams >= 3) && (pmask < 0 || pmask > 0xff) )
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->lock);

	if (pmask == -1) {
		rc = ams_i2c_write(chip->client, chip->shadow, preg, pval);
	} else {
		rc = ams_i2c_modify(chip, chip->shadow,
			preg, pmask, pval);
	}

	AMS_MUTEX_UNLOCK(&chip->lock);

	return rc ? rc : size;
}

struct device_attribute tmd2725_attrs[] = {
	__ATTR(regs, 0644, tmd2725_regs_dump, tmd2725_reg_set),
};

int tmd2725_attrs_size = ARRAY_SIZE(tmd2725_attrs);
#endif // #ifdef ABI_SET_GET_REGISTERS


static int tmd2725_get_id(struct tmd2725_chip *chip, u8 *id, u8 *rev, u8 *auxid)
{
	int ret = 0;
	ret = ams_i2c_read(chip->client, TMD2725_REG_AUXID, auxid);
	if (ret < 0) {
		SENSOR_LOG_ERROR("i2c read fail\n");
		return ret;
	}

	ret = ams_i2c_read(chip->client, TMD2725_REG_REVID, rev);
	if (ret < 0) {
		SENSOR_LOG_ERROR("i2c read fail\n");
		return ret;
	}
	ret = ams_i2c_read(chip->client, TMD2725_REG_ID, id);
	if (ret < 0) {
		SENSOR_LOG_ERROR("i2c read fail\n");
		return ret;
	}
	return 0;
}

static int tmd2725_parse_dt(struct device *dev,
		struct tmd2725_platform_data *pdata,
		struct tmd2725_chip *data) {

	struct device_node *np = dev->of_node;

	unsigned int tmp = 0;
	int rc = 0;
	bool supported;

	/* irq gpio */
	rc = of_get_named_gpio(dev->of_node,"tmd,irq-gpio", 0);
	if (rc < 0) {
		SENSOR_LOG_ERROR("Unable to read irq gpio\n");
		return rc;
	}
	data->irq_gpio = rc;
	SENSOR_LOG_INFO("irq gpio is %d\n", data->irq_gpio);

	/* ps tuning data*/
	supported = of_property_read_u32(np, "tmd,power_always_on", &tmp);
	pdata->power_always_on = tmp;
	SENSOR_LOG_INFO("power_always_on is %d\n", pdata->power_always_on);

	supported = of_property_read_u32(np, "tmd,has_als", &tmp);
	pdata->has_als = tmp;
	SENSOR_LOG_INFO("has_als is %s\n", pdata->has_als?"true":"false");

	supported = of_property_read_u32(np, "tmd,has_ps", &tmp);
	pdata->has_ps = tmp;
	SENSOR_LOG_INFO("has_ps is %s\n", pdata->has_ps?"true":"false");

	return 0;
}
/*********************
* platform init  *
*********************/
static int tmd2725_platform_hw_power_on(struct tmd2725_chip *chip, bool on)
{
	int err;
	if (unlikely(IS_ERR_OR_NULL(chip->pdata)))
		return -ENODEV;
	if (!chip->pdata->power_always_on) {
		if (chip->pdata->power_state == POWER_OFF) {
			err = sensor_regulator_power_on(chip, on);
		}
	} else {
		err = sensor_regulator_power_on(chip, on);
	}
	return err;
}

static int tmd2725_common_hw_init(struct tmd2725_chip *chip)
{
	int ret;
	tmd2725_set_defaults(chip);

	ret = tmd2725_flush_regs(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("flush reg error\n");
		return -ENODEV;
	}
	SENSOR_LOG_INFO("tmd2725 common hw init finished\n");
	return 0;
}
static int tmd2725_platform_hw_exit(struct tmd2725_chip *chip)
{
	return 0;
}
/********************************************************************
*Validate the appropriate ams device is available for this driver
********************************************************************/
static int tmd2725_check_device_id(struct tmd2725_chip *chip)
{
	int ret;
	u8 id, rev, auxid, i;
	ret = tmd2725_get_id(chip, &id, &rev, &auxid);
	if (ret < 0) {
		SENSOR_LOG_ERROR("i2c read fail\n");
		return -ENODEV;
	}
	SENSOR_LOG_INFO("device id:%02x device aux id:%02x device rev:%02x\n", id, auxid, rev);

	id &= 0xfc; /* clear the 2 LSbits, they indicate the bus voltage */
	rev &= 0xe0; /* clear all but fuse bits */
	for (i = 0; i < ARRAY_SIZE(tmd2725_ids)/3; i++) {
		if (id == (tmd2725_ids[i*3+0]))
			if (auxid == (tmd2725_ids[i*3+1]))
				if (rev == (tmd2725_ids[i*3+2]))
					break;
	}
	if (i < ARRAY_SIZE(tmd2725_names)) {
		SENSOR_LOG_DEBUG("'%s rev. 0x%x' detected\n",
			tmd2725_names[i], rev);
		chip->device_index = i;
	} else {
		SENSOR_LOG_ERROR("not supported chip id\n");
		return -ENODEV;
	}
	return 0;
}
/********************************************************************
*prepare ams device state
********************************************************************/
int tmd2725_device_warm_up(struct tmd2725_chip *chip)
{
	int ret;
	ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_ENABLE, 0x01);
	ret = tmd2725_update_enable_reg(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("update enable reg fail\n");
		return ret;
	}
	return 0;
}
/********************************************************************
* device suspend/resume callback interfaces
********************************************************************/
static int tmd2725_suspend(struct device *dev)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	chip->wakeup_from_suspend = true;
	if (chip->als_enabled)
		cancel_delayed_work(&chip->als_work);

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	}
	return 0;
}

static int tmd2725_resume(struct device *dev)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	if (chip->als_enabled)
		schedule_delayed_work(&chip->als_work, msecs_to_jiffies(0));

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
	}

	return 0;
}
/********************************************************************
* struct tmd2725_platfotm_ops
********************************************************************/
static struct tmd2725_platfotm_ops p_ops = {
	.platform_init = tmd2725_common_hw_init,
	.platform_power = tmd2725_platform_hw_power_on,
	.platform_exit = tmd2725_platform_hw_exit,
};
/********************************************************************
* device probe
********************************************************************/
static int tmd2725_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	int ret;

	struct device *dev = &client->dev;
	static struct tmd2725_chip *chip;
	struct tmd2725_platform_data *pdata = dev->platform_data;
	SENSOR_LOG_INFO("probe start\n");

	chip = kzalloc(sizeof(struct tmd2725_chip),GFP_KERNEL);
	if (IS_ERR_OR_NULL(chip)) {
		SENSOR_LOG_ERROR("alloc failed\n");
		ret = -ENOMEM;
		goto exit;
	}
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		SENSOR_LOG_ERROR("i2c smbus byte data unsupported\n");
		ret = -ENODEV;
		goto exit_alloc_failed;
	}

	if (client->dev.of_node) {
		pdata = kzalloc(sizeof(struct tmd2725_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			SENSOR_LOG_ERROR("Failed to allocate memory\n");
			ret = -ENOMEM;
			goto exit_alloc_failed;
		}
		client->dev.platform_data = pdata;
	}

	wake_lock_init(&chip->ps_wlock, WAKE_LOCK_SUSPEND, "tmd2725");
	chip->client = client;
	chip->pdata = pdata;
	mutex_init(&chip->pdata->lock);
	mutex_init(&chip->pdata->i2c_lock);
	i2c_set_clientdata(client, chip);
	chip->pdata->ops = &p_ops;

	ret = sensor_regulator_configure(chip, true);
	if (ret < 0) {
		SENSOR_LOG_ERROR("unable to configure regulator\n");
		goto exit_alloc_failed;
	}

	ret = sensor_hw_pinctrl_init(chip, &chip->client->dev);
	if (ret < 0) {
		SENSOR_LOG_ERROR("pinctrl init error\n");
		goto exit_regulator_put;
	}

	/* validate if hw exits */
	ret = tmd2725_check_device_id(chip);
	if (ret < 0){
		SENSOR_LOG_ERROR("check device info error\n");
		goto exit_regulator_put;
	}

	/* chip hw init */
	if (pdata->ops->platform_init) {
		ret = pdata->ops->platform_init(chip);
		if (ret < 0)
			goto exit_regulator_put;
	}

	/* parse device tree */
	ret = tmd2725_parse_dt(&client->dev, pdata, chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("parse dt() err\n");
		goto exit_platform_init_failed;
	}

	if (pdata->has_als) {
		pdata->als_name = INPUT_ALS_NAME;
		ret = tmd2725_als_device_register(chip, &tmd2725_driver);
		if (ret < 0) {
			SENSOR_LOG_ERROR("als device register error\n");
			goto exit_platform_init_failed;
		}
	}
	if (pdata->has_ps) {
		pdata->prox_name = INPUT_PS_NAME;
		ret = tmd2725_ps_device_register(chip, &tmd2725_driver);
		if (ret < 0) {
			SENSOR_LOG_ERROR("ps device register error\n");
			goto exit_als_unregister;
		}
	}

	ret = tmd2725_device_warm_up(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("enable als/ps fail.\n");
		goto exit_ps_unregister;
	}

	SENSOR_LOG_INFO("Probe ok.\n");
	return 0;

exit_ps_unregister:
	if (chip->pdata->has_ps)
		tmd2725_ps_device_unregister(chip);
exit_als_unregister:
	if (chip->pdata->has_als)
		tmd2725_als_device_unregister(chip);
exit_platform_init_failed:
	if (chip->pdata->ops->platform_exit)
		chip->pdata->ops->platform_exit(chip);
exit_regulator_put:
	sensor_regulator_configure(chip, false);
exit_alloc_failed:
	wake_lock_destroy(&chip->ps_wlock);
	if (chip->pdata)
		kfree(chip->pdata);
	if (chip)
		kfree(chip);
exit:
	SENSOR_LOG_ERROR("Probe failed.\n");
	return ret;
}

static int tmd2725_remove(struct i2c_client *client)
{
	struct tmd2725_chip *chip = i2c_get_clientdata(client);
	SENSOR_LOG_INFO("\nTMD2725: REMOVE()\n");
	if (chip->pdata->has_ps)
		tmd2725_ps_device_unregister(chip);
	if (chip->pdata->has_als)
		tmd2725_als_device_unregister(chip);
	if (chip->pdata->ops->platform_exit)
		chip->pdata->ops->platform_exit(chip);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return 0;
}

static struct i2c_device_id tmd2725_idtable[] = {
	{ "tmd2725", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tmd2725_idtable);

static const struct dev_pm_ops tmd2725_pm_ops = {
	.suspend = tmd2725_suspend,
	.resume  = tmd2725_resume,
};

static struct i2c_driver tmd2725_driver = {
	.driver = {
		.name = "tmd2725",
		.pm = &tmd2725_pm_ops,
	},
	.id_table = tmd2725_idtable,
	.probe = tmd2725_probe,
	.remove = tmd2725_remove,
};

static int __init tmd2725_init(void)
{
	int rc;
	SENSOR_LOG_INFO("TMD2725: init()\n");
	rc = i2c_add_driver(&tmd2725_driver);
	return rc;
}

static void __exit tmd2725_exit(void)
{
	SENSOR_LOG_INFO("TMD2725: exit()\n");
	i2c_del_driver(&tmd2725_driver);
}

module_init(tmd2725_init);
module_exit(tmd2725_exit);

MODULE_AUTHOR("J. August Brenner<jon.brenner@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tmd2725 ALS, Prox, Gesture, Beam, RemCon sensor driver");
MODULE_LICENSE("GPL");
