/*
 * This file is part of the PA224 sensor driver.
 * PA224 is combined proximity, and VCSEL.
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
 *
 *
 when         	who         		Remark : what, where, why          		version
---------   ------------     	-----------------------------------   	------------------
2016/7/11   Bao QI                          Pa22a code refcatoring                      v2.0.0
==========================================================================================*/

#include "pa22_common.h"

/* I2C Read */
int i2c_read_reg(struct i2c_client *client, u8 reg, u8 *buf)
{
	int ret = 0;
	int i = 0;

	struct pa22_platform_data *pdata = client->dev.platform_data;

	mutex_lock(&pdata->lock);

	for (i = 0; i < I2C_RETRY_TIMES; i++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to read i2c addr=%x\n", pdata->client->addr);
			msleep(I2C_RETRY_DELAY);
		} else {
			*buf = (u8) ret;
			mutex_unlock(&pdata->lock);
			return 0;
		}
	}
	mutex_unlock(&pdata->lock);

	return ret;
}
/* I2C Write */
int i2c_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret = 0;
	int i = 0;

	struct pa22_platform_data *pdata = client->dev.platform_data;

	mutex_lock(&pdata->lock);
	for (i = 0; i < I2C_RETRY_TIMES; i++)
	{
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0) {
			SENSOR_LOG_ERROR("failed to write i2c addr=%x\n", pdata->client->addr);
			msleep(I2C_RETRY_DELAY);
		} else {
			mutex_unlock(&pdata->lock);
			return 0;
		}
	}
	mutex_unlock(&pdata->lock);

	return ret;
}

/* Calibration file handle*/
int pa22_read_file(char *file_path, char *read_buf, int count)
{
	struct file *file_p;
	mm_segment_t old_fs;
	int vfs_retval = -EINVAL;
	bool file_exist = true;
	char *buf = NULL;

	SENSOR_LOG_DEBUG("read infomation : size =%d\n", count);
	if (NULL == file_path) {
		SENSOR_LOG_ERROR("file_path is NULL\n");
		return -EINVAL;
	}

	file_p = filp_open(file_path, O_RDONLY , 0444);
	if (IS_ERR(file_p)) {
		file_exist = false;
		SENSOR_LOG_INFO("file does not exist\n");
		buf = kzalloc(count * sizeof(char), GFP_KERNEL);
		if (IS_ERR_OR_NULL(buf)) {
			SENSOR_LOG_ERROR("alloc mem failed\n");
			goto error;
		}
	} else {
		filp_close(file_p, NULL);
	}

	file_p = filp_open(file_path, O_CREAT|O_RDWR , 0666);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (!file_exist) {
		SENSOR_LOG_DEBUG("init file memory\n");
		if (!IS_ERR_OR_NULL(buf)) {
			vfs_retval = vfs_write(file_p, (char *)buf, sizeof(buf), &file_p->f_pos);
			if (vfs_retval < 0) {
				SENSOR_LOG_ERROR("[write file <%s>failed]\n",file_path);
				goto file_close;
			}
		}

	}

	file_p->f_pos = 0;
	vfs_retval = vfs_read(file_p, (char*)read_buf, count, &file_p->f_pos);
	if (vfs_retval < 0) {
		SENSOR_LOG_ERROR("[write file <%s>failed]\n",file_path);
		goto file_close;
	}

	SENSOR_LOG_INFO("read ok\n");

file_close:
	set_fs(old_fs);
	filp_close(file_p, NULL);
error:
	if (!IS_ERR_OR_NULL(buf))
		kfree(buf);
	return vfs_retval;
}

int pa22_write_file(char *file_path, const char *write_buf, int count)
{
	struct file *file_p;
	mm_segment_t old_fs;
	int vfs_retval = -EINVAL;

	SENSOR_LOG_DEBUG("write infomation : size =%d\n", count);
	if (NULL == file_path) {
		SENSOR_LOG_ERROR("file_path is NULL\n");
		return -EINVAL;
	}

	file_p = filp_open(file_path, O_CREAT|O_RDWR|O_TRUNC , 0666);
	if (IS_ERR(file_p)) {
		SENSOR_LOG_ERROR("[open file <%s>failed]\n",file_path);
		goto error;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_retval = vfs_write(file_p, (char*)write_buf, count, &file_p->f_pos);
	if (vfs_retval < 0) {
		SENSOR_LOG_ERROR("[write file <%s>failed]\n",file_path);
		goto file_close;
	}

	SENSOR_LOG_INFO("write ok\n");

file_close:
	set_fs(old_fs);
	filp_close(file_p, NULL);
error:
	return vfs_retval;
}

int create_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count)
{
	int i;
	for (i = 0; i < count; i++)
	{
		if (device_create_file(dev, dev_attrs + i))
			return -ENODEV;
	}
	return 0;
}

void remove_sysfs_interfaces(struct device *dev, struct device_attribute *dev_attrs, int count)
{
	int i;

	for (i = 0; i < count; i++)
		device_remove_file(dev, dev_attrs + i);
	return;
}
int sensor_regulator_configure(struct pa22_platform_data *pdata, bool on)
{
	int rc;

	if (unlikely(IS_ERR_OR_NULL(pdata))) {
		SENSOR_LOG_ERROR("null pointer.\n");
		return PTR_ERR(pdata);
	}

	if (!on) {
		if (regulator_count_voltages(pdata->vdd) > 0)
			regulator_set_voltage(pdata->vdd, 0,
				PA224_VDD_MAX_UV);

		regulator_put(pdata->vdd);
		regulator_disable(pdata->vdd);
	} else {
		pdata->vdd = regulator_get(&pdata->client->dev, "vdd");
		if (IS_ERR_OR_NULL(pdata->vdd)) {
			rc = PTR_ERR(pdata->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (!IS_ERR_OR_NULL(pdata->vdd)) {
		    if (regulator_count_voltages(pdata->vdd) > 0) {
			    rc = regulator_set_voltage(pdata->vdd,
				    PA224_VDD_MIN_UV, PA224_VDD_MAX_UV);
			    if (rc) {
				    SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",
					    rc);
				    goto reg_vdd_put;
			    }
		    }

		    rc = regulator_enable(pdata->vdd);
		    if (rc) {
			    SENSOR_LOG_ERROR(
				    "Regulator enable vdd failed. rc=%d\n", rc);
			    goto reg_vdd_put;
		    }
		}
	}

	return 0;

reg_vdd_put:
	regulator_put(pdata->vdd);
	return rc;
}


static int sensor_regulator_power_on(struct pa22_platform_data *pdata, bool on)
{
	int rc = 0;

	if (!on) {
	    if (!IS_ERR_OR_NULL(pdata->vdd)) {
	        rc = regulator_disable(pdata->vdd);
	        if (rc) {
			    SENSOR_LOG_ERROR(
				    "Regulator vdd disable failed rc=%d\n", rc);
			    return rc;
	        }
	    }
	} else {
	    if (!IS_ERR_OR_NULL(pdata->vdd)) {
	        rc = regulator_enable(pdata->vdd);
	        if (rc) {
			    SENSOR_LOG_ERROR(
				    "Regulator vdd enable failed rc=%d\n", rc);
			    return rc;
	        }
	    }
	}

	SENSOR_LOG_INFO("power state : (%s)\n", on ? "on":"off");

	mdelay(5);

	return rc;
}

int sensor_platform_hw_power_on(struct pa22_platform_data *pdata, bool on)
{
	int err;
	if (unlikely(IS_ERR_OR_NULL(pdata)))
	return -ENODEV;
	if (!pdata->vdd_always_on) {
		if (!pdata->enable_mask) {
			err = sensor_regulator_power_on(pdata, on);
		}
	} else {
		err = sensor_regulator_power_on(pdata, on);
	}
	return err;
}


