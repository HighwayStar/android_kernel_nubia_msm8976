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
#include "ams_common.h"
#include "ams_tmd2725.h"

int sensor_read_file(char *file_path, char *read_buf, int count)
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

int sensor_write_file(char *file_path, const char *write_buf, int count)
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

int sensor_create_sysfs_interfaces(struct device *dev,
	struct device_attribute *attr, int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, attr + i))
			goto exit;
	return 0;
exit:
	for (; i >= 0 ; i--)
		device_remove_file(dev, attr + i);
	SENSOR_LOG_ERROR("failed to create sysfs interface\n");
	return -ENODEV;
}

void sensor_remove_sysfs_interfaces(struct device *dev,
	struct device_attribute *attr, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, attr + i);
}

int sensor_regulator_configure(struct tmd2725_chip *chip, bool on)
{
	int rc;
	if (unlikely(IS_ERR_OR_NULL(chip->pdata))) {
		SENSOR_LOG_ERROR("null pointer.\n");
		rc = -PTR_ERR(chip->pdata);
		return rc;
	}

	if (!on) {
		if(!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			if (regulator_count_voltages(chip->pdata->vdd) > 0)
				regulator_set_voltage(chip->pdata->vdd, 0, POWER_VDD_MAX_UV);
			regulator_put(chip->pdata->vdd);
			regulator_disable(chip->pdata->vdd);
		}
		if(!IS_ERR_OR_NULL(chip->pdata->vio)) {
			if (regulator_count_voltages(chip->pdata->vio) > 0)
				regulator_set_voltage(chip->pdata->vio, 0, POWER_VIO_MAX_UV);
			regulator_put(chip->pdata->vio);
			regulator_disable(chip->pdata->vio);
		}
	} else {
		chip->pdata->vdd = regulator_get(&chip->client->dev, "vdd");
		if (IS_ERR_OR_NULL(chip->pdata->vdd)) {
			rc = -PTR_ERR(chip->pdata->vdd);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			if (regulator_count_voltages(chip->pdata->vdd) > 0) {
				rc = regulator_set_voltage(chip->pdata->vdd,
					POWER_VDD_MIN_UV, POWER_VDD_MAX_UV);
				if (rc) {
					SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",
						rc);
					rc = - EINVAL;
					goto reg_vdd_put;
				}
			}

			rc = regulator_enable(chip->pdata->vdd);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator enable vdd failed. rc=%d\n", rc);
				rc = - EINVAL;
				goto reg_vdd_put;
			}
			SENSOR_LOG_INFO("vdd regulator ok\n");
		}

		chip->pdata->vio = regulator_get(&chip->client->dev, "vio");
		if (IS_ERR_OR_NULL(chip->pdata->vio)) {
			rc = -PTR_ERR(chip->pdata->vio);
			SENSOR_LOG_ERROR("Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vio)) {
			if (regulator_count_voltages(chip->pdata->vio) > 0) {
				rc = regulator_set_voltage(chip->pdata->vio,
					POWER_VIO_MIN_UV, POWER_VIO_MAX_UV);
				if (rc) {
					SENSOR_LOG_ERROR("Regulator set failed vdd rc=%d\n",
						rc);
					rc = - EINVAL;
					goto reg_vio_put;
				}
			}

			rc = regulator_enable(chip->pdata->vio);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator enable vdd failed. rc=%d\n", rc);
				rc = - EINVAL;
				goto reg_vio_put;
			}
			SENSOR_LOG_INFO("vio regulator ok\n");
		}
	}
	return 0;
reg_vio_put:
	regulator_put(chip->pdata->vio);
reg_vdd_put:
	regulator_put(chip->pdata->vdd);
	return rc;
}


int sensor_regulator_power_on(struct tmd2725_chip *chip, bool on)
{
	int rc = 0;

	if (IS_ERR_OR_NULL(chip->pdata))
		return -EINVAL;

	if (!on) {
		if (!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			rc = regulator_disable(chip->pdata->vdd);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vio)) {
			rc = regulator_disable(chip->pdata->vio);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator vdd disable failed rc=%d\n", rc);
				return rc;
			}
		}
	} else {
		if (!IS_ERR_OR_NULL(chip->pdata->vdd)) {
			rc = regulator_enable(chip->pdata->vdd);
			if (rc) {
				SENSOR_LOG_ERROR(
					"Regulator vdd enable failed rc=%d\n", rc);
				return rc;
			}
		}
		if (!IS_ERR_OR_NULL(chip->pdata->vio)) {
			rc = regulator_enable(chip->pdata->vio);
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

int sensor_hw_pinctrl_init(struct tmd2725_chip *chip, struct device *dev)
{
	int rc;
	if (unlikely(IS_ERR_OR_NULL(chip) || IS_ERR_OR_NULL(dev)))
		return -EINVAL;

	chip->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(chip->pinctrl)) {
		SENSOR_LOG_ERROR("data->pinctrl is NULL\n");
		return PTR_ERR(chip->pinctrl);
	}

	chip->pin_default = pinctrl_lookup_state(chip->pinctrl, "tmd2725_default");
	if (IS_ERR_OR_NULL(chip->pin_default)) {
		SENSOR_LOG_ERROR("lookup default state failed\n");
		return PTR_ERR(chip->pin_default);
	}

	chip->pin_sleep = pinctrl_lookup_state(chip->pinctrl, "tmd2725_sleep");
	if (IS_ERR_OR_NULL(chip->pin_sleep)) {
		SENSOR_LOG_ERROR("lookup sleep state failed\n");
		return PTR_ERR(chip->pin_sleep);
	}

	if (!IS_ERR_OR_NULL(chip->pinctrl)) {
		rc = pinctrl_select_state(chip->pinctrl, chip->pin_default);
		if (rc) {
			SENSOR_LOG_ERROR("select default state failed\n");
			return rc;
		}
	}
	SENSOR_LOG_INFO("pinctrl init success\n");
	return 0;
}

void sensor_irq_enable(struct tmd2725_chip *data, bool enable, bool flag_sync)
{
	SENSOR_LOG_DEBUG_IF(data->pdata->debug_level, "irq %s\n",enable ? "enable" : "disable");
	if (enable == data->irq_enabled) {
		SENSOR_LOG_DEBUG("doubule %s irq %d\n",enable? "enable" : "disable",
			data->irq);
		return;
	} else {
			data->irq_enabled = enable;
	}
	if (enable) {
		enable_irq(data->irq);
	} else {
		if (flag_sync) {
			disable_irq(data->irq);
		} else {
			disable_irq_nosync(data->irq);
		}
	}
}
void sensor_quick_sort(int *pdata, int len, int left, int right)
{
	int i, j, tmp, t;

	if(left > right || IS_ERR_OR_NULL(pdata) || len == 0)
		return;
	tmp = pdata[left];
	i = left;
	j = right;
	while(i != j){
		while(pdata[j] >= tmp && i < j)
			j--;
		while(pdata[i] <= tmp && i < j)
			i++;

		if (i < j) {
			t = pdata[i];
			pdata[i] = pdata[j];
			pdata[j] = t;
		}
	}
	pdata[left] = pdata[i];
	pdata[i] = tmp;
	sensor_quick_sort(pdata, len, left, i-1);
	sensor_quick_sort(pdata, len, i + 1, right);
}

