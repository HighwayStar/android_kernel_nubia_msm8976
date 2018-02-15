/*
 * This file is part of the PA22A sensor driver.
 * PA22A is combined proximity, and VCSEL.
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
-----------   	------------     	-----------------------------------   	------------------
2016/7/11       Bao QI              Pa22a code refcatoring                   V2.0
==========================================================================================
*/

#include "pa22_common.h"
#include "pa22_als.h"

/* Global Variant */
static int ALSTemp[10];
static int ALSTempIndex = 0;
static int ALSAVGStart = 0;

/* pa22a als range coefficient */
static int pa22_als_range_coeff[4] = {149,1363,2833,13897};

/* range */
static dev_t pa22_als_dev_t;
static struct class *als_class;

static int pa22_get_als_range(struct pa22_als_data *data)
{
        int err = 0;
		u8 regdata = 0;
        struct pa22_platform_data *pdata = data->platform_data;
        struct i2c_client *client = pdata->client;

        err = i2c_read_reg(client, REG_CFG0, &regdata);
		if (err < 0) {
			SENSOR_LOG_ERROR("i2c read failed.\n");
			return -1;
		}
        return ((regdata >> 4) & 0x03);
}
static int pa22_als_set_enable(struct pa22_als_data *data, int enable)
{
        int err = 0;
        u8 regdata = 0;
        struct i2c_client *client;
        if (unlikely(IS_ERR_OR_NULL(data))) {
            SENSOR_LOG_ERROR("null pointer.\n");
            return -PTR_ERR(data);
        }

        client= data->platform_data->client;
        mutex_lock(&data->platform_data->reg_lock);
        err = i2c_read_reg(client, REG_CFG0, &regdata);
		if (err < 0) {
			SENSOR_LOG_ERROR("i2c read failed\n");
            mutex_unlock(&data->platform_data->reg_lock);
			return err;
		}
        regdata &= ~(ALS_ACTIVE);
        err = i2c_write_reg(client, REG_CFG0, regdata | enable);
		if (err < 0) {
			SENSOR_LOG_ERROR("i2c read failed\n");
            mutex_unlock(&data->platform_data->reg_lock);
			return err;
		}
        mutex_unlock(&data->platform_data->reg_lock);
        return err;

}
static int pa22_als_init_client(struct pa22_als_data *data)
{
        int err;
        struct pa22_platform_data *pdata = data->platform_data;
        struct i2c_client *client = pdata->client;
        /* Dealy time setting */
        data->als_poll_delay = PA22_ALS_POLL_DELAY;
        data->als_enable_delay = PA22_ALS_ENABLE_DELAY;
        data->als_debug = false;
        pa22_als_calidata_init(data);
        /* Initialize Sensor */
        err = i2c_write_reg(client, REG_CFG0, (PA2A_AFSR << 4));
        if (err < 0)
            return err;
        return 0;
}
static int pa22_als_enable(struct pa22_als_data *data, int enable, bool poll_enable)
{
        int err;
        struct pa22_platform_data *pdata = data->platform_data;

        data->als_enable = enable;
        if (enable) {
            if (!pdata->vdd_always_on) {
				sensor_platform_hw_power_on(pdata, true);
				pa22_als_init_client(data);
				mutex_lock(&pdata->lock);
				pdata->enable_mask |= 1 << ALS;
				mutex_unlock(&pdata->lock);
            }
            err = pa22_als_set_enable(data, 1);
            if (err < 0) {
                SENSOR_LOG_ERROR("enable als fail.\n");
                return err;
            }

            if (ALS_POLLING && poll_enable)
                schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));

        } else {
            err = pa22_als_set_enable(data, 0);
            if (err < 0) {
                SENSOR_LOG_ERROR("disable als fail.\n");
                return err;
            }
            if (!pdata->vdd_always_on) {
				mutex_lock(&pdata->lock);
				pdata->enable_mask &= ~(1 << ALS);
				mutex_unlock(&pdata->lock);
				sensor_platform_hw_power_on(pdata, false);
            }
			cancel_delayed_work(&data->als_dwork);
        }
        return 0;
}

static int pa22_get_als_value(struct pa22_als_data *data)
{
        u8 lsb = 0, msb = 0;
        int als_adc = 0;
        int i = 0;
        int sum = 0;
        int err = 0;

        struct pa22_platform_data *pdata = data->platform_data;
        struct i2c_client *client = pdata->client;

        mutex_lock(&data->als_lock);
        err = i2c_read_reg(client, REG_ALS_DATA_LSB, &lsb);
        if (err < 0) {
            mutex_unlock(&data->als_lock);
            return -ERR_DEV_OPS;
        }
        err = i2c_read_reg(client, REG_ALS_DATA_MSB, &msb);
        if (err < 0) {
            mutex_unlock(&data->als_lock);
            return -ERR_DEV_OPS;
        }
        als_adc = ( (msb << 8) | lsb);
        mutex_unlock(&data->als_lock);

        SENSOR_LOG_INFO("ALS ADC Count=%d\n", als_adc);

        if (ALS_AVG_ENABLE) {
            if(ALSAVGStart) {
                ALSAVGStart=0;
                ALSTempIndex=0;
                for (i = 0; i < 10; i++)
                    ALSTemp[i]=als_adc;
            } else {
                if(ALSTempIndex<=9)
                    ALSTempIndex++;
                else {
                    ALSTempIndex=0;
                    ALSTemp[ALSTempIndex]=als_adc;
                    for (i = 0; i < 10; i++)
                        sum += ALSTemp[i];
                    return (sum/10);
                }
            }
        }
        return als_adc;
}

static int pa22_get_lux_value(struct pa22_als_data *data)
{
        int lux = 0;
        int als_adc = pa22_get_als_value(data);
        int range = pa22_get_als_range(data);
        lux = (als_adc * pa22_als_range_coeff[range]) >> 10;
		/*ensure to avoid invalid lux*/
		lux = lux < 0 ? data->als_prev_lux : lux;
		lux = lux > 10000 ? 10000 :lux;
		data->als_cal_lux = lux * SCALE_FACTOR(data->lux_cal_data.cur.lux,
		        data->lux_cal_data.base.lux);
		data->als_lux = lux;
		data->als_prev_lux = lux;
        SENSOR_LOG_INFO("lux = %d\n", lux);
        return lux;
}
/*
 * Initialization function
 */
static void pa22_als_calidata_init(struct pa22_als_data *data)
{
		data->lux_cal_data.cur.lux = 1;
		data->lux_cal_data.base.lux = 1;
		data->lux_cal_data.flag = 0;
}
static int pa22_als_cal_init(struct pa22_als_data *data)
{
		int err = 0;
        err = pa22_read_file(LUX_CAL_PATH,
                               (char *)&(data->lux_cal_data),
                               sizeof(struct light_cal_parameter));
        if (err < 0) {
			SENSOR_LOG_ERROR("read factory cal parameters failed\n");
        }
        if (data->lux_cal_data.cur.lux == 0) {
			pa22_als_calidata_init(data);
        }
		return err;
}
/*
 * core calibration algo impletation
 */
static int pa22_als_sample_lux(struct pa22_als_data *data)
{
		int err = 0;
		int sum = 0, mean = 0;
		int retry_times = 5;
        int sample_times = retry_times;
        SENSOR_LOG_INFO("sample lux...\n");
		while(retry_times--) {
			err = pa22_get_lux_value(data);
			sum += err;
			msleep(data->als_poll_delay);
		}
        mean = sum/sample_times;
        SENSOR_LOG_INFO("mean = %d\n", mean);
		return mean;
}
static int pa22_als_calibrate_work(struct pa22_als_data *data, const char *cal_data)
{
		int err = 0;
        int als_enable = 0;
		if (unlikely(cal_data == NULL)) {
			SENSOR_LOG_ERROR("NULL\n");
			return -1;
		}
        als_enable = data->als_enable;
		if (!data->als_enable) {
			err = pa22_als_enable(data, 1, false);
			if (err < 0) {
				SENSOR_LOG_ERROR("enable failed.\n");
				goto als_cal_exit;
			}
		}

		/*copy mem directly instead of parse string*/
		memcpy(&data->lux_cal_data.base, cal_data, sizeof(data->lux_cal_data.base));
		memcpy(&data->lux_cal_data.cur, cal_data, sizeof(data->lux_cal_data.cur));

		err = pa22_als_sample_lux(data);
		if (err < 0) {
			SENSOR_LOG_ERROR("get mean lux value error\n");
			goto als_cal_exit;
		}
		data->lux_cal_data.cur.lux = (err > 0) ? err : 1;

		SENSOR_LOG_INFO("lux_cal_data.base.lux = %d\n", data->lux_cal_data.base.lux);
		SENSOR_LOG_INFO("lux_cal_data.cur.lux = %d\n", data->lux_cal_data.cur.lux);

		data->lux_cal_data.flag = (data->lux_cal_data.base.lux > 0) ? 1 : 0;

		if (data->lux_cal_data.flag) {
			mutex_lock(&data->als_lock);
			data->als_cal_lux= data->als_prev_lux * SCALE_FACTOR(data->lux_cal_data.base.lux,
				data->lux_cal_data.cur.lux);
			mutex_unlock(&data->als_lock);
		} else {
			pa22_als_calidata_init(data);
		}

		SENSOR_LOG_INFO("lux_cal_data.flag = %d\n", data->lux_cal_data.flag);

		err = pa22_write_file(LUX_CAL_PATH,
                                (const char *)&(data->lux_cal_data),
                                sizeof(struct light_cal_parameter));
		if (err < 0) {
			SENSOR_LOG_ERROR("save rgb cal parameters failed\n");
			goto als_cal_exit;
		}

als_cal_exit:
		err = pa22_als_enable(data, als_enable, false);
		if (err < 0) {
			SENSOR_LOG_ERROR("disable failed.\n");
		}
		return err;
}

/* For HAL to Enable ALS */
static ssize_t pa22_enable_als_sensor_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct pa22_als_data *data = dev_get_drvdata(dev);
        return sprintf(buf, "%d\n", data->als_enable);
}
static ssize_t pa22_enable_als_sensor_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
        struct pa22_als_data *data = dev_get_drvdata(dev);
        unsigned long val = simple_strtoul(buf, NULL, 10);

        SENSOR_LOG_INFO("enable als sensor ( %ld)\n", val);

        if ((val != 0) && (val != 1)) {
                SENSOR_LOG_INFO("enable als sensor=%ld\n", val);
                return count;
        }

        pa22_als_enable(data, val, true);

        return count;
}
static ssize_t pa22_als_poll_delay_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct pa22_als_data *data = dev_get_drvdata(dev);
        return sprintf(buf, "%d\n", data->als_poll_delay);
}

static ssize_t pa22_als_poll_delay_store(struct device *dev,
                                        struct device_attribute *attr, const char *buf, size_t count)
{
        struct pa22_als_data *data = dev_get_drvdata(dev);
        unsigned long val = simple_strtoul(buf, NULL, 10);

        if (val < 10)
			val = 10;

        data->als_poll_delay = (unsigned int)val;
        return count;
}


/* ALS Value */
static ssize_t pa22_als_lux_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct pa22_als_data *data = dev_get_drvdata(dev);
        return sprintf(buf, "%d\n", data->als_lux);
}
static ssize_t pa22_als_chip_id_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
		return sprintf(buf, "%s\n", CHIP_NAME_ALS);
}
/*calibrate*/
static ssize_t pa22_als_calibrate_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
        struct pa22_als_data *data = dev_get_drvdata(dev);
		return sprintf(buf, "%d\n", data->lux_cal_data.flag);
}
static ssize_t pa22_als_calibrate_store(struct device *dev,
										struct device_attribute *attr, const char *buf, size_t count)
{
		int err = 0;
		int val = 0;
		struct pa22_als_data *data = dev_get_drvdata(dev);
		err = kstrtoint(buf, 0, &val);
		if (err < 0) {
			SENSOR_LOG_ERROR("kstrtoint failed\n");
			return err;
		}
		err = pa22_als_calibrate_work(data, (const char *)&val);
		if (err < 0) {
			SENSOR_LOG_ERROR("als calibrate fail.\n");
		}

		return count;
}
static ssize_t pa22_als_factory_calibrate_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
		return 0;
}
static ssize_t pa22_als_factory_calibrate_store(struct device *dev,
                                        struct device_attribute *attr, const char *buf, size_t count)
{
		int err = 0;
		struct pa22_als_data *data = dev_get_drvdata(dev);

		err = pa22_als_calibrate_work(data, buf);
		if (err < 0) {
			SENSOR_LOG_ERROR("als calibrate fail.\n");
		}

		return count;
}
static ssize_t pa22_als_debug_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
        struct pa22_als_data *data = dev_get_drvdata(dev);
        return sprintf(buf, "%d-%d\n", data->als_prev_lux, data->als_cal_lux);
}
static ssize_t pa22_als_debug_store(struct device *dev,
										struct device_attribute *attr, const char *buf, size_t count)
{
		int err = 0;
		int val = 0;
		struct pa22_als_data *data = dev_get_drvdata(dev);
		err = kstrtoint(buf, 0, &val);
		if (err < 0) {
			SENSOR_LOG_ERROR("kstrtoint failed\n");
			return err;
		}
		SENSOR_LOG_INFO("val = %d\n", val);
		if (val)
			data->als_debug = true;
		else 
			data->als_debug = false;
		return count;
}
static ssize_t pa22_als_dev_init_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
        return 0;
}
static ssize_t pa22_als_dev_init_store(struct device *dev,
										struct device_attribute *attr, const char *buf, size_t count)
{
		int err = 0;
		struct pa22_als_data *data = dev_get_drvdata(dev);
		err = pa22_als_cal_init(data);
		if (err > 0) {
			 SENSOR_LOG_INFO("als dev init ok.\n");
		}
		return count;
}

static struct device_attribute attrs_als_device[] = {
		__ATTR(enable, 0664, pa22_enable_als_sensor_show, pa22_enable_als_sensor_store),
		__ATTR(delay, 0664, pa22_als_poll_delay_show, pa22_als_poll_delay_store),
		__ATTR(light_value, 0444, pa22_als_lux_show, NULL),
		__ATTR(calibrate, 0664, pa22_als_calibrate_show, pa22_als_calibrate_store),
		__ATTR(fac_calibrate, 0444, pa22_als_factory_calibrate_show, pa22_als_factory_calibrate_store),
		__ATTR(dev_init, 0664, pa22_als_dev_init_show, pa22_als_dev_init_store),
		__ATTR(chip_name, 0444, pa22_als_chip_id_show, NULL),
		__ATTR(debug, 0664, pa22_als_debug_show, pa22_als_debug_store),
};
/*work que function*/
static void pa22_work_func_light(struct work_struct *work)
{
        struct pa22_als_data *data = container_of(work, struct pa22_als_data, als_dwork.work);
        int lux;

        lux = pa22_get_lux_value(data);
        if (data->als_debug)
			SENSOR_LOG_INFO("als_lux =%d : als_cal_lux = %d\n", data->als_lux,
			data->als_cal_lux);

        if (!data->lux_cal_data.flag)
			input_report_rel(data->als_input_dev, REL_X, data->als_lux + 1);
        else
			input_report_rel(data->als_input_dev, REL_X, data->als_cal_lux + 1);
        input_sync(data->als_input_dev);

        if(ALS_POLLING && data->als_enable)
			schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));

}

/*Suspend/Resume*/
static int pa22_als_suspend(struct pa22_platform_device *dev_node)
{
        struct pa22_als_data *data;
        data = container_of(dev_node, struct pa22_als_data, device_node);
        if (data && data->als_enable) {
            SENSOR_LOG_INFO("enter\n");
			cancel_delayed_work(&data->als_dwork);
        }
        SENSOR_LOG_INFO("exit\n");
        return 0;
}

static int pa22_als_resume(struct pa22_platform_device *dev_node)
{
        struct pa22_als_data *data;
        data = container_of(dev_node, struct pa22_als_data, device_node);
        if (data && data->als_enable) {
            SENSOR_LOG_INFO("enter\n");
            schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
        }
        SENSOR_LOG_INFO("exit\n");
        return 0;
}
static int pa22_als_hw_init(struct pa22_platform_data *pdata)
{
        struct pa22_platform_device *pdev_node;
        struct pa22_als_data *data;
        if (unlikely(IS_ERR_OR_NULL(pdata))) {
            SENSOR_LOG_ERROR("null pointer exception\n");
			return -ENODEV;
        }
        list_for_each_entry(pdev_node, &pdata->hw_device_list, node) {
            /*Device Initialize*/
            if (pdev_node->handle == ALS) {
                data = container_of(pdev_node, struct pa22_als_data, device_node);
                if(data)
                    pa22_als_init_client(data);
            }
        }
        return 0;
}

static void pa22_als_hw_exit(struct pa22_platform_data *pdata)
{
        if (unlikely(IS_ERR_OR_NULL(pdata))) {
            SENSOR_LOG_ERROR("null pointer exception\n");
			return;
        }
}
static int pa22_als_input_device_init(struct pa22_als_data *data)
{
        int err = 0;
        if (unlikely(IS_ERR_OR_NULL(data))) {
            SENSOR_LOG_ERROR("null pointer exception\n");
			return -ENODEV;
        }

		/* allocate light input_device */
        data->als_input_dev = input_allocate_device();
        if (IS_ERR_OR_NULL(data->als_input_dev)) {
			err = -ENOMEM;
			SENSOR_LOG_ERROR("could not allocate input device\n");
			goto exit;
        }

        data->als_input_dev->name = DEV_NAME_ALS;
        set_bit(EV_REL, data->als_input_dev->evbit);
        set_bit(REL_X,  data->als_input_dev->relbit);

		//ALS INPUT regist
        err = input_register_device(data->als_input_dev);
        if (err) {
			err = -ENOMEM;
			SENSOR_LOG_ERROR("Unable to register input device als: %s\n",
				data->als_input_dev->name);
			goto input_register_err;
		}
		return err;

input_register_err:
		input_free_device(data->als_input_dev);
exit:
		return err;
}

static struct pa22_platform_ops als_platform_ops = {
		.init = pa22_als_hw_init,
		.exit = pa22_als_hw_exit,
        .resume = pa22_als_resume,
        .suspend = pa22_als_suspend,
        .unregister = pa22_als_unregister,
};

/*
 * I2C init/probing/exit functions
 */
int pa22_als_register(struct pa22_platform_data *pdata, struct i2c_driver *driver)
{
        int err = 0;
        u8 tmp = 0;
        struct pa22_als_data *data;
        if (unlikely(IS_ERR_OR_NULL(pdata))) {
            SENSOR_LOG_ERROR("pdata is null.\n");
            return -EINVAL;
        }
        SENSOR_LOG_INFO("start als probe\n");

        err = i2c_read_reg(pdata->client, REG_ALS_DATA_LSB, &tmp);
        if (err < 0) {
            SENSOR_LOG_ERROR("do not support als\n");
            goto exit;
        }

        data = kzalloc(sizeof(struct pa22_als_data), GFP_KERNEL);
		if (IS_ERR_OR_NULL(data)) {
			SENSOR_LOG_ERROR("Failed to allocate memory\n");
			err = -ENOMEM;
			goto exit;
		}

		/* add client data to list */
		data->device_node.handle = ALS;
		data->device_node.name = "als";
		data->device_node.ops = &als_platform_ops;

		/* initialize pa22_als_data */        
		data->platform_data = pdata;
        INIT_DELAYED_WORK(&data->als_dwork, pa22_work_func_light);

		als_class = class_create(THIS_MODULE, "light");
		alloc_chrdev_region(&pa22_als_dev_t, 0, 1, DEV_NAME_ALS);
		data->als_dev = device_create(als_class, NULL, pa22_als_dev_t, driver, DEV_NAME_ALS);
		if (IS_ERR(data->als_dev)) {
			err = -EINVAL;
			SENSOR_LOG_ERROR("device_create als failed\n");
			goto exit_alloc_failed;
		}

		dev_set_drvdata(data->als_dev, data);
		err = create_sysfs_interfaces(data->als_dev, attrs_als_device, ARRAY_SIZE(attrs_als_device));
		if (err < 0) {
			SENSOR_LOG_ERROR("create sysfs interfaces failed\n");
			goto create_sysfs_interface_error;
		}

        mutex_init(&data->als_lock);

        SENSOR_LOG_INFO("registering light sensor input device\n");
		err = pa22_als_input_device_init(data);
		if (err < 0) {
			SENSOR_LOG_ERROR("input device init fail.\n");
			goto exit_probe;
		}

        /* probe finish, and add als client to list */
		list_add_tail(&data->device_node.node, &pdata->hw_device_list);

        /* h/w initialization */
        if (data) {
		    err = data->device_node.ops->init(pdata);
            if (err < 0)
                goto exit_probe;
        }
        SENSOR_LOG_INFO("als probe finish\n");
        return 0;

exit_probe:
		list_del(&data->device_node.node);
		mutex_destroy(&data->als_lock);
create_sysfs_interface_error:
		remove_sysfs_interfaces(data->als_dev, attrs_als_device, ARRAY_SIZE(attrs_als_device));
exit_alloc_failed:
		data->als_dev = NULL;
		device_destroy(als_class, pa22_als_dev_t);
		class_destroy(als_class);
exit:
		return err;
}

static int pa22_als_unregister(struct pa22_platform_device *pdev_node)
{
        struct pa22_als_data *data;
        if(unlikely(IS_ERR_OR_NULL(pdev_node)))
            return -EINVAL;

        data = container_of(pdev_node, struct pa22_als_data, device_node);
        if (data->als_enable)
			pa22_als_enable(data, 1, true);
        list_del(&data->device_node.node);
        input_unregister_device(data->als_input_dev);
        input_free_device(data->als_input_dev);
        remove_sysfs_interfaces(data->als_dev, attrs_als_device, ARRAY_SIZE(attrs_als_device));
        mutex_destroy(&data->als_lock);
        kfree(data);
        return 0;
}
