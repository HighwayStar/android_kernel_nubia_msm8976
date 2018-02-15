/*
 * This file is part of the APDS9922 sensor driver.
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
-----------   	------------     	-----------------------------------   	------------------
2016/7/11       Bao QI              Pa22a code refcatoring                   V2.0
==========================================================================================
*/

#include "apds9922_common.h"

static int apds9922_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int apds9922_remove(struct i2c_client *client);
int sensor_debug = 0;
/*Suspend/Resume*/
static int apds9922_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9922_platform_data *pdata = i2c_get_clientdata(client);
	struct apds9922_platform_device *device_node;
	if (pdata)
	{
		SENSOR_LOG_INFO("enter\n");
		list_for_each_entry(device_node, &pdata->hw_device_list, node)
		{
			device_node->ops->suspend(device_node);
		}
	}

	SENSOR_LOG_INFO("exit\n");
	return 0;
}

static int apds9922_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct apds9922_platform_data *pdata = i2c_get_clientdata(client);
	struct apds9922_platform_device *device_node;
	if (pdata)
	{
		SENSOR_LOG_INFO("enter\n");
		list_for_each_entry(device_node, &pdata->hw_device_list, node)
		{
			device_node->ops->resume(device_node);
		}
	}
	SENSOR_LOG_INFO("exit\n");
	return 0;
}

static int sensor_parse_dt(struct device *dev,struct apds9922_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	unsigned int tmp = 0;
	int rc = 0;

	/* vdd-always-on flag */
	rc = of_property_read_u32(np, "avago,vdd-always-on", &tmp);
	if (rc < 0)
	{
		SENSOR_LOG_ERROR("Unable to read vdd always on flag\n");
		return rc;
	}
	pdata->vdd_always_on = tmp;

	SENSOR_LOG_INFO("vdd always-on flag is %d\n", pdata->vdd_always_on);
	return 0;
}
static int apds9922_read_device_id(struct apds9922_platform_data *data)
{
	int retry_times = 3;
	u8 device_id;
	int res = -1;
	struct i2c_client *client = data->client;
	while (retry_times--)
	{
		res = apds9922_i2c_read_reg(client, APDS9921_DD_PART_ID_ADDR, &device_id);
		if (res >= 0)
		{
			SENSOR_LOG_INFO("device_id = %d\n", device_id);
			if (device_id == 0xB3)
			{
				SENSOR_LOG_INFO("read device id success\n");
				return 0;
			}
		}
	}
	SENSOR_LOG_ERROR("read device id failed\n");
	return res;
}

static const struct i2c_device_id apds9922_id[] = {
	{ APDS9922_DRV_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, apds9922_id);

static struct of_device_id apds9922_match_table[] = {
	{ .compatible = "avago,apda9922",},
	{ },
};

static const struct dev_pm_ops apds9922_pm_ops = {
	.suspend	= apds9922_suspend,
	.resume 	= apds9922_resume,
};


static struct i2c_driver apds9922_driver = {
	.driver = {
		.name	= APDS9922_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = apds9922_match_table,
		.pm = &apds9922_pm_ops,
	},
	.probe	= apds9922_probe,
	.remove	= apds9922_remove,
	 .id_table = apds9922_id,
};
/*
 * I2C init/probing/exit functions
 */

static int apds9922_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct apds9922_platform_data *pdata=client->dev.platform_data;
	int err = 0;

	SENSOR_LOG_INFO("probe start\n");
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
	{
		SENSOR_LOG_ERROR("i2c_check_functionality error");
		err = -ENODEV;
		goto exit_probe;
	}

	if (client->dev.of_node)
	{
		pdata = kzalloc(sizeof(struct apds9922_platform_data),GFP_KERNEL);
		if (!pdata)
		{
			SENSOR_LOG_ERROR("Failed to allocate memory\n");
			err = -ENOMEM;
			goto exit_platform_failed;
		}
		client->dev.platform_data = pdata;

	}

    /* parse device tree */
	err = sensor_parse_dt(&client->dev, pdata);
	if (err < 0)
	{
		SENSOR_LOG_ERROR("sensor_parse_dt() err\n");
		goto exit_platform_failed;
	}

	/* set platform data */
	mutex_init(&pdata->lock);
	mutex_init(&pdata->reg_lock);
	pdata->client = client;
	i2c_set_clientdata(client, pdata);
	INIT_LIST_HEAD(&pdata->hw_device_list);

	err = apds9922_sensor_regulator_configure(pdata, true);
	if (err)
	{
		SENSOR_LOG_ERROR("unable to configure regulator\n");
		goto exit_platform_failed;
	}

    /* read device id */
	err = apds9922_read_device_id(pdata);
	if (err < 0)
	{
		SENSOR_LOG_ERROR("apds9922_read_device_id failed\n");
		goto exit_platform_failed;
	}
	pdata->debug_enable = false;
 /* register als sensor */
	SENSOR_LOG_INFO("support als.\n");
	err = apds9922_als_register(pdata, &apds9922_driver);
	if (err < 0)
	{
		SENSOR_LOG_ERROR("register als fail\n");
		goto exit_platform_failed;
	}

	/* register ps sensor */
	SENSOR_LOG_INFO("support ps.\n");
	err = apds9922_ps_register(pdata, &apds9922_driver);
	if (err < 0)
	{
		SENSOR_LOG_ERROR("register ps fail\n");
		goto exit_platform_failed;
	}

	/* power off ic when probe finsihed if need */
	if (!pdata->vdd_always_on)
	{
		apds9922_sensor_platform_hw_power_on(pdata, false);
	}

	SENSOR_LOG_INFO("probe ok.\n");

	return 0;

exit_platform_failed:
	apds9922_sensor_regulator_configure(pdata, false);
exit_probe:
	return err;
}

static int apds9922_remove(struct i2c_client *client)
{
	struct apds9922_platform_data *pdata = i2c_get_clientdata(client);
	struct apds9922_platform_device *device_node;
	if (pdata)
	{
		SENSOR_LOG_INFO("enter\n");
		list_for_each_entry(device_node, &pdata->hw_device_list, node)
		{
			device_node->ops->unregister(device_node);
		}
	}
	apds9922_sensor_regulator_configure(pdata, false);
	mutex_destroy(&pdata->lock);
	mutex_destroy(&pdata->reg_lock);
	if(pdata)
		kfree(pdata);
	return 0;
}

static int __init apds9922_init(void)
{
	return i2c_add_driver(&apds9922_driver);
}

static void __exit apds9922_exit(void)
{
	i2c_del_driver(&apds9922_driver);
}

MODULE_AUTHOR("Sensor Team, TXC");
MODULE_DESCRIPTION("PA224 proximity sensor driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(apds9922_init);
module_exit(apds9922_exit);
