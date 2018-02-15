/*
*  ti_usbtype_c.c is linux kernel modules for ti usb type c chip
*
 *  Copyright (C) 2015 nubia software department
 *  Xiaojun Xue <xue.xiaojun@zte.com.cn>
 *  Copyright (c) 2015-2020, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>


#define INT_INDEX	0
#define CC_EN		1
#define NUM_GPIO	2


#define GPIO_CC_INT	106
#define GPIO_CC_ENABLE		52


struct ti_usbtypec_chip {
	struct i2c_client *client;
	bool power_enable;
	struct pinctrl *usbtypec_pinctrl;
	struct pinctrl_state *gpio_power_active;
	struct pinctrl_state *gpio_power_supend;
};

struct gpio ti_typec_gpio[NUM_GPIO] = {
	{GPIO_CC_INT, GPIOF_IN, "CC_intr"},
	{GPIO_CC_ENABLE, GPIOF_OUT_INIT_HIGH, "CC_En"}
};


static int ti_usb_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
/*

static int ti_usb_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}
*/
static int ti_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int value;

	value = of_get_named_gpio_flags(np, "ti,rst-gpio", 0, NULL);
	if (value >= 0)
		ti_typec_gpio[INT_INDEX].gpio = value;
	else
		return -ENODEV;
	value = of_get_named_gpio_flags(np, "ti,enable-gpio", 0, NULL);
	if (value >= 0)
		ti_typec_gpio[CC_EN].gpio = value;
	else
		return -ENODEV;

	pr_info("Interrupt GPIO = %d\n", ti_typec_gpio[INT_INDEX].gpio);
	pr_info("CC_ENABLE GPIO = %d\n", ti_typec_gpio[CC_EN].gpio);
	return 0;
}


static int ti_usbtypec_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
	struct ti_usbtypec_chip * usbtypec_data = NULL;
	int ret = 0;


	if (client->dev.of_node) {
		ret = ti_parse_dt(&client->dev);
		if (ret)
			return -ENODEV;
	} else
		return -ENODEV;

	#ifdef CONFIG_ZTEMT_CHARGER_CNTL_TYPEC
	//the charger will control the gpio
	#else
	ret = gpio_request_array(ti_typec_gpio, ARRAY_SIZE(ti_typec_gpio));

	if (ret < 0) {
		dev_err(&client->dev, "gpio_request_array failed");
		return -EINVAL;
	}
	#endif

	if(!i2c_check_functionality(client->adapter,
	I2C_FUNC_SMBUS_BYTE_DATA)){
		dev_err(&client->dev,"no support for i2c rw\n");
		return -EIO;
	}

	usbtypec_data = kzalloc(sizeof(struct ti_usbtypec_chip),GFP_KERNEL);

	if(!usbtypec_data){
		ret = -ENOMEM;
		goto mem_alloc_fail;
	}
	usbtypec_data->client = client;
	i2c_set_clientdata(client, usbtypec_data);
	printk(KERN_ERR"=chip id = 0x%x",ti_usb_read_reg(client,0x01));
	dev_info(&client->dev,"%s,probe successful!!!",__func__);

	return ret;

 mem_alloc_fail:
	return ret;
}

static int ti_usbtypec_remove(struct i2c_client *client)
{
	struct ti_usbtypec_chip *typec_data = i2c_get_clientdata(client);

	typec_data->power_enable=false;
	kfree(typec_data);
	return 0;

}
#ifdef CONFIG_PM
static int ti_usbtypec_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	return ret;
}
static int ti_usbtypec_resume(struct i2c_client *client)
{
	int ret = 0;
	return ret;
}
#else
#define  ti_usbtypec_suspend NULL
#define  ti_usbtypec_resume  NULL
#endif


static const struct i2c_device_id ti_usbtypec_id[] = {
	{ "ti_typec", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ti_usbtypec_id);


#ifdef CONFIG_OF
static struct of_device_id ti_usbtypec_match_table[] = {
	{ .compatible = "ti,usbtypec",},
	{},
};
#else
#define  ti_usbtypec_match_table NULL
#endif

static struct i2c_driver usb_type_c_driver = {
	.driver = {
		.name ="ti_usbtypec",
		.of_match_table = ti_usbtypec_match_table,
	},
	.probe = ti_usbtypec_probe,
	.remove = ti_usbtypec_remove,
#ifdef CONFIG_PM
	.suspend = ti_usbtypec_suspend,
	.resume = ti_usbtypec_resume,
#endif
	.id_table = ti_usbtypec_id,
};

static int  __init usb_type_c_init(void)
{
	return i2c_add_driver(&usb_type_c_driver);
}
static void  __exit usb_type_c_exit(void)
{
	i2c_del_driver(&usb_type_c_driver);
}
module_init(usb_type_c_init);
module_exit(usb_type_c_exit);
MODULE_AUTHOR("xiaojun xue <xue.xiaojun@zte.com.cn>");
MODULE_DESCRIPTION("ti usb type c chip drivers");
MODULE_LICENSE("GPL");
