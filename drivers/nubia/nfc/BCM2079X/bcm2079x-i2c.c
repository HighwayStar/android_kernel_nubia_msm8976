/*
 * Copyright (C) 2012 Broadcom Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/version.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>
#include <linux/syscalls.h>
#include <linux/wakelock.h>

#include "bcm2079x.h"

/* do not change below */
#define MAX_BUFFER_SIZE		780

/* for DEBUG */
//#define DEBUG_NFC	//DEBUG SWITCH
#define LOG_TAG "NFC-bcm2079x"

#define NFC_LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
                                        LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define NFC_LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
                                        LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_NFC
#define NFC_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
                                        LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define NFC_LOG_DEBUG(fmt, args...)
#endif

/* Read data */
#define PACKET_HEADER_SIZE_NCI	(4)
#define PACKET_HEADER_SIZE_HCI	(3)

#define PACKET_TYPE_NCI		(16)
#define PACKET_TYPE_HCIEV	(4)
#define MAX_PACKET_SIZE		(PACKET_HEADER_SIZE_NCI + 255)
#define BCM2079X_STATE_ACTIVE  "bcm2079x_pin_active"
#define BCM2079X_STATE_SUSPEND "bcm2079x_pin_suspend"

struct bcm2079x_dev_t {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice bcm2079x_device;
	unsigned int wake_gpio;
	unsigned int en_gpio;
	unsigned int irq_gpio;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	unsigned int count_irq;
	struct wake_lock wakelock;
};

static struct clk *clk_rf;

/* add by liuyunli for compile
static void bcm2079x_init_stat(struct bcm2079x_dev_t *bcm2079x_dev)
{
	bcm2079x_dev->count_irq = 0;
}
*/
static void bcm2079x_disable_irq(struct bcm2079x_dev_t *bcm2079x_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->irq_enabled) {
		NFC_LOG_DEBUG("disable_irq.\n");
		disable_irq_nosync(bcm2079x_dev->client->irq);
		bcm2079x_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

static void bcm2079x_enable_irq(struct bcm2079x_dev_t *bcm2079x_dev)
{
	unsigned long flags;
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (!bcm2079x_dev->irq_enabled) {
		NFC_LOG_DEBUG("enable_irq.\n");
		bcm2079x_dev->irq_enabled = true;
		bcm2079x_dev->count_irq = 0;
		enable_irq(bcm2079x_dev->client->irq);
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
}

/*
 The alias address 0x79, when sent as a 7-bit address from the host processor
 will match the first byte (highest 2 bits) of the default client address
 (0x1FA) that is programmed in bcm20791.
 When used together with the first byte (0xFA) of the byte sequence below,
 it can be used to address the bcm20791 in a system that does not support
 10-bit address and change the default address to 0x38.
 the new address can be changed by changing the CLIENT_ADDRESS below if 0x38
 conflicts with other device on the same i2c bus.
 */
#define ALIAS_ADDRESS	  0x79

static int change_client_addr(struct bcm2079x_dev_t *bcm2079x_dev, int addr)
{
	struct i2c_client *client;
	int ret;
	int i;
	char addr_data[] = {
		0xFA, 0xF2, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00, 0x2A
	};
	client = bcm2079x_dev->client;
	client->addr = ALIAS_ADDRESS;
	client->flags &= ~I2C_CLIENT_TEN;

	addr_data[5] = addr & 0xFF;
	ret = 0;
	for (i = 1; i < sizeof(addr_data) - 1; ++i)
		ret += addr_data[i];
	addr_data[sizeof(addr_data) - 1] = (ret & 0xFF);
	NFC_LOG_INFO("Change client device from (0x%04X) flag = %04x, addr_data[%zu] = %02x\n",
		 client->addr, client->flags, sizeof(addr_data) - 1,
		 addr_data[sizeof(addr_data) - 1]);
	ret = i2c_master_send(client, addr_data, sizeof(addr_data));
	if (ret != sizeof(addr_data)) {
		client->addr = ALIAS_ADDRESS;
		client->flags &= ~I2C_CLIENT_TEN;
		NFC_LOG_INFO("Change client device from (0x%04X) flag = %04x, addr_data[%zu] = %02x\n",
			 client->addr, client->flags, sizeof(addr_data) - 1,
			 addr_data[sizeof(addr_data) - 1]);
		ret = i2c_master_send(client, addr_data, sizeof(addr_data));
	}
	client->addr = addr_data[5];
	NFC_LOG_INFO("client device changed to (0x%04X) flag = %04x, ret = %d\n",
		 client->addr, client->flags, ret);
	return (ret == sizeof(addr_data) ? 0 : -EIO);
}

static irqreturn_t bcm2079x_dev_irq_handler(int irq, void *dev_id)
{
	struct bcm2079x_dev_t *bcm2079x_dev = dev_id;
	unsigned long flags;

	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	bcm2079x_dev->count_irq++;
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);
	wake_lock_timeout(&bcm2079x_dev->wakelock, HZ);
	NFC_LOG_DEBUG("bcm2079x_dev_irq_handler, count_irq = %d\n",
			bcm2079x_dev->count_irq);
	wake_up(&bcm2079x_dev->read_wq);

	return IRQ_HANDLED;
}

static unsigned int bcm2079x_dev_poll(struct file *filp, poll_table *wait)
{
	struct bcm2079x_dev_t *bcm2079x_dev = filp->private_data;
	unsigned int mask = 0;
	unsigned long flags;

	NFC_LOG_INFO("poll wait, irq count %d.\n", bcm2079x_dev->count_irq);
	poll_wait(filp, &bcm2079x_dev->read_wq, wait);

	if (bcm2079x_dev->count_irq > 1)
		NFC_LOG_ERROR("error after poll_wait, count_irq=%d.\n",
			bcm2079x_dev->count_irq);
	spin_lock_irqsave(&bcm2079x_dev->irq_enabled_lock, flags);
	if (bcm2079x_dev->count_irq > 0) {
		bcm2079x_dev->count_irq--;
		mask |= POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&bcm2079x_dev->irq_enabled_lock, flags);

	return mask;
}

static ssize_t bcm2079x_dev_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *offset)
{
	struct bcm2079x_dev_t *bcm2079x_dev = filp->private_data;
	unsigned char tmp[MAX_BUFFER_SIZE];
	int total;
//	dev_info(&bcm2079x_dev->client->dev,"bcm2079x_dev_read\n");

	total = 0;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&bcm2079x_dev->read_mutex);
	/** Raw mode Read the full length and return.**/
	total = i2c_master_recv(bcm2079x_dev->client, tmp, count);
	mutex_unlock(&bcm2079x_dev->read_mutex);

	if (total != count) {
		NFC_LOG_ERROR("failed in i2c bus driver, total = 0x%x count = %d\n",
				total, (int)count);
		total = -EIO;
	} else if (copy_to_user(buf, tmp, total)) {
		NFC_LOG_ERROR("copy failed, total = %d\n", total);
		total = -EFAULT;
	}
	NFC_LOG_DEBUG("total:%d count:%zu, 0x%02x, 0x%02x, 0x%02x \n",
			 total, count, tmp[0], tmp[1], tmp[2]);
	return total;
}

static ssize_t bcm2079x_dev_write(struct file *filp, const char __user *buf,
				   size_t count, loff_t *offset)
{
	struct bcm2079x_dev_t *bcm2079x_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
//	dev_info(&bcm2079x_dev->client->dev,"bcm2079x_dev_write\n");

	if (count > MAX_BUFFER_SIZE) {
		NFC_LOG_ERROR("failed,out of memory.count=%zu .\n", count);
		return -ENOMEM;
	}

	if (copy_from_user(tmp, buf, count)) {
		NFC_LOG_ERROR("copy failed, count=%zu\n", count);
		return -EFAULT;
	}

	mutex_lock(&bcm2079x_dev->read_mutex);
	/* Write data */
	ret = i2c_master_send(bcm2079x_dev->client, tmp, count);
	mutex_unlock(&bcm2079x_dev->read_mutex);

	if (ret != count) {
		NFC_LOG_ERROR("failed in i2c bus driver, ret= %d\n", ret);
		ret = -EIO;
	}
	NFC_LOG_DEBUG("count:%zu, 0x%02x, 0x%02x, 0x%02x",
			 count, tmp[0], tmp[1], tmp[2]);
	return ret;
}

static int bcm2079x_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	struct bcm2079x_dev_t *bcm2079x_dev = container_of(filp->private_data,
							   struct bcm2079x_dev_t,
							   bcm2079x_device);
	filp->private_data = bcm2079x_dev;
	//bcm2079x_init_stat(bcm2079x_dev);
	bcm2079x_enable_irq(bcm2079x_dev);
	NFC_LOG_INFO("dev_open  %d,%d , count_irq=%d\n",
			imajor(inode), iminor(inode), bcm2079x_dev->count_irq);
	return ret;
}

static long bcm2079x_dev_unlocked_ioctl(struct file *filp,
					 unsigned int cmd, unsigned long arg)
{
	struct bcm2079x_dev_t *bcm2079x_dev = filp->private_data;
	//dev_info(&bcm2079x_dev->client->dev,"bcm2079x_dev_unlocked_ioctl, cmd = %x \n", cmd);
	switch (cmd) {
		case BCMNFC_READ_FULL_PACKET:
			NFC_LOG_ERROR("BCMNFC_READ_FULL_PACKET (%x, %lx)\n",
								cmd, arg);
			break;
		case BCMNFC_READ_MULTI_PACKETS:
			NFC_LOG_ERROR("BCMNFC_READ_MULTI_PACKETS (%x, %lx)\n",
								cmd, arg);
			break;
		case BCMNFC_CHANGE_ADDR:
			NFC_LOG_INFO("BCMNFC_CHANGE_ADDR (%x, %lx)\n",
							cmd, arg);
			change_client_addr(bcm2079x_dev, arg);
			break;
		case BCMNFC_POWER_CTL:
			NFC_LOG_INFO("BCMNFC_POWER_CTL (%x, %lx)\n",
							cmd, arg);
			gpio_set_value(bcm2079x_dev->en_gpio, arg);
			bcm2079x_dev->count_irq = 0;
			break;
		case BCMNFC_WAKE_CTL:
			NFC_LOG_INFO("BCMNFC_WAKE_CTL (%x, %lx)\n",
							cmd, arg);
			gpio_set_value(bcm2079x_dev->wake_gpio, arg);
			break;
		case BCMNFC_READ_MODE:
			NFC_LOG_INFO("get read mode[0x%x] \n", cmd);
			return READ_RAW_PACKET;
		default:
			NFC_LOG_ERROR("unknown cmd (%x, %lx)\n",
						cmd, arg);
		return 0;
	}
	return 0;
}

static const struct file_operations bcm2079x_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.poll = bcm2079x_dev_poll,
	.read = bcm2079x_dev_read,
	.write = bcm2079x_dev_write,
	.open = bcm2079x_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = bcm2079x_dev_unlocked_ioctl,
#endif
	.unlocked_ioctl = bcm2079x_dev_unlocked_ioctl
};

struct bcm2079x_pinctrl_info {
	struct pinctrl *pinctrl;
	struct pinctrl_state *nfc_gpio_state_active;
	struct pinctrl_state *nfc_gpio_state_suspend;
};
#ifdef CONFIG_OF
static struct bcm2079x_platform_data *
bcm2079x_of_init(struct i2c_client *client)
{
	struct bcm2079x_platform_data *pdata;
	struct device_node *np = client->dev.of_node;

	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		NFC_LOG_ERROR("bcm20797x_platform_data allocation fail.\n");
		return NULL;
	}

	pdata->wake_gpio = of_get_gpio(np, 0);
	pdata->irq_gpio = of_get_gpio(np, 1);
	pdata->en_gpio = of_get_gpio(np, 2);

	return pdata;
}
#else
static inline struct bcm2079x_platform_data *
			bcm2079x_of_init(struct i2c_client *client)
{
	return NULL;
}
#endif

static struct bcm2079x_pinctrl_info bcm2079x_pctrl;

static int nfc_pinctrl_init(struct device *dev)
{
	bcm2079x_pctrl.pinctrl = devm_pinctrl_get(dev);

	if (IS_ERR_OR_NULL(bcm2079x_pctrl.pinctrl)) {
		NFC_LOG_ERROR("Getting pinctrl handle failed.\n");
		return -EINVAL;
	}
	bcm2079x_pctrl.nfc_gpio_state_active = pinctrl_lookup_state(
                                                 bcm2079x_pctrl.pinctrl,
                                                 BCM2079X_STATE_ACTIVE);

	if (IS_ERR_OR_NULL(bcm2079x_pctrl.nfc_gpio_state_active)) {
		NFC_LOG_ERROR("Failed to get the active state pinctrl handle.\n");
		return -EINVAL;
	}
	bcm2079x_pctrl.nfc_gpio_state_suspend = pinctrl_lookup_state(
						 bcm2079x_pctrl.pinctrl,
						BCM2079X_STATE_SUSPEND);

	if (IS_ERR_OR_NULL(bcm2079x_pctrl.nfc_gpio_state_active)) {
		NFC_LOG_ERROR("Failed to get the suspend state pinctrl handle.\n");
		return -EINVAL;
	}
	return 0;
}

static int bcm2079x_parse_dt(struct device *dev,
	struct bcm2079x_platform_data * pdata)
{
	struct device_node *np = dev ->of_node;
	int clk_gpio;
	clk_gpio = of_get_named_gpio(np, "broadcom,clk-gpio", 0);
	pdata->en_gpio =
		of_get_named_gpio_flags(np, "broadcom,en-gpio", 0, NULL);
	pdata->wake_gpio =
		of_get_named_gpio_flags(np, "broadcom,wake-gpio", 0, NULL);
	pdata->irq_gpio =
		of_get_named_gpio_flags(np, "broadcom,irq-gpio", 0, NULL);
	gpio_request(clk_gpio, "gpio");
	gpio_direction_input(clk_gpio);
	NFC_LOG_INFO("clk_gpio: %d, en_gpio: %d, wake_gpio: %d, irq_gpio %d.\n",
		clk_gpio, pdata ->en_gpio, pdata ->wake_gpio, pdata ->irq_gpio);
	return 0;
}
#ifdef ZTEMT_FOR_NFC_PIN_TEST
static ssize_t attr_nfc_nable_setting(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	gpio_set_value(platform_data->en_gpio, val);
	return size;
}

static ssize_t attr_nfc_wake_setting(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	unsigned long val;
	//struct bcm2079x_dev *bcm2079x_dev;
	//bcm2079x_dev = container_of(bcm2079x_dev,dev,);
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	gpio_set_value(platform_data->wake_gpio, val);
	return size;
}

static struct device_attribute attributes[] = {
	//set nfc_enable pin to 1-high or 0-low
	__ATTR(nfc_nable_setting, 0644, NULL, attr_nfc_nable_setting),
	__ATTR(nfc_wake_setting,0644, NULL, attr_nfc_wake_setting),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	NFC_LOG_ERROR("Unable to create interface.\n");
	return -1;
}

#endif

static void  bcm2079x_clk_enable(struct device *dev)
{
	int ret = -1;

	clk_rf = clk_get(dev, "ref_clk");
	if (IS_ERR(clk_rf)) {
		NFC_LOG_ERROR("failed to get nfc clk:ref_clk.\n");
		return;
	}
	ret = clk_prepare(clk_rf);
	if (ret) {
		NFC_LOG_ERROR("failed to call clk_prepare, ret = %d\n",ret);
		return;
	}
	NFC_LOG_INFO("success get and prepare nfc clk.\n ");
	return;
}

static void bcm2079x_clk_disable(void)
{
	if (IS_ERR(clk_rf)) {
		NFC_LOG_ERROR("disable clock error, skiped.\n");
		return ;
	}
	clk_unprepare(clk_rf);
	clk_put(clk_rf);
	clk_rf = NULL;
	NFC_LOG_INFO("success release nfc clk.\n");
	return ;
}
static int bcm2079x_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	int ret;
	struct bcm2079x_dev_t *bcm2079x_dev;
	struct bcm2079x_platform_data *platform_data;

	NFC_LOG_INFO("probing bcm2079x driver start, flags = %x\n",
							client->flags);
	if (client->dev.of_node)
		platform_data = bcm2079x_of_init(client);
	else
		platform_data = client->dev.platform_data;
	if (platform_data == NULL) {
		NFC_LOG_ERROR("probe fail with null platform_data.\n");
		return -ENOMEM;
	}
	bcm2079x_parse_dt(&client ->dev, platform_data);
	//dump_stack();
	nfc_pinctrl_init(&client->dev);

//	ret = pinctrl_select_state(bcm2079x_pctrl.pinctrl,
//				   bcm2079x_pctrl.nfc_gpio_state_active);
//	if (ret)
//		NFC_LOG_ERROR("cannot set pin to nfc_gpio_state_active state");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		NFC_LOG_ERROR("probe fail with i2c function error.\n");
		return -ENODEV;
	}

	ret = gpio_request_one(platform_data->irq_gpio, GPIOF_IN, "nfc_int");
	if (ret)
		return -ENODEV;
	ret = gpio_request_one(platform_data->en_gpio, GPIOF_OUT_INIT_LOW, "nfc_en");
	if (ret)
		goto err_en;
	ret = gpio_request_one(platform_data->wake_gpio, GPIOF_OUT_INIT_LOW, "nfc_wake");
	if (ret)
		goto err_wake;

	gpio_set_value(platform_data->en_gpio, 0);
	gpio_set_value(platform_data->wake_gpio, 0);

	bcm2079x_clk_enable(&client->dev);

	bcm2079x_dev = kzalloc(sizeof(*bcm2079x_dev), GFP_KERNEL);
	if (bcm2079x_dev == NULL) {
		NFC_LOG_ERROR("probe failed with allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	bcm2079x_dev->wake_gpio = platform_data->wake_gpio;
	bcm2079x_dev->irq_gpio = platform_data->irq_gpio;
	bcm2079x_dev->en_gpio = platform_data->en_gpio;
	bcm2079x_dev->client = client;

	wake_lock_init(&bcm2079x_dev->wakelock, WAKE_LOCK_SUSPEND, "bcm2079x");

	/* init mutex and queues */
	init_waitqueue_head(&bcm2079x_dev->read_wq);
	mutex_init(&bcm2079x_dev->read_mutex);
	spin_lock_init(&bcm2079x_dev->irq_enabled_lock);

	bcm2079x_dev->bcm2079x_device.minor = MISC_DYNAMIC_MINOR;
	bcm2079x_dev->bcm2079x_device.name = "bcm2079x";
	bcm2079x_dev->bcm2079x_device.fops = &bcm2079x_dev_fops;

	ret = misc_register(&bcm2079x_dev->bcm2079x_device);
	if (ret) {
		NFC_LOG_ERROR("probe failed with misc_register.\n");
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	bcm2079x_dev->irq_enabled = true;
	ret = request_irq(client->irq, bcm2079x_dev_irq_handler,
			  IRQF_TRIGGER_RISING|IRQF_NO_SUSPEND,
			  client->name, bcm2079x_dev);
	if (ret) {
		NFC_LOG_ERROR("probe failed in request_irq func.\n");
		goto err_request_irq_failed;
	}
	if (unlikely(irq_set_irq_wake(client->irq, 1)))
		NFC_LOG_ERROR("unable to make irq %d wakeup.\n", client->irq);
	bcm2079x_disable_irq(bcm2079x_dev);
	i2c_set_clientdata(client, bcm2079x_dev);
#ifdef ZTEMT_FOR_NFC_PIN_TEST
	ret= create_sysfs_interfaces(&client->dev);
	 if (ret < 0) {
		NFC_LOG_ERROR("probe failed in sysfs register.\n");
		return ret;
	}
#endif
	NFC_LOG_INFO("probe successfully, exit.\n");
	return 0;

err_request_irq_failed:
	misc_deregister(&bcm2079x_dev->bcm2079x_device);
err_misc_register:
	mutex_destroy(&bcm2079x_dev->read_mutex);
	wake_unlock(&bcm2079x_dev->wakelock);
	wake_lock_destroy(&bcm2079x_dev->wakelock);

	kfree(bcm2079x_dev);
err_exit:
	gpio_free(platform_data->wake_gpio);
err_wake:
	gpio_free(platform_data->en_gpio);
	bcm2079x_clk_disable();
err_en:
	gpio_free(platform_data->irq_gpio);
	return ret;
}

static int bcm2079x_remove(struct i2c_client *client)
{
	struct bcm2079x_dev_t *bcm2079x_dev;

	bcm2079x_dev = i2c_get_clientdata(client);
	free_irq(client->irq, bcm2079x_dev);
	misc_deregister(&bcm2079x_dev->bcm2079x_device);
	mutex_destroy(&bcm2079x_dev->read_mutex);

	gpio_free(bcm2079x_dev->irq_gpio);
	gpio_free(bcm2079x_dev->en_gpio);
	gpio_free(bcm2079x_dev->wake_gpio);
	wake_unlock(&bcm2079x_dev->wakelock);
	wake_lock_destroy(&bcm2079x_dev->wakelock);
	bcm2079x_clk_disable();
	kfree(bcm2079x_dev);

	return 0;
}

static struct of_device_id bcm2079x_table[] = {
	{ .compatible = "broadcom,bcm2079x_nfc",},
	{ },
};

static const struct i2c_device_id bcm2079x_id[] = {
	{"bcm2079x-i2c", 0},
	{}
};

static struct i2c_driver bcm2079x_driver = {
	.id_table = bcm2079x_id,
	.probe = bcm2079x_probe,
	.remove = bcm2079x_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "bcm2079x-i2c",
		.of_match_table = bcm2079x_table,
	},
};

/*
 * module load/unload record keeping
 */

/*
static int __init bcm2079x_dev_init(void)
{
	return i2c_add_driver(&bcm2079x_driver);
}
module_init(bcm2079x_dev_init);

static void __exit bcm2079x_dev_exit(void)
{
	i2c_del_driver(&bcm2079x_driver);
}
module_exit(bcm2079x_dev_exit);
*/

module_i2c_driver(bcm2079x_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("NFC bcm2079x driver");
MODULE_LICENSE("GPL");
