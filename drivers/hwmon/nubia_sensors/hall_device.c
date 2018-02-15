/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * Version: 1.4
 * Revision record:
 * 	V1.4:
 * 		Add label name for each hall device when
 * 		request gpio and irq. 2015/07/01
 * 	V1.3:
 * 		optimum hall device wakelock. 2015/03/19
 * 	V1.2:
 * 		Support the old board which have only one hall device.
 * 		Release the wake lock in old board. 2015/03/17
 * 	V1.1:
 * 		Do not regulator_disable when probe. 2015/02/02
 * 	V1.0:
 * 		Add version information. 2015/01/08
 *
 */

#include "hall_device.h"

#define HALL_DRIVER_VERSION "v1.4"

#define LOG_TAG "HALL_DEVICE"
//#define DEBUG_ON

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

/* POWER SUPPLY VOLTAGE RANGE */
#define HALL_VIO_MIN_UV	1750000
#define HALL_VIO_MAX_UV	1950000

static unsigned int hall_code[] = {
	REL_RX,
	REL_RY,
	REL_RZ,
};

static dev_t const hall_device_dev_t = MKDEV(MISC_MAJOR, 252);

static void hall_device_irq_enable(struct hall_device_irq *irq, bool enable, bool flag_sync)
{
	if (enable == irq->enabled) {
		SENSOR_LOG_INFO("doubule %s irq %d, retern here\n",
				enable ? "enable" : "disable", irq->irq_num);
		return;
	}
	irq->enabled = enable;
	SENSOR_LOG_INFO("%s irq %d\n", enable ? "enable" : "disable", irq->irq_num);

	if (enable) {
		enable_irq(irq->irq_num);
	}
	else {
		if (flag_sync)
			disable_irq(irq->irq_num);
		else
			disable_irq_nosync(irq->irq_num);
	}
}

static void hall_device_wakelock_ops(struct hall_device_wake_lock *wakelock, bool enable)
{
	if (enable == wakelock->locked) {
		SENSOR_LOG_INFO("doubule %s %s, retern here\n",
				enable ? "lock" : "unlock", wakelock->name);
		return;
	}
	wakelock->locked = enable;
	SENSOR_LOG_INFO("%s %s\n", enable ? "lock" : "unlock", wakelock->name);

	if (enable)
		wake_lock(&wakelock->lock);
	else
		wake_unlock(&wakelock->lock);
}

static enum hrtimer_restart hall_device_unlock_wakelock_work_func(struct hrtimer *timer)
{
	struct hall_device_chip *chip = container_of(timer,
			struct hall_device_chip, unlock_wakelock_timer);

	if (chip->on_irq_working == false)
		hall_device_wakelock_ops(&(chip->wakeup_wakelock), false);

	return HRTIMER_NORESTART;
}

static void hall_device_get_value_and_report(struct hall_device_chip *chip)
{
	struct hall_hw_device *hw_device;

	list_for_each_entry(hw_device, &chip->hw_device_list, node) {
		hw_device->state = gpio_get_value(hw_device->irq.irq_pin) ?
					MAGNETIC_DEVICE_FAR : MAGNETIC_DEVICE_NEAR;
		SENSOR_LOG_INFO("MAGNETIC_DEVICE gpio %d [%s]\n", hw_device->irq.irq_pin,
				hw_device->state == MAGNETIC_DEVICE_NEAR ? "NEAR" : "FAR");
		input_report_rel(chip->idev, hw_device->code, hw_device->state);
	}
	input_sync(chip->idev);
};

static void hall_device_irq_work_func(struct work_struct *work)
{
	unsigned int state;
	struct hall_hw_device *hw_device =
		container_of(work, struct hall_hw_device, irq_work);
	struct hall_device_chip *chip = hw_device->chip;

	mutex_lock(&chip->lock);
	state = gpio_get_value(hw_device->irq.irq_pin) ?
				MAGNETIC_DEVICE_FAR : MAGNETIC_DEVICE_NEAR;
	if (hw_device->state == state)
		SENSOR_LOG_INFO("gpio %d [%s] same state\n", hw_device->irq.irq_pin,
				state == MAGNETIC_DEVICE_NEAR ? "NEAR" : "FAR");
	else
		hall_device_get_value_and_report(chip);

	if (hw_device->state == MAGNETIC_DEVICE_NEAR)
		hall_device_wakelock_ops(&(chip->wakeup_wakelock), false);
	else
		hrtimer_start(&chip->unlock_wakelock_timer, ktime_set(3, 0), HRTIMER_MODE_REL);

	chip->on_irq_working = false;
	hall_device_irq_enable(&(hw_device->irq), true, true);
	mutex_unlock(&chip->lock);
};

static struct hall_hw_device *hall_get_pending_hw_device(struct hall_device_chip *chip, int pending_irq)
{
	struct hall_hw_device *hw_device;

	list_for_each_entry(hw_device, &chip->hw_device_list, node) {
		if (hw_device->irq.irq_num == pending_irq)
			return hw_device;
	}

	return NULL;
}

static irqreturn_t hall_device_irq(int irq, void *handle)
{
	struct hall_hw_device *hw_device;
	struct hall_device_chip *chip = handle;

	if (!chip->enabled) {
		SENSOR_LOG_ERROR("chip is not enabled, the irq %d no useful\n", irq);
		return IRQ_HANDLED;
	}

	chip->on_irq_working = true;
	hw_device = hall_get_pending_hw_device(chip, irq);
	hall_device_irq_enable(&hw_device->irq, false, false);
	hrtimer_cancel(&chip->unlock_wakelock_timer);
	hall_device_wakelock_ops(&(chip->wakeup_wakelock), true);

	if (schedule_work(&hw_device->irq_work) == 0)
		SENSOR_LOG_ERROR("schedule_work failed!\n");

	return IRQ_HANDLED;
}

static void hall_device_enable_all_irq(struct hall_device_chip *chip)
{
	struct hall_hw_device *hw_device;
	list_for_each_entry(hw_device, &chip->hw_device_list, node)
		hall_device_irq_enable(&hw_device->irq, true, true);
}

static void hall_device_disable_all_irq(struct hall_device_chip *chip)
{
	struct hall_hw_device *hw_device;
	list_for_each_entry(hw_device, &chip->hw_device_list, node)
		hall_device_irq_enable(&hw_device->irq, false, true);
}

static int hall_device_enable(struct hall_device_chip *chip, int on)
{
	int ret = 0;

	SENSOR_LOG_INFO("%s hall_device\n", on ? "enable" : "disable");
	if (on) {
		hall_device_get_value_and_report(chip);
		ret = hall_device_power_on(chip, 1);
		if (ret < 0)
			return ret;
		hall_device_enable_all_irq(chip);
	}
	else {
		hall_device_disable_all_irq(chip);
		ret = hall_device_power_on(chip, 0);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static ssize_t hall_drvier_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", HALL_DRIVER_VERSION);
}

static ssize_t hall_hw_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hall_device_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->hall_hw_device_count);
}

static ssize_t hall_device_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hall_device_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->enabled);
}

static ssize_t hall_device_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	bool enable;
	struct hall_device_chip *chip = dev_get_drvdata(dev);

	if (strtobool(buf, &enable)) {
		SENSOR_LOG_ERROR("get value faield\n");
		return -EINVAL;
	}

	if (enable == chip->enabled) {
		SENSOR_LOG_INFO("already %s\n", enable ? "enable" : "disable");
		return size;
	}

	chip->enabled = enable;
	ret = hall_device_enable(chip, chip->enabled);
	if (ret) {
		chip->enabled = !enable;
		SENSOR_LOG_ERROR("hall device %s failed\n", enable ? "enable" : "disable");
		return -EINVAL;
	}

	return size;
}
static ssize_t hall_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct hall_device_chip *chip = dev_get_drvdata(dev);
	if(IS_ERR_OR_NULL(chip)) {
		SENSOR_LOG_ERROR("chip is null.\n");
		return -EINVAL;
	}
	schedule_delayed_work(&chip->flush_work, msecs_to_jiffies(0));
	return 0;
}

static struct device_attribute attrs_hall_device[] = {
	__ATTR(driver_version, 0444, hall_drvier_version_show, NULL),
	__ATTR(hall_hw_count, 0444, hall_hw_count_show, NULL),
	__ATTR(enable, 0640, hall_device_enable_show, hall_device_enable_store),
	__ATTR(hall_value, 0444, hall_value_show, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(attrs_hall_device); i++) {
		ret = device_create_file(dev, attrs_hall_device + i);
		if (ret)
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attrs_hall_device + i);
	SENSOR_LOG_ERROR("Unable to create interface\n");
	return ret;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attrs_hall_device); i++)
		device_remove_file(dev, attrs_hall_device + i);
}
static void hall_flush_work_func(struct work_struct *work)
{
	struct hall_device_chip *chip = container_of((struct delayed_work *)work, struct hall_device_chip, flush_work);
	if(IS_ERR_OR_NULL(chip))
	{
		SENSOR_LOG_ERROR("chip is null.\n");
		return;
	}
	mutex_lock(&chip->lock);
	hall_device_get_value_and_report(chip);
	mutex_unlock(&chip->lock);
}
static void hall_device_chip_data_init(struct hall_device_chip *chip)
{
	chip->enabled = false;
	chip->wakeup_wakelock.name = "hall_device_wakelock";
	chip->wakeup_wakelock.locked = false;
	mutex_init(&chip->lock);
	INIT_LIST_HEAD(&chip->hw_device_list);
	INIT_DELAYED_WORK(&chip->flush_work, hall_flush_work_func);
}

static int hall_device_parse_dt(struct hall_device_chip *chip)
{
	int i;
	int rc = 0;
	u32 num;
	unsigned char propname[32];
	struct hall_hw_device *hw_device;
	struct device_node *np = chip->pdev->dev.of_node;

	rc = of_property_read_u32(np, "hall_device,hw_device_count", &num);
	if (rc && (rc != -EINVAL)) {
		SENSOR_LOG_ERROR("Unable to read hw_device_count\n");
		return -1;
	}

	if (num > ARRAY_SIZE(hall_code) || num <= 0) {
		SENSOR_LOG_ERROR("hw_device_count error\n");
		return -1;
	}

	chip->hall_hw_device_count = num;
	SENSOR_LOG_INFO("hall_hw_device_count is %d\n", chip->hall_hw_device_count);

	for (i = 0; i < num; i++) {
		snprintf(propname, 32, "%s%d", "hall_device,irq-gpio-", i);
		rc = of_get_named_gpio_flags(np, propname, 0, NULL);
		if (rc < 0) {
			SENSOR_LOG_ERROR("Unable to read irq gpio\n");
			return rc;
		}
		hw_device = kzalloc(sizeof(struct hall_hw_device), GFP_KERNEL);
		if (!hw_device) {
			SENSOR_LOG_ERROR("no enough memory\n");
			return -ENOMEM;
		}
		hw_device->irq.irq_pin = rc;
		SENSOR_LOG_INFO("hw_device[%d] irq_pin is %d\n",
						i, hw_device->irq.irq_pin);
		list_add_tail(&hw_device->node, &chip->hw_device_list);
	}

	return 0;
}

static void hall_device_dt_free(struct hall_device_chip *chip)
{
	struct hall_hw_device *hw_device;
	struct hall_hw_device *hw_device_tmp;
	list_for_each_entry_safe(hw_device, hw_device_tmp,
			&chip->hw_device_list, node) {
		list_del(&hw_device->node);
		kfree(hw_device);
	}
}

static int hall_device_power_init(struct hall_device_chip *chip, bool on)
{
	int rc = 0;

	if (!on) {
		if (regulator_count_voltages(chip->vio) > 0)
			regulator_set_voltage(chip->vio, 0,
				HALL_VIO_MAX_UV);
		regulator_put(chip->vio);
	} else {
		chip->vio = regulator_get(&chip->pdev->dev, "vio");
		if (IS_ERR(chip->vio)) {
			rc = PTR_ERR(chip->vio);
			SENSOR_LOG_ERROR("Regulator get failed vio rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(chip->vio) > 0) {
			rc = regulator_set_voltage(chip->vio,
				HALL_VIO_MIN_UV, HALL_VIO_MAX_UV);
			if (rc) {
				SENSOR_LOG_ERROR("Regulator set failed vio rc=%d\n", rc);
				goto err_vio_set;
			}
		}
	}
	return 0;

err_vio_set:
	regulator_put(chip->vio);
	return rc;
}

static int hall_device_power_on(struct hall_device_chip *chip, bool on)
{
	int rc = 0;

	if (on)
		rc = regulator_enable(chip->vio);
	else
		rc = regulator_disable(chip->vio);
	if (rc)
		SENSOR_LOG_ERROR("Regulator vio %s failed rc=%d\n", on ? "enable" : "disable", rc);

	return rc;
}

static int hall_hw_device_init(struct hall_device_chip *chip)
{
	int i = 0;
	int ret = 0;
	struct hall_hw_device *hw_device;

	list_for_each_entry(hw_device, &chip->hw_device_list, node) {
		snprintf(hw_device->label_name, HALL_LABEL_NAME_LEN, "%s%d", "hall_device_irq_pin", i);
		ret = gpio_request_one(hw_device->irq.irq_pin, GPIOF_DIR_IN, hw_device->label_name);
		if (ret) {
			SENSOR_LOG_ERROR("gpio %d is busy and then to free it\n", hw_device->irq.irq_pin);
			gpio_free(hw_device->irq.irq_pin);
			ret = gpio_request_one(hw_device->irq.irq_pin, GPIOF_DIR_IN, hw_device->label_name);
			if (ret) {
				SENSOR_LOG_ERROR("Failed to request gpio %d\n", hw_device->irq.irq_pin);
				return -1;
			}
		}

		hw_device->irq.irq_num = gpio_to_irq(hw_device->irq.irq_pin);
		ret = request_threaded_irq(hw_device->irq.irq_num, NULL,
				hall_device_irq,
				IRQF_TRIGGER_FALLING |
				IRQF_TRIGGER_RISING |
				IRQF_ONESHOT,
				hw_device->label_name, chip);
		if (ret) {
			SENSOR_LOG_ERROR("Failed to request irq %d\n", hw_device->irq.irq_num);
			return -1;
		}

		hw_device->state = MAGNETIC_DEVICE_UNKNOW;
		hw_device->code = hall_code[i++];
		hw_device->irq.enabled = true;
		hw_device->chip = chip;
		INIT_WORK(&hw_device->irq_work, hall_device_irq_work_func);
	}

	return 0;
}

static void hall_hw_device_deinit(struct hall_device_chip *chip)
{
	struct hall_hw_device *hw_device;
	list_for_each_entry(hw_device, &chip->hw_device_list, node) {
		free_irq(hw_device->irq.irq_num, chip);
		gpio_free(hw_device->irq.irq_pin);
	}
}

static int hall_device_probe(struct platform_device *pdev)
{
	int i;
	int ret = 0;
	struct hall_device_chip *chip = NULL;

	SENSOR_LOG_INFO("probe start\n");

	chip = kzalloc(sizeof(struct hall_device_chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	chip->pdev = pdev;

	hall_device_chip_data_init(chip);

	ret = hall_device_parse_dt(chip);
	if (ret) {
		SENSOR_LOG_ERROR("parse dt failed\n");
		goto parse_dt_failed;
	}

	platform_set_drvdata(pdev, chip);

	chip->hall_device_class = class_create(THIS_MODULE, "hall_device");
	if (IS_ERR(chip->hall_device_class)) {
		ret = PTR_ERR(chip->hall_device_class);
		goto class_create_failed;
	}

	chip->hall_device_dev = device_create(chip->hall_device_class,
			NULL, hall_device_dev_t, NULL, "hall_device");
	if (IS_ERR(chip->hall_device_dev)) {
		ret = PTR_ERR(chip->hall_device_dev);
		goto create_hall_device_failed;
	}

	dev_set_drvdata(chip->hall_device_dev, chip);

	ret = hall_hw_device_init(chip);
	if (ret) {
		SENSOR_LOG_ERROR("hall device init failed\n");
		goto hall_hw_device_init_failed;
	}

	chip->idev = input_allocate_device();
	if (!chip->idev) {
		SENSOR_LOG_ERROR("no memory for idev\n");
		ret = -ENODEV;
		goto input_alloc_failed;
	}
	chip->idev->name = "hall_device";
	chip->idev->id.bustype = BUS_VIRTUAL;

	set_bit(EV_REL,	chip->idev->evbit);
	for (i = 0; i < chip->hall_hw_device_count; i++)
		set_bit(hall_code[i], chip->idev->relbit);

	ret = input_register_device(chip->idev);
	if (ret) {
		SENSOR_LOG_ERROR("cant register input '%s'\n", chip->idev->name);
		goto input_register_failed;
	}

	ret = create_sysfs_interfaces(chip->hall_device_dev);
	if (ret) {
		SENSOR_LOG_ERROR("create sysfs faield\n");
		goto create_sysfs_failed;
	}

	hall_device_disable_all_irq(chip);

	wake_lock_init(&chip->wakeup_wakelock.lock, WAKE_LOCK_SUSPEND, chip->wakeup_wakelock.name);
	hrtimer_init(&chip->unlock_wakelock_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	chip->unlock_wakelock_timer.function = hall_device_unlock_wakelock_work_func;

	ret = hall_device_power_init(chip, 1);
	if (ret < 0)
		goto power_init_failed;

	SENSOR_LOG_INFO("HALL Driver version %s\n", HALL_DRIVER_VERSION);
	SENSOR_LOG_INFO("probe success\n");
	return 0;

power_init_failed:
	wake_lock_destroy(&chip->wakeup_wakelock.lock);
	remove_sysfs_interfaces(chip->hall_device_dev);
create_sysfs_failed:
	input_unregister_device(chip->idev);
	chip->idev = NULL;
input_register_failed:
	input_free_device(chip->idev);
input_alloc_failed:
	hall_hw_device_deinit(chip);
hall_hw_device_init_failed:
	device_destroy(chip->hall_device_class, hall_device_dev_t);
create_hall_device_failed:
	class_destroy(chip->hall_device_class);
class_create_failed:
	hall_device_dt_free(chip);
parse_dt_failed:
	kfree(chip);
malloc_failed:
	SENSOR_LOG_ERROR("probe failed\n");
	return ret;
}

static int hall_device_remove(struct platform_device *pdev)
{
	struct hall_device_chip *chip = platform_get_drvdata(pdev);

	SENSOR_LOG_INFO("hall_device_remove\n");

	hall_device_power_init(chip, 0);
	wake_lock_destroy(&chip->wakeup_wakelock.lock);
	remove_sysfs_interfaces(chip->hall_device_dev);
	input_unregister_device(chip->idev);
	hall_hw_device_deinit(chip);
	device_destroy(chip->hall_device_class, hall_device_dev_t);
	class_destroy(chip->hall_device_class);
	hall_device_dt_free(chip);
	kfree(chip);
	return 0;
}

static int hall_device_resume(struct device *dev)
{
	struct hall_hw_device *hw_device;
	struct hall_device_chip *chip = dev_get_drvdata(dev);

	SENSOR_LOG_DEBUG("enter\n");
	if (chip->enabled) {
		list_for_each_entry(hw_device, &chip->hw_device_list, node)
			disable_irq_wake(hw_device->irq.irq_num);
	}
	SENSOR_LOG_DEBUG("eixt\n");
	return 0 ;
}

static int hall_device_suspend(struct device *dev)
{
	struct hall_hw_device *hw_device;
	struct hall_device_chip *chip = dev_get_drvdata(dev);

	SENSOR_LOG_DEBUG("enter\n");
	if (chip->enabled) {
		list_for_each_entry(hw_device, &chip->hw_device_list, node)
			enable_irq_wake(hw_device->irq.irq_num);
	}
	SENSOR_LOG_DEBUG("eixt\n");
	return 0 ;
}

static struct of_device_id of_hall_device_idtable[] = {
	{.compatible = "nubia,hall_device",},
	{}
};

static const struct dev_pm_ops hall_device_pm_ops = {
	.suspend	= hall_device_suspend,
	.resume		= hall_device_resume,
};

static struct platform_driver hall_plat_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "hall_device",
		.of_match_table = of_hall_device_idtable,
		.pm = &hall_device_pm_ops,
	},
	.probe = hall_device_probe,
	.remove = hall_device_remove,
};

static int __init hall_device_init(void)
{
	return platform_driver_register(&hall_plat_driver);
}

static void __exit hall_device_exit(void)
{
	platform_driver_unregister(&hall_plat_driver);
}

module_init(hall_device_init);
module_exit(hall_device_exit);

MODULE_DESCRIPTION("hall_device driver");
MODULE_AUTHOR("NUBIA");
MODULE_LICENSE("GPL");

