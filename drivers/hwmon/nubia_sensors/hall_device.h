#ifndef __HALL_DEVICE_H__
#define __HALL_DEVICE_H__

#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/wakelock.h>
#include <linux/list.h>

#define MAGNETIC_DEVICE_NEAR		1
#define MAGNETIC_DEVICE_FAR		2
#define MAGNETIC_DEVICE_UNKNOW		-1
#define HALL_LABEL_NAME_LEN		32

struct hall_device_irq {
	unsigned int irq_num;
	unsigned int irq_pin;
	bool enabled;
};

struct hall_device_wake_lock {
	struct wake_lock lock;
	bool   locked;
	char   *name;
};

struct hall_hw_device {
	unsigned int state;
	unsigned int code;
	struct list_head node;
	struct hall_device_chip *chip;
	struct hall_device_irq irq;
	struct work_struct irq_work;
	unsigned char label_name[HALL_LABEL_NAME_LEN];
};

struct hall_device_chip {
	struct platform_device *pdev;

	int hall_hw_device_count;
	struct list_head hw_device_list;

	struct mutex lock;
	struct input_dev *idev;

	struct class *hall_device_class;

	struct device *hall_device_dev;

	struct hall_device_wake_lock wakeup_wakelock;

	struct delayed_work flush_work;

	struct hrtimer unlock_wakelock_timer;

	struct regulator *vio;

	bool enabled;
	bool on_irq_working;
};

static void hall_device_get_value_and_report(struct hall_device_chip *chip);
static int hall_device_enable(struct hall_device_chip *chip, int on);
static ssize_t hall_device_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t hall_device_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static void hall_device_irq_enable(struct hall_device_irq * irq, bool enable, bool flag_sync);
static irqreturn_t hall_device_irq(int irq, void *handle);
static void hall_device_irq_work_func(struct work_struct *work);
static int create_sysfs_interfaces(struct device *dev);
static int hall_device_suspend(struct device *dev);
static int hall_device_resume(struct device *dev);
static int hall_device_probe(struct platform_device *pdev);
static int hall_device_remove(struct platform_device *pdev);
static int __init hall_device_init(void);
static void __exit hall_device_exit(void);
static void hall_device_chip_data_init(struct hall_device_chip *chip);
static void hall_device_wakelock_ops(struct hall_device_wake_lock *wakelock, bool enable);
static int hall_device_power_on(struct hall_device_chip *chip, bool on);

#endif /* __HALL_DEVICE_PAIR_OUT_H */
