/* drivers/input/keyreset.c
 *
 * Copyright (C) 2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/input.h>
#include <linux/keyreset.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/of.h>
#include <soc/qcom/restart.h>
#include <linux/kernel.h>


struct keyreset_state {
	struct input_handler input_handler;
	unsigned long keybit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long upbit[BITS_TO_LONGS(KEY_CNT)];
	unsigned long key[BITS_TO_LONGS(KEY_CNT)];
	spinlock_t lock;
	int key_down_target;
	int key_down;
	int key_up;
	int restart_disabled;
	int need_panic;
	int (*reset_fn)(void);
	int (*dump_fn)(int);
	int dump_pressed;
	struct delayed_work key_down_work;
	int delay;
	int key_is_down;
	struct work_struct key_up_work;
	struct  workqueue_struct *wq;
};

#define DEFAULT_PANIC_KEY_DOWN_DELAY	20

int restart_requested;
static void deferred_restart(struct work_struct *dummy)
{
	restart_requested = 2;
	sys_sync();
	restart_requested = 3;
	kernel_restart(NULL);
}
static DECLARE_WORK(restart_work, deferred_restart);

static void do_key_down(struct work_struct *work)
{
	char *killer = NULL;

	printk(KERN_EMERG"Keyreset: Trigger a crash!!!\n");
	panic_on_oops = 1;
	msm_set_dload_mode(1);
	wmb();
	*killer = 1;	/* force a NULL pointer panic */
	//panic("Kernel Panic trigger by keyboard!!!\n");
	//while(1);
}

static void do_key_up(struct work_struct *work)
{
}

static void keyreset_event(struct input_handle *handle, unsigned int type,
			   unsigned int code, int value)
{
	unsigned long flags;
	struct keyreset_state *state = handle->private;

	if (type != EV_KEY)
		return;

	if (code >= KEY_MAX)
		return;

	if (!test_bit(code, state->keybit))
		return;

	spin_lock_irqsave(&state->lock, flags);
	if (!test_bit(code, state->key) == !value)
		goto done;
	__change_bit(code, state->key);
	if (test_bit(code, state->upbit)) {
		if (value) {
			state->restart_disabled = 1;
			state->key_up++;
		} else
			state->key_up--;
	} else {
		if (value)
			state->key_down++;
		else
			state->key_down--;
	}
	if (state->key_down == 0 && state->key_up == 0)
		state->restart_disabled = 0;

	pr_debug("reset key changed %d %d new state %d-%d-%d\n", code, value,
		 state->key_down, state->key_up, state->restart_disabled);

	if (value && !state->restart_disabled &&
	    state->key_down == state->key_down_target) {
		state->restart_disabled = 1;
		state->key_is_down = 1;
		if (state->need_panic) {
			if (!queue_delayed_work(state->wq, &state->key_down_work, state->delay))
				pr_debug("Key down work already queued!\n");
		}

	}else if (state->key_is_down == 1) {
		if (!cancel_delayed_work(&state->key_down_work)) {
			queue_work(state->wq, &state->key_up_work);
		}
		state->key_is_down = 0;
	}

	if (state->key_down == state->key_down_target) {
		state->dump_pressed = 1;
		if (state->dump_fn)
			restart_requested = state->dump_fn(1);
	} else if (state->dump_pressed == 1) {
		state->dump_pressed = 0;
		if (state->dump_fn)
			restart_requested = state->dump_fn(0);
	}

done:
	spin_unlock_irqrestore(&state->lock, flags);
}

static int keyreset_connect(struct input_handler *handler,
					  struct input_dev *dev,
					  const struct input_device_id *id)
{
	int i;
	int ret;
	struct input_handle *handle;
	struct keyreset_state *state =
		container_of(handler, struct keyreset_state, input_handler);

	for (i = 0; i < KEY_MAX; i++) {
		if (test_bit(i, state->keybit) && test_bit(i, dev->keybit))
			break;
	}
	if (i == KEY_MAX)
		return -ENODEV;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "keyreset";
	handle->private = state;

	ret = input_register_handle(handle);
	if (ret)
		goto err_input_register_handle;

	ret = input_open_device(handle);
	if (ret)
		goto err_input_open_device;

	pr_info("using input dev %s for key reset\n", dev->name);

	return 0;

err_input_open_device:
	input_unregister_handle(handle);
err_input_register_handle:
	kfree(handle);
	return ret;
}

static void keyreset_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id keyreset_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};
MODULE_DEVICE_TABLE(input, keyreset_ids);

static int keyreset_probe(struct platform_device *pdev)
{
	int ret;
	int key, *keyp;
	struct keyreset_state *state;
	struct keyreset_platform_data *pdata = pdev->dev.platform_data;
	struct device_node *np = pdev->dev.of_node;
	const struct property *prop;
	const __be32 *val;
	int count, i;
	int delay;

	if (!pdata && !np)
		return -EINVAL;

	if (pdata)
		keyp = pdata->keys_down;

	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	spin_lock_init(&state->lock);
#ifndef CONFIG_OF
	while ((key = *keyp++)) {
		if (key >= KEY_MAX)
			continue;
		state->key_down_target++;
		__set_bit(key, state->keybit);
	}
	if (pdata->keys_up) {
		keyp = pdata->keys_up;
		while ((key = *keyp++)) {
			if (key >= KEY_MAX)
				continue;
			__set_bit(key, state->keybit);
			__set_bit(key, state->upbit);
		}
	}

	if (pdata->reset_fn)
		state->reset_fn = pdata->reset_fn;
	if (pdata->panic_before_reset)
		state->need_panic = 1;
	else
		state->need_panic = 0;
	if (pdata->dump_fn)
		state->dump_fn = pdata->dump_fn;
#else
	prop = of_find_property(np, "keys-down", NULL);
	if (!prop || !prop->value) {
		pr_err("Invalid keys-down");
		kfree(state);
		return -ENOMEM;
	}
	count = prop->length / sizeof(u32);
	val = prop->value;
	if (count > BITS_TO_LONGS(KEY_CNT))
		count = BITS_TO_LONGS(KEY_CNT);
	for (i = 0; i < count; i++) {
		key = (unsigned short)be32_to_cpup(val++);
		printk("key = %x\n", key);
		if (key == KEY_RESERVED || key > KEY_MAX)
			continue;
		state->key_down_target++;
		__set_bit(key, state->keybit);
	}
	state->need_panic = of_property_read_bool(np,
					"keyreset-need-panic");
	if (of_property_read_u32(np,"key_down_delay", &delay)) {
		pr_err("no key_down_delay property set\n");
		delay = DEFAULT_PANIC_KEY_DOWN_DELAY;
	}
	state->delay = msecs_to_jiffies(delay*1000);
#endif
	state->wq = alloc_ordered_workqueue("keyreset_nubia", 0);
	if (!state->wq) {
		kfree(state);
		return -ENOMEM;
	}
	INIT_DELAYED_WORK(&state->key_down_work, do_key_down);
	INIT_WORK(&state->key_up_work, do_key_up);
	state->input_handler.event = keyreset_event;
	state->input_handler.connect = keyreset_connect;
	state->input_handler.disconnect = keyreset_disconnect;
	state->input_handler.name = KEYRESET_NAME;
	state->input_handler.id_table = keyreset_ids;
	ret = input_register_handler(&state->input_handler);
	if (ret) {
		kfree(state);
		return ret;
	}
	platform_set_drvdata(pdev, state);
	return 0;
}

int keyreset_remove(struct platform_device *pdev)
{
	struct keyreset_state *state = platform_get_drvdata(pdev);
	input_unregister_handler(&state->input_handler);
	kfree(state);
	return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id keyreset_dt_match[] = {
	{ .compatible = "nubia,keyreset" },
	{},
};
MODULE_DEVICE_TABLE(of, keyreset_dt_match);
#endif

struct platform_driver keyreset_driver = {
	.probe = keyreset_probe,
	.remove = keyreset_remove,
	.driver		= {
		.name	= KEYRESET_NAME,
		.of_match_table = of_match_ptr(keyreset_dt_match),
		.owner	= THIS_MODULE,
	},

};

static int __init keyreset_init(void)
{
	return platform_driver_register(&keyreset_driver);
}

static void __exit keyreset_exit(void)
{
	return platform_driver_unregister(&keyreset_driver);
}

module_init(keyreset_init);
module_exit(keyreset_exit);
