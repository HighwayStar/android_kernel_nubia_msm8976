/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012-2015 Synaptics Incorporated. All rights reserved.
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, SYNAPTICS'
 * TOTAL CUMULATIVE LIABILITY TO ANY PARTY SHALL NOT EXCEED ONE HUNDRED U.S.
 * DOLLARS.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/types.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include "synaptics_dsx.h"
#include "synaptics_dsx_core.h"

#define SYN_I2C_RETRY_TIMES 3

/*
#define I2C_BURST_LIMIT 255
*/
/*
#define XFER_MSGS_LIMIT 8
*/

static unsigned char *wr_buf;

static struct synaptics_dsx_hw_interface hw_if;

static struct platform_device *synaptics_dsx_i2c_device;

extern void schedule_reset_work(void);

#ifdef CONFIG_OF
static int parse_dt(struct device *dev, struct synaptics_dsx_board_data *bdata)
{
	int retval;
	u32 value;
	const char *name;
	struct property *prop;
	struct device_node *np = dev->of_node;
	int i = 0;
	unsigned char propname[32];

	bdata->irq_gpio = of_get_named_gpio_flags(np,
			"synaptics,irq-gpio", 0, NULL);
	pr_err("[TP]%s: synaptics irq-gpio = %d\n", __func__, bdata->irq_gpio);

	retval = of_property_read_u32(np, "synaptics,tp-recovery-enable",
			&value);
	if (retval < 0)
		bdata->tp_recovery_enable = false;
	else{
		if(value > 0)
			bdata->tp_recovery_enable = true;
		else
			bdata->tp_recovery_enable = false;
	}
	pr_err("[TP]%s: tp-recovery-enable = %d\n", __func__, bdata->tp_recovery_enable);

	retval = of_property_read_u32(np, "synaptics,irq-on-state",
			&value);
	if (retval < 0)
		bdata->irq_on_state = 0;
	else
		bdata->irq_on_state = value;
	pr_err("[TP]%s: synaptics irq-on-state = %d\n", __func__, bdata->irq_on_state);

	retval = of_property_read_u32(np, "synaptics,irq-flags", &value);
	if (retval < 0)
		return retval;
	else
		bdata->irq_flags = value;

	retval = of_property_read_string(np, "synaptics,pwr-reg-name", &name);
	if (retval == -EINVAL)
		bdata->pwr_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->pwr_reg_name = name;

	retval = of_property_read_string(np, "synaptics,lcd-reg-name", &name);
	if (retval == -EINVAL)
		bdata->lcd_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->lcd_reg_name = name;

	retval = of_property_read_string(np, "synaptics,bus-reg-name", &name);
	if (retval == -EINVAL)
		bdata->bus_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->bus_reg_name = name;

		bdata->power_gpio = of_get_named_gpio_flags(np,
				"synaptics,power-gpio", 0, NULL);
	if (bdata->power_gpio < 0){
		bdata->power_gpio = -1;
		pr_err("[TP]%s: can't get synaptics power-gpio !\n", __func__);
	}
	pr_err("[TP]%s: synaptics,power-gpio = %d \n", __func__, bdata->power_gpio);

		retval = of_property_read_u32(np, "synaptics,power-on-state",
				&value);
	if (retval < 0){
		bdata->power_on_state = -1;
		pr_err("[TP]%s: can't get synaptics power-on-state !\n", __func__);
	}
	else{
		bdata->power_on_state = value;
	}
	pr_err("[TP]%s: synaptics,power-on-state = %d \n", __func__, bdata->power_on_state);

		retval = of_property_read_u32(np, "synaptics,power-delay-ms",
				&value);
	if (retval < 0){
		bdata->power_delay_ms = 0;
		pr_err("[TP]%s: can't get synaptics power-delay-ms!\n", __func__);
	}
	else{
			bdata->power_delay_ms = value;
	}

		bdata->reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,reset-gpio", 0, NULL);
	if (bdata->reset_gpio < 0){
		bdata->reset_gpio = -1;
		pr_err("[TP]%s: can't get synaptics reset gpio !\n", __func__);
	}
	pr_err("[TP]%s: synaptics reset-gpio = %d\n", __func__, bdata->reset_gpio);

		retval = of_property_read_u32(np, "synaptics,reset-on-state",
				&value);
	if (retval < 0){
		bdata->reset_on_state = -1;
		pr_err("[TP]%s: can't get synaptics reset-on-state !\n", __func__);
	}
	else{
		bdata->reset_on_state = value;
	}
	pr_err("[TP]%s: synaptics reset-on-state = %d\n", __func__, bdata->reset_on_state);

		retval = of_property_read_u32(np, "synaptics,reset-active-ms",
				&value);
	if (retval < 0){
		bdata->reset_active_ms = 0;
		pr_err("[TP]%s: can't get synaptics reset-active-ms !\n", __func__);
	}
	else{
		bdata->reset_active_ms = value;
	}
	pr_err("[TP]%s: synaptics reset-active-ms = %d\n", __func__, bdata->reset_active_ms);

	retval = of_property_read_u32(np, "synaptics,reset-delay-ms",
				&value);
	if (retval < 0){
		bdata->reset_delay_ms = 0;
		pr_err("[TP]%s: can't get synaptics reset_delay_ms !\n", __func__);
	}
	else{
		bdata->reset_delay_ms = value;
	}
	pr_err("[TP]%s: synaptics reset_delay_ms = %d\n", __func__, bdata->reset_delay_ms);

	retval = of_property_read_u32(np, "synaptics,max-y-for-2d",
				&value);
	if (retval < 0){
		bdata->max_y_for_2d = -1;
		pr_err("[TP]%s: synaptics can't get max_y_for_2d !\n", __func__);
	}
	else{
		bdata->max_y_for_2d = value;
	}

	bdata->lcd_reset_gpio = of_get_named_gpio_flags(np,
				"synaptics,lcd-reset-gpio", 0, NULL);
	if (bdata->lcd_reset_gpio < 0){
		bdata->lcd_reset_gpio = -1;
		pr_err("[TP]%s: can't get synaptics lcd reset gpio !\n", __func__);
	}
	pr_err("[TP]%s: synaptics lcd-reset-gpio = %d\n", __func__, bdata->lcd_reset_gpio);

	bdata->swap_axes = of_property_read_bool(np, "synaptics,swap-axes");

	bdata->x_flip = of_property_read_bool(np, "synaptics,x-flip");

	bdata->y_flip = of_property_read_bool(np, "synaptics,y-flip");

	retval = of_property_read_u32(np, "synaptics,ub-i2c-addr",
				&value);
	if (retval < 0){
		bdata->ub_i2c_addr = -1;
		pr_err("[TP]%s: synaptics can't get ub_i2c_addr !\n", __func__);
	}
	else{
		bdata->ub_i2c_addr = (unsigned short)value;
	}

	prop = of_find_property(np, "synaptics,cap-button-codes", NULL);
	if (prop && prop->length) {
		bdata->cap_button_map->map = devm_kzalloc(dev,
				prop->length,
				GFP_KERNEL);
		if (!bdata->cap_button_map->map)
			return -ENOMEM;
		bdata->cap_button_map->nbuttons = prop->length / sizeof(u32);
		retval = of_property_read_u32_array(np,
				"synaptics,cap-button-codes",
				bdata->cap_button_map->map,
				bdata->cap_button_map->nbuttons);
		if (retval < 0) {
			bdata->cap_button_map->nbuttons = 0;
			bdata->cap_button_map->map = NULL;
		}
	} else {
		bdata->cap_button_map->nbuttons = 0;
		bdata->cap_button_map->map = NULL;
	}

	prop = of_find_property(np, "synaptics,vir-button-codes", NULL);
	if (prop && prop->length) {
		bdata->vir_button_map->map = devm_kzalloc(dev,
				prop->length,
				GFP_KERNEL);
		if (!bdata->vir_button_map->map)
			return -ENOMEM;
		bdata->vir_button_map->nbuttons = prop->length / sizeof(u32);
		bdata->vir_button_map->nbuttons /= 5;
		retval = of_property_read_u32_array(np,
				"synaptics,vir-button-codes",
				bdata->vir_button_map->map,
				bdata->vir_button_map->nbuttons * 5);
		if (retval < 0) {
			bdata->vir_button_map->nbuttons = 0;
			bdata->vir_button_map->map = NULL;
		}
	} else {
		bdata->vir_button_map->nbuttons = 0;
		bdata->vir_button_map->map = NULL;
	}

	retval = of_property_read_u32(np, "synaptics,virkeynum",&value);
	if (retval < 0)
	{
		bdata->virkey_num = 0;
		pr_err("[TP]%s: can't get synaptics3330 virkeynum !\n", __func__);
	}
	else
	{
		bdata->virkey_num = value;
	}

	retval = of_property_read_u32(np, "synaptics,key-isfactory",&value);
	if (retval < 0)
	{
		bdata->key_isfactory = 0;
		pr_err("[TP]%s: can't get synaptics3330 key-isfactory !\n", __func__);
	}
	else
	{
		bdata->key_isfactory = value;
	}

	retval = of_property_read_u32(np, "synaptics,virkeyrange",&value);
	if (retval < 0)
	{
		bdata->virkeyrange = 0;
		pr_err("[TP]%s: can't get synaptics3330 virkeyrange !\n", __func__);
	}
	else
	{
		bdata->virkeyrange = value;
	}

	retval = of_property_read_u32(np, "synaptics,keylooptime",&value);
	if (retval < 0)
	{
		bdata->keylooptime = 0;
		pr_err("[TP]%s: can't get synaptics3330 keylooptime !\n", __func__);
	}
	else
	{
		bdata->keylooptime = value;
	}


	if(bdata->virkey_num != 0)
	{
		bdata->virkey_name= devm_kzalloc(dev,bdata->virkey_num*sizeof(char *),GFP_KERNEL);
		if (!bdata->virkey_name)
			return -ENOMEM;
		bdata->virkey_minval = devm_kzalloc(dev,bdata->virkey_num*sizeof(int),GFP_KERNEL);
		if (!bdata->virkey_minval)
			return -ENOMEM;
		bdata->virkey_maxval = devm_kzalloc(dev,bdata->virkey_num*sizeof(int),GFP_KERNEL);
		if (!bdata->virkey_maxval)
			return -ENOMEM;
		bdata->virkey_mincap = devm_kzalloc(dev,bdata->virkey_num*sizeof(int),GFP_KERNEL);
		if (!bdata->virkey_mincap)
			return -ENOMEM;
		bdata->virkey_maxcap = devm_kzalloc(dev,bdata->virkey_num*sizeof(int),GFP_KERNEL);
		if (!bdata->virkey_maxcap)
			return -ENOMEM;
		bdata->virkey_val = devm_kzalloc(dev,bdata->virkey_num*sizeof(int),GFP_KERNEL);
		if (!bdata->virkey_val)
			return -ENOMEM;
	}

	for (i = 0; i < bdata->virkey_num; i++)
	{
		snprintf(propname, 32, "%s%d%s", "synaptics,key", i,"name");
		retval = of_property_read_string(np, propname, &name);
		if (retval == -EINVAL)
			bdata->virkey_name[i] = NULL;
		else if (retval < 0)
			return retval;
		else
			bdata->virkey_name[i]  = name;

		snprintf(propname, 32, "%s%d%s", "synaptics,key", i,"minval");
		retval = of_property_read_u32(np, propname,&value);
		if (retval < 0)
		{
			bdata->virkey_minval[i] = -1;
			pr_err("[TP]%s: can't get synaptics3330 keyminval !\n", __func__);
		}
		else
		{
			bdata->virkey_minval[i] = value;
		}

		snprintf(propname, 32, "%s%d%s", "synaptics,key", i,"maxval");
		retval = of_property_read_u32(np, propname,&value);
		if (retval < 0)
		{
			bdata->virkey_maxval[i] = -1;
			pr_err("[TP]%s: can't get synaptics3330 keymaxval !\n", __func__);
		}
		else
		{
			bdata->virkey_maxval[i] = value;
		}

		snprintf(propname, 32, "%s%d%s", "synaptics,key", i,"mincap");
		retval = of_property_read_u32(np, propname,&value);
		if (retval < 0)
		{
			bdata->virkey_mincap[i] = -1;
			pr_err("[TP]%s: can't get synaptics3330 keymincap !\n", __func__);
		}
		else
		{
			bdata->virkey_mincap[i] = value;
		}

		snprintf(propname, 32, "%s%d%s", "synaptics,key", i,"maxcap");
		retval = of_property_read_u32(np, propname,&value);
		if (retval < 0)
		{
			bdata->virkey_maxcap[i] = -1;
			pr_err("[TP]%s: can't get synaptics3330 keymaxcap !\n", __func__);
		}
		else
		{
			bdata->virkey_maxcap[i] = value;
		}

		bdata->virkey_val[i] = 0xffffffff;

	}

	return 0;
}
#endif

static int synaptics_rmi4_i2c_alloc_buf(struct synaptics_rmi4_data *rmi4_data,
		unsigned int count)
{
	static unsigned int buf_size;

	if (count > buf_size) {
		if (buf_size)
			kfree(wr_buf);
		wr_buf = kzalloc(count, GFP_KERNEL);
		if (!wr_buf) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: Failed to alloc mem for buffer\n",
					__func__);
			buf_size = 0;
			return -ENOMEM;
		}
		buf_size = count;
	}

	return 0;
}

static void synaptics_rmi4_i2c_check_addr(struct synaptics_rmi4_data *rmi4_data,
		struct i2c_client *i2c)
{
	if (hw_if.board_data->ub_i2c_addr == -1)
		return;

	if (hw_if.board_data->i2c_addr == i2c->addr)
		hw_if.board_data->i2c_addr = hw_if.board_data->ub_i2c_addr;
	else
		hw_if.board_data->i2c_addr = i2c->addr;

	return;
}

static int synaptics_rmi4_i2c_set_page(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr)
{
	int retval;
	unsigned char retry;
	unsigned char buf[PAGE_SELECT_LEN];
	unsigned char page;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[1];

	msg[0].addr = hw_if.board_data->i2c_addr;
	msg[0].flags = 0;
	msg[0].len = PAGE_SELECT_LEN;
	msg[0].buf = buf;

	page = ((addr >> 8) & MASK_8BIT);
	buf[0] = MASK_8BIT;
	buf[1] = page;

	if (page != rmi4_data->current_page) {
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
				rmi4_data->tp_reset_count = 0;
				rmi4_data->current_page = page;
				retval = PAGE_SELECT_LEN;
				break;
			}
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: I2C retry %d\n",
					__func__, retry + 1);
			msleep(20);

			if (retry == SYN_I2C_RETRY_TIMES / 2) {
				synaptics_rmi4_i2c_check_addr(rmi4_data, i2c);
				msg[0].addr = hw_if.board_data->i2c_addr;
			}
			if(rmi4_data->hw_if->board_data->tp_recovery_enable && rmi4_data->tp_reset_start){
				if(retry > (SYN_I2C_RETRY_TIMES-2)){
					schedule_reset_work();
					if(retval< 0 ){
						dev_err(rmi4_data->pdev->dev.parent,
							"%s: I2C hw sw reset error = %d\n",
							__func__, retval);
					}
				}
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return retval;
}

static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	unsigned char buf;
#ifdef I2C_BURST_LIMIT
	unsigned char ii;
	unsigned char rd_msgs = ((length - 1) / I2C_BURST_LIMIT) + 1;
#else
	unsigned char rd_msgs = 1;
#endif
	unsigned char index = 0;
	unsigned char xfer_msgs;
	unsigned char remaining_msgs;
	unsigned short i2c_addr;
	unsigned short data_offset = 0;
	unsigned short remaining_length = length;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_adapter *adap = i2c->adapter;
	struct i2c_msg msg[rd_msgs + 1];

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	msg[0].addr = hw_if.board_data->i2c_addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf;

#ifdef I2C_BURST_LIMIT
	for (ii = 0; ii < (rd_msgs - 1); ii++) {
		msg[ii + 1].addr = hw_if.board_data->i2c_addr;
		msg[ii + 1].flags = I2C_M_RD;
		msg[ii + 1].len = I2C_BURST_LIMIT;
		msg[ii + 1].buf = &data[data_offset];
		data_offset += I2C_BURST_LIMIT;
		remaining_length -= I2C_BURST_LIMIT;
	}
#endif

	msg[rd_msgs].addr = hw_if.board_data->i2c_addr;
	msg[rd_msgs].flags = I2C_M_RD;
	msg[rd_msgs].len = remaining_length;
	msg[rd_msgs].buf = &data[data_offset];

	buf = addr & MASK_8BIT;

	remaining_msgs = rd_msgs + 1;

	while (remaining_msgs) {
#ifdef XFER_MSGS_LIMIT
		if (remaining_msgs > XFER_MSGS_LIMIT)
			xfer_msgs = XFER_MSGS_LIMIT;
		else
			xfer_msgs = remaining_msgs;
#else
		xfer_msgs = remaining_msgs;
#endif
		for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
			retval = i2c_transfer(adap, &msg[index], xfer_msgs);
			if (retval == xfer_msgs){
				rmi4_data->tp_reset_count = 0;
				break;
			}
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: I2C retry %d\n",
					__func__, retry + 1);
			msleep(20);

			if (retry == SYN_I2C_RETRY_TIMES / 2) {
				synaptics_rmi4_i2c_check_addr(rmi4_data, i2c);
				i2c_addr = hw_if.board_data->i2c_addr;
				msg[0].addr = i2c_addr;
#ifdef I2C_BURST_LIMIT
				for (ii = 0; ii < (rd_msgs - 1); ii++)
					msg[ii + 1].addr = i2c_addr;
#endif
				msg[rd_msgs].addr = i2c_addr;
			}
			if(rmi4_data->hw_if->board_data->tp_recovery_enable && rmi4_data->tp_reset_start){
				if(retry > (SYN_I2C_RETRY_TIMES-2)){
					schedule_reset_work();
					if(retval< 0 ){
						dev_err(rmi4_data->pdev->dev.parent,
							"%s: I2C hw sw reset error = %d\n",
							__func__, retval);
					}
				}
			}
		}

		if (retry == SYN_I2C_RETRY_TIMES) {
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: I2C read over retry limit\n",
					__func__);
			retval = -EIO;
			goto exit;
		}

		remaining_msgs -= xfer_msgs;
		index += xfer_msgs;
	}

	retval = length;

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[1];

	retval = synaptics_rmi4_i2c_alloc_buf(rmi4_data, length + 1);
	if (retval < 0)
		return retval;

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = synaptics_rmi4_i2c_set_page(rmi4_data, addr);
	if (retval != PAGE_SELECT_LEN) {
		retval = -EIO;
		goto exit;
	}

	msg[0].addr = hw_if.board_data->i2c_addr;
	msg[0].flags = 0;
	msg[0].len = length + 1;
	msg[0].buf = wr_buf;

	wr_buf[0] = addr & MASK_8BIT;
	retval = secure_memcpy(&wr_buf[1], length, &data[0], length, length);
	if (retval < 0) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: Failed to copy data\n",
				__func__);
		goto exit;
	}

	for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
		if (i2c_transfer(i2c->adapter, msg, 1) == 1) {
			retval = length;
			rmi4_data->tp_reset_count = 0;
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);

		if (retry == SYN_I2C_RETRY_TIMES / 2) {
			synaptics_rmi4_i2c_check_addr(rmi4_data, i2c);
			msg[0].addr = hw_if.board_data->i2c_addr;
		}

		if(rmi4_data->hw_if->board_data->tp_recovery_enable && rmi4_data->tp_reset_start){
			if(retry > (SYN_I2C_RETRY_TIMES-2)){
				schedule_reset_work();
				if(retval< 0 ){
					dev_err(rmi4_data->pdev->dev.parent,
							"%s: I2C hw sw reset error = %d\n",
							__func__, retval);
				}
			}
		}
	}

	if (retry == SYN_I2C_RETRY_TIMES) {
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C write over retry limit\n",
				__func__);
		retval = -EIO;
	}

exit:
	mutex_unlock(&rmi4_data->rmi4_io_ctrl_mutex);

	return retval;
}

static struct synaptics_dsx_bus_access bus_access = {
	.type = BUS_I2C,
	.read = synaptics_rmi4_i2c_read,
	.write = synaptics_rmi4_i2c_write,
};

static void synaptics_rmi4_i2c_dev_release(struct device *dev)
{
	kfree(synaptics_dsx_i2c_device);

	return;
}

static int synaptics_rmi4_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
	pr_err("[TP]%s\n", __func__);

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}

	synaptics_dsx_i2c_device = kzalloc(
			sizeof(struct platform_device),
			GFP_KERNEL);
	if (!synaptics_dsx_i2c_device) {
		dev_err(&client->dev,
				"%s: Failed to allocate memory for synaptics_dsx_i2c_device\n",
				__func__);
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		hw_if.board_data = devm_kzalloc(&client->dev,
				sizeof(struct synaptics_dsx_board_data),
				GFP_KERNEL);
		if (!hw_if.board_data) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for board data\n",
					__func__);
			return -ENOMEM;
		}
		hw_if.board_data->cap_button_map = devm_kzalloc(&client->dev,
				sizeof(struct synaptics_dsx_button_map),
				GFP_KERNEL);
		if (!hw_if.board_data->cap_button_map) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for 0D button map\n",
					__func__);
			return -ENOMEM;
		}
		hw_if.board_data->vir_button_map = devm_kzalloc(&client->dev,
				sizeof(struct synaptics_dsx_button_map),
				GFP_KERNEL);
		if (!hw_if.board_data->vir_button_map) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for virtual button map\n",
					__func__);
			return -ENOMEM;
		}
		parse_dt(&client->dev, hw_if.board_data);
	}
#else
	hw_if.board_data = client->dev.platform_data;
#endif

	hw_if.bus_access = &bus_access;
	hw_if.board_data->i2c_addr = client->addr;

	synaptics_dsx_i2c_device->name = PLATFORM_DRIVER_NAME;
	synaptics_dsx_i2c_device->id = 0;
	synaptics_dsx_i2c_device->num_resources = 0;
	synaptics_dsx_i2c_device->dev.parent = &client->dev;
	synaptics_dsx_i2c_device->dev.platform_data = &hw_if;
	synaptics_dsx_i2c_device->dev.release = synaptics_rmi4_i2c_dev_release;

	retval = platform_device_register(synaptics_dsx_i2c_device);
	if (retval) {
		dev_err(&client->dev,
				"%s: Failed to register platform device\n",
				__func__);
		return -ENODEV;
	}

	return 0;
}

static int synaptics_rmi4_i2c_remove(struct i2c_client *client)
{
	platform_device_unregister(synaptics_dsx_i2c_device);

	return 0;
}

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

#ifdef CONFIG_OF
static struct of_device_id synaptics_rmi4_of_match_table[] = {
	{
		.compatible = "synaptics,dsx-i2c",
	},
	{},
};
MODULE_DEVICE_TABLE(of, synaptics_rmi4_of_match_table);
#else
#define synaptics_rmi4_of_match_table NULL
#endif

static struct i2c_driver synaptics_rmi4_i2c_driver = {
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = synaptics_rmi4_of_match_table,
	},
	.probe = synaptics_rmi4_i2c_probe,
	.remove = synaptics_rmi4_i2c_remove,
	.id_table = synaptics_rmi4_id_table,
};

int synaptics_rmi4_bus_init(void)
{
	pr_err("[TP]%s\n", __func__);
	return i2c_add_driver(&synaptics_rmi4_i2c_driver);
}
EXPORT_SYMBOL(synaptics_rmi4_bus_init);

void synaptics_rmi4_bus_exit(void)
{
	kfree(wr_buf);

	i2c_del_driver(&synaptics_rmi4_i2c_driver);

	return;
}
EXPORT_SYMBOL(synaptics_rmi4_bus_exit);

MODULE_AUTHOR("Synaptics, Inc.");
MODULE_DESCRIPTION("Synaptics DSX I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
