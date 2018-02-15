/*
 * BTN_Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012-2015 BTN_Synaptics Incorporated. All rights reserved.
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
 * INFORMATION CONTAINED IN THIS DOCUMENT IS PROVIDED "AS-IS," AND BTN_SYNAPTICS
 * EXPRESSLY DISCLAIMS ALL EXPRESS AND IMPLIED WARRANTIES, INCLUDING ANY
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * AND ANY WARRANTIES OF NON-INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS.
 * IN NO EVENT SHALL BTN_SYNAPTICS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, PUNITIVE, OR CONSEQUENTIAL DAMAGES ARISING OUT OF OR IN CONNECTION
 * WITH THE USE OF THE INFORMATION CONTAINED IN THIS DOCUMENT, HOWEVER CAUSED
 * AND BASED ON ANY THEORY OF LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, AND EVEN IF BTN_SYNAPTICS WAS ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE. IF A TRIBUNAL OF COMPETENT JURISDICTION DOES
 * NOT PERMIT THE DISCLAIMER OF DIRECT DAMAGES OR ANY OTHER DAMAGES, BTN_SYNAPTICS'
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
#include "btn_synaptics.h"
#include "btn_synaptics_core.h"

#define SYN_I2C_RETRY_TIMES 3

/*
#define I2C_BURST_LIMIT 255
*/
/*
#define XFER_MSGS_LIMIT 8
*/

static unsigned char *wr_buf;

static struct btn_synaptics_hw_interface hw_if;

static struct platform_device *btn_synaptics_i2c_device;

#ifdef CONFIG_OF
static int parse_dt(struct device *dev, struct btn_synaptics_board_data *bdata)
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
	pr_info("[TP]%s: synaptics irq-gpio = %d\n", __func__, bdata->irq_gpio);

	retval = of_property_read_u32(np, "synaptics,irq-on-state",
			&value);
	if (retval < 0)
		bdata->irq_on_state = 0;
	else
		bdata->irq_on_state = value;
	pr_info("[TP]%s: synaptics1302 irq-on-state = %d\n", __func__, bdata->irq_on_state);

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

	retval = of_property_read_string(np, "synaptics,bus-reg-name", &name);
	if (retval == -EINVAL)
		bdata->bus_reg_name = NULL;
	else if (retval < 0)
		return retval;
	else
		bdata->bus_reg_name = name;

	bdata->power_gpio_vdd = of_get_named_gpio_flags(np,"synaptics,power-gpio-vdd", 0, NULL);
	if (bdata->power_gpio_vdd < 0)
	{
		bdata->power_gpio_vdd = -1;
		pr_err("[TP]%s: can't get synaptics1302 power-gpio-vdd !\n", __func__);
	}

	bdata->power_gpio_vcc = of_get_named_gpio_flags(np,"synaptics,power-gpio-vcc", 0, NULL);
	if (bdata->power_gpio_vcc < 0)
	{
		bdata->power_gpio_vcc = -1;
		pr_err("[TP]%s: can't get synaptics1302 power-gpio-vcc !\n", __func__);
	}

	retval = of_property_read_u32(np, "synaptics,power-on-state",&value);
	if (retval < 0)
	{
		bdata->power_on_state = -1;
		pr_err("[TP]%s: can't get synaptics1302 power-on-state !\n", __func__);
	}
	else
	{
		bdata->power_on_state = value;
	}

	retval = of_property_read_u32(np, "synaptics,power-delay-ms",&value);
	if (retval < 0){
		bdata->power_delay_ms = 0;
		pr_err("[TP]%s: can't get synaptics1302 power-delay-ms!\n", __func__);
	}
	else
	{
			bdata->power_delay_ms = value;
	}

	bdata->reset_gpio = of_get_named_gpio_flags(np,"synaptics,reset-gpio", 0, NULL);
	if (bdata->reset_gpio < 0)
	{
		bdata->reset_gpio = -1;
		pr_err("[TP]%s: can't get synaptics1302 reset gpio !\n", __func__);
	}
	pr_info("[TP]%s: synaptics reset-gpio = %d\n", __func__, bdata->reset_gpio);

	retval = of_property_read_u32(np, "synaptics,reset-on-state",&value);
	if (retval < 0){
		bdata->reset_on_state = -1;
		pr_err("[TP]%s: can't get synaptics1302 reset-on-state !\n", __func__);
	}
	else
	{
		bdata->reset_on_state = value;
	}
	pr_info("[TP]%s: synaptics1302 reset-on-state = %d\n", __func__, bdata->reset_on_state);

	retval = of_property_read_u32(np, "synaptics,reset-active-ms",&value);
	if (retval < 0)
	{
		bdata->reset_active_ms = 0;
		pr_err("[TP]%s: can't get synaptics1302 reset-active-ms !\n", __func__);
	}
	else
	{
		bdata->reset_active_ms = value;
	}
	pr_info("[TP]%s: synaptics1302 reset-active-ms = %d\n", __func__, bdata->reset_active_ms);

	retval = of_property_read_u32(np, "synaptics,reset-delay-ms",&value);
	if (retval < 0)
	{
		bdata->reset_delay_ms = 0;
		pr_err("[TP]%s: can't get synaptics1302 reset_delay_ms !\n", __func__);
	}
	else
	{
		bdata->reset_delay_ms = value;
	}
	pr_info("[TP]%s: synaptics1302 reset_delay_ms = %d\n", __func__, bdata->reset_delay_ms);

	retval = of_property_read_u32(np, "synaptics,max-y-for-2d",
				&value);
	if (retval < 0)
	{
		bdata->max_y_for_2d = -1;
		pr_err("[TP]%s: synaptics1302 can't get max_y_for_2d !\n", __func__);
	}
	else
	{
		bdata->max_y_for_2d = value;
	}

	bdata->lcd_reset_gpio = of_get_named_gpio_flags(np,"synaptics,lcd-reset-gpio", 0, NULL);
	if (bdata->lcd_reset_gpio < 0)
	{
		bdata->lcd_reset_gpio = -1;
		pr_err("[TP]%s: can't get synaptics1302 lcd reset gpio !\n", __func__);
	}
	pr_info("[TP]%s: synaptics1302 lcd-reset-gpio = %d\n", __func__, bdata->lcd_reset_gpio);

	bdata->swap_axes = of_property_read_bool(np, "synaptics,swap-axes");

	bdata->x_flip = of_property_read_bool(np, "synaptics,x-flip");

	bdata->y_flip = of_property_read_bool(np, "synaptics,y-flip");

	retval = of_property_read_u32(np, "synaptics,ub-i2c-addr",&value);
	if (retval < 0)
	{
		bdata->ub_i2c_addr = -1;
		pr_err("[TP]%s: synaptics1302 can't get ub_i2c_addr !\n", __func__);
	}
	else
	{
		bdata->ub_i2c_addr = (unsigned short)value;
	}

	prop = of_find_property(np, "synaptics,cap-button-codes", NULL);
	if (prop && prop->length)
	{
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
		if (retval < 0)
		{
			bdata->cap_button_map->nbuttons = 0;
			bdata->cap_button_map->map = NULL;
		}
	} else
	{
		bdata->cap_button_map->nbuttons = 0;
		bdata->cap_button_map->map = NULL;
	}

	prop = of_find_property(np, "synaptics,vir-button-codes", NULL);
	if (prop && prop->length)
	{
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
		if (retval < 0)
		{
			bdata->vir_button_map->nbuttons = 0;
			bdata->vir_button_map->map = NULL;
		}
	} else
       {
		bdata->vir_button_map->nbuttons = 0;
		bdata->vir_button_map->map = NULL;
	}


	retval = of_property_read_u32(np, "synaptics,virkeynum",&value);
	if (retval < 0)
	{
		bdata->virkey_num = 0;
		pr_err("[TP]%s: can't get synaptics1302 virkeynum !\n", __func__);
	}
	else
	{
		bdata->virkey_num = value;
	}

	if(bdata->virkey_num != 0)
	{
		bdata->virkey_name= devm_kzalloc(dev,bdata->virkey_num*sizeof(char *),GFP_KERNEL);
		if (!bdata->virkey_name)
			return -ENOMEM;
		bdata->virkey_TX = devm_kzalloc(dev,bdata->virkey_num*sizeof(int),GFP_KERNEL);
		if (!bdata->virkey_TX)
			return -ENOMEM;
		bdata->virkey_RX = devm_kzalloc(dev,bdata->virkey_num*sizeof(int),GFP_KERNEL);
		if (!bdata->virkey_RX)
			return -ENOMEM;
		bdata->virkey_minval = devm_kzalloc(dev,bdata->virkey_num*sizeof(int),GFP_KERNEL);
		if (!bdata->virkey_minval)
			return -ENOMEM;
		bdata->virkey_maxval = devm_kzalloc(dev,bdata->virkey_num*sizeof(int),GFP_KERNEL);
		if (!bdata->virkey_maxval)
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

		snprintf(propname, 32, "%s%d%s", "synaptics,key", i,"TX");
		retval = of_property_read_u32(np, propname,&value);
		if (retval < 0)
		{
			bdata->virkey_TX[i] = -1;
			pr_err("[TP]%s: can't get synaptics1302 key TX !\n", __func__);
		}
		else
		{
			bdata->virkey_TX[i] = value;
		}

		snprintf(propname, 32, "%s%d%s", "synaptics,key", i,"RX");
		retval = of_property_read_u32(np, propname,&value);
		if (retval < 0)
		{
			bdata->virkey_RX[i] = -1;
			pr_err("[TP]%s: can't get synaptics1302 key RX !\n", __func__);
		}
		else
		{
			bdata->virkey_RX[i] = value;
		}

		snprintf(propname, 32, "%s%d%s", "synaptics,key", i,"minval");
		retval = of_property_read_u32(np, propname,&value);
		if (retval < 0)
		{
			bdata->virkey_minval[i] = -1;
			pr_err("[TP]%s: can't get synaptics1302 keyminval !\n", __func__);
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
			pr_err("[TP]%s: can't get synaptics1302 keymaxval !\n", __func__);
		}
		else
		{
			bdata->virkey_maxval[i] = value;
		}
		bdata->virkey_val[i] = 0xffffffff;

	}

	return 0;
}
#endif

static int btn_synaptics_rmi4_i2c_alloc_buf(struct btn_synaptics_rmi4_data *rmi4_data,
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

static void btn_synaptics_rmi4_i2c_check_addr(struct btn_synaptics_rmi4_data *rmi4_data,
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

static int btn_synaptics_rmi4_i2c_set_page(struct btn_synaptics_rmi4_data *rmi4_data,
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
				rmi4_data->current_page = page;
				retval = PAGE_SELECT_LEN;
				break;
			}
			dev_err(rmi4_data->pdev->dev.parent,
					"%s: I2C retry %d\n",
					__func__, retry + 1);
			msleep(20);

			if (retry == SYN_I2C_RETRY_TIMES / 2) {
				btn_synaptics_rmi4_i2c_check_addr(rmi4_data, i2c);
				msg[0].addr = hw_if.board_data->i2c_addr;
			}
		}
	} else {
		retval = PAGE_SELECT_LEN;
	}

	return retval;
}

static int btn_synaptics_rmi4_i2c_read(struct btn_synaptics_rmi4_data *rmi4_data,
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

	retval = btn_synaptics_rmi4_i2c_set_page(rmi4_data, addr);
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
			if (retval == xfer_msgs)
				break;

			dev_err(rmi4_data->pdev->dev.parent,
					"%s: I2C retry %d\n",
					__func__, retry + 1);
			msleep(20);

			if (retry == SYN_I2C_RETRY_TIMES / 2) {
				btn_synaptics_rmi4_i2c_check_addr(rmi4_data, i2c);
				i2c_addr = hw_if.board_data->i2c_addr;
				msg[0].addr = i2c_addr;
#ifdef I2C_BURST_LIMIT
				for (ii = 0; ii < (rd_msgs - 1); ii++)
					msg[ii + 1].addr = i2c_addr;
#endif
				msg[rd_msgs].addr = i2c_addr;
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

static int btn_synaptics_rmi4_i2c_write(struct btn_synaptics_rmi4_data *rmi4_data,
		unsigned short addr, unsigned char *data, unsigned short length)
{
	int retval;
	unsigned char retry;
	struct i2c_client *i2c = to_i2c_client(rmi4_data->pdev->dev.parent);
	struct i2c_msg msg[1];

	retval = btn_synaptics_rmi4_i2c_alloc_buf(rmi4_data, length + 1);
	if (retval < 0)
		return retval;

	mutex_lock(&rmi4_data->rmi4_io_ctrl_mutex);

	retval = btn_synaptics_rmi4_i2c_set_page(rmi4_data, addr);
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
			break;
		}
		dev_err(rmi4_data->pdev->dev.parent,
				"%s: I2C retry %d\n",
				__func__, retry + 1);
		msleep(20);

		if (retry == SYN_I2C_RETRY_TIMES / 2) {
			btn_synaptics_rmi4_i2c_check_addr(rmi4_data, i2c);
			msg[0].addr = hw_if.board_data->i2c_addr;
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

static struct btn_synaptics_bus_access bus_access = {
	.type = BUS_I2C,
	.read = btn_synaptics_rmi4_i2c_read,
	.write = btn_synaptics_rmi4_i2c_write,
};

static void btn_synaptics_rmi4_i2c_dev_release(struct device *dev)
{
	kfree(btn_synaptics_i2c_device);

	return;
}

static int btn_synaptics_rmi4_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
	int retval;
	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev,
				"%s: SMBus byte data commands not supported by host\n",
				__func__);
		return -EIO;
	}

	btn_synaptics_i2c_device = kzalloc(
			sizeof(struct platform_device),
			GFP_KERNEL);
	if (!btn_synaptics_i2c_device) {
		dev_err(&client->dev,
				"%s: Failed to allocate memory for btn_synaptics_i2c_device\n",
				__func__);
		return -ENOMEM;
	}

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		hw_if.board_data = devm_kzalloc(&client->dev,
				sizeof(struct btn_synaptics_board_data),
				GFP_KERNEL);
		if (!hw_if.board_data) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for board data\n",
					__func__);
			return -ENOMEM;
		}
		hw_if.board_data->cap_button_map = devm_kzalloc(&client->dev,
				sizeof(struct btn_synaptics_button_map),
				GFP_KERNEL);
		if (!hw_if.board_data->cap_button_map) {
			dev_err(&client->dev,
					"%s: Failed to allocate memory for 0D button map\n",
					__func__);
			return -ENOMEM;
		}
		hw_if.board_data->vir_button_map = devm_kzalloc(&client->dev,
				sizeof(struct btn_synaptics_button_map),
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

	btn_synaptics_i2c_device->name = PLATFORM_DRIVER_NAME;
	btn_synaptics_i2c_device->id = 0;
	btn_synaptics_i2c_device->num_resources = 0;
	btn_synaptics_i2c_device->dev.parent = &client->dev;
	btn_synaptics_i2c_device->dev.platform_data = &hw_if;
	btn_synaptics_i2c_device->dev.release = btn_synaptics_rmi4_i2c_dev_release;

	retval = platform_device_register(btn_synaptics_i2c_device);
	if (retval) {
		dev_err(&client->dev,
				"%s: Failed to register platform device\n",
				__func__);
		return -ENODEV;
	}

	return 0;
}

static int btn_synaptics_rmi4_i2c_remove(struct i2c_client *client)
{
	platform_device_unregister(btn_synaptics_i2c_device);

	return 0;
}

static const struct i2c_device_id btn_synaptics_rmi4_id_table[] = {
	{I2C_DRIVER_NAME, 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, btn_synaptics_rmi4_id_table);

#ifdef CONFIG_OF
static struct of_device_id btn_synaptics_rmi4_of_match_table[] = {
	{
		.compatible = "synaptics,s1302",
	},
	{},
};
MODULE_DEVICE_TABLE(of, btn_synaptics_rmi4_of_match_table);
#else
#define btn_synaptics_rmi4_of_match_table NULL
#endif

static struct i2c_driver btn_synaptics_rmi4_i2c_driver = {
	.driver = {
		.name = I2C_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = btn_synaptics_rmi4_of_match_table,
	},
	.probe = btn_synaptics_rmi4_i2c_probe,
	.remove = btn_synaptics_rmi4_i2c_remove,
	.id_table = btn_synaptics_rmi4_id_table,
};

int btn_synaptics_rmi4_bus_init(void)
{
	return i2c_add_driver(&btn_synaptics_rmi4_i2c_driver);
}
EXPORT_SYMBOL(btn_synaptics_rmi4_bus_init);

void btn_synaptics_rmi4_bus_exit(void)
{
	kfree(wr_buf);

	i2c_del_driver(&btn_synaptics_rmi4_i2c_driver);

	return;
}
EXPORT_SYMBOL(btn_synaptics_rmi4_bus_exit);

MODULE_AUTHOR("BTN_Synaptics, Inc.");
MODULE_DESCRIPTION("BTN_Synaptics DSX I2C Bus Support Module");
MODULE_LICENSE("GPL v2");
