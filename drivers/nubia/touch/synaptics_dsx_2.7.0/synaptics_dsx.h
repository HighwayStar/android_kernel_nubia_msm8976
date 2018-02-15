/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012-2016 Synaptics Incorporated. All rights reserved.
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

#ifndef _SYNAPTICS_DSX_H_
#define _SYNAPTICS_DSX_H_

#ifndef NUBIA_TOUCH_SYNAPTICS
#define NUBIA_TOUCH_SYNAPTICS
#endif
#ifdef NUBIA_TOUCH_SYNAPTICS
#define PLATFORM_DRIVER_NAME "nubia_synaptics_dsx"
#else
#define PLATFORM_DRIVER_NAME "synaptics_dsx"
#endif
#define STYLUS_DRIVER_NAME "synaptics_dsx_stylus"
#define ACTIVE_PEN_DRIVER_NAME "synaptics_dsx_active_pen"
#define PROXIMITY_DRIVER_NAME "synaptics_dsx_proximity"
#define GESTURE_DRIVER_NAME "synaptics_dsx_gesture"
#define I2C_DRIVER_NAME "synaptics_dsx_i2c"
#define SPI_DRIVER_NAME "synaptics_dsx_spi"

/*
 * struct synaptics_dsx_button_map - button map
 * @nbuttons: number of buttons
 * @map: pointer to array of button codes
 */
struct synaptics_dsx_button_map {
	unsigned char nbuttons;
	unsigned int *map;
};

#ifdef NUBIA_TOUCH_SYNAPTICS
struct synaptics_vreg_config {
	unsigned int min_voltage;
	unsigned int max_voltage;
	unsigned int enable_load;
	unsigned int disable_load;
};

struct synaptics_gpio_vreg_config {
	int power_on_state;
};

struct synaptics_vreg_seq {
	unsigned int pre_on_sleep;
	unsigned int post_on_sleep;
	unsigned int pre_off_sleep;
	unsigned int post_off_sleep;
};
#endif

/*
 * struct synaptics_dsx_board_data - DSX board data
 * @x_flip: x flip flag
 * @y_flip: y flip flag
 * @swap_axes: swap axes flag
 * @irq_gpio: attention interrupt GPIO
 * @irq_on_state: attention interrupt active state
 * @power_gpio: power switch GPIO
 * @power_on_state: power switch active state
 * @reset_gpio: reset GPIO
 * @reset_on_state: reset active state
 * @max_y_for_2d: maximum y value for 2D area when virtual buttons are present
 * @irq_flags: IRQ flags
 * @i2c_addr: I2C slave address
 * @ub_i2c_addr: microbootloader mode I2C slave address
 * @device_descriptor_addr: HID device descriptor address
 * @panel_x: x-axis resolution of display panel
 * @panel_y: y-axis resolution of display panel
 * @power_delay_ms: delay time to wait after powering up device
 * @reset_delay_ms: delay time to wait after resetting device
 * @reset_active_ms: reset active time
 * @byte_delay_us: delay time between two bytes of SPI data
 * @block_delay_us: delay time between two SPI transfers
 * @addr_delay_us: delay time after sending address word
 * @pwr_reg_name: pointer to name of regulator for power control
 * @bus_reg_name: pointer to name of regulator for bus pullup control
 * @cap_button_map: pointer to 0D button map
 * @vir_button_map: pointer to virtual button map
 */
struct synaptics_dsx_board_data {
	bool x_flip;
	bool y_flip;
	bool swap_axes;
	int irq_gpio;
	int irq_on_state;
	int power_gpio;
	int power_on_state;
	int reset_gpio;
	int reset_on_state;
	int max_y_for_2d;
	unsigned long irq_flags;
	unsigned short i2c_addr;
	unsigned short ub_i2c_addr;
	unsigned short device_descriptor_addr;
	unsigned int panel_x;
	unsigned int panel_y;
	unsigned int power_delay_ms;
	unsigned int reset_delay_ms;
	unsigned int reset_active_ms;
	unsigned int byte_delay_us;
	unsigned int block_delay_us;
	unsigned int addr_delay_us;
	const char *pwr_reg_name;
	const char *bus_reg_name;
	struct synaptics_dsx_button_map *cap_button_map;
	struct synaptics_dsx_button_map *vir_button_map;
#ifdef NUBIA_TOUCH_SYNAPTICS
	struct synaptics_vreg_config *avdd_config;
	struct synaptics_vreg_config *vdd_config;
	int power_gpio_avdd;
	int power_gpio_vdd;
	struct synaptics_gpio_vreg_config *gpio_avdd_config;
	struct synaptics_gpio_vreg_config *gpio_vdd_config;
	struct synaptics_vreg_seq *avdd_seq;
	struct synaptics_vreg_seq *vdd_seq;
	bool power_on_gpio_first;
	bool	power_off_gpio_first;
	bool power_on_avdd_first;
	bool power_off_avdd_first;
	bool avdd_always_on;
	bool vdd_always_on;
	bool touch_resume_first;
	bool touch_suspend_first;
	bool open_charge_bit;
	bool fb_ready_reset;
	unsigned short palm_reg;
	const char *fw_image_name1;
	const char *fw_image_name2;
	const char *fw_image_name3;
	const char * panel1_product_id1;
	const char * panel1_product_id2;
	const char * panel2_product_id1;
	const char * panel2_product_id2;
	const char * panel3_product_id1;
	const char * panel3_product_id2;
	unsigned int check_cap_2d_min;
	unsigned int check_cap_2d_max;
	unsigned int reset_on_sleep;
	unsigned int reset_off_sleep;
#endif
};

#endif
