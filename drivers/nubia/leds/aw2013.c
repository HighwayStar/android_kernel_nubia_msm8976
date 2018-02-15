/***********************************************************************************/
/* File Name: aw2013.c */
/* File Description: this file is used to make aw2013 driver to be added in kernel or module. */

/*  Copyright (c) 2002-2015, ZTEMT, Inc.  All rights reserved.             */
/*  No part of this work may be reproduced, modified, distributed, transmitted,    */
/*  transcribed, or translated into any language or computer format, in any form   */
/*  or by any means without written permission of: ZTEMT, Inc.,            */
/***********************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/errno.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/io.h>
#include <linux/kthread.h>

#include <linux/bug.h>
#include <linux/err.h>
#include <linux/i2c.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/spinlock_types.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include "aw2013.h"
#include  <../../include/linux/printk.h>
#include <linux/ctype.h>

#include <linux/regulator/consumer.h>

#define LOG_TAG "AW2013"
//#define DEBUG_ON //DEBUG SWITCH
#define LOG_ERROR(fmt, args...) printk(KERN_ERR "[%s] [%s:%d] " fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#define LOG_INFO(fmt, args...)  printk(KERN_INFO "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#ifdef  DEBUG_ON
#define LOG_DEBUG(fmt, args...) printk(KERN_DEBUG "[%s] [%s:%d] "  fmt,\
					LOG_TAG, __FUNCTION__, __LINE__, ##args)
#else
#define LOG_DEBUG(fmt, args...)
#endif

#define LOG_DEBUG_IF(fmt, args...)  	\
{									\
	if(led->debug_mark)				\
		printk(KERN_DEBUG"[%s][%s:%d]" fmt,LOG_TAG, __FUNCTION__, __LINE__, ##args); \
}


enum aw_outn_mode{
	AW_SW_RESET,	    // 0  soft_reset , all regs revert to default value.
	AW_CONST_ON,	    // 1 work on a constant lightness.
	AW_CONST_OFF,	    // 2 darkness is comming
	AW_AUTO_BREATH, 	// 3 self breathing, used in sences such as missing message.
	AW_STEP_FADE_IN,	// 4  fade in means that the lightness is getting stronger.
	AW_STEP_FADE_OUT,	// 5  fade out means that the lightness is getting weaker
	AW_BREATH_ONCE,     // 6 only breath once, touch the home menu for instance.
	AW_RESERVED,		// 7 reserverd.
};

#define GRADE_PARAM_LEN 20
#define CONST_MIN_GRADE  0
#define CONST_MAX_GRADE  200
#define BREATH_MAX_GRADE  200
#define FADE_PARAM_LEN 20
#define AW2013_I2C_MAX_LOOP 		50
#define LED_MIDDLE_CHANNEL           0x00
#define  BLINK_ONCE_MIN_GRADE_SCLACE    2
#define  HOLD_TIME_CONSTANT             2
#define  LED_MODE_CONFIG	0x70

#define Delay_time   0x00    //t0, LED Delay time ,0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00     //rpt,LED breath period number,0x00=forever,0x01=1,0x02=2.....0x0f=15

static struct aw2013_regs_data aw2013_regs =
{
	.soft_reset = 0x00,
	.enable_led = 0x01,
	.irq_state = 0x20,
	.enable_ledx = 0x30,

	.led0_mode_conf = 0x31,
	.led1_mode_conf = 0x32,
	.led2_mode_conf = 0x33,

	.pwm0_max_lightness = 0x34,
	.pwm1_max_lightness = 0x35,
	.pwm2_max_lightness = 0x36,

	.led0_rise_and_hold_time = 0x37,
	.led0_fall_and_off_time = 0x38,
	.led0_delay_and_repeat_time = 0x39,

	.led1_rise_and_hold_time = 0x3A,
	.led1_fall_and_off_time = 0x3B,
	.led1_delay_and_repeat_time = 0x3C,

	.led2_rise_and_hold_time = 0x3D,
	.led2_fall_and_off_time = 0x3E,
	.led2_delay_and_repeat_time = 0x3F,
};


#ifdef CONFIG_OF
static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "aw2013", },
	{}
};
#endif


// basic funtion---i2c function
static int aw2013_i2c_rx_byte_data(
		struct i2c_client *i2c,
		unsigned char  reg,
		unsigned char* buf)
{

	struct i2c_msg msgs[2];

	if(NULL == i2c)
		return -EINVAL;
	msgs[0].addr = i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = 1;
	msgs[0].buf = &reg;

	msgs[1].addr = i2c->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = buf;

	if (i2c_transfer(i2c->adapter, msgs, 2) < 0) {
		LOG_ERROR("transfer failed\n");
		return -4;
	}
	LOG_DEBUG("read buf[0]=0x%x!\n",buf[0]);

	return 0;
}

static int aw2013_i2c_tx_byte_data(
		struct i2c_client *i2c,
		unsigned char reg,
		unsigned char buf)
{
	struct i2c_msg msgs;
	char bufwr[2];
	if(NULL == i2c)
		return -EINVAL;

	bufwr[0] = reg;
	bufwr[1] = buf;
	msgs.addr = i2c->addr;
	msgs.flags = 0;
	msgs.len = 2;
	msgs.buf = bufwr;
	LOG_DEBUG("aw2013 : write reg[0x%x] , data[%d]!\n",reg, buf);
	if (i2c_transfer(i2c->adapter, &msgs, 1) < 0) {
		LOG_ERROR("transfer failed\n");
		return -EAGAIN;
	}

	return 0;
}


unsigned char aw2013_read_reg(struct aw2013_control_data *led,unsigned char regaddr)
{
	unsigned char rdbuf[1], wrbuf[1], ret;
	if(NULL == led)
		return -EINVAL;

	wrbuf[0] = regaddr;

	ret = aw2013_i2c_rx_byte_data(led->i2c_client,regaddr,rdbuf); // then read the content at the regaddr
	if (ret < 0)
		LOG_ERROR("aw2013_read_reg failed\n");
	return rdbuf[0];
}


static int write_reg(struct aw2013_control_data *led,unsigned char reg,unsigned char data)
{

	int ret = 0;
	unsigned char i;
	if(NULL == led)
		return -EINVAL;

	for (i=0; i<AW2013_I2C_MAX_LOOP; i++)
	{
		ret = aw2013_i2c_tx_byte_data(led->i2c_client,reg,data);
		LOG_DEBUG("write reg[0x%x] , data[%d]!\n",reg,data);
		if (ret >= 0) // ack success
			break;
		}
	return ret;
}

static int aw2013_soft_reset( struct aw2013_control_data *led)
{
	char buf;
	int ret = 0;
	buf = 0x55;

	if(NULL == led)
		return -EINVAL;
	ret = aw2013_i2c_tx_byte_data(led->i2c_client,aw2013_regs.soft_reset,buf);
	if(ret < 0){
		LOG_ERROR("%s: write reg[0x%x] fail!\n",__func__,aw2013_regs.soft_reset);
		return ret;
	}
	ret = write_reg(led,aw2013_regs.enable_led,0x01);
	return ret;
}

static int led_const_on(struct aw2013_control_data *led)
{
	unsigned char buf;

	if(NULL == led)
		return -EINVAL;
	write_reg(led,aw2013_regs.led0_mode_conf + led->outn,led->max_current);
	//The value of led->grade_scale_fractions and led->grade_scale_numerator range from 1 to 5.
	if((led->min_grade == CONST_MIN_GRADE) ||((led->min_grade*led->middle_grade_percentage) < 100))
		write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);
	else
	{
		if(led->outn == led->middle_light_which_channel)
			write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,
				led->min_grade*led->middle_grade_percentage/100 + led->grade_scale_offset);
		else
			write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,
				led->min_grade*led->side_grade_percentage/100+led->grade_scale_offset);
	}
	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));
	msleep(6);
	return 0;
}

static int led_const_off(struct aw2013_control_data *led)
{
	int buf;
	if(NULL == led)
		return -EINVAL;
	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx,buf & (~(0x01 << led->outn))); // outn 0 --0xFE , 1--0xFD, 2--0xFB

	msleep(1);
	return 0;
}

static int led_auto_breath(struct aw2013_control_data *led) // self breath 0 - 255 version.
{
	unsigned char buf;
	if(NULL == led)
		return -EINVAL;
	write_reg(led,aw2013_regs.led0_mode_conf + led->outn, LED_MODE_CONFIG + led->max_current); //FO=FI=MD=1, Imax=10
	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->blink_max_grade);
	//  rise is equal to fall  rise , there are 3 time. rise_fall, hold, off
	write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Hold_time);
	write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Off_time+HOLD_TIME_CONSTANT);
	write_reg(led,aw2013_regs.led0_delay_and_repeat_time + led->outn*3, (Delay_time << 4) + Period_Num);

	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx, buf | 0x01 << led->outn);
	msleep(1);
	return 0;
}

static int led_step_fade_in(struct aw2013_control_data *led)
{
	int buf;
	if(NULL == led)
		return -EINVAL;

	led_const_on(led);
	buf = aw2013_read_reg(led,aw2013_regs.led0_mode_conf + led->outn);
	write_reg(led,aw2013_regs.led0_mode_conf + led->outn, (buf & 0xEF) | 0x20); // set pwm mode and fade in mode
	write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn*3, (led->Rise_Fall_time <<4) + led->Hold_time);
	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));
	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,  led->max_grade);
	msleep(1);
	led_const_on(led);
	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,  led->min_grade);
	return 0;
}



static int led_step_fade_out(struct aw2013_control_data *led)
{
	int buf;
	if(NULL == led)
		return -EINVAL;

	led_const_on(led);
	buf = aw2013_read_reg(led,aw2013_regs.led0_mode_conf + led->outn);
	write_reg(led,aw2013_regs.led0_mode_conf+led->outn, buf | 0x42); // set pwm mode and fade out mode
	write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Off_time);
	//enable ledx
	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));
	// set lightness
	write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);
	msleep(1);
	return 0;
}

static int led_breath_once(struct aw2013_control_data *led)
{
	if(NULL == led)
		return -EINVAL;
	write_reg(led,aw2013_regs.enable_ledx,0x07);  //enable all leds
	write_reg(led,aw2013_regs.led0_mode_conf,0x60 + led->max_current); // fi fo md imax : 0110 0010

	write_reg(led,aw2013_regs.led0_fall_and_off_time , (led->Rise_Fall_time << 4) | led->Off_time);
	write_reg(led,aw2013_regs.led0_rise_and_hold_time, (led->Rise_Fall_time <<4) | led->Hold_time);
	write_reg(led,aw2013_regs.led0_delay_and_repeat_time, 0x00);
	write_reg(led,aw2013_regs.pwm0_max_lightness, led->blink_max_grade);
	msleep((0x0d << led->Rise_Fall_time)*10 + 100);
	if ((led->min_grade == CONST_MIN_GRADE) ||((led->min_grade*led->middle_grade_percentage) < 100) ){
		write_reg(led,aw2013_regs.pwm0_max_lightness, led->min_grade);
	}
	else{
		write_reg(led,aw2013_regs.pwm0_max_lightness, led->min_grade*led->middle_grade_percentage/100+ led->grade_scale_offset);
	}
	msleep((0x0d << led->Rise_Fall_time)*10 + 100);
	return 0;

}

//static enum led_brightness aw2013_breath_mode_get(struct led_classdev *led_cdev)
static ssize_t aw2013_blink_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return sprintf(buf, "%d\n", led->blink_mode);
}

static ssize_t  aw2013_blink_mode_set(struct device *dev,
		struct device_attribute *attr, const char *buf,size_t count)
{

	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);
	sscanf(buf, "%d", &led->blink_mode);
	LOG_DEBUG_IF("ztemt_blink_mode=%d \n",led->blink_mode);
	switch (led->blink_mode) {
		case AW_SW_RESET:
			aw2013_soft_reset(led);
			break;
		case AW_CONST_ON:
			led_const_on(led);
			break;
		case AW_CONST_OFF:
			led_const_off(led);
			break;
		case AW_AUTO_BREATH:
			if(!led->Rise_Fall_time && !led->Hold_time && !led->Off_time)
			{
				led->Rise_Fall_time = led->onceblink_fade_time;
				led->Hold_time = led->onceblink_fullon_time;
				led->Off_time = led->onceblink_fulloff_time;
			}
			led_auto_breath(led);
			break;
		case AW_STEP_FADE_IN:
			led_step_fade_in(led);
			break;
		case AW_STEP_FADE_OUT:
			led_step_fade_out(led);
			break;
		case AW_BREATH_ONCE:
			if(!led->Rise_Fall_time && !led->Hold_time && !led->Off_time)
			{
				led->Rise_Fall_time = led->onceblink_fade_time;
				led->Hold_time = led->onceblink_fullon_time;
				led->Off_time = led->onceblink_fulloff_time;
			}
			led_breath_once(led);
			break;
		case AW_RESERVED:
			break;
		default:
			break;
	}
	return count;
}

static ssize_t fade_parameter_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct aw2013_control_data *led;
	char *after, *parm2,*parm3;

	unsigned long delay_off,delay_off_1;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long delay_on = simple_strtoul(buf, &after, 10);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	while(isspace(*after))
		after++;
	parm2 = after;
	delay_off = simple_strtoul(parm2, &after, 10);

	while(isspace(*after))
		after++;
	parm3 = after;
	delay_off_1 = simple_strtoul(parm3, &after, 10);
	led->Rise_Fall_time = (int)delay_on;
	led->Hold_time = (int)delay_off;
	led->Off_time = (int)delay_off_1;
	LOG_DEBUG_IF("fade_time=%d ,on_time=%d , off_time=%d\n",
		led->Rise_Fall_time,led->Hold_time,led->Off_time);
	return count;
}

static ssize_t fade_parameter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return snprintf(buf, FADE_PARAM_LEN, "%4d %4d %4d\n",
			led->Rise_Fall_time, led->Hold_time, led->Off_time);
}

static ssize_t grade_parameter_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{

	struct aw2013_control_data *led;
	char *after, *parm2;
	unsigned long parameter_two;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	unsigned long parameter_one = simple_strtoul(buf, &after, 10);

	led = container_of(led_cdev, struct aw2013_control_data, cdev);
	while(isspace(*after))
		after++;
	parm2 = after;
	parameter_two = simple_strtoul(parm2, &after, 10);
	led->min_grade=(int)parameter_one;
	led->max_grade=(int)parameter_two;
	
	LOG_DEBUG_IF("min_grade=%d , max_grade=%d\n",
			led->min_grade,led->max_grade);
	return count;
}

static ssize_t grade_parameter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return snprintf(buf, GRADE_PARAM_LEN,	"%4d %4d\n",
			led->min_grade,led->max_grade);
}

static ssize_t outn_store(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct aw2013_control_data *led;
	char *after;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	unsigned long parameter_one = simple_strtoul(buf, &after, 10);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	parameter_one = (parameter_one >> 4) & 0x0f;
	switch(parameter_one)
	{
		case 0x00:
			led->outn = led->left_light_which_channel;
			break;
		case 0x01:
			led->outn = led->middle_light_which_channel;
			break;
		case 0x02:
			led->outn = led->right_light_which_channel;
			break;
		default:
			break;
	}

	LOG_DEBUG_IF("ztemt_channel=%d \n",led->outn);
	return count;
}

static ssize_t outn_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);

	return sprintf(buf, "%d\n",led->outn);
}

static ssize_t debug_mark_show(struct device *dev, struct device_attribute *attr, char*buf)
{
	struct aw2013_control_data *led;
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	led = container_of(led_cdev, struct aw2013_control_data, cdev);
	led->debug_mark = !led->debug_mark;
	return sprintf(buf,"%d\n",led->debug_mark);
}

static DEVICE_ATTR(fade_parameter, 0664, fade_parameter_show, fade_parameter_store);
static DEVICE_ATTR(grade_parameter, 0664, grade_parameter_show, grade_parameter_store);
static DEVICE_ATTR(outn, 0664, outn_show, outn_store);
static DEVICE_ATTR(blink_mode, 0664, aw2013_blink_mode_get, aw2013_blink_mode_set);
static DEVICE_ATTR(debug_mark, 0644, debug_mark_show, NULL);


static struct attribute *aw2013_attrs[] = {
	&dev_attr_fade_parameter.attr,
	&dev_attr_grade_parameter.attr,
	&dev_attr_outn.attr,
	&dev_attr_blink_mode.attr,
	&dev_attr_debug_mark.attr,
	NULL
};

static const struct attribute_group aw2013_attr_group = {
	.attrs = aw2013_attrs,
};

static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}

static int aw2013_power_on(struct device *dev)
{

	int rc;
	static struct regulator *vcc_ana;
	static struct regulator *vcc_i2c;

	LOG_INFO("power_on\n");
	vcc_ana = regulator_get(dev, "vdd_ana");
	if (IS_ERR(vcc_ana))
	{
		rc = PTR_ERR(vcc_ana);
		LOG_ERROR("Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vcc_ana) > 0)
	{
		rc = regulator_set_voltage(vcc_ana, 2850000, 3300000);
		if (rc)
		{
			LOG_ERROR("Regulator set ana vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}

	rc = reg_set_optimum_mode_check(vcc_ana, 15000);
	if (rc < 0)
	{
		LOG_ERROR("Regulator vcc_ana set_opt failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(vcc_ana);
	if (rc)
	{
		LOG_ERROR("Regulator vcc_ana enable failed rc=%d\n", rc);
		goto error_reg_en_vcc_ana;
	}

	vcc_i2c = regulator_get(dev, "vcc_i2c");

	if (IS_ERR(vcc_i2c))
	{
		rc = PTR_ERR(vcc_i2c);
		LOG_ERROR("Regulator get failed rc=%d\n", rc);
		goto error_reg_opt_vcc_dig;
	}

	if (regulator_count_voltages(vcc_i2c) > 0)
	{
		rc = regulator_set_voltage(vcc_i2c, 1800000, 1800000);
		if (rc)
		{
			LOG_ERROR("Regulator set i2c vtg failed rc=%d\n", rc);
			goto error_set_vtg_i2c;
		}
	}

	rc = reg_set_optimum_mode_check(vcc_i2c, 10000);
	if (rc < 0)
	{
		LOG_ERROR("Regulator vcc_i2c set_opt failed rc=%d\n", rc);
		goto error_set_vtg_i2c;
	}

	rc = regulator_enable(vcc_i2c);
	if (rc)
	{
		LOG_ERROR("Regulator vcc_i2c enable failed rc=%d\n", rc);
		goto error_reg_en_vcc_i2c;
	}

	msleep(5);
	return 0;


error_reg_en_vcc_i2c:
	reg_set_optimum_mode_check(vcc_i2c, 0);
error_set_vtg_i2c:
	regulator_put(vcc_i2c);
error_reg_opt_vcc_dig:
	regulator_disable(vcc_ana);
error_reg_en_vcc_ana:
	reg_set_optimum_mode_check(vcc_ana, 0);
error_set_vtg_vcc_ana:
	regulator_put(vcc_ana);
	return rc;

}

static int  aw2013_probe(struct i2c_client *client,

		const struct i2c_device_id *dev_id)

{
	int ret = 0;
	char buf = 0x0;
	int retry_times = 5;
	struct aw2013_control_data *aw2013_data;

	LOG_INFO("start probe:\n");

	aw2013_data = devm_kzalloc(&client->dev,sizeof(struct aw2013_control_data) , GFP_KERNEL);
	if (!aw2013_data) {
		LOG_ERROR("Unable to allocate memory\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(client->dev.of_node, "middle_grade_percentage",
		&aw2013_data->middle_grade_percentage);
	if (ret < 0) {
		LOG_ERROR("Failure reading middle_grade_percentage, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "side_grade_percentage",
		&aw2013_data->side_grade_percentage);
	if (ret < 0) {
		LOG_ERROR("Failure reading side_grade_percentage, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "grade_scale-offset",
		&aw2013_data->grade_scale_offset);
	if (ret < 0) {
		LOG_ERROR("Failure reading grade_scale-offset, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "blink_max_grade",
		&aw2013_data->blink_max_grade);
	if (ret < 0) {
		LOG_ERROR("Failure reading blink_max_grade, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "onceblink_fade_time",
		&aw2013_data->onceblink_fade_time);
	if (ret < 0) {
		LOG_ERROR("Failure reading onceblink_fade_time, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "onceblink_fullon_time",
		&aw2013_data->onceblink_fullon_time);
	if (ret < 0) {
		LOG_ERROR("Failure reading onceblink_fullon_time, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "onceblink_fulloff_time",
		&aw2013_data->onceblink_fulloff_time);
	if (ret < 0) {
		LOG_ERROR("Failure reading onceblink_fulloff_time, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "autoblink_fade_time",
		&aw2013_data->autoblink_fade_time);
	if (ret < 0) {
		LOG_ERROR("Failure reading autoblink_fade_time, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "autoblink_fullon_time",
		&aw2013_data->autoblink_fullon_time);
	if (ret < 0) {
		LOG_ERROR("Failure reading autoblink_fullon_time, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "autoblink_fulloff_time",
		&aw2013_data->autoblink_fulloff_time);
	if (ret < 0) {
		LOG_ERROR("Failure reading autoblink_fulloff_time, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "max_current",&aw2013_data->max_current);
	if (ret < 0) {
		LOG_ERROR("Failure reading max_current, rc =  %d\n", ret);
		return -EINVAL;
	}

	ret = of_property_read_u32(client->dev.of_node, "left_light_which_channel", &aw2013_data->left_light_which_channel);
	if(ret < 0)
		aw2013_data->left_light_which_channel = 1;
	ret = of_property_read_u32(client->dev.of_node, "middle_light_which_channel", &aw2013_data->middle_light_which_channel);
	if(ret < 0)
		aw2013_data->middle_light_which_channel = 0;
	ret = of_property_read_u32(client->dev.of_node, "right_light_which_channel", &aw2013_data->right_light_which_channel);
	if(ret < 0)
		aw2013_data->right_light_which_channel = 2;

	ret = aw2013_power_on(&client->dev);
	if (ret < 0) {
		LOG_ERROR("aw2013_power_on failed\n");
		return -EINVAL;
	}

	aw2013_data->i2c_client = client;
	i2c_set_clientdata(client, aw2013_data);
	aw2013_data->cdev.name = "nubia_led";

	ret = aw2013_i2c_rx_byte_data(aw2013_data->i2c_client,aw2013_regs.soft_reset,&buf);
	if(ret < 0)	{
		LOG_ERROR("aw2013_i2c_rx_byte_data  read chip id fail!!!\n");
		return ret;
	}

	if(0x33 != buf){
		LOG_ERROR(" the chip id is not 0x33!\n");
		return -ENODEV;
	}
	else
		LOG_ERROR("the chip id  is 0x33\n");

	ret = aw2013_soft_reset(aw2013_data);
	if(ret){
		LOG_ERROR("the chip reset failed\n");
	}
	msleep(3);

	buf = 0x02; // this is the reg address for aw2013's isr reg.
	while(retry_times--)
	{
		ret = aw2013_read_reg(aw2013_data,buf);
		if (ret >= 0) {
			LOG_INFO("aw2013_debug: reg[0x02] = %d\n", ret);
			break;
		}
		LOG_ERROR("aw2013_debug: can not get the isr states\n");
		msleep(3);
	}

	write_reg(aw2013_data, 0x01, 0x01);
	aw2013_data->i2c_suspend = false;
	
	ret = led_classdev_register(&client->dev, &aw2013_data->cdev);
	if (ret) {
		LOG_ERROR("unable to register breath_led ret=%d\n",ret);
		goto init_fail;
	}

	ret = sysfs_create_group(&aw2013_data->cdev.dev->kobj,
			&aw2013_attr_group);
	if (ret)
		goto init_fail;

	LOG_INFO("probe ok\n");
	return 0;

init_fail:
	return ret;

}

static int aw2013_remove(struct i2c_client *client)
{
	struct aw2013_control_data *aw2013_data;
	aw2013_data = i2c_get_clientdata(client);
	led_classdev_unregister(&aw2013_data->cdev);
	sysfs_remove_group(&aw2013_data->cdev.dev->kobj, &aw2013_attr_group);
	return 0;
}


static int aw2013_suspend(struct i2c_client *cl, pm_message_t mesg)
{
	struct aw2013_control_data *aw2013_data = i2c_get_clientdata(cl);
	u8 regdata = 0;

	aw2013_i2c_rx_byte_data(aw2013_data->i2c_client, aw2013_regs.enable_ledx, &regdata);
	if(!(regdata & 0x07) && !aw2013_data->i2c_suspend)
	{
		write_reg(aw2013_data, 0x01, 0x00);
		aw2013_data->i2c_suspend = true;
	}
	// for aw2013 suspend setting GCR Register 0
	LOG_INFO("blink_mode=%d \n",aw2013_data->blink_mode );
	return 0;
};

static int aw2013_resume(struct i2c_client *cl)
{
	struct aw2013_control_data *aw2013_data = i2c_get_clientdata(cl);
	if(aw2013_data->i2c_suspend)
	{
		write_reg(aw2013_data, 0x01, 0x01);// for aw2013 resume setting GCR Register 1
		aw2013_data->i2c_suspend = false;
	}
	LOG_INFO("blink_mode=%d \n",aw2013_data->blink_mode );
	return 0;
};


static const struct i2c_device_id aw2013_id[] = {
	{ "aw2013", 1 },
	{},
};
MODULE_DEVICE_TABLE(i2c, BQ27520_id);

static struct i2c_driver aw2013_driver = {
	.driver = {
		.name = "aw2013",
#ifdef CONFIG_OF
		.of_match_table = aw2013_match_table,
#endif
	},
	.id_table 	= aw2013_id,
	.probe 		= aw2013_probe,
	.remove 	= aw2013_remove,
	.suspend	= aw2013_suspend,
	.resume 	= aw2013_resume,
};


static int __init aw2013_init(void)
{
	return i2c_add_driver(&aw2013_driver);
}

static void __exit aw2013_exit(void)
{
	i2c_del_driver(&aw2013_driver);
}


late_initcall(aw2013_init);
module_exit(aw2013_exit);

MODULE_VERSION("1.0");
MODULE_AUTHOR("xuexiaojun,zhouqifang <xue.xiaojun@zte.com.cn,zhou.qifang7@zte.com.cn>");
MODULE_DESCRIPTION("aw2013 Linux driver");
MODULE_ALIAS("platform:aw2013");

