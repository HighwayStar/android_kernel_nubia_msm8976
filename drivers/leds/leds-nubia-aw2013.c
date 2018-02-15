/***********************************************************************************/
/* File Name: aw2013.c */
/* File Description: this file is used to make aw2013 driver to be added in kernel or module. */

/*  Copyright (c) 2002-2012, ZTEMT, Inc.  All rights reserved.             */
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
#include "leds-nubia-aw2013.h"
#include  <../../include/linux/printk.h>
#include <linux/ctype.h>
#include <linux/regulator/consumer.h>

enum aw_outn_mode{
	AW_SW_RESET,	    // 0 to power off the IC
	AW_CONST_ON,	    // 1 to work on a constant lightness
	AW_CONST_OFF,	    //AW_CONST_OFF,		// 2
	AW_AUTO_BREATH, 	// 3 used in sence such as charing issue, is the repeatation of mode 6
	AW_STEP_FADE_IN,	// 4
	AW_STEP_FADE_OUT,	// 5
	AW_BREATH_ONCE,     // 6
	AW_RESERVED,		// 7
};

#define GRADE_PARAM_LEN 20
#define CONST_MIN_GRADE  10
#define CONST_MAX_GRADE  200
#define FADE_PARAM_LEN 20
#define AW2013_I2C_MAX_LOOP 		50
#define Delay_time   0x00    //t0, LED Delay time ,0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00     //rpt,LED breath period number,0x00=forever,0x01=1,0x02=2.....0x0f=15
#define  NUBIA_MIDDLE_CHANNEL  0x00
#define  MIN_GRADE_PARAMETER_SCALE      2
#define  MIN_GRADE_CONSTANT             120
#define  BLINK_ONCE_MIN_GRADE_SCLACE    2
#define  BLINK_ONCE_CONSTANT            120
#define  MAX_CURRENT_PARAMETER          0x02
#define  BLINK_ONCE_DELAY_TIME          500
#define  BLINK_ONCE_DELAY_SCALE          10
#define  HOLD_TIME_CONSTANT             1

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

#ifdef CONFIG_NUBIA_LED_DEBUG
#define LED_DEBUG(fmt,args...) printk(KERN_ERR"nubia:%s,%d" fmt,__func__,__LINE__,##args)
#else
#define LED_DEBUG(fmt,args...)  do {} while (0)
#endif

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
		dev_err(&i2c->dev, "%s: transfer failed->\n", __func__);
		return -EAGAIN;
	}
	LED_DEBUG("return  buf[0]=0x%x!\n",buf[0]);
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
	LED_DEBUG("aw2013 : write reg[0x%x] , data[%d]!\n",reg,buf);
	if (i2c_transfer(i2c->adapter, &msgs, 1) < 0) {
		dev_err(&i2c->dev, "%s: transfer failed \n", __func__);
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
		dev_err(&led->i2c_client->dev,"aw2013_read_reg failed %s\n", __func__);
    return rdbuf[0];
}


static int aw2013_write_reg(struct aw2013_control_data *led,unsigned char reg,unsigned char data)
{

	int ret = 0;
	unsigned char i;
    if(NULL == led)
        return -EINVAL;

	for (i=0; i<AW2013_I2C_MAX_LOOP; i++){
		ret = aw2013_i2c_tx_byte_data(led->i2c_client,reg,data);
		LED_DEBUG("aw2013 : write reg[0x%x] , data[%d]!\n",reg,data);
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
		dev_err(&led->i2c_client->dev,"%s: write reg[0x%x] fail!\n",__func__,aw2013_regs.soft_reset);
        return ret;
    }
    ret = aw2013_write_reg(led,aw2013_regs.enable_led,0x01);
	return ret;
}


static int led_const_on(struct aw2013_control_data *led)
{
	unsigned char buf;
    if(NULL == led)
        return -EINVAL;

    aw2013_write_reg(led,aw2013_regs.led0_mode_conf + led->outn,MAX_CURRENT_PARAMETER);
    if(led->min_grade > 0 )
	    aw2013_write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade*MIN_GRADE_PARAMETER_SCALE+MIN_GRADE_CONSTANT);
    else
	    aw2013_write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);
	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	aw2013_write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));
    msleep(6);
    return 0;
}



static int led_const_off(struct aw2013_control_data *led)
{
	int buf;
    if(NULL == led)
        return -EINVAL;
	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	aw2013_write_reg(led,aw2013_regs.enable_ledx,buf & (~(0x01 << led->outn))); // outn 0 --0xFE , 1--0xFD, 2--0xFB
	msleep(1);
    return 0;
}

static int led_auto_breath(struct aw2013_control_data *led) // self breath 0 - 255 version.
{
    if(NULL == led)
        return -EINVAL;

		// 3. set ledx as auto mode and set it's max current.as the Imax is a unchanged value, no need to set again.
	aw2013_write_reg(led,aw2013_regs.led0_mode_conf + led->outn, 0x72); //FO=FI=MD=1, Imax=10
	aw2013_write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->max_grade);

		// 4. set auto mode's time
		//  rise is equal to fall  rise , there are 3 time. rise_fall, hold, off
	aw2013_write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Hold_time);
	aw2013_write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Off_time);
	aw2013_write_reg(led,aw2013_regs.led0_delay_and_repeat_time + led->outn*3, (Delay_time << 4) + Period_Num);
		// 5. enable ledx
	aw2013_write_reg(led,aw2013_regs.enable_ledx, 0x01 << led->outn);
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
	aw2013_write_reg(led,aw2013_regs.led0_mode_conf + led->outn, (buf & 0xEF) | 0x20); // set pwm mode and fade in mode
	aw2013_write_reg(led,aw2013_regs.led0_rise_and_hold_time + led->outn, led->Rise_Fall_time <<4);
	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	aw2013_write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));
	aw2013_write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,  led->max_grade);
	msleep(1);
	led_const_on(led);
	aw2013_write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn,  led->min_grade);
    return 0;
}



static int led_step_fade_out(struct aw2013_control_data *led)
{
	int buf;
    if(NULL == led)
        return -EINVAL;

	led_const_on(led);
	buf = aw2013_read_reg(led,aw2013_regs.led0_mode_conf + led->outn);
	aw2013_write_reg(led,aw2013_regs.led0_mode_conf+led->outn, buf | 0x42); // set pwm mode and fade in mode
	aw2013_write_reg(led,aw2013_regs.led0_fall_and_off_time + led->outn*3, (led->Rise_Fall_time << 4) + led->Off_time);
	//enable ledx
	buf = aw2013_read_reg(led,aw2013_regs.enable_ledx);
	aw2013_write_reg(led,aw2013_regs.enable_ledx, buf | (0x01 << led->outn));
	// set lightness
	aw2013_write_reg(led,aw2013_regs.pwm0_max_lightness + led->outn, led->min_grade);
	msleep(1);
    return 0;
}

static int led_breath_once(struct aw2013_control_data *led)
{
    if(NULL == led)
        return -EINVAL;
	aw2013_write_reg(led,aw2013_regs.enable_ledx,0x07);  //enable all leds
	aw2013_write_reg(led,aw2013_regs.led0_mode_conf,0x62); // fi fo md imax : 0110 0010
	aw2013_write_reg(led,aw2013_regs.led0_fall_and_off_time , (led->Rise_Fall_time << 4) | led->Off_time);
	aw2013_write_reg(led,aw2013_regs.led0_rise_and_hold_time, (led->Rise_Fall_time <<4) | (led->Hold_time+HOLD_TIME_CONSTANT));
	aw2013_write_reg(led,aw2013_regs.led0_delay_and_repeat_time, 0x00);
	aw2013_write_reg(led,aw2013_regs.pwm0_max_lightness, led->max_grade);
	msleep((0x0d << led->Rise_Fall_time)*BLINK_ONCE_DELAY_SCALE + BLINK_ONCE_DELAY_TIME);
    aw2013_write_reg(led,aw2013_regs.pwm0_max_lightness, (led->min_grade*BLINK_ONCE_MIN_GRADE_SCLACE+BLINK_ONCE_CONSTANT));
    msleep((0x0d << led->Rise_Fall_time)*BLINK_ONCE_DELAY_SCALE + BLINK_ONCE_DELAY_TIME);
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
			led_auto_breath(led);
			break;
		case AW_STEP_FADE_IN:
		  led_step_fade_in(led);
			break;
		case AW_STEP_FADE_OUT:
			led_step_fade_out(led);
			break;
		case AW_BREATH_ONCE:
            if(led->outn == NUBIA_MIDDLE_CHANNEL)
                led_breath_once(led);
            else
                led_const_on(led);
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
		LED_DEBUG("%s : %d : fade_time=%d ,on_time=%d , off_time=%d\n",
			__func__,__LINE__,led->Rise_Fall_time,led->Hold_time,led->Off_time);
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
		LED_DEBUG("%s : %d : min_grade=%d , max_grade=%d\n",
			__func__,__LINE__,led->min_grade,led->max_grade);
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

    /***channel convert adaptor for chip needed****/
	parameter_one = (parameter_one >> 4) & 0x0f;
	if(parameter_one == 0x01)
		parameter_one = 0x00;
	else if(parameter_one == 0x00)
		parameter_one = 0x01;

    led->outn =(int) parameter_one;
	LED_DEBUG("%s : %d : ztemt_channel=%d \n",__func__,__LINE__,led->outn);
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

static DEVICE_ATTR(fade_parameter, 0664, fade_parameter_show, fade_parameter_store);
static DEVICE_ATTR(grade_parameter, 0664, grade_parameter_show, grade_parameter_store);
static DEVICE_ATTR(outn, 0664, outn_show, outn_store);
static DEVICE_ATTR(blink_mode, 0664, aw2013_blink_mode_get, aw2013_blink_mode_set);

static struct attribute *aw2013_attrs[] = {
	&dev_attr_fade_parameter.attr,
	&dev_attr_grade_parameter.attr,
	&dev_attr_outn.attr,
	&dev_attr_blink_mode.attr,
	NULL
};

static const struct attribute_group aw2013_attr_group = {
	.attrs = aw2013_attrs,
};

/* Regulator utility functions */
static int aw2013_vreg_init_reg(struct device *dev,
					struct aw2013_reg_data *vreg)
{
	int ret = 0;

	/* check if regulator is already initialized? */
	if (vreg->reg)
		goto out;

	/* Get the regulator handle */
	vreg->reg = devm_regulator_get(dev, vreg->name);
	if (IS_ERR(vreg->reg)) {
		ret = PTR_ERR(vreg->reg);
		printk(KERN_ERR"%s: devm_regulator_get(%s) failed. ret=%d\n",
			__func__, vreg->name, ret);
		goto out;
	}

	if (regulator_count_voltages(vreg->reg) > 0) {
		vreg->set_voltage_sup = true;
		/* sanity check */
		if (!vreg->high_vol_level || !vreg->hpm_uA) {
			pr_err("%s: %s invalid constraints specified\n",
			       __func__, vreg->name);
			ret = -EINVAL;
		}
	}

out:
	return ret;
}

static void aw2013_vreg_deinit_reg(struct aw2013_reg_data *vreg)
{
	if (vreg->reg)
		devm_regulator_put(vreg->reg);
}

static int aw2013_vreg_set_optimum_mode(struct aw2013_reg_data
						  *vreg, int uA_load)
{
	int ret = 0;

	/*
	 * regulators that do not support regulator_set_voltage also
	 * do not support regulator_set_optimum_mode
	 */
	if (vreg->set_voltage_sup) {
		ret = regulator_set_optimum_mode(vreg->reg, uA_load);
		if (ret < 0)
			pr_err("%s: regulator_set_optimum_mode(reg=%s,uA_load=%d) failed. ret=%d\n",
			       __func__, vreg->name, uA_load, ret);
		else
			/*
			 * regulator_set_optimum_mode() can return non zero
			 * value even for success case.
			 */
			ret = 0;
	}
	return ret;
}

static int aw2013_vreg_set_voltage(struct aw2013_reg_data *vreg,
					int min_uV, int max_uV)
{
	int ret = 0;
	if (vreg->set_voltage_sup) {
		ret = regulator_set_voltage(vreg->reg, min_uV, max_uV);
		if (ret) {
			pr_err("%s: regulator_set_voltage(%s)failed. min_uV=%d,max_uV=%d,ret=%d\n",
			       __func__, vreg->name, min_uV, max_uV, ret);
		}
	}

	return ret;
}

static int aw2013_vreg_enable(struct aw2013_reg_data *vreg)
{
	int ret = 0;

	/* Put regulator in HPM (high power mode) */
	ret = aw2013_vreg_set_optimum_mode(vreg, vreg->hpm_uA);
	if (ret < 0)
		return ret;

	if (!vreg->is_enabled) {
		/* Set voltage level */
		ret = aw2013_vreg_set_voltage(vreg, vreg->high_vol_level,
						vreg->high_vol_level);
		if (ret)
			return ret;
	}
	ret = regulator_enable(vreg->reg);
	if (ret) {
		pr_err("%s: regulator_enable(%s) failed. ret=%d\n",
				__func__, vreg->name, ret);
		return ret;
	}
	vreg->is_enabled = true;
	return ret;
}

static int aw2013_vreg_disable(struct aw2013_reg_data *vreg)
{
	int ret = 0;

	/* Never disable regulator marked as always_on */
	if (vreg->is_enabled && !vreg->is_always_on) {
		ret = regulator_disable(vreg->reg);
		if (ret) {
			pr_err("%s: regulator_disable(%s) failed. ret=%d\n",
				__func__, vreg->name, ret);
			goto out;
		}
		vreg->is_enabled = false;

		ret = aw2013_vreg_set_optimum_mode(vreg, 0);
		if (ret < 0)
			goto out;

		/* Set min. voltage level to 0 */
		ret = aw2013_vreg_set_voltage(vreg, 0, vreg->high_vol_level);
		if (ret)
			goto out;
	} else if (vreg->is_enabled && vreg->is_always_on) {
		if (vreg->lpm_sup) {
			/* Put always_on regulator in LPM (low power mode) */
			ret = aw2013_vreg_set_optimum_mode(vreg,
							      vreg->lpm_uA);
			if (ret < 0)
				goto out;
		}
	}
out:
	return ret;
}

static int aw2013_setup_vreg(struct aw2013_control_data *pdata,
			bool enable, bool is_init)
{
	int ret = 0;
	struct aw2013_reg_data *vreg_table;

	vreg_table = pdata->vreg_data;
	if(!vreg_table)
		goto jump_out;
	if (enable)
		ret = aw2013_vreg_enable(vreg_table);
	else
		ret = aw2013_vreg_disable(vreg_table);
	if (ret)
		goto jump_out;

jump_out:
	return ret;
}

/*
 * Reset vreg by ensuring it is off during probe. A call
 * to enable vreg is needed to balance disable vreg
 */
static int aw2013_vreg_reset(struct aw2013_control_data *pdata)
{
	int ret;

	ret = aw2013_setup_vreg(pdata, 1, true);
	if (ret)
		return ret;
	ret = aw2013_setup_vreg(pdata, 0, true);
	return ret;
}
#define MAX_PROP_SIZE 32

static int aw2013_dt_parse_vreg_info(struct device *dev,
		struct aw2013_reg_data  **vreg_data, const char *vreg_name)
{
	int len, ret = 0;
	const __be32 *prop;
	char prop_name[MAX_PROP_SIZE];
	struct aw2013_reg_data *vreg;
	struct device_node *np = dev->of_node;

	snprintf(prop_name, MAX_PROP_SIZE, "%s-supply", vreg_name);
	if (!of_parse_phandle(np, prop_name, 0)) {
		dev_info(dev, "No vreg data found for %s\n", vreg_name);
		return ret;
	}

	vreg = devm_kzalloc(dev, sizeof(*vreg), GFP_KERNEL);
	if (!vreg) {
		dev_err(dev, "No memory for vreg: %s\n", vreg_name);
		ret = -ENOMEM;
		return ret;
	}

	vreg->name = vreg_name;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-always-on", vreg_name);
	if (of_get_property(np, prop_name, NULL))
		vreg->is_always_on = true;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-lpm-sup", vreg_name);
	if (of_get_property(np, prop_name, NULL))
		vreg->lpm_sup = true;

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-voltage-level", vreg_name);
	prop = of_get_property(np, prop_name, &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		dev_warn(dev, "%s %s property\n",
			prop ? "invalid format" : "no", prop_name);
	} else {
		vreg->low_vol_level = be32_to_cpup(&prop[0]);
		vreg->high_vol_level = be32_to_cpup(&prop[1]);
	}

	snprintf(prop_name, MAX_PROP_SIZE,
			"qcom,%s-current-level", vreg_name);
	prop = of_get_property(np, prop_name, &len);
	if (!prop || (len != (2 * sizeof(__be32)))) {
		dev_warn(dev, "%s %s property\n",
			prop ? "invalid format" : "no", prop_name);
	} else {
		vreg->lpm_uA = be32_to_cpup(&prop[0]);
		vreg->hpm_uA = be32_to_cpup(&prop[1]);
	}

	*vreg_data = vreg;
	LED_DEBUG("%s: %s %s vol=[%d %d]uV, curr=[%d %d]uA\n",
		vreg->name, vreg->is_always_on ? "always_on," : "",
		vreg->lpm_sup ? "lpm_sup," : "", vreg->low_vol_level,
		vreg->high_vol_level, vreg->lpm_uA, vreg->hpm_uA);

	return ret;
}
/* This init function should be called only once for each SDHC slot */
static int aw2013_vreg_init(struct device *dev,
				struct aw2013_control_data *pdata,
				bool is_init)
{
	int ret = 0;
	struct aw2013_reg_data *curr_vdd_reg;

	curr_vdd_reg = pdata->vreg_data;
	if (!is_init)
		/* Deregister all regulators from regulator framework */
		goto vdd_reg_deinit;

	/*
	 * Get the regulator handle from voltage regulator framework
	 * and then try to set the voltage level for the regulator
	 */
	LED_DEBUG("ZTEMT:aw2013_vreg_init\n");
	if (curr_vdd_reg) {
		ret = aw2013_vreg_init_reg(dev, curr_vdd_reg);
		if (ret)
			goto out;
	}
	ret = aw2013_vreg_reset(pdata);
	if (ret)
		printk(KERN_ERR"vreg reset failed (%d)\n", ret);
	goto out;


vdd_reg_deinit:
	if (curr_vdd_reg)
		aw2013_vreg_deinit_reg(curr_vdd_reg);
out:
	return ret;
}

static int aw2013_nubia_power_on(struct aw2013_control_data *aw2013_data,struct device *dev)
{
    /* Setup regulators */
    int ret = 0;
    ret = aw2013_vreg_init(dev,aw2013_data, true);
    if (ret) {
        dev_err(dev, "nubia:vreg init failed (%d)\n", ret);
    }

    ret = aw2013_setup_vreg(aw2013_data, 1, true);
    if (ret){
        dev_err(dev, "nubia:aw2013_setup_vreg failed (%d)\n", ret);
    }

    return ret;
}


static int  aw2013_probe(struct i2c_client *client,

		const struct i2c_device_id *dev_id)

{
	int ret = 0;
	char buf = 0x0;

	struct aw2013_control_data *aw2013_data;

	LED_DEBUG("%s: start probe:\n",__func__);

	aw2013_data = devm_kzalloc(&client->dev,
	sizeof(struct aw2013_control_data) , GFP_KERNEL);
	if (!aw2013_data) {
		dev_err(&client->dev, "Unable to allocate memory\n");
		   return -ENOMEM;
	}

    if(aw2013_dt_parse_vreg_info(&client->dev,&aw2013_data->vreg_data, "vdd")){
         dev_err(&client->dev,"failed parsing vdd data\n");
    }

	aw2013_data->i2c_client = client;
    i2c_set_clientdata(client,aw2013_data);
	aw2013_data->cdev.name = "nubia_led";

    ret = aw2013_nubia_power_on(aw2013_data,&client->dev);
    if(ret < 0){
       dev_err(&client->dev,"aw2013 poweron failed\n");
    }

    mdelay(1000);
	ret = aw2013_i2c_rx_byte_data(aw2013_data->i2c_client,aw2013_regs.soft_reset,&buf);
	if(ret < 0)	{
		printk("aw2013_i2c_rx_byte_data  read chip id fail!!!\n");
		return ret;
	}

	if(0x33 != buf){
		dev_err(&client->dev,"the chip id is not 0x33!\n");
		return -ENODEV;
    }
    ret = aw2013_soft_reset(aw2013_data);
    if(ret){
    dev_err(&client->dev,"the chip reset failed\n");
    }
    ret = led_classdev_register(&client->dev, &aw2013_data->cdev);
	if (ret) {
		pr_err("unable to register breath_led ret=%d\n",ret);
		goto init_fail;
	}
     // create sys files
	ret = sysfs_create_group(&aw2013_data->cdev.dev->kobj,
			&aw2013_attr_group);
	if (ret)
		 goto init_fail;

	LED_DEBUG("%s: finish probe:\n",__func__);
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

 if((aw2013_data->outn == NUBIA_MIDDLE_CHANNEL) &&
        ((aw2013_data->blink_mode == AW_SW_RESET)|
            (aw2013_data->blink_mode == AW_CONST_OFF)))
    aw2013_write_reg(aw2013_data, 0x01, 0x00);// for aw2013 suspend setting GCR Register 0

	return 0;
};

static int aw2013_resume(struct i2c_client *cl)
{
       struct aw2013_control_data *aw2013_data = i2c_get_clientdata(cl);
    
    if((aw2013_data->outn == NUBIA_MIDDLE_CHANNEL) &&
           ((aw2013_data->blink_mode == AW_SW_RESET)|
               (aw2013_data->blink_mode == AW_CONST_OFF)))
        aw2013_write_reg(aw2013_data, 0x01, 0x01);// for aw2013 resume setting GCR Register 1

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

