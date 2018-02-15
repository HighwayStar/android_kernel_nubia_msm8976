/*
 * This file is part of the apds9922 sensor driver.
 * apds9922 is combined proximity, and VCSEL.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *Reversion
 *
 *
 *
 when         	who         		Remark : what, where, why          		version
-----------   	------------     	-----------------------------------   	------------------
2016/7/11       Bao QI              Pa22a code refcatoring                   V2.0
==========================================================================================
*/
#include "apds9922_common.h"
#include "apds9922_ps.h"

static dev_t const apds9922_proximity_dev_t = MKDEV(MISC_MAJOR, 101);
static struct class         *proximity_class;

/*
 * Management functions
 */

static int apds9921_dd_set_prx_meas_rate(struct i2c_client *client, int prx_meas)
{
	return apds9922_i2c_write_reg(client, APDS9921_DD_PRX_MEAS_RATE_ADDR, prx_meas);
}

static int apds9921_dd_set_pers(struct i2c_client *client, int pers)
{
	return apds9922_i2c_write_reg(client, APDS9921_DD_INT_PERSISTENCE_ADDR, pers);
}

static int apds9921_dd_set_prx_led(struct i2c_client *client, int prx_led)
{
	return apds9922_i2c_write_reg(client, APDS9921_DD_PRX_LED_ADDR, prx_led);
}

static int apds9921_dd_set_prx_pulses(struct i2c_client *client, int prx_pulses)
{
	return apds9922_i2c_write_reg(client, APDS9921_DD_PRX_PULSES_ADDR, prx_pulses);
}

static int apds9921_dd_set_prx_thresh(struct i2c_client *client, int thres_low, int thres_up)
{
	int err =0;
	u8 low_lsb,low_msb,up_lsb,up_msb;

	low_lsb = (thres_low & 0xff);
	low_msb =(thres_low >> 8) & 0x07;
	up_lsb = (thres_up & 0xff);
	up_msb =(thres_up >> 8) & 0x07;

	err=apds9922_i2c_write_reg(client, APDS9921_DD_PRX_THRES_UP_0_ADDR, up_lsb);
	err=apds9922_i2c_write_reg(client, APDS9921_DD_PRX_THRES_UP_1_ADDR, up_msb);
	err=apds9922_i2c_write_reg(client, APDS9921_DD_PRX_THRES_LOW_0_ADDR, low_lsb);
	err=apds9922_i2c_write_reg(client, APDS9921_DD_PRX_THRES_LOW_1_ADDR, low_msb);

	return err;
}


static int apds9922_get_ps_value(struct apds9922_ps_data *data)
{
	u8 lsb,msb;
	int ps_data = 0;
	int err = 0;
	struct apds9922_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;

	mutex_lock(&data->ps_lock);
	err = apds9922_i2c_read_reg(client, APDS9921_DD_PRX_DATA_0_ADDR,&lsb);
	if (err < 0)
	{
		mutex_unlock(&data->ps_lock);
		return -ERR_DEV_OPS;
	}
	err = apds9922_i2c_read_reg(client, APDS9921_DD_PRX_DATA_1_ADDR, &msb);
	if (err < 0)
	{
		mutex_unlock(&data->ps_lock);
		return -ERR_DEV_OPS;
	}
	ps_data = ( (msb << 8) | lsb);

	data->ps_data = ps_data & 0x7FF;
	mutex_unlock(&data->ps_lock);
	return data->ps_data;
}


static void apds9922_ps_load_calibration_param(char *filename, struct apds9922_ps_data *data)
{
	u8 param[2] = {1,1};
	int ret;

	ret = apds9922_read_file(filename, param, sizeof(param));
	if (ret < 0 || 0==param[1])
	{
		data->crosstalk = data->ps_offset;
		data->near_diff_cnt = data->ps_thrd_high;
		data->far_diff_cnt = data->ps_thrd_low;
	}
	else
	{
		data->crosstalk = param[0];
		data->near_diff_cnt = param[1];
		data->far_diff_cnt = param[1] / 2;

		data->ps_thrd_high = data->near_diff_cnt;
		data->ps_thrd_low = data->far_diff_cnt ;
		data->ps_offset = data->crosstalk;

		apds9921_dd_set_prx_thresh(data->platform_data->client, data->ps_thrd_low+data->ps_offset, data->ps_thrd_high+data->ps_offset);

		SENSOR_LOG_DEBUG("prox debug: crosstalk = %d near_diff_cnt = %d\n", data->crosstalk, data->near_diff_cnt);
	}
}


static int apds9922_ps_set_int_enable(struct apds9922_ps_data *data, int enable)
{
	int err = 0;
	u8 regdata = 0;
	struct i2c_client *client;
	if (unlikely(IS_ERR_OR_NULL(data)))
	{
		SENSOR_LOG_ERROR("null pointer./n");
		return -PTR_ERR(data);
	}
	client = data->platform_data->client;
	mutex_lock(&data->platform_data->reg_lock);
	err = apds9922_i2c_read_reg(data->platform_data->client, APDS9921_DD_INT_CFG_ADDR, &regdata);
	if (err < 0)
	{
		SENSOR_LOG_ERROR("i2c read failed\n");
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}

	regdata &= ~(APDS9921_DD_PRX_INT_EN);
	regdata |= enable ;

	err = apds9922_i2c_write_reg(client, APDS9921_DD_INT_CFG_ADDR, (regdata&&0xFD));
	if (err < 0)
	{
		SENSOR_LOG_ERROR("i2c read failed\n");
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}
	mutex_unlock(&data->platform_data->reg_lock);
	return err;
}


static int apds9922_ps_set_enable(struct apds9922_ps_data *data, int enable)
{
	int err = 0;
	u8 regdata = 0;
	struct i2c_client *client;
	if (unlikely(IS_ERR_OR_NULL(data)))
	{
		SENSOR_LOG_ERROR("null pointer./n");
		return -PTR_ERR(data);
	}
	client = data->platform_data->client;
	mutex_lock(&data->platform_data->reg_lock);
	err = apds9922_i2c_read_reg(data->platform_data->client, APDS9921_DD_MAIN_CTRL_ADDR, &regdata);
	if (err < 0)
	{
		SENSOR_LOG_ERROR("i2c read failed\n");
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}

	regdata &= ~(APDS9922_PS_ACTIVE);
	regdata |= enable ;

	err = apds9922_i2c_write_reg(client, APDS9921_DD_MAIN_CTRL_ADDR, (regdata));
	if (err < 0)
	{
		SENSOR_LOG_ERROR("i2c read failed\n");
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}
	mutex_unlock(&data->platform_data->reg_lock);
	return err;
}

static int apds9922_enable_ps(struct apds9922_ps_data *data, int enable)
{
	struct apds9922_platform_data *pdata;
	struct i2c_client *client;
	int ps_data;
	int err;

	if (unlikely(IS_ERR_OR_NULL(data)))
	{
		SENSOR_LOG_ERROR("null pointer exception.\n");
		return -EINVAL;
	}
	pdata = data->platform_data;
	client = pdata->client;
	data->ps_enable = enable;

	if (PS_POLLING)
	{
		err = apds9922_ps_set_enable(data, enable);
		if (err < 0)
		{
			SENSOR_LOG_ERROR("enable ps fail\n");
			return err;
		}
		data->ps_last_status = APDS9922_PS_UNKOWN_DISTANCE;
		if (enable)
			schedule_delayed_work(&data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));
		else
			cancel_delayed_work(&data->ps_dwork);
	}
	else if(!PS_POLLING)
	{
		if (enable)
		{
			if (!pdata->vdd_always_on)
			{
				apds9922_sensor_platform_hw_power_on(pdata, true);
				apds9922_init_client(data);
				mutex_lock(&pdata->lock);
				pdata->enable_mask |= 1 << PS;
				mutex_unlock(&pdata->lock);
			}

			data->ps_last_status = APDS9922_PS_UNKOWN_DISTANCE;
			apds9922_ps_set_int_enable(data,0);
			apds9922_ps_set_enable(data, 1);
			msleep(50);
			ps_data = apds9922_get_ps_value(data);
			if(data->prox_debug)
			{
				input_report_rel(data->ps_input_dev, REL_MISC, data->ps_data > 0 ? data->ps_data : 1);
				input_sync(data->ps_input_dev);
			}
			else {
            			if(ps_data > data->ps_thrd_high+data->ps_offset)
					data->ps_status = APDS9922_PS_NEAR_DISTANCE;
				else if(ps_data < data->ps_thrd_low+data->ps_offset)
					data->ps_status = APDS9922_PS_FAR_DISTANCE;
				apds9922_report_event(data);
			}
			apds9922_ps_set_int_enable(data,1);
		}
		else
		{
			apds9922_ps_set_enable(data, 0);
			apds9921_dd_set_prx_thresh(client, data->ps_thrd_low+data->ps_offset, data->ps_thrd_high+data->ps_offset);
			if (!pdata->vdd_always_on)
			{
				mutex_lock(&pdata->lock);
				pdata->enable_mask &= ~(1 << PS);
				mutex_unlock(&pdata->lock);
				apds9922_sensor_platform_hw_power_on(pdata, false);
			}
		}
	}

	return 0;
}

static int apds9922_get_psoffset(struct apds9922_ps_data *data)
{
	int i = 0;
	int ps_data  = 0;

	apds9922_ps_set_int_enable(data,0);

	for (i=0; i<APDS9921_PS_CAL_LOOP; i++)
	{
		mdelay(50);	// must be greater than prx meas rate
		ps_data +=(apds9922_get_ps_value(data) & 0xff);
	}
		ps_data = ps_data/i;

	SENSOR_LOG_DEBUG("APDS9921_PS_CAL pdata = %d\n", ps_data);

	if ((ps_data <= APDS9921_PS_CAL_CROSSTALK_HIGH) &&(ps_data >= APDS9921_PS_CAL_CROSSTALK_LOW))
	{
		data->crosstalk = ps_data;
	}
	else
	{
		SENSOR_LOG_ERROR("crosstalk =%d > %d ,crosstalk calibration faile\n", ps_data,APDS9921_PS_CAL_CROSSTALK_HIGH);
		return -1;
	}
	return ps_data;
}
/*
 * return value
 * -1: need naked calibration
 * -2: need 3cm gray-card cailibraion
 */
static int apds9922_run_calibration(struct apds9922_ps_data *data)
{
	u8 buftmp[2];
	int ps_data = 0;
	int i;

	apds9922_ps_set_int_enable(data,0);
	for (i=0; i<APDS9921_PS_CAL_LOOP; i++) {
		mdelay(50);	// must be greater than prx meas rate
		ps_data +=(apds9922_get_ps_value(data) & 0xff);
	}

	data->near_diff_cnt = ps_data/i;
	if((data->near_diff_cnt <= data->crosstalk) || (data->near_diff_cnt >= APDS9922_PS_THRESH_DATA_MAX))
	{
		SENSOR_LOG_ERROR("calibration value = %d ,fail",data->near_diff_cnt);
		return -1;
	}

	buftmp[0] = (u8)data->crosstalk;
	buftmp[1] = (u8)(data->near_diff_cnt - data->crosstalk);

	if (apds9922_write_file(PS_CAL_FILE_PATH, buftmp, sizeof(buftmp)) < 0) {
		SENSOR_LOG_ERROR("Open PS calibration file error!!");
		return -ERR_FILE_OPS;
	}
	data->ps_thrd_high = buftmp[1];
	data->ps_thrd_low = data->ps_thrd_high / 2;
	data->ps_offset = data->crosstalk;
	data->near_diff_cnt = data->ps_thrd_high;
	data->far_diff_cnt = data->ps_thrd_low;

	SENSOR_LOG_DEBUG("APDS9921_PS_CAL data->crosstalk=%d,data->near_diff_cnt = %d,data->far_diff_cnt = %d\n", data->crosstalk,data->near_diff_cnt,data->far_diff_cnt);

	return (data->crosstalk + data->near_diff_cnt);
}

static void apds9922_report_event(struct apds9922_ps_data *data)
{
	if (data->ps_status != data->ps_last_status) {
		input_report_rel(data->ps_input_dev, REL_RZ, data->ps_status);
		input_sync(data->ps_input_dev);
		data->ps_last_status = data->ps_status;
		SENSOR_LOG_DEBUG("data->ps_status = %d\n",data->ps_status);
	}
	return;
}
/*
 * Initialization function
 */

static int apds9922_init_client(struct apds9922_ps_data *data)
{
	struct apds9922_platform_data *pdata;
	struct i2c_client *client;
	int err;

	if (unlikely(IS_ERR_OR_NULL(data))) {
		SENSOR_LOG_ERROR("null pointer\n");
		return -EINVAL;
	}

	pdata = data->platform_data;
	client = pdata->client;
	/* Initialize Sensor */

	/* PS_LED */
	err = apds9921_dd_set_prx_led(client, APDS9921_DD_PRX_DEFAULT_LED_FREQ|APDS9921_DD_PRX_DEFAULT_LED_CURRENT);
	if (err < 0)
		return err;

	/* PS_PULSES */
	err = apds9921_dd_set_prx_pulses(client, data->ps_pulse);
	if (err < 0)
		return err;

	/* PS_MEAS_RATE */
	err = apds9921_dd_set_prx_meas_rate(client, 0x40|APDS9921_DD_PRX_DEFAULT_RES|APDS9921_DD_PRX_DEFAULT_MEAS_RATE);
	if (err < 0)
		return err;

	/* INT_PERSISTENCE */
	err = apds9921_dd_set_pers(client, APDS9921_DD_PRX_PERS_1|APDS9921_DD_ALS_PERS_1);
	if (err < 0)
	return err;

	/* PS_THRES_UP & PS_THRES_DOWN */
	err = apds9921_dd_set_prx_thresh(client, data->ps_thrd_low+data->ps_offset, data->ps_thrd_high+data->ps_offset); // init threshold for proximity
	if (err < 0)
		return err;

	/* PS Offset */

//	err = apds9921_dd_set_prx_can(client, data->ps_offset);
//	if (err < 0)
//		return err;

	if(!PS_POLLING)
		err = apds9922_ps_set_int_enable(data,1);
	else
		err = apds9922_ps_set_int_enable(data,0);
	 if (err < 0)
		return err;

	SENSOR_LOG_INFO("apds9922_init_client ok\n");
	return 0;
}

static int apds9922_get_object(struct apds9922_ps_data *data)
{
	int psdata = apds9922_get_ps_value(data);

	SENSOR_LOG_DEBUG("ps:%d\n", psdata);
	SENSOR_LOG_DEBUG("ps_status:%d\n", data->ps_status);
	SENSOR_LOG_DEBUG("ps_thrd_low:%d\n", data->ps_thrd_low);
	SENSOR_LOG_DEBUG("ps_thrd_high:%d\n", data->ps_thrd_high);

	if (psdata < data->ps_thrd_low) {
		data->ps_status = APDS9922_PS_FAR_DISTANCE;
		SENSOR_LOG_INFO("Object Far\n");
	}else if (psdata > data->ps_thrd_high) {
		data->ps_status = APDS9922_PS_NEAR_DISTANCE;
		SENSOR_LOG_INFO("Object Near\n");
	}

	return data->ps_status;
}
/*----------------------------------------------------------------------------*/

/* For HAL to Enable PS */
static ssize_t apds9922_show_enable_ps_sensor(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->ps_enable);
}

static ssize_t apds9922_store_enable_ps_sensor(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	int val;
	int err = -1;
	err = kstrtoint(buf, 0, &val);
	if (err) {
		SENSOR_LOG_ERROR("kstrtoint fail.\n");
		return count;
	}
	SENSOR_LOG_INFO("enable ps sensor (%d)\n", val);
	if ((val != 0) && (val != 1)) {
		SENSOR_LOG_INFO("enable ps sensor=%d\n", val);
		return count;
	}

	err = apds9922_enable_ps(data, val);
	if (err < 0)
		SENSOR_LOG_ERROR("apds9922_enable_ps error\n");
	return count;
}

static ssize_t apds9922_show_ps_poll_delay(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->ps_poll_delay);	// return in micro-second
}

static ssize_t apds9922_store_ps_poll_delay(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if (val < 100)
		val = 100;

	data->ps_poll_delay = (unsigned int)val;

	return count;
}

/* PS Value */
static ssize_t apds9922_show_ps(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	input_report_rel(data->ps_input_dev, REL_RZ, data->ps_status);
	input_sync(data->ps_input_dev);
	return sprintf(buf, "%d\n", apds9922_get_ps_value(data));
}
/* Write/Read Register data */
static ssize_t apds9922_show_reg(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	struct apds9922_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;
	int ret;
	int i = 0;
	int count = 0;
	u8 regdata = 0;

	for (i = 0; i < 0x28; i++)
	{
		ret = apds9922_i2c_read_reg(client, 0x00+i, &regdata);
		if (ret < 0)
			break;
		else
	count += sprintf(buf+count, "[%x] = (%x)\n", 0x00+i, regdata);
	}
	return count;
}

static ssize_t apds9922_store_reg(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	struct apds9922_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;
	int addr, cmd, ret;

	if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		SENSOR_LOG_ERROR("invalid format: '%s'\n", buf);
		return count;
	}
	ret = apds9922_i2c_write_reg(client, addr, cmd);
	return count;
}

static ssize_t apds9922_ps_offset_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);

	apds9922_get_psoffset(data);
	return count;
}

static ssize_t apds9922_ps_offset_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", apds9922_get_psoffset(data));
}

static ssize_t apds9922_ps_uncover_data_min_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", APDS9922_PS_UNCOVER_DATA_MIN);
}
static ssize_t apds9922_ps_uncover_data_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", APDS9922_PS_UNCOVER_DATA_MAX);
}

static ssize_t apds9922_ps_min_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", APDS9922_PS_THRESH_DATA_MIN);
}
static ssize_t apds9922_ps_max_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", APDS9922_PS_THRESH_DATA_MAX);
}
/* PS Calibration */
static ssize_t apds9922_ps_calibration_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	apds9922_run_calibration(data);
	return count;
}
static ssize_t apds9922_ps_calibration_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", apds9922_run_calibration(data));
}
/* Device init */
static ssize_t apds9922_show_dev_init(struct device *dev,struct device_attribute *attr,	char *buf)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "[%d,%d,%d]\n", data->ps_thrd_high,data->ps_thrd_low,data->ps_offset);
}

static ssize_t apds9922_store_dev_init(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	apds9922_ps_load_calibration_param(PS_CAL_FILE_PATH, data);
	return count;
}
static ssize_t apds9922_chip_name_show(struct device *dev,struct device_attribute *attr,	char *buf)
{
	sprintf(buf, "%s\n",APDS9922_CHIP_NAME);
	return strlen(buf);
}

static ssize_t apds9922_cal_debug_show(struct device *dev,struct device_attribute *attr,	char *buf)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "prox debug: [%d,%d,%d],crosstalk = %d near_diff_cnt = %d\n",data->ps_thrd_high ,data->ps_thrd_low,data->ps_offset,data->crosstalk, data->near_diff_cnt);
}

static ssize_t apds9922_prox_debug_show(struct device *dev,struct device_attribute *attr,	char *buf)
{
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "prox debug flag is %s\n", data->prox_debug? "true" : "false");

}
static ssize_t apds9922_prox_debug_store(struct device *dev,struct device_attribute *attr,	const char *buf, size_t size)
{
	int val = 0;
	int err;
	struct apds9922_ps_data *data = dev_get_drvdata(dev);
	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint fail.\n");
		return -EINVAL;
	}
	if (val) {
		data->prox_debug = true;
		data->ps_cal_low= 150;
		data->ps_cal_high= 150;
		apds9921_dd_set_prx_thresh(data->platform_data->client, data->ps_cal_low, data->ps_cal_high);
		apds9922_enable_ps(data, 1);
	} else {
		data->prox_debug = false;
		apds9921_dd_set_prx_thresh(data->platform_data->client, data->ps_thrd_low+data->ps_offset, data->ps_thrd_high+data->ps_offset);
		apds9922_enable_ps(data, 0);
		apds9922_ps_set_int_enable(data,1);
	}
	SENSOR_LOG_INFO("exit\n");

	return size;
}
static ssize_t apds9922_prox_data_max_show(struct device *dev,struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%d", APDS9922_PS_TH_MAX);
}

static struct device_attribute attrs_ps_device[] = {
	__ATTR(enable, 0664, apds9922_show_enable_ps_sensor, apds9922_store_enable_ps_sensor),
	__ATTR(delay, 0664, apds9922_show_ps_poll_delay, apds9922_store_ps_poll_delay),
	__ATTR(prox_init, 0664, apds9922_show_dev_init, apds9922_store_dev_init),
	__ATTR(prox_value, 0444, apds9922_show_ps, NULL),
	__ATTR(prox_offset_cal, 0664, apds9922_ps_offset_show, apds9922_ps_offset_store),
	__ATTR(prox_thres, 0664, apds9922_ps_calibration_show, apds9922_ps_calibration_store),
	__ATTR(prox_debug, 0660, apds9922_prox_debug_show, apds9922_prox_debug_store),
	__ATTR(chip_name, 0444, apds9922_chip_name_show, NULL),
	__ATTR(prox_data_max, 0444, apds9922_prox_data_max_show, NULL),
	__ATTR(prox_uncover_min, 0440, apds9922_ps_uncover_data_min_show, NULL),
	__ATTR(prox_uncover_max, 0440, apds9922_ps_uncover_data_max_show, NULL),
	__ATTR(prox_thres_max,0440, apds9922_ps_max_thres_show, NULL),
	__ATTR(prox_thres_min, 0440, apds9922_ps_min_thres_show, NULL),
	__ATTR(prox_cal_debug, 0444, apds9922_cal_debug_show, NULL),
	__ATTR(reg, 0664, apds9922_show_reg, apds9922_store_reg),

};
/*work que function*/
static void apds9922_work_func_proximity(struct work_struct *work)
{
	struct apds9922_ps_data *data = container_of(work,struct apds9922_ps_data, ps_dwork.work);
	int Pval;

	Pval = apds9922_get_object(data);
	SENSOR_LOG_DEBUG("PS value: %d\n", Pval);

	apds9922_report_event(data);
	if(PS_POLLING && data->ps_enable)
		schedule_delayed_work(&data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));
}

static int apds9922_check_intr(struct apds9922_ps_data *data)
{
	struct apds9922_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;
	u8 status;
	int err;

	err = apds9922_i2c_read_reg(client, APDS9921_DD_MAIN_STATUS_ADDR,&status);
	if (err < 0) {
		SENSOR_LOG_ERROR(" read ps status fail");
	}

	data->ps_detection = (status>>2)&0x01;

	err = apds9922_get_ps_value(data);
	if (err < 0) {
		SENSOR_LOG_ERROR(" read ps value fail");
		goto check_intr_exit;
	}

	if (data->prox_debug) {
		input_report_rel(data->ps_input_dev, REL_MISC, data->ps_data > 0 ? data->ps_data : 1);
		input_sync(data->ps_input_dev);
		if(data->ps_detection == 1 )
		{
			data->ps_detection = 0;
			data->ps_cal_low= 255;
			data->ps_cal_high= 255;
			apds9921_dd_set_prx_thresh(client, data->ps_cal_low , data->ps_cal_high);
		}
		else
		{
			data->ps_cal_low= 0;
			data->ps_cal_high= 0;
			apds9921_dd_set_prx_thresh(client, data->ps_cal_low , data->ps_cal_high);
		}
		goto check_intr_exit;
	}

	SENSOR_LOG_DEBUG("ps ps_data=%d  [%d,%d ,%d]\n", data->ps_data,  data->ps_thrd_low ,data->ps_thrd_high,data->ps_offset);


	if ( data->ps_detection == 1 ) {
		/* far-to-near detected */
		data->ps_status = APDS9922_PS_NEAR_DISTANCE;
		apds9922_report_event(data);
		SENSOR_LOG_DEBUG("ps far-to-near detected\n");

		if(data->ps_data > 254){
			data->ps_oil_occurred = 1;
			apds9921_dd_set_prx_thresh(client, data->ps_thrd_low+data->ps_offset+data->ps_oil_value, 255);     //¼ÙÉè´¥Åöµ½ÓÍÎÛ
		}else{
			apds9921_dd_set_prx_thresh(client, data->ps_thrd_low+data->ps_offset, 254);
		}

	}
	else {
		/* near-to-far detected */
		data->ps_status = APDS9922_PS_FAR_DISTANCE;
		apds9922_report_event(data);
		if(data->ps_oil_occurred)
		{
		 	data->ps_oil_occurred= 0;
			apds9921_dd_set_prx_thresh(client, data->ps_thrd_low+data->ps_offset+data->ps_oil_value, data->ps_thrd_high+data->ps_offset+data->ps_oil_value);
		}else{
			apds9921_dd_set_prx_thresh(client, data->ps_thrd_low+data->ps_offset, data->ps_thrd_high+data->ps_offset);
		}
		SENSOR_LOG_DEBUG("ps near-to-far detected\n");
	}

//	apds9921_dd_set_prx_pulses(client, data->ps_pulse);   //»áÓ°ÏìALSÂðå£¿  // ÄÜ·ñÊ¹Í¬×´Ì¬ÖÐ¶Ï

check_intr_exit:

	SENSOR_LOG_INFO("exit.\n");
	return 0;
}

static void apds9922_irq_enable(struct apds9922_ps_data *data, bool enable, bool flag_sync)
{
	if (enable == data->irq_enabled) {
		SENSOR_LOG_DEBUG("doubule %s irq %d\n",enable? "enable" : "disable",data->irq);
		return;
	} else {
		data->irq_enabled = enable;
	}

	if (enable) {
		enable_irq(data->irq);
	} else {
		if (flag_sync) {
			disable_irq(data->irq);
		} else {
			disable_irq_nosync(data->irq);
		}
	}
}

static void apds9922_work_func_irq(struct work_struct *work)
{
	struct apds9922_ps_data *data;
	SENSOR_LOG_DEBUG("IRQ Work INT\n");
	data = container_of((struct work_struct *)work, struct apds9922_ps_data, irq_dwork);
	/* Add Oil Alg */
	wake_lock_timeout(&data->apds9922_wake_lock, msecs_to_jiffies(1000));

	apds9922_check_intr(data);
}

static irqreturn_t apds9922_irq(int irq, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct apds9922_platform_data *pdata = client->dev.platform_data;
	struct apds9922_platform_device *device_node;
	struct apds9922_ps_data *data;

      /*Device Initialize*/
	list_for_each_entry(device_node, &pdata->hw_device_list, node) {
		if (device_node->handle == PS) {
			data = container_of(device_node, struct apds9922_ps_data, device_node);
		}
	}
	if (!IS_ERR_OR_NULL(data)) {
		if (0 == queue_work(data->irq_work_queue, &data->irq_dwork))
			SENSOR_LOG_ERROR("schedule_work failed!\n");
	}

	return IRQ_HANDLED;
}

/*Suspend/Resume*/
static int apds9922_ps_suspend(struct apds9922_platform_device *dev_node)
{
	return 0;
}

static int apds9922_ps_resume(struct apds9922_platform_device *dev_node)
{
	return 0;
}

static int apds9922_pinctrl_init(struct apds9922_ps_data *data, struct device *dev)
{
	int rc;

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
	SENSOR_LOG_ERROR("data->pinctrl is NULL\n");
	return PTR_ERR(data->pinctrl);
	}

	data->pin_default = pinctrl_lookup_state(data->pinctrl, "apds9922_default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		SENSOR_LOG_ERROR("lookup default state failed\n");
		return PTR_ERR(data->pin_default);
	}

	data->pin_sleep = pinctrl_lookup_state(data->pinctrl, "apds9922_sleep");
	if (IS_ERR_OR_NULL(data->pin_sleep)) {
		SENSOR_LOG_ERROR("lookup sleep state failed\n");
		return PTR_ERR(data->pin_sleep);
	}

	if (!IS_ERR_OR_NULL(data->pinctrl)) {
		rc = pinctrl_select_state(data->pinctrl, data->pin_default);
		if (rc) {
		SENSOR_LOG_ERROR("select default state failed\n");
		return rc;
		}
	}
	SENSOR_LOG_INFO("pinctrl init success\n");
	return 0;
}

static int apds9922_ps_hw_init(struct apds9922_platform_data *pdata)
{
	int err;
	struct i2c_client *client;
	struct apds9922_ps_data *data;
	struct apds9922_platform_device *pdev_node;
	if (unlikely(IS_ERR_OR_NULL(pdata))) {
	SENSOR_LOG_ERROR("null pointer exception.\n");
	return -EINVAL;
	}

	list_for_each_entry(pdev_node, &pdata->hw_device_list, node) {
	/*Device Initialize*/
		if (pdev_node->handle == PS) {
			data = container_of(pdev_node, struct apds9922_ps_data, device_node);
		}
	}

	if (unlikely(IS_ERR_OR_NULL(data))) {
		SENSOR_LOG_ERROR("data is null\n");
		return -EINVAL;
	}

	client = pdata->client;

	err = apds9922_pinctrl_init(data, &pdata->client->dev);
	if (err) {
		SENSOR_LOG_ERROR("apds9922 ps pinctrl init failed\n");
		return -ENODEV;
	}

	/*Device Initialize*/
	err = apds9922_init_client(data);
	if (err < 0) {
		SENSOR_LOG_ERROR("init apds9922 failed when probe\n");
		return -ENODEV;
	}
	if (gpio_is_valid(data->irq_gpio)) {
		/* configure apds9922 irq gpio */
		SENSOR_LOG_INFO("gpio value is %d \n", gpio_get_value(data->irq_gpio));
		err = gpio_request_one(data->irq_gpio,GPIOF_DIR_IN,"apds9922_irq_gpio");
		if (err) {
			SENSOR_LOG_ERROR("unable to request gpio %d\n",data->irq_gpio);
		return -ENODEV;
	}

	data->irq = client->irq =gpio_to_irq(data->irq_gpio);
	} else {
		SENSOR_LOG_ERROR("irq gpio not provided\n");
	}
	return 0;
}

static void apds9922_ps_hw_exit(struct apds9922_platform_data *pdata)
{
	struct apds9922_ps_data *data;
	struct apds9922_platform_device *device_node;
	if (IS_ERR_OR_NULL(pdata)) {
		SENSOR_LOG_ERROR("null pointer exception.\n");
		return;
	}
	list_for_each_entry(device_node, &pdata->hw_device_list, node) {
        /*Device Initialize*/
	if (device_node->handle == PS) {
		data = container_of(device_node, struct apds9922_ps_data, device_node);
	}
	}
	if (data && gpio_is_valid(data->irq_gpio))
		gpio_free(data->irq_gpio);
}

static int apds9922_ps_parse_dt(struct device *dev,struct apds9922_platform_data *pdata,struct apds9922_ps_data *data)
{
	struct device_node *np = dev->of_node;

	unsigned int tmp = 0;
	int rc = 0;

	/* irq gpio */
	rc = of_get_named_gpio(dev->of_node,"avago,irq-gpio", 0);
	if (rc < 0) {
		SENSOR_LOG_ERROR("Unable to read irq gpio\n");
		return rc;
	}
	data->irq_gpio = rc;
	SENSOR_LOG_INFO("irq gpio is %d\n", data->irq_gpio);

	/* ps tuning data*/
	rc = of_property_read_u32(np, "avago,ps_threshold_low", &tmp);
	data->ps_thrd_low= (!rc ? tmp : 38);
	SENSOR_LOG_INFO("ps_threshold_low is %d\n", data->ps_thrd_low);

	rc = of_property_read_u32(np, "avago,ps_threshold_high", &tmp);
	data->ps_thrd_high = (!rc ? tmp : 50);

	rc = of_property_read_u32(np, "avago,ps_offset", &tmp);
	data->ps_offset = (!rc ? tmp : 60);

	rc = of_property_read_u32(np, "avago,ps-pulse", &tmp);
	data->ps_pulse = (!rc ? tmp : 16);
	SENSOR_LOG_INFO("ps_pulse is %d\n", data->ps_pulse);

	return 0;
}
static int apds9922_ps_input_device_init(struct apds9922_ps_data *data)
{
	int err = 0;
	if (unlikely(IS_ERR_OR_NULL(data))) {
		SENSOR_LOG_ERROR("null pointer exception\n");
		return -ENODEV;
	}
	/* allocate proximity input_device */
	data->ps_input_dev = input_allocate_device();
	if (IS_ERR_OR_NULL(data->ps_input_dev)) {
		err = -ENOMEM;
		SENSOR_LOG_INFO("could not allocate input device\n");
		goto exit;
	}

	input_set_drvdata(data->ps_input_dev, data);
	data->ps_input_dev->name = INPUT_NAME_PS;
	data->ps_input_dev->id.bustype = BUS_I2C;
	set_bit(EV_REL, data->ps_input_dev->evbit);
	set_bit(REL_RZ,  data->ps_input_dev->relbit);
	set_bit(REL_MISC,  data->ps_input_dev->relbit);

	SENSOR_LOG_INFO("registering proximity input device\n");
	err = input_register_device(data->ps_input_dev);
	if (err < 0) {
		SENSOR_LOG_INFO("could not register input device\n");
		err = -ENOMEM;
		goto input_register_err;
	}
input_register_err:
	input_free_device(data->ps_input_dev);
exit:
	return err;
}

static struct apds9922_platform_ops ps_platform_ops = {
      .init = apds9922_ps_hw_init,
      .exit = apds9922_ps_hw_exit,
      .resume = apds9922_ps_resume,
      .suspend = apds9922_ps_suspend,
      .unregister = apds9922_ps_unregister,
};

int apds9922_ps_register(struct apds9922_platform_data *pdata, struct i2c_driver *driver)
{
	int err = 0;
	struct apds9922_ps_data *data;
	struct i2c_client *client = pdata->client;

	if (unlikely(IS_ERR_OR_NULL(pdata))) {
		SENSOR_LOG_ERROR("null pdata\n");
		return -EINVAL;
	}

	SENSOR_LOG_DEBUG("probe start\n");
	data = kzalloc(sizeof(struct apds9922_ps_data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(data)) {
		SENSOR_LOG_ERROR("kzalloc apds9922_ps_data failed\n");
		err = -ENOMEM;
		goto exit;
	}
	/*parse device tree*/
	err = apds9922_ps_parse_dt(&client->dev, pdata, data);
	if (err < 0) {
		SENSOR_LOG_ERROR("sensor_parse_dt() err\n");
		goto exit_alloc_failed;
	}


	data->ps_poll_delay = APDS9922_PS_POLL_DELAY;
	data->ps_detection = 0;	/* default to no detection */
	data->ps_oil_occurred = 0;
	data->ps_oil_value = 20;
	data->prox_debug = false;


	/*add client data to list*/
	data->device_node.handle = PS;
	data->device_node.name = "prox";
	data->device_node.ops = &ps_platform_ops;

	/*initialize apds9922_ps_data*/
	mutex_init(&data->ps_lock);
	data->platform_data = pdata;

	proximity_class = class_create(THIS_MODULE, "proximity");
	data->proximity_dev = device_create(proximity_class, NULL, apds9922_proximity_dev_t, driver ,"proximity");
	if (IS_ERR_OR_NULL(data->proximity_dev)) {
		err = -ENODEV;
		SENSOR_LOG_ERROR("device_create proximity failed\n");
		goto exit_alloc_failed;
	}

	dev_set_drvdata(data->proximity_dev, data);

	err = apds9922_create_sysfs_interfaces(data->proximity_dev, attrs_ps_device, ARRAY_SIZE(attrs_ps_device));
	if (err < 0) {
		SENSOR_LOG_ERROR("create sysfs interfaces failed\n");
		goto create_proximity_dev_failed;
	}

	SENSOR_LOG_INFO("registering proximity sensor input device\n");
	err = apds9922_ps_input_device_init(data);
	if (err < 0) {
		SENSOR_LOG_ERROR("input device init fail.\n");
		goto create_proximity_dev_failed;
	}
	wake_lock_init(&data->apds9922_wake_lock, WAKE_LOCK_SUSPEND ,"apds9922_wake_lock");

	if (PS_POLLING){
		INIT_DELAYED_WORK(&data->ps_dwork, apds9922_work_func_proximity);
	}else{
		INIT_WORK(&data->irq_dwork, apds9922_work_func_irq);
	}

	data->irq_work_queue = create_singlethread_workqueue("apds9922_work_queue");
	if (IS_ERR_OR_NULL(data->irq_work_queue)){
		err = -ENOMEM;
		SENSOR_LOG_ERROR( "cannot create work taos_work_queue, err = %d",err);
		goto exit_remove_sysfs_interfaces;
	}

	/* probe finish, and add ps client to list */
	list_add_tail(&data->device_node.node, &pdata->hw_device_list);

	/* h/w initialization */
	if (data) {
		err = data->device_node.ops->init(pdata);
	if (err < 0)
		goto exit_remove_sysfs_interfaces;
	}

	irq_set_irq_wake(client->irq, 1);
	/*Interrupt Regist*/
	if (!PS_POLLING ) {
		err = request_irq(data->irq, apds9922_irq,IRQF_TRIGGER_FALLING | IRQF_ONESHOT,APDS9922_DRV_NAME, (void *)client);
		if (err) {
			SENSOR_LOG_ERROR("Could not get IRQ\n");
			goto exit_probe;
		}
	}

	apds9922_irq_enable(data, true, false);
	SENSOR_LOG_INFO("probe ps finish.\n");
	return 0;
exit_probe:
	if(data->device_node.ops->exit)
		data->device_node.ops->exit(pdata);
	list_del(&data->device_node.node);
exit_remove_sysfs_interfaces:
	wake_lock_destroy(&data->apds9922_wake_lock);
	mutex_destroy(&data->ps_lock);
	apds9922_remove_sysfs_interfaces(data->proximity_dev, attrs_ps_device, ARRAY_SIZE(attrs_ps_device));
create_proximity_dev_failed:
	data->proximity_dev = NULL;
	device_destroy(proximity_class, apds9922_proximity_dev_t);
	class_destroy(proximity_class);
exit_alloc_failed:
	kfree(data);
exit:
	return err;
}

static int apds9922_ps_unregister(struct apds9922_platform_device *pdev_node)
{
	struct apds9922_ps_data *data;
	struct device *dev;
	struct apds9922_platform_data *pdata;
	struct i2c_client *client;
	if (IS_ERR_OR_NULL(pdev_node))
	  return -EINVAL;

	data = container_of(pdev_node, struct apds9922_ps_data, device_node);
	dev = data->proximity_dev;
	pdata = data->platform_data;
	client = pdata->client;

	if (data->ps_enable)
		apds9922_enable_ps(data, 0);
	list_del(&data->device_node.node);
	input_unregister_device(data->ps_input_dev);
	input_free_device(data->ps_input_dev);
	apds9922_remove_sysfs_interfaces(dev, attrs_ps_device, ARRAY_SIZE(attrs_ps_device));
	wake_lock_destroy(&data->apds9922_wake_lock);

	if (!PS_POLLING)
	free_irq(data->irq, client);

	if (data->device_node.ops->exit)
	data->device_node.ops->exit(pdata);

	mutex_destroy(&data->ps_lock);
	kfree(data);
	return 0;
}
