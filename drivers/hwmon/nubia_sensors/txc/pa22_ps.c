/*
 * This file is part of the pa22 sensor driver.
 * pa22 is combined proximity, and VCSEL.
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
#include "pa22_common.h"
#include "pa22_ps.h"

static dev_t const pa22_proximity_dev_t = MKDEV(MISC_MAJOR, 101);
static struct class         *proximity_class;

static int far_ps_min = PA24_PS_OFFSET_MAX;
static int saturation_flag = 0;
static const int ps_steady = ps_ary_size + 4;
static int oil_occurred = 0;
static int ps_had_load_cal_param = 0;

u8 ps_seq_far[ps_ary_size];
u8 ps_seq_oil[ps_ary_size];
u8 ps_seq_near[ps_ary_size];

static void pa22_ps_load_calibration_param(char *filename, struct pa22_ps_data *data)
{
	u8 param[2] = {PA24_PS_OFFSET_DEFAULT, PA24_MIN_NEAR_CNT};
	int ret;

	ret = pa22_read_file(filename, param, sizeof(param));
	if (ret < 0) {
		data->crosstalk = PA24_PS_OFFSET_DEFAULT;
		data->near_diff_cnt = PA24_MIN_NEAR_CNT;
		data->far_diff_cnt = PA24_MIN_NEAR_CNT / 2;
	} else {
		data->crosstalk = param[0];
		data->near_diff_cnt = param[1];
		data->far_diff_cnt = param[1] / 2;
	}
	SENSOR_LOG_INFO("prox debug: crosstalk = %d near_diff_cnt = %d\n", data->crosstalk, data->near_diff_cnt);

	ps_had_load_cal_param = 1;
}
static int pa22_ps_set_enable(struct pa22_ps_data *data, int enable)
{
	int err = 0;
	u8 regdata = 0;
	struct i2c_client *client;
	if (unlikely(IS_ERR_OR_NULL(data))) {
		SENSOR_LOG_ERROR("null pointer./n");
		return -PTR_ERR(data);
	}
	client = data->platform_data->client;
	mutex_lock(&data->platform_data->reg_lock);
	err = i2c_read_reg(data->platform_data->client, REG_CFG0, &regdata);
	if (err < 0) {
		SENSOR_LOG_ERROR("i2c read failed\n");
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}

	regdata &= ~(PS_ACTIVE);
	regdata |= (enable << 1);

	err = i2c_write_reg(client, REG_CFG0, (regdata));
	if (err < 0) {
		SENSOR_LOG_ERROR("i2c read failed\n");
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}
	mutex_unlock(&data->platform_data->reg_lock);
	return err;
}
static int pa22_enable_ps(struct pa22_ps_data *data, int enable)
{
	struct pa22_platform_data *pdata;
	struct i2c_client *client;
	int i = 0;
	int err = 0;
	u8 psdata = 0, regdata = 0;
	if (unlikely(IS_ERR_OR_NULL(data))) {
	    SENSOR_LOG_ERROR("null pointer exception.\n");
	    return -EINVAL;
	}
	pdata = data->platform_data;
	client = pdata->client;

	if (!ps_had_load_cal_param)
		pa22_ps_load_calibration_param(PS_CAL_FILE_PATH, data);

	data->ps_enable = enable;

	if (PS_POLLING) {
		i2c_write_reg(client, REG_CFG0, (regdata));
		if (enable)
			schedule_delayed_work(&data->ps_dwork, msecs_to_jiffies(data->ps_enable_delay));
	} else if (!PS_POLLING) {
		if (enable) {
			if (!pdata->vdd_always_on) {
				sensor_platform_hw_power_on(pdata, true);
				pa22_init_client(data);
				mutex_lock(&pdata->lock);
				pdata->enable_mask |= 1 << PS;
				mutex_unlock(&pdata->lock);
			}
			saturation_flag = 0;
			oil_occurred = 0;
			data->ps_status = PA24_PS_FAR_DISTANCE;
			data->ps_last_status = PA24_PS_UNKOWN_DISTANCE;

			for (i = 0;i < ps_ary_size; i++)
			{
				ps_seq_far[i] = 255;
				ps_seq_oil[i] = 255;
				ps_seq_near[i]= 255;
			}

			i2c_write_reg(client, REG_PS_TH, 0xFF);
			i2c_write_reg(client, REG_PS_TL, 0x00);

			i2c_write_reg(client, REG_CFG1 ,(PA24_LED_CURR << 4) | (PA24_PS_PRST << 2));
			i2c_write_reg(client, REG_CFG2, (PA24_PS_MODE << 6) | (PA24_PS_SET << 2));
			err = pa22_ps_set_enable(data, enable);
			if (err < 0) {
				SENSOR_LOG_ERROR("enable ps fail\n");
				return err;
			}
			msleep(PA24_PS_ENABLE_DELAY);

			/* Window type */
			data->ps_thrd_high = PA24_PS_OFFSET_MAX;
			data->ps_thrd_low = PA24_PS_OFFSET_MAX - 1;

			/* report far/near event when first enable*/
			err= i2c_read_reg(client, REG_PS_DATA, &psdata);
			if (err < 0) {
			    SENSOR_LOG_ERROR("i2c_read function err = %d\n",err);
			    return -1;
			}
			if (psdata <= data->ps_thrd_low)
			    data->ps_status = PA24_PS_FAR_DISTANCE;
			else if (psdata >= data->ps_thrd_high)
			    data->ps_status = PA24_PS_NEAR_DISTANCE;
			pa22_report_event(data);

			/* set threshold and start irq*/
			pa22_irq_enable(data, true, false);
			i2c_write_reg(client, REG_PS_TH, data->ps_thrd_high);
			i2c_write_reg(client, REG_PS_TL, data->ps_thrd_low);

		} else {
			err = pa22_ps_set_enable(data, enable);
            if (err < 0) {
                return err;
            }
			pa22_irq_enable(data, false, true);
			if (!pdata->vdd_always_on) {
                mutex_lock(&pdata->lock);
                pdata->enable_mask &= ~(1 << PS);
                mutex_unlock(&pdata->lock);
                sensor_platform_hw_power_on(pdata, false);
			}
		}
	}

	return 0;
}

void pa22_swap(u8 *x, u8 *y)
{
	u8 temp = *x;
	*x = *y;
	*y = temp;
}

static int pa22_get_psoffset(struct pa22_ps_data *data)
{
	struct pa22_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;

	int i, j;
	int ret;
	u16 sum_of_pdata = 0;
	int tmp_uncover_data = 0;
	u8 temp_pdata[20], cfg0data = 0, cfg2data = 0;
	unsigned int ArySize = 12;
	int scale = ArySize / 4;

	SENSOR_LOG_INFO("START proximity sensor calibration\n");

	/*Offset mode & disable intr from ps*/
	ret = i2c_read_reg(client, REG_CFG2, &cfg2data);
	ret = i2c_write_reg(client, REG_CFG2, 0x08);

	/*Set crosstalk = 0*/
	ret = i2c_write_reg(client, REG_PS_OFFSET, 0x00);

	/*PS On*/
	ret = i2c_read_reg(client, REG_CFG0, &cfg0data);
	ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x02);

	for (i = 0; i < ArySize; i++)
	{
		mdelay(50);
		ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata + i);
		SENSOR_LOG_INFO("temp_data = %d\n", temp_pdata[i]);
	}

	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
	for (j = i + 1; j < ArySize; j++)
		if (temp_pdata[i] > temp_pdata[j])
			pa22_swap(temp_pdata + i, temp_pdata + j);

	/* calculate the cross-talk using central 10 data */
	for (i = scale; i < ArySize - scale; i++)
	{
		SENSOR_LOG_INFO("temp_pdata = %d\n", temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}
	tmp_uncover_data = sum_of_pdata * 2 / ArySize;
	if (tmp_uncover_data > PA24_PS_UNCOVER_MAX_SAFE) {
		SENSOR_LOG_ERROR("uncover data too big, keep sensor naked or structure is not qualified!\n");
		ret = i2c_write_reg(client, REG_CFG2, cfg2data);
		return tmp_uncover_data;
	}
	mutex_lock(&data->ps_lock);
	data->crosstalk = tmp_uncover_data;
	mutex_unlock(&data->ps_lock);
	SENSOR_LOG_INFO("sum_of_pdata = %d   cross_talk = %d\n",
                        sum_of_pdata, data->crosstalk);

	/* Restore CFG2 */
	ret = i2c_write_reg(client, REG_CFG2, cfg2data);

	SENSOR_LOG_INFO("FINISH get proximity sensor\n");

	return data->crosstalk;
}
/*
 * return value
 * -1: need naked calibration
 * -2: need 3cm gray-card cailibraion
 */
static int pa22_run_calibration(struct pa22_ps_data *data)
{
	struct pa22_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;
	int i, j;
	int ret;
	u16 sum_of_pdata = 0;
	int tmp_near_diff = 0;
	u8 temp_pdata[20], buftemp[2], cfg0data = 0, cfg2data = 0;
	unsigned int ArySize = 12;
	int scale = ArySize / 4;

	SENSOR_LOG_INFO("start calc base-noise value\n");
	if (data->crosstalk < PA24_PS_UNCOVER_MIN_SAFE || data->crosstalk > PA24_PS_UNCOVER_MAX_SAFE) {
		SENSOR_LOG_ERROR("Need Run Naked Calibration first or fail!\n");
		return -ERR_NAKED_CAL;
	}

	/* Prevent interrput */
	/*Offset mode & disable intr from ps*/
	ret = i2c_read_reg(client, REG_CFG2, &cfg2data);
	ret = i2c_write_reg(client, REG_CFG2, 0x08);

	/*Set crosstalk = 0*/
	ret = i2c_write_reg(client, REG_PS_OFFSET, 0x00);

	/*PS On*/
	ret = i2c_read_reg(client, REG_CFG0, &cfg0data);
	ret = i2c_write_reg(client, REG_CFG0, cfg0data | 0x02);

	for (i = 0; i < ArySize; i++)
	{
		mdelay(50);
		ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata + i);
		SENSOR_LOG_INFO("temp_data = %d\n", temp_pdata[i]);
	}

	/* pdata sorting */
	for (i = 0; i < ArySize - 1; i++)
	for (j = i + 1; j < ArySize; j++)
		if (temp_pdata[i] > temp_pdata[j])
			pa22_swap(temp_pdata + i, temp_pdata + j);

	/* calculate the cross-talk using central 10 data */
	for (i = scale; i < ArySize - scale; i++)
	{
		SENSOR_LOG_INFO("temp_pdata = %d\n",temp_pdata[i]);
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	tmp_near_diff = sum_of_pdata * 2 / ArySize - data->crosstalk;
	SENSOR_LOG_INFO("tmp_near_diff = %d\n", tmp_near_diff);
	if (tmp_near_diff < PA24_MIN_NEAR_CNT || tmp_near_diff > PA24_MAX_NEAR_CNT) {
		SENSOR_LOG_ERROR("thres calibration failed\n");
		ret = i2c_write_reg(client, REG_CFG2, cfg2data);
		return -ERR_THRES_CAL;
	}
	mutex_lock(&data->ps_lock);
	data->near_diff_cnt = tmp_near_diff;
	data->far_diff_cnt = tmp_near_diff / 2;
	mutex_unlock(&data->ps_lock);
	SENSOR_LOG_INFO("sum_of_pdata = %d   near_diff_cnt = %d\n",
                        sum_of_pdata, data->near_diff_cnt);

	ret = i2c_write_reg(client, REG_CFG2, cfg2data);


	buftemp[0] = (u8)data->crosstalk;
	buftemp[1] = (u8)data->near_diff_cnt;

	if (pa22_write_file(PS_CAL_FILE_PATH, buftemp, sizeof(buftemp)) < 0) {
		SENSOR_LOG_INFO("Open PS calibration file error!!");
		return -ERR_FILE_OPS;
	}

	SENSOR_LOG_INFO("FINISH proximity sensor calibration\n");

	return data->near_diff_cnt;
}

static int pa22_get_ps_value(struct i2c_client *client)
{
	u8 regdata = 0;
	i2c_read_reg(client, REG_PS_DATA, &regdata);
	return regdata;
}

static void pa22_report_event(struct pa22_ps_data *data)
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

static int pa22_init_client(struct pa22_ps_data *data)
{
	struct pa22_platform_data *pdata;
	struct i2c_client *client;

	if (unlikely(IS_ERR_OR_NULL(data))) {
		SENSOR_LOG_ERROR("null pointer\n");
		return -EINVAL;
	}

	pdata = data->platform_data;
	client = pdata->client;
	/* Dealy time setting */
	data->ps_poll_delay = PA24_PS_POLL_DELAY;
	data->ps_enable_delay = PA24_PS_ENABLE_DELAY;
	/* Initialize Sensor */
	i2c_write_reg(client, REG_CFG1,
		(PA24_LED_CURR	<< 4)| (PA24_PS_PRST << 2) );
	i2c_write_reg(client, REG_CFG3,
		(PA24_INT_TYPE	<< 6)| (PA24_PS_PERIOD << 3) );
	i2c_write_reg(client, REG_PS_SET, 0x82);
	i2c_write_reg(client, REG_CFG4, 0x0C);
	i2c_write_reg(client, REG_CFG2,
		((PA24_PS_MODE	<< 6)|(PA24_INT_TYPE << 2)));
	SENSOR_LOG_INFO("pa22_init_client ok\n");
	return 0;
}
static int pa22_get_object(struct pa22_ps_data *data)
{
	struct pa22_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;

	u8 psdata = pa22_get_ps_value(client);

	SENSOR_LOG_INFO("PS:%d\n", psdata);
	SENSOR_LOG_INFO("ps_status:%d\n", data->ps_status);
	SENSOR_LOG_INFO("ps_thrd_low:%d\n", data->ps_thrd_low);
	SENSOR_LOG_INFO("ps_thrd_high:%d\n", data->ps_thrd_high);
	switch (data->ps_status) {
		case PA24_PS_NEAR_DISTANCE:
			if (psdata < data->ps_thrd_low) {
				data->ps_status = PA24_PS_FAR_DISTANCE;
				SENSOR_LOG_INFO("Object Far\n");
			}
			break;
		case PA24_PS_FAR_DISTANCE:
			if (psdata > data->ps_thrd_high) {
				data->ps_status = PA24_PS_NEAR_DISTANCE;
				SENSOR_LOG_INFO("Object Near\n");
			}
			break;
	}

	return data->ps_status;
}
/*----------------------------------------------------------------------------*/

/* For HAL to Enable PS */
static ssize_t pa22_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->ps_enable);
}
static ssize_t pa22_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
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

	err = pa22_enable_ps(data, val);
	if (err < 0)
		SENSOR_LOG_ERROR("pa22_enable_ps error\n");
	return count;
}

static ssize_t pa22_show_ps_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->ps_poll_delay);	// return in micro-second
}

static ssize_t pa22_store_ps_poll_delay(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if (val < 10)
	    val = 10;

	data->ps_poll_delay = (unsigned int)val;

	return count;
}

/* PS Value */
static ssize_t pa22_show_ps(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	struct pa22_platform_data *pdata = data->platform_data;
	input_report_rel(data->ps_input_dev, REL_RZ, data->ps_status);
	input_sync(data->ps_input_dev);
	return sprintf(buf, "%d\n", pa22_get_ps_value(pdata->client));
}
/* Write/Read Register data */
static ssize_t pa22_show_reg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	struct pa22_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;
	int ret;
	int i = 0;
	int count = 0;
	u8 regdata = 0;

	for (i = 0; i < 19; i++)
	{
		ret = i2c_read_reg(client, 0x00+i, &regdata);
		if (ret < 0)
			break;
		else
			count += sprintf(buf+count, "[%x] = (%x)\n", 0x00+i, regdata);
	}
	return count;
}
static ssize_t pa22_store_reg(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
    struct pa22_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;
	int addr, cmd, ret;

	if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		SENSOR_LOG_ERROR("invalid format: '%s'\n", buf);
		return count;
	}
	ret = i2c_write_reg(client, addr, cmd);
	return count;	
}
static ssize_t pa22_ps_offset_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	pa22_get_psoffset(data);
	return count;
}
static ssize_t pa22_ps_offset_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pa22_get_psoffset(data));
}
/* PS Calibration */
static ssize_t pa22_ps_calibration_store(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	pa22_run_calibration(data);
	return count;
}
static ssize_t pa22_ps_calibration_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", pa22_run_calibration(data));
}
/* Device init */
static ssize_t pa22_store_dev_init(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}
static ssize_t pa22_chip_name_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	sprintf(buf, "%s\n",PA22_DRV_NAME);
	return strlen(buf);
}
static ssize_t pa22_prox_debug_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "prox debug flag is %s\n", data->prox_debug? "true" : "false");
}
static ssize_t pa22_cal_debug_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "prox debug: crosstalk = %d near_diff_cnt = %d\n",
					data->crosstalk, data->near_diff_cnt);
}
static ssize_t pa22_prox_uncover_data_min_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	SENSOR_LOG_DEBUG("max uncover data is %d\n", PA24_PS_UNCOVER_MIN_SAFE);
	return sprintf(buf, "%d\n", PA24_PS_UNCOVER_MIN_SAFE);
}
static ssize_t pa22_prox_uncover_data_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	SENSOR_LOG_DEBUG("max uncover data is %d\n", PA24_PS_UNCOVER_MAX_SAFE);
	return sprintf(buf, "%d\n", PA24_PS_UNCOVER_MAX_SAFE);
}
static ssize_t pa22_ps_max_thres_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->crosstalk + PA24_MAX_NEAR_CNT);
}
static ssize_t pa22_ps_min_thres_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->crosstalk + PA24_MIN_NEAR_CNT);
}

static ssize_t pa22_prox_debug_store(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	int val = 0;
	int err;
	struct pa22_ps_data *data = dev_get_drvdata(dev);
	SENSOR_LOG_ERROR("enter\n");
	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint fail.\n");
		return -EINVAL;
	}

	if (val) {
		data->prox_debug = true;
	} else {
		data->prox_debug = false;
	}
	SENSOR_LOG_ERROR("exit\n");
	return size;
}
static ssize_t pa22_prox_data_max_show(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return sprintf(buf, "%d", PA24_PS_TH_MAX);
}
static struct device_attribute attrs_ps_device[] = {
	__ATTR(enable, 0664, pa22_show_enable_ps_sensor, pa22_store_enable_ps_sensor),
	__ATTR(delay, 0664, pa22_show_ps_poll_delay, pa22_store_ps_poll_delay),
	__ATTR(prox_value, 0444, pa22_show_ps, NULL),
	__ATTR(reg, 0664, pa22_show_reg, pa22_store_reg),
	__ATTR(prox_thres, 0664, pa22_ps_calibration_show, pa22_ps_calibration_store),
	__ATTR(prox_thres_max, 0440, pa22_ps_max_thres_show, NULL),
	__ATTR(prox_thres_min, 0440, pa22_ps_min_thres_show, NULL),
	__ATTR(prox_init, 0220, NULL, pa22_store_dev_init),
	__ATTR(prox_debug, 0660, pa22_prox_debug_show, pa22_prox_debug_store),
	__ATTR(prox_data_max, 0444, pa22_prox_data_max_show, NULL),
	__ATTR(prox_uncover_min, 0444, pa22_prox_uncover_data_min_show, NULL),
	__ATTR(prox_uncover_max, 0444, pa22_prox_uncover_data_max_show, NULL),
	__ATTR(chip_name, 0444, pa22_chip_name_show, NULL),
	__ATTR(prox_offset_cal, 0664, pa22_ps_offset_show, pa22_ps_offset_store),
	__ATTR(prox_cal_debug, 0444, pa22_cal_debug_show, NULL),
};
/*work que function*/
static void pa22_work_func_proximity(struct work_struct *work)
{
	struct pa22_ps_data *data = container_of(work,
						struct pa22_ps_data, ps_dwork.work);
	int Pval;
  	Pval = pa22_get_object(data);
	SENSOR_LOG_INFO("PS value: %d\n", Pval);

	input_report_rel(data->ps_input_dev, REL_DIAL, Pval ? PA24_PS_FAR_DISTANCE: PA24_PS_NEAR_DISTANCE);
	input_sync(data->ps_input_dev);

	if(PS_POLLING && data->ps_enable)
		schedule_delayed_work(&data->ps_dwork, msecs_to_jiffies(data->ps_poll_delay));
}

static void pa22_get_ps_slope_array(u8 *ps_seq, int *slope, u8 ps, int arysize)
{
	int i;

	for (i=0; i<arysize-1; i++)
	{
		ps_seq[arysize-1-i] = ps_seq[arysize-1-i-1];
		if (ps_seq[arysize-1-i] == 0)
			ps_seq[arysize-1-i] = ps;
	}
	ps_seq[0] = (int)ps;

	for (i=0; i<arysize-1; i++)
	{
		slope[i] = (int)(ps_seq[i] - ps_seq[i+1]);
	}
	return;
}
static int pa22_check_intr(struct pa22_ps_data *data)
{
	struct pa22_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;
	int res;
	u8 psdata=0;
	u8 cfgdata=0;
	static int far_loop = 0;

	int slope[ps_ary_size-1];
	int sum = 0, abs_sum = 0, ps_sum = 0;
	SENSOR_LOG_INFO("enter.\n");
	res = i2c_read_reg(client, REG_PS_DATA, &psdata);
	if (res < 0) {
		SENSOR_LOG_ERROR("i2c_read function err res = %d\n",res);
		return -1;
	}

	if (data->prox_debug) {
		input_report_rel(data->ps_input_dev, REL_MISC, psdata > 0 ? psdata : 1);
		input_sync(data->ps_input_dev);
		goto check_intr_exit;
	}
	//SUNLIGHT
	if (psdata == 0) {

		if (data->ps_status == PA24_PS_NEAR_DISTANCE) {
			i2c_write_reg(client,REG_CFG1,
						(PA24_LED_CURR << 4)| (PA24_PS_PRST << 2) );
		}
		saturation_flag = 1;

		if (oil_occurred && far_ps_min < PA24_PS_OFFSET_MAX) {
			data->ps_thrd_high = far_ps_min + OIL_EFFECT + data->near_diff_cnt;
			data->ps_thrd_low = far_ps_min + OIL_EFFECT;
		} else if (!oil_occurred && far_ps_min < PA24_PS_OFFSET_MAX) {
			data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
			data->ps_thrd_low = far_ps_min + data->near_diff_cnt - data->far_diff_cnt;//data->near_diff_cnt - PA24_NEAR_FAR_CNT;
		} else if (far_ps_min == PA24_PS_OFFSET_MAX) {
			data->ps_thrd_high = PA24_PS_OFFSET_MAX;
			data->ps_thrd_low = PA24_PS_OFFSET_MAX - 1;
		}
		msleep(saturation_delay);
		SENSOR_LOG_INFO("Sun light!!, ht=%d, lt=%d, far_ps_min=%d\n", data->ps_thrd_high, data->ps_thrd_low, far_ps_min);
		data->ps_status = PA24_PS_FAR_DISTANCE;
		goto check_intr_exit;
	}
	//FARTHER AWAY
	if (psdata < data->ps_thrd_low && data->ps_status == PA24_PS_FAR_DISTANCE) {

		pa22_get_ps_slope_array(ps_seq_far, slope, psdata, ps_ary_size);
		TXC_SUM(ps_seq_far, ps_sum);
		TXC_SUM(slope, sum);
		TXC_ABS_SUM(slope, abs_sum);
		SENSOR_LOG_INFO("slope : %d %d %d\n", slope[2], slope[1], slope[0]);
		SENSOR_LOG_INFO("value : %d %d %d %d\n", ps_seq_far[3], ps_seq_far[2], ps_seq_far[1], ps_seq_far[0]);
		SENSOR_LOG_INFO("saturation_flag=%d\n", saturation_flag);
		//If saturation happened, the average ps value must be greater than (far_ps_min-10) and also  steady
		if ( (saturation_flag && ps_sum/ps_ary_size >= ( far_ps_min > 10 ? (far_ps_min - 10) : far_ps_min ))
			  || !saturation_flag || (saturation_flag && far_ps_min == PA24_PS_OFFSET_MAX) )
		{
			//STEADY
			if (abs_sum < ps_steady) {
				if (saturation_flag)
					saturation_flag = 0;

				data->ps_status = PA24_PS_FAR_DISTANCE;
				oil_occurred = 0;
				far_ps_min = ps_sum / ps_ary_size;
				data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
				data->ps_thrd_low = far_ps_min > 15 ? (far_ps_min - 5) : 15;
				i2c_write_reg(client, REG_CFG3, (PA24_INT_TYPE << 6) | (PA24_PS_PERIOD << 3));
				SENSOR_LOG_INFO("FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, data->ps_thrd_high, data->ps_thrd_low);
				pa22_report_event(data);
			}
		}
		msleep(sequence_dealy);
	}
	//NEAR 
	else if (psdata > data->ps_thrd_high)
	{
		int i = 0;
		for (i = 0; i < ps_ary_size; i++) {
			res = i2c_read_reg(client, REG_PS_DATA, ps_seq_near+i);
			if (i > 0)
				slope[i-1] = (int)(ps_seq_near[i] - ps_seq_near[i-1]);
			mdelay(5);
		}
		//pa22_get_ps_slope_array(ps_seq_near, slope, psdata, ps_ary_size);
		SENSOR_LOG_ERROR("slope : %d %d %d\n", slope[2], slope[1], slope[0]);
		SENSOR_LOG_ERROR("value : %d %d %d %d\n", ps_seq_near[3], ps_seq_near[2], ps_seq_near[1], ps_seq_near[0]);
		TXC_ABS_SUM(slope, abs_sum);
		oil_occurred = 0;
		if (abs_sum < ps_steady) {
			data->ps_status = PA24_PS_NEAR_DISTANCE;
			i2c_write_reg(client,REG_CFG1,
					(PA24_LED_CURR << 4)| (1 << 2) );

			if (psdata >= 254) {
				far_loop = 0;
				oil_occurred = 1;
				data->ps_thrd_low = far_ps_min + OIL_EFFECT;
				data->ps_thrd_high = 0xFF;
			} else {
				data->ps_thrd_low = far_ps_min + (data->near_diff_cnt - data->far_diff_cnt);//(PA24_NEAR_FAR_CNT - 3));
				data->ps_thrd_high = 254;//(far_ps_min + OIL_EFFECT);
			}
			SENSOR_LOG_INFO("NER, far_ps_min:%3d psdata:%3d high low:%3d %3d\n", far_ps_min, psdata, data->ps_thrd_high, data->ps_thrd_low);
            pa22_report_event(data);
		} else if (abs_sum > 20) {
			/*Flicker light*/
			i2c_write_reg(client, REG_CFG3, (PA24_INT_TYPE << 6)| (0 << 3));
			SENSOR_LOG_ERROR("Flicker light!!!!");
		}
	}
	//FAR AWAY
	if (psdata < data->ps_thrd_low && data->ps_status == PA24_PS_NEAR_DISTANCE)
	{
		if (oil_occurred) {
			far_loop++;
			pa22_get_ps_slope_array(ps_seq_oil, slope, psdata, ps_ary_size);
			TXC_SUM(ps_seq_oil, ps_sum);
			TXC_SUM(slope, sum);
			TXC_ABS_SUM(slope, abs_sum);
			SENSOR_LOG_INFO("slope : %d %d %d\n", slope[2], slope[1], slope[0]);
			SENSOR_LOG_INFO("value : %d %d %d %d\n", ps_seq_oil[3], ps_seq_oil[2], ps_seq_oil[1], ps_seq_oil[0]);
			//STEADY
			if (abs_sum < ps_steady || far_loop > 10) {
				i2c_write_reg(client,REG_CFG1,
					(PA24_LED_CURR << 4)| (PA24_PS_PRST << 2) );
				data->ps_status = PA24_PS_FAR_DISTANCE;
				oil_occurred = 0;
				if (far_loop <= 10) {
					far_ps_min = ps_sum / ps_ary_size;
					data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
					data->ps_thrd_low = far_ps_min > 5 ? (far_ps_min - 5) : 5;
				} else {
					SENSOR_LOG_INFO("far_loop > 10\n");
					far_ps_min = far_ps_min + 15;
					data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
					data->ps_thrd_low = far_ps_min + (data->near_diff_cnt - data->far_diff_cnt);//PA24_NEAR_FAR_CNT);
				}
				i2c_write_reg(client, REG_CFG3, (PA24_INT_TYPE << 6)| (PA24_PS_PERIOD << 3));
				SENSOR_LOG_INFO("OIL to FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, data->ps_thrd_high, data->ps_thrd_low);
                pa22_report_event(data);
            }
			msleep(sequence_dealy);
		} else {
			i2c_write_reg(client,REG_CFG1,
					(PA24_LED_CURR << 4)| (PA24_PS_PRST << 2) );
			data->ps_status = PA24_PS_FAR_DISTANCE;
			data->ps_thrd_high = far_ps_min + data->near_diff_cnt;
			data->ps_thrd_low = far_ps_min + data->near_diff_cnt - data->far_diff_cnt;//PA24_NEAR_FAR_CNT;
			SENSOR_LOG_ERROR("FAR, far_ps_min %3d high low : %3d %3d\n", far_ps_min, data->ps_thrd_high, data->ps_thrd_low);
			pa22_report_event(data);
        }

	}

check_intr_exit:

	i2c_write_reg(client, REG_PS_TL, data->ps_thrd_low);
	i2c_write_reg(client, REG_PS_TH, data->ps_thrd_high);

	/* Clear PS INT FLAG */
	res = i2c_read_reg(client, REG_CFG2, &cfgdata);
	if (res < 0) {
		SENSOR_LOG_ERROR("i2c_read function err res = %d\n",res);
		return -ERR_DEV_OPS;
	}
	cfgdata = cfgdata & 0xFD;

	res = i2c_write_reg(client,REG_CFG2, cfgdata);
	if (res < 0) {
		SENSOR_LOG_ERROR("i2c_send function err res = %d\n",res);
		return -ERR_DEV_OPS;
	}
	SENSOR_LOG_INFO("exit.\n");
	return 0;
}

static void pa22_irq_enable(struct pa22_ps_data *data, bool enable, bool flag_sync)
{
	if (enable == data->irq_enabled) {
	    SENSOR_LOG_DEBUG("doubule %s irq %d\n",enable? "enable" : "disable",
		    data->irq);
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

static void pa22_work_func_irq(struct work_struct *work)
{
	struct pa22_ps_data *data;
	SENSOR_LOG_INFO("IRQ Work INT\n");
	data = container_of((struct work_struct *)work, struct pa22_ps_data, irq_dwork);
	/* Add Oil Alg */
	wake_lock_timeout(&data->pa22_wake_lock, msecs_to_jiffies(1000));
	pa22_check_intr(data);
}

static irqreturn_t pa22_irq(int irq, void *info)
{
	struct i2c_client *client=(struct i2c_client *)info;
	struct pa22_platform_data *pdata = client->dev.platform_data;
    struct pa22_platform_device *device_node;
    struct pa22_ps_data *data;

    /*Device Initialize*/
    list_for_each_entry(device_node, &pdata->hw_device_list, node) {
        if (device_node->handle == PS) {
            data = container_of(device_node, struct pa22_ps_data, device_node);
        }
    }
    if (!IS_ERR_OR_NULL(data)) {
	    if (0 == queue_work(data->irq_work_queue, &data->irq_dwork))
	        SENSOR_LOG_ERROR("schedule_work failed!\n");
	}
	return IRQ_HANDLED;
}

/*Suspend/Resume*/
static int pa22_ps_suspend(struct pa22_platform_device *dev_node)
{
        return 0;
}

static int pa22_ps_resume(struct pa22_platform_device *dev_node)
{
        return 0;
}

static int pa22_pinctrl_init(struct pa22_ps_data *data, struct device *dev)
{
	int rc;

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		SENSOR_LOG_ERROR("data->pinctrl is NULL\n");
		return PTR_ERR(data->pinctrl);
	}

	data->pin_default = pinctrl_lookup_state(data->pinctrl, "pa22x_default");
	if (IS_ERR_OR_NULL(data->pin_default)) {
		SENSOR_LOG_ERROR("lookup default state failed\n");
		return PTR_ERR(data->pin_default);
	}

	data->pin_sleep = pinctrl_lookup_state(data->pinctrl, "pa22x_sleep");
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

static int pa22_ps_hw_init(struct pa22_platform_data *pdata)
{
	int err;
	struct i2c_client *client;
	struct pa22_ps_data *data;
	struct pa22_platform_device *pdev_node;
	if (unlikely(IS_ERR_OR_NULL(pdata))) {
        SENSOR_LOG_ERROR("null pointer exception.\n");
		return -EINVAL;
	}

	list_for_each_entry(pdev_node, &pdata->hw_device_list, node) {
	/*Device Initialize*/
		if (pdev_node->handle == PS) {
			data = container_of(pdev_node, struct pa22_ps_data, device_node);
		}
    }

	if (unlikely(IS_ERR_OR_NULL(data))) {
		SENSOR_LOG_ERROR("data is null\n");
		return -EINVAL;
	}

	client = pdata->client;

	err = pa22_pinctrl_init(data, &pdata->client->dev);
	if (err) {
		SENSOR_LOG_ERROR("pa22 ps pinctrl init failed\n");
		return -ENODEV;
	}
	/*Device Initialize*/
	err = pa22_init_client(data);
	if (err < 0) {
		SENSOR_LOG_ERROR("init pa22 failed when probe\n");
		return -ENODEV;
	}
	if (gpio_is_valid(data->irq_gpio)) {
		/* configure pa22 irq gpio */
		SENSOR_LOG_INFO("gpio value is %d \n", gpio_get_value(data->irq_gpio));
		err = gpio_request_one(data->irq_gpio,
				GPIOF_DIR_IN,
				"pa22_irq_gpio");
		if (err) {
			SENSOR_LOG_ERROR("unable to request gpio %d\n",
				data->irq_gpio);
			return -ENODEV;
		}

		data->irq = client->irq =
			gpio_to_irq(data->irq_gpio);
	} else {
		SENSOR_LOG_ERROR("irq gpio not provided\n");
	}
	return 0;
}

static void pa22_ps_hw_exit(struct pa22_platform_data *pdata)
{
    struct pa22_ps_data *data;
    struct pa22_platform_device *device_node;
	if (IS_ERR_OR_NULL(pdata)) {
        SENSOR_LOG_ERROR("null pointer exception.\n");
		return;
	}
    list_for_each_entry(device_node, &pdata->hw_device_list, node) {
        /*Device Initialize*/
        if (device_node->handle == PS) {
            data = container_of(device_node, struct pa22_ps_data, device_node);
        }
    }
	if (data && gpio_is_valid(data->irq_gpio))
		gpio_free(data->irq_gpio);
}
static int pa22_ps_parse_dt(struct device *dev,
		struct pa22_platform_data *pdata,
		struct pa22_ps_data *data) {

	struct device_node *np = dev->of_node;

	unsigned int tmp = 0;
	int rc = 0;

	/* irq gpio */
	rc = of_get_named_gpio(dev->of_node,"txc,irq-gpio", 0);
	if (rc < 0) {
		SENSOR_LOG_ERROR("Unable to read irq gpio\n");
		return rc;
	}
	data->irq_gpio = rc;
	SENSOR_LOG_INFO("irq gpio is %d\n", data->irq_gpio);

	/* ps tuning data*/
	rc = of_property_read_u32(np, "txc,ps_threshold_low", &tmp);
	data->ps_thrd_low= (!rc ? tmp : 30);
	SENSOR_LOG_INFO("ps_threshold_low is %d\n", data->ps_thrd_low);

	rc = of_property_read_u32(np, "txc,ps_threshold_high", &tmp);
	data->ps_thrd_high = (!rc ? tmp : 120);
	SENSOR_LOG_INFO("ps_threshold_low is %d\n", data->ps_thrd_high);

	return 0;
}
static int pa22_ps_input_device_init(struct pa22_ps_data *data)
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

static struct pa22_platform_ops ps_platform_ops = {
		.init = pa22_ps_hw_init,
		.exit = pa22_ps_hw_exit,
        .resume = pa22_ps_resume,
        .suspend = pa22_ps_suspend,
        .unregister = pa22_ps_unregister,
};

int pa22_ps_register(struct pa22_platform_data *pdata, struct i2c_driver *driver)
{
	int err = 0;
	struct pa22_ps_data *data;
	struct i2c_client *client = pdata->client;
	if (unlikely(IS_ERR_OR_NULL(pdata))) {
		SENSOR_LOG_ERROR("null pdata\n");
		return -EINVAL;
	}

	SENSOR_LOG_DEBUG("probe start\n");
	data = kzalloc(sizeof(struct pa22_ps_data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(data)) {
		SENSOR_LOG_ERROR("kzalloc pa22_ps_data failed\n");
		err = -ENOMEM;
		goto exit;
	}
	/*parse device tree*/
	err = pa22_ps_parse_dt(&client->dev, pdata, data);
	if (err < 0) {
		SENSOR_LOG_ERROR("sensor_parse_dt() err\n");
		goto exit_alloc_failed;
	}

	/*add client data to list*/
	data->device_node.handle = PS;
	data->device_node.name = "prox";
	data->device_node.ops = &ps_platform_ops;

	/*initialize pa22_ps_data*/
	mutex_init(&data->ps_lock);
	data->platform_data = pdata;
	data->prox_debug = false;

	proximity_class = class_create(THIS_MODULE, "proximity");
	data->proximity_dev = device_create(proximity_class, NULL, pa22_proximity_dev_t, driver ,"proximity");
	if (IS_ERR_OR_NULL(data->proximity_dev)) {
		err = -ENODEV;
		SENSOR_LOG_ERROR("device_create proximity failed\n");
		goto exit_alloc_failed;
	}

	dev_set_drvdata(data->proximity_dev, data);

	err = create_sysfs_interfaces(data->proximity_dev, attrs_ps_device, ARRAY_SIZE(attrs_ps_device));
	if (err < 0) {
		SENSOR_LOG_ERROR("create sysfs interfaces failed\n");
		goto create_proximity_dev_failed;
	}

	SENSOR_LOG_INFO("registering proximity sensor input device\n");
	err = pa22_ps_input_device_init(data);
	if (err < 0) {
	    SENSOR_LOG_ERROR("input device init fail.\n");
	    goto create_proximity_dev_failed;
	}
	wake_lock_init(&data->pa22_wake_lock, WAKE_LOCK_SUSPEND ,"pa22_wake_lock");

	if (PS_POLLING)
		INIT_DELAYED_WORK(&data->ps_dwork, pa22_work_func_proximity);
	else
		INIT_WORK(&data->irq_dwork, pa22_work_func_irq);

	data->irq_work_queue = create_singlethread_workqueue("pa22_work_queue");
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
		err = request_irq(data->irq, pa22_irq,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				PA22_DRV_NAME, (void *)client);

		if (err) {
			SENSOR_LOG_INFO("Could not get IRQ\n");
            goto exit_probe;
		}
	}

	pa22_irq_enable(data, false, true);
	SENSOR_LOG_INFO("probe ps finish.\n");
	return 0;
exit_probe:
	if(data->device_node.ops->exit)
		data->device_node.ops->exit(pdata);
	list_del(&data->device_node.node);
exit_remove_sysfs_interfaces:
	wake_lock_destroy(&data->pa22_wake_lock);
	mutex_destroy(&data->ps_lock);
	remove_sysfs_interfaces(data->proximity_dev, attrs_ps_device, ARRAY_SIZE(attrs_ps_device));
create_proximity_dev_failed:
	data->proximity_dev = NULL;
	device_destroy(proximity_class, pa22_proximity_dev_t);
	class_destroy(proximity_class);
exit_alloc_failed:
	kfree(data);
exit:
	return err;
}

static int pa22_ps_unregister(struct pa22_platform_device *pdev_node)
{
	struct pa22_ps_data *data;
	struct device *dev;
	struct pa22_platform_data *pdata;
	struct i2c_client *client;
	if (IS_ERR_OR_NULL(pdev_node))
		return -EINVAL;

	data = container_of(pdev_node, struct pa22_ps_data, device_node);
	dev = data->proximity_dev;
	pdata = data->platform_data;
	client = pdata->client;

	if (data->ps_enable)
		pa22_enable_ps(data, 0);
	list_del(&data->device_node.node);
	input_unregister_device(data->ps_input_dev);
	input_free_device(data->ps_input_dev);
	remove_sysfs_interfaces(dev, attrs_ps_device, ARRAY_SIZE(attrs_ps_device));
	wake_lock_destroy(&data->pa22_wake_lock);

	if (!PS_POLLING)
		free_irq(data->irq, client);

	if (data->device_node.ops->exit)
		data->device_node.ops->exit(pdata);

	mutex_destroy(&data->ps_lock);
	kfree(data);
	return 0;
}
