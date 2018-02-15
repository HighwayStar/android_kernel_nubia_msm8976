/*
 * This file is part of the PA22A sensor driver.
 * PA22A is combined proximity, and VCSEL.
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
#include "apds9922_als.h"

/* range */
static dev_t apds9922_als_dev_t;
static struct class *als_class;

static int apds9922_als_set_enable(struct apds9922_als_data *data, int enable)
{
	int err = 0;
	u8 regdata = 0;
	struct i2c_client *client;
	if (unlikely(IS_ERR_OR_NULL(data))) {
		SENSOR_LOG_ERROR("null pointer.\n");
	return -PTR_ERR(data);
	}

	client= data->platform_data->client;
	mutex_lock(&data->platform_data->reg_lock);
	err = apds9922_i2c_read_reg(client, APDS9921_DD_MAIN_CTRL_ADDR, &regdata);
	if (err < 0) {
		SENSOR_LOG_ERROR("i2c read failed\n");
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}

	if(false == enable){
		regdata &= ~(APDS9922_ALS_ACTIVE);
	}else
	{
		regdata |= APDS9922_ALS_ACTIVE;
	}

	err = apds9922_i2c_write_reg(client, APDS9921_DD_MAIN_CTRL_ADDR, regdata );
	if (err < 0) {
		SENSOR_LOG_ERROR("i2c read failed\n");
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}
	mutex_unlock(&data->platform_data->reg_lock);

	return err;

}

static int apds9922_als_init_client(struct apds9922_als_data *data)
{
	int err;
	u8 int_cfg = 0;
	u8 main_cfg =0;
	struct apds9922_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;

        /* Dealy time setting */

	apds9922_als_cal_init(data);
        /* Initialize Sensor */
	mutex_lock(&data->platform_data->reg_lock);
	err = apds9922_i2c_read_reg(client, APDS9921_DD_MAIN_CTRL_ADDR,&main_cfg);
	if (err < 0) {
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}
	err = apds9922_i2c_write_reg(client, APDS9921_DD_MAIN_CTRL_ADDR, main_cfg&~APDS9922_ALS_ACTIVE);
	if (err < 0) {
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}
	mutex_unlock(&data->platform_data->reg_lock);

	/* ALS_MEAS_RATE */
	err = apds9922_i2c_write_reg(client,APDS9921_DD_ALS_MEAS_RATE_ADDR, APDS9921_DD_ALS_DEFAULT_RES|APDS9921_DD_ALS_DEFAULT_MEAS_RATE);
	if (err < 0)
		return err;
	data->als_res_index = APDS9921_DD_ALS_RES_20BIT;	// 100ms conversion time

	/* ALS_GAIN */
	err = apds9922_i2c_write_reg(client, APDS9921_DD_ALS_GAIN_ADDR,APDS9921_DD_ALS_DEFAULT_GAIN);
	if (err < 0)
		return err;
	data->als_gain_index = APDS9921_DD_ALS_GAIN_18X;	// 3x GAIN

	/* INT_PERSISTENCE */ //disable als int
	err = apds9922_i2c_write_reg(client, APDS9921_DD_INT_PERSISTENCE_ADDR,0);
	if (err < 0)
		return err;

	mutex_lock(&data->platform_data->reg_lock);
	err = apds9922_i2c_read_reg(client, APDS9921_DD_INT_CFG_ADDR,&int_cfg);
	if (err < 0) {
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}

	err = apds9922_i2c_write_reg(client, APDS9921_DD_INT_CFG_ADDR,(int_cfg&0x13));
	if (err < 0) {
		mutex_unlock(&data->platform_data->reg_lock);
		return err;
	}
	mutex_unlock(&data->platform_data->reg_lock);

	err = apds9922_als_set_enable(data, 1);
	if (err < 0) {
		return err;
	}
	return 0;
}

static int apds9922_als_enable(struct apds9922_als_data *data, int enable)
{
	int err;
	struct apds9922_platform_data *pdata = data->platform_data;

	data->als_enable = enable;
	if (enable) {
		if (!pdata->vdd_always_on) {
			err = apds9922_sensor_platform_hw_power_on(pdata, true);
			err = apds9922_als_init_client(data);
			mutex_lock(&pdata->lock);
			pdata->enable_mask |= 1 << ALS;
			mutex_unlock(&pdata->lock);
		}
	} else {
		if (!pdata->vdd_always_on) {
			mutex_lock(&pdata->lock);
			pdata->enable_mask &= ~(1 << ALS);
			mutex_unlock(&pdata->lock);
			apds9922_sensor_platform_hw_power_on(pdata, false);
		}
	}

	if (data->als_enable)
		schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
	else
		cancel_delayed_work(&data->als_dwork);

	return 0;
}

static int LuxCalculation(struct apds9922_als_data *data,int als_data)
{
	int luxValue=0;
	luxValue = ((als_data*data->als_lux_factor))/((apds9921_als_meas_rate_tb[data->als_res_index])*apds9921_als_gain_tb[data->als_gain_index]);
	return luxValue;
}

static int apds9922_get_als_value(struct apds9922_als_data *data)
{
	u8 lsb,msb,hsb;
	int als_data = 0;
	int err = 0;
	struct apds9922_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;

	mutex_lock(&data->als_lock);
	err = apds9922_i2c_read_reg(client, APDS9921_DD_ALS_DATA_0_ADDR,&lsb);
	if (err < 0) {
		mutex_unlock(&data->als_lock);
		return -ERR_DEV_OPS;
	}
	err = apds9922_i2c_read_reg(client, APDS9921_DD_ALS_DATA_1_ADDR, &msb);
	if (err < 0) {
		mutex_unlock(&data->als_lock);
		return -ERR_DEV_OPS;
	}
	err = apds9922_i2c_read_reg(client, APDS9921_DD_ALS_DATA_2_ADDR, &hsb);
	if (err < 0) {
		mutex_unlock(&data->als_lock);
		return -ERR_DEV_OPS;
	}
	als_data = ( ((hsb&0x0f) << 16) | (msb << 8) | lsb);
	mutex_unlock(&data->als_lock);

	return als_data;
}

static void apds9922_calibrate_lux_value(struct apds9922_als_data *data,int luxValue)
{
	data->als_cal_lux = luxValue * SCALE_FACTOR(data->lux_cal_data.base.lux,data->lux_cal_data.cur.lux);
	data->als_cal_lux = data->als_cal_lux > 30000 ? 30000 :data->als_cal_lux;
	luxValue = luxValue > 30000 ? 30000 :luxValue;
	data->als_lux = luxValue;
}

static int apds9922_get_lux_value(struct apds9922_als_data *data)
{
	int als_data = 0;
	int luxValue=0;
	int change_again=0;
	int als_threshold_l;
	int als_threshold_h;
	struct apds9922_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;

	als_data = apds9922_get_als_value(data);
	luxValue = LuxCalculation(data,als_data);

	SENSOR_LOG_DEBUG("als lux=%d als_data=%d again=%d als_res=%d\n", luxValue, als_data, apds9921_als_gain_tb[data->als_gain_index], data->als_res_index);

	als_threshold_l =  (apds9921_als_res_tb[data->als_res_index] * APDS9921_ALS_THRESHOLD_HSYTERESIS ) /100;
	als_threshold_h = (apds9921_als_res_tb[data->als_res_index] * (100-APDS9921_ALS_THRESHOLD_HSYTERESIS) ) /100;

	if (als_data >= als_threshold_h) {
	 // lower AGAIN if possible
		if (data->als_gain_index != APDS9921_DD_ALS_GAIN_1X) {
			data->als_gain_index--;
			change_again = 1;
		}
	}
	else if (als_data < als_threshold_l) {
		// increase AGAIN if possible
		if (data->als_gain_index != APDS9921_DD_ALS_GAIN_18X) {
			data->als_gain_index++;
			change_again = 1;
		}
	}

	if (change_again) {
		apds9922_i2c_write_reg(client, APDS9921_DD_ALS_GAIN_ADDR,apds9921_als_gain_bit_tb[data->als_gain_index]);
		SENSOR_LOG_DEBUG("write changer gain Succese\n");
	}

	return luxValue;
}

/*
 * Initialization function
 */
static void apds9922_als_calidata_init(struct apds9922_als_data *data)
{
	data->lux_cal_data.cur.lux = 1;
	data->lux_cal_data.base.lux = 1;
	data->lux_cal_data.flag = 0;
}
static int apds9922_als_cal_init(struct apds9922_als_data *data)
{
	int err = 0;
	err = apds9922_read_file(LUX_CAL_PATH,(char *)&(data->lux_cal_data),sizeof(struct light_cal_parameter));
	if (err < 0 ) {
		SENSOR_LOG_ERROR("read factory cal parameters failed\n");
	}
	if (data->lux_cal_data.cur.lux == 0 || data->lux_cal_data.base.lux == 0 ||data->lux_cal_data.flag == 0) {
		apds9922_als_calidata_init(data);
	}
	return err;
}
/*
 * core calibration algo impletation
 */
static int apds9922_als_sample_lux(struct apds9922_als_data *data)
{
	int err = 0;
	int sum = 0, mean = 0;
	int retry_times = 5;
	int sample_times = 0;

	SENSOR_LOG_DEBUG("sample lux...\n");

	while(retry_times--) {
		err = apds9922_get_lux_value(data);
		if(err < 0)
			continue;
		sum += err;
		sample_times++;
		msleep(data->als_poll_delay);
	}
	mean = sum/sample_times;
	SENSOR_LOG_DEBUG("mean = %d\n", mean);
	return mean;

}
static int apds9922_als_calibrate_work(struct apds9922_als_data *data, const char *cal_data)
{
	int err = 0;
	int lux_value = 0;
	int als_enable = 0;

	if (unlikely(cal_data == NULL)) {
		SENSOR_LOG_ERROR("NULL\n");
		return -1;
	}

	als_enable = data->als_enable;

	err = apds9922_als_enable(data, 0);
	if (err < 0) {
		SENSOR_LOG_ERROR("enable failed.\n");
		goto als_cal_exit;
	}

	/*copy mem directly instead of parse string*/
	memcpy(&data->lux_cal_data.base, cal_data, sizeof(data->lux_cal_data.base));
	memcpy(&data->lux_cal_data.cur, cal_data, sizeof(data->lux_cal_data.cur));

	lux_value = apds9922_als_sample_lux(data);
	if ( (lux_value < ((data->lux_cal_data.base.lux * 70)/100))  ||  (lux_value >((data->lux_cal_data.base.lux *130)/100)) )
	{
		SENSOR_LOG_ERROR("get mean lux value error %d\n",err);
		goto als_cal_exit;
	}
	data->lux_cal_data.cur.lux = (lux_value > 0) ? lux_value : 1;
	data->lux_cal_data.flag = (data->lux_cal_data.base.lux > 0) ? 1 : 0;

	SENSOR_LOG_DEBUG("lux_cal_data.base.lux = %d\n", data->lux_cal_data.base.lux);
	SENSOR_LOG_DEBUG("lux_cal_data.cur.lux = %d\n", data->lux_cal_data.cur.lux);
	SENSOR_LOG_DEBUG("lux_cal_data.flag = %d\n", data->lux_cal_data.flag);

	err = apds9922_write_file(LUX_CAL_PATH,(const char *)&(data->lux_cal_data),sizeof(struct light_cal_parameter));
	if (err < 0) {
		SENSOR_LOG_ERROR("save rgb cal parameters failed\n");
		goto als_cal_exit;
	}
	apds9922_als_enable(data,als_enable);
	return lux_value;

als_cal_exit:
	apds9922_als_calidata_init(data);
	apds9922_als_enable(data,als_enable);
	return -1;
}

/* For HAL to Enable ALS */
static ssize_t apds9922_enable_als_sensor_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
        struct apds9922_als_data *data = dev_get_drvdata(dev);
        return sprintf(buf, "%d\n", data->als_enable);
}
static ssize_t apds9922_enable_als_sensor_store(struct device *dev,
                                struct device_attribute *attr, const char *buf, size_t count)
{
    struct apds9922_als_data *data = dev_get_drvdata(dev);
    unsigned long val = simple_strtoul(buf, NULL, 10);

     SENSOR_LOG_DEBUG("enable als sensor ( %ld)\n", val);

    if ((val != 0) && (val != 1)) {
          SENSOR_LOG_INFO("enable als sensor=%ld\n", val);
          return count;
     }

     apds9922_als_enable(data, val);

    return count;
}

static ssize_t apds9922_als_poll_delay_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct apds9922_als_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->als_poll_delay);
}

static ssize_t apds9922_als_poll_delay_store(struct device *dev,
                                        struct device_attribute *attr, const char *buf, size_t count)
{
	u8 als_meas_rate;
	struct apds9922_als_data *data = dev_get_drvdata(dev);
	struct apds9922_platform_data *pdata = data->platform_data;
	struct i2c_client *client = pdata->client;
	unsigned long msec = simple_strtoul(buf, NULL, 10);

	SENSOR_LOG_INFO("als_poll_delay=%ld\n", msec);

	if (msec < 0) return -1;

	if (msec > 1000) {
		data->als_poll_delay = 1000;
		als_meas_rate = APDS9921_DD_ALS_MEAS_RES_20_BIT|APDS9921_DD_ALS_MEAS_RATE_1000_MS;
		data->als_res_index = APDS9921_DD_ALS_RES_20BIT;
	} else if (msec > 500) {
		data->als_poll_delay = 500;
		als_meas_rate = APDS9921_DD_ALS_MEAS_RES_20_BIT|APDS9921_DD_ALS_MEAS_RATE_500_MS;
		data->als_res_index = APDS9921_DD_ALS_RES_20BIT;
	} else if (msec > 200) {
		data->als_poll_delay = 200;
		als_meas_rate = APDS9921_DD_ALS_MEAS_RES_19_BIT|APDS9921_DD_ALS_MEAS_RATE_200_MS;
		data->als_res_index = APDS9921_DD_ALS_RES_19BIT;
	} else if (msec > 100) {
		data->als_poll_delay = 100;
		als_meas_rate = APDS9921_DD_ALS_MEAS_RES_18_BIT|APDS9921_DD_ALS_MEAS_RATE_100_MS;
		data->als_res_index = APDS9921_DD_ALS_RES_18BIT;
	} else if (msec > 50) {
		data->als_poll_delay = 50;
		als_meas_rate = APDS9921_DD_ALS_MEAS_RES_17_BIT|APDS9921_DD_ALS_MEAS_RATE_50_MS;
		data->als_res_index = APDS9921_DD_ALS_RES_17BIT;
	}	else {
		data->als_poll_delay = 25;
		als_meas_rate = APDS9921_DD_ALS_MEAS_RES_16_BIT|APDS9921_DD_ALS_MEAS_RATE_25_MS;
		data->als_res_index = APDS9921_DD_ALS_RES_16BIT;
	}

	apds9922_i2c_write_reg(client,APDS9921_DD_ALS_MEAS_RATE_ADDR,als_meas_rate);
    return count;
}

static ssize_t apds9922_als_lux_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct apds9922_als_data *data = dev_get_drvdata(dev);
	int luxValue = 0;

	if(true == data->als_enable)
	{
		msleep(data->als_poll_delay);
		if(true == data->lux_cal_data.flag)
			return sprintf(buf, "%d\n", data->als_cal_lux);
		else
			return sprintf(buf, "%d\n", data->als_lux);
	}
	else
	{
		msleep(data->als_poll_delay);
		luxValue = apds9922_get_lux_value(data);
		apds9922_calibrate_lux_value(data,luxValue);

		if(1 == data->lux_cal_data.flag)
			return sprintf(buf, "%d\n", data->als_cal_lux+1);
		 else
			return sprintf(buf, "%d\n", data->als_lux+1);
	}
	return 0;
}

static ssize_t apds9922_als_flush_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
	struct apds9922_als_data *data = dev_get_drvdata(dev);

	if (!data->lux_cal_data.flag)
		input_report_rel(data->als_input_dev, REL_X, data->als_lux + 1);
	else
		input_report_rel(data->als_input_dev, REL_X, data->als_cal_lux + 1);
		input_sync(data->als_input_dev);

	return 0;
}

static ssize_t apds9922_als_chip_id_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", APDS9922_CHIP_NAME);
}

/*calibrate*/
static ssize_t apds9922_als_calibrate_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	struct apds9922_als_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->lux_cal_data.flag);
}

static ssize_t apds9922_als_calibrate_store(struct device *dev,
										struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	int val = 0;
	struct apds9922_als_data *data = dev_get_drvdata(dev);

	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint failed\n");
		return err;
	}
	err = apds9922_als_calibrate_work(data, (const char *)&val);
	if (err < 0) {
		SENSOR_LOG_ERROR("als calibrate fail.\n");
	}

	return count;
}

static ssize_t apds9922_als_factory_calibrate_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	struct apds9922_als_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", data->lux_cal_data.flag);

}
static ssize_t apds9922_als_factory_calibrate_store(struct device *dev,
                                        struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	struct apds9922_als_data *data = dev_get_drvdata(dev);

	err = apds9922_als_calibrate_work(data, buf);
	if (err < 0) {
		SENSOR_LOG_ERROR("als calibrate fail.\n");
	}

	return count;
}

static ssize_t apds9922_als_debug_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	struct apds9922_als_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "cal_lux=%d\n", data->als_cal_lux);
}
static ssize_t apds9922_als_debug_store(struct device *dev,
										struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	int val = 0;
	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint failed\n");
		return err;
	}
	SENSOR_LOG_INFO("val = %d\n", val);
	if (val)
		sensor_debug= true;
	else
		sensor_debug = false;

	return count;
}

static ssize_t apds9922_als_dev_init_show(struct device *dev,
										struct device_attribute *attr, char *buf)
{
	struct apds9922_als_data *data = dev_get_drvdata(dev);
	return sprintf(buf, "flag=%d-base=%d-cur=%d\n", data->lux_cal_data.flag,data->lux_cal_data.base.lux, data->lux_cal_data.cur.lux);
}
static ssize_t apds9922_als_dev_init_store(struct device *dev,
										struct device_attribute *attr, const char *buf, size_t count)
{
	int err = 0;
	struct apds9922_als_data *data = dev_get_drvdata(dev);
	err = apds9922_als_cal_init(data);
	if (err > 0) {
		SENSOR_LOG_INFO("als dev init ok.\n");
	}
	return count;
}


/* Write/Read Register data */
static ssize_t apds9922_show_als_reg(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct apds9922_als_data *data = dev_get_drvdata(dev);
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


static ssize_t apds9922_store_als_reg(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct apds9922_als_data *data = dev_get_drvdata(dev);
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


static struct device_attribute attrs_als_device[] = {
	__ATTR(enable, 0664, apds9922_enable_als_sensor_show, apds9922_enable_als_sensor_store),
	__ATTR(delay, 0664, apds9922_als_poll_delay_show, apds9922_als_poll_delay_store),
	__ATTR(chip_name, 0444, apds9922_als_chip_id_show, NULL),
	__ATTR(dev_init, 0664, apds9922_als_dev_init_show, apds9922_als_dev_init_store),
	__ATTR(light_value, 0444, apds9922_als_lux_show, NULL),
	__ATTR(flush, 0444, apds9922_als_flush_show, NULL),
	__ATTR(calibrate, 0664, apds9922_als_calibrate_show, apds9922_als_calibrate_store),
	__ATTR(fac_calibrate, 0664, apds9922_als_factory_calibrate_show, apds9922_als_factory_calibrate_store),
	__ATTR(debug, 0664, apds9922_als_debug_show, apds9922_als_debug_store),
	__ATTR(reg, 0664, apds9922_show_als_reg, apds9922_store_als_reg),
};
/*work que function*/
static void apds9922_work_func_light(struct work_struct *work)
{
	struct apds9922_als_data *data = container_of(work, struct apds9922_als_data, als_dwork.work);
	int luxValue = 0;

	luxValue = apds9922_get_lux_value(data);
	/*ensure to avoid invalid lux*/
	apds9922_calibrate_lux_value(data,luxValue);

	SENSOR_LOG_DEBUG("als_lux =%d : als_cal_lux = %d\n", data->als_lux,data->als_cal_lux);

	if (!data->lux_cal_data.flag)
		input_report_rel(data->als_input_dev, REL_X, data->als_lux + 1);
	else
		input_report_rel(data->als_input_dev, REL_X, data->als_cal_lux + 1);
	input_sync(data->als_input_dev);

	if(APDS9922_ALS_POLLING && data->als_enable)
		schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));

}

/*Suspend/Resume*/
static int apds9922_als_suspend(struct apds9922_platform_device *dev_node)
{
	struct apds9922_als_data *data;
	data = container_of(dev_node, struct apds9922_als_data, device_node);
	if (data && data->als_enable) {
	SENSOR_LOG_INFO("enter\n");
		cancel_delayed_work(&data->als_dwork);
	}
	SENSOR_LOG_INFO("exit\n");
	return 0;
}

static int apds9922_als_resume(struct apds9922_platform_device *dev_node)
{
	struct apds9922_als_data *data;
	data = container_of(dev_node, struct apds9922_als_data, device_node);
	if (data && data->als_enable) {
		SENSOR_LOG_INFO("enter\n");
		schedule_delayed_work(&data->als_dwork, msecs_to_jiffies(data->als_poll_delay));
	}
	SENSOR_LOG_INFO("exit\n");
	return 0;
}
static int apds9922_als_hw_init(struct apds9922_platform_data *pdata)
{
	struct apds9922_platform_device *pdev_node;
	struct apds9922_als_data *data;
	int err = 0;

	if (unlikely(IS_ERR_OR_NULL(pdata))) {
		SENSOR_LOG_ERROR("null pointer exception\n");
		return -ENODEV;
	}

	list_for_each_entry(pdev_node, &pdata->hw_device_list, node) {
          /*Device Initialize*/
		if (pdev_node->handle == ALS) {
			data = container_of(pdev_node, struct apds9922_als_data, device_node);
			if(data)
			{
				err = apds9922_als_init_client(data);
			}
			return err;
		}
	}

	return 0;
}

static void apds9922_als_hw_exit(struct apds9922_platform_data *pdata)
{
     if (unlikely(IS_ERR_OR_NULL(pdata))) {
         SENSOR_LOG_ERROR("null pointer exception\n");
	  return;
    }
}
static int apds9922_als_input_device_init(struct apds9922_als_data *data)
{
	int err = 0;
	if (unlikely(IS_ERR_OR_NULL(data))) {
	SENSOR_LOG_ERROR("null pointer exception\n");
		return -ENODEV;
	}

		/* allocate light input_device */
	data->als_input_dev = input_allocate_device();
	if (IS_ERR_OR_NULL(data->als_input_dev)) {
		err = -ENOMEM;
		SENSOR_LOG_ERROR("could not allocate input device\n");
		goto exit;
	}

	data->als_input_dev->name = DEV_NAME_ALS;
	set_bit(EV_REL, data->als_input_dev->evbit);
	set_bit(REL_X,  data->als_input_dev->relbit);

		//ALS INPUT regist
	err = input_register_device(data->als_input_dev);
	if (err) {
		err = -ENOMEM;
		SENSOR_LOG_ERROR("Unable to register input device als: %s\n",data->als_input_dev->name);
		goto input_register_err;
	}

	return err;

input_register_err:
	input_free_device(data->als_input_dev);
exit:
    return err;
}

static struct apds9922_platform_ops als_platform_ops = {
	.init = apds9922_als_hw_init,
	.exit = apds9922_als_hw_exit,
       .resume = apds9922_als_resume,
       .suspend = apds9922_als_suspend,
       .unregister = apds9922_als_unregister,
};

/*
 * I2C init/probing/exit functions
 */
static int als_parse_dt(struct device *dev, struct apds9922_als_data *pdata)
{
	struct device_node *np = dev->of_node;
	unsigned int tmp;
	int rc = 0;
	/* ALS tuning value */
	rc = of_property_read_u32(np, "avago,als-lux-factor", &tmp);
	if (rc) {
		pr_err("Unable to read als lux factor\n");
		return rc;
	}
	pdata->als_lux_factor = tmp;
	SENSOR_LOG_INFO("%s als_lux_factor = %d\n", __func__, pdata->als_lux_factor);

	return 0;
}



int apds9922_als_register(struct apds9922_platform_data *pdata, struct i2c_driver *driver)
{
	int err = 0;
	u8 tmp = 0;
	struct apds9922_als_data *data;
	if (unlikely(IS_ERR_OR_NULL(pdata))) {
		SENSOR_LOG_ERROR("pdata is null.\n");
	return -EINVAL;
	}
	SENSOR_LOG_INFO("start als probe\n");

	err = apds9922_i2c_read_reg(pdata->client, APDS9921_DD_ALS_DATA_ADDR, &tmp);
	if (err < 0) {
		SENSOR_LOG_ERROR("do not support als\n");
		goto exit;
	}

	data = kzalloc(sizeof(struct apds9922_als_data), GFP_KERNEL);
	if (IS_ERR_OR_NULL(data)) {
		SENSOR_LOG_ERROR("Failed to allocate memory\n");
		err = -ENOMEM;
		goto exit;
	}

	if(pdata->client->dev.of_node)
	{
		err = als_parse_dt(&pdata->client->dev,data);
		if (err < 0) {
			SENSOR_LOG_ERROR("do not support als\n");
		goto exit_parse_failed;
		}
	}
	data->als_enable = false;
	data->als_poll_delay = APDS9922_ALS_POLL_DELAY;
	data ->als_cal_lux = 0;
	data->als_lux = 0;

	 /* add client data to list */
	data->device_node.handle = ALS;
	data->device_node.name = "als";
	data->device_node.ops = &als_platform_ops;

	 /* initialize apds9922_als_data */
	data->platform_data = pdata;
	INIT_DELAYED_WORK(&data->als_dwork, apds9922_work_func_light);

	als_class = class_create(THIS_MODULE, "light");
	alloc_chrdev_region(&apds9922_als_dev_t, 0, 1, DEV_NAME_ALS);
	data->als_dev = device_create(als_class, NULL, apds9922_als_dev_t, driver, DEV_NAME_ALS);
	if (IS_ERR(data->als_dev)) {
		err = -EINVAL;
		SENSOR_LOG_ERROR("device_create als failed\n");
		goto exit_alloc_failed;
	}

	dev_set_drvdata(data->als_dev, data);
	err = apds9922_create_sysfs_interfaces(data->als_dev, attrs_als_device, ARRAY_SIZE(attrs_als_device));
	if (err < 0) {
		SENSOR_LOG_ERROR("create sysfs interfaces failed\n");
		goto create_sysfs_interface_error;
	}

	mutex_init(&data->als_lock);

	SENSOR_LOG_INFO("registering light sensor input device\n");
	err = apds9922_als_input_device_init(data);
	if (err < 0) {
		SENSOR_LOG_ERROR("input device init fail.\n");
		goto exit_probe;
	}

        /* probe finish, and add als client to list */
	list_add_tail(&data->device_node.node, &pdata->hw_device_list);
        /* h/w initialization */
	if (data) {
		err = data->device_node.ops->init(pdata);
		if (err < 0)
			goto exit_probe;
	}
	SENSOR_LOG_INFO("als probe finish\n");
	return 0;

exit_probe:
	list_del(&data->device_node.node);
	mutex_destroy(&data->als_lock);
create_sysfs_interface_error:
	apds9922_remove_sysfs_interfaces(data->als_dev, attrs_als_device, ARRAY_SIZE(attrs_als_device));
exit_alloc_failed:
	data->als_dev = NULL;
	device_destroy(als_class, apds9922_als_dev_t);
	class_destroy(als_class);
exit_parse_failed:
exit:
	return err;
}

static int apds9922_als_unregister(struct apds9922_platform_device *pdev_node)
{
	struct apds9922_als_data *data;
	if(unlikely(IS_ERR_OR_NULL(pdev_node)))
		return -EINVAL;
	data = container_of(pdev_node, struct apds9922_als_data, device_node);
	list_del(&data->device_node.node);
	input_unregister_device(data->als_input_dev);
	input_free_device(data->als_input_dev);
	apds9922_remove_sysfs_interfaces(data->als_dev, attrs_als_device, ARRAY_SIZE(attrs_als_device));
	mutex_destroy(&data->als_lock);
	if(data)
		kfree(data);
	return 0;
}
