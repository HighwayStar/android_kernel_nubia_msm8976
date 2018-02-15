/*
* This file is part of the tmd2725 sensor driver.
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
when         	who         		Remark : what, where, why          		version
-----------   	------------     	-------------------------           	--------
201x/x/xx       AMS															 V1.0
2016/7/11       Bao QI              Tmd2725 code refcatoring                 V2.0
====================================================================================
*/
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#include "ams_tmd2725.h"
#include "ams_i2c.h"
#include "ams_common.h"

#undef LOG_TAG
#define LOG_TAG "TMD2725-ALS"

static int tmd2725_configure_als_mode(struct tmd2725_chip *chip, u8 state);
static int tmd2725_get_lux(struct tmd2725_chip *chip);
static int tmd2725_read_als(struct tmd2725_chip *chip);
static void tmd2725_report_als(struct tmd2725_chip *chip);
static void tmd2725_als_parameter_init(struct tmd2725_chip *chip);
#define DEV_ALS_NAME         "light"
#define ALS_CAL_PATH		 "/persist/sensors/als_cal_data"
#define PARSE_DTSI_NUMBER    7
#define MODULE_MANUFACTURE_NUMBER 3
//#define LUX_DBG

#define GAIN1	0
#define GAIN4	1
#define GAIN16	2
#define GAIN64	3
#define SCALE_FACTOR(x, y) (x)/(y)

static dev_t tmd2725_als_dev_t;
static struct class *als_class;
static u8 const als_gains[] = {
	1,
	4,
	16,
	64
};

static u8 const restorable_als_regs[] = {
	TMD2725_REG_ATIME,
	TMD2725_REG_WTIME,
	TMD2725_REG_PERS,
	TMD2725_REG_CFG0,
	TMD2725_REG_CFG1,
};

static const char *dts_array_name[MODULE_MANUFACTURE_NUMBER] = {
	"tmd,tp0",
	"tmd,tp1",
	"tmd,tp2",
};
static struct tp_als_parameter tp_module_parameter[MODULE_MANUFACTURE_NUMBER] = {
	{.tp_module_id = 0x00},
	{.tp_module_id = 0x01},
	{.tp_module_id = 0x02}
};
static int tmd2725_flush_als_regs(struct tmd2725_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_als_regs); i++) {
		reg = restorable_als_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg, chip->shadow[reg]);
		if (rc) {
			SENSOR_LOG_ERROR("err on reg 0x%02x\n", reg);
			break;
		}
	}

	return rc;
}

static int tmd2725_read_als(struct tmd2725_chip *chip)
{
	int ret;

	ret = ams_i2c_blk_read(chip->client, TMD2725_REG_CH0DATA,
			&chip->shadow[TMD2725_REG_CH0DATA], 4 * sizeof(u8));

	if (ret >= 0) {
		chip->als_inf.als_ch0 = le16_to_cpu(
			*((const __le16 *) &chip->shadow[TMD2725_REG_CH0DATA]));
		chip->als_inf.als_ch1 = le16_to_cpu(
			*((const __le16 *) &chip->shadow[TMD2725_REG_CH1DATA]));
		ret = 0;
	}

	return ret;
}

static void tmd2725_calc_lux_coef(struct tmd2725_chip *chip)
{
	struct tmd2725_lux_segment *pls = &chip->params.lux_segment[0];

	chip->als_inf.lux1_ch0_coef = chip->params.d_factor * pls[0].ch0_coef;
	chip->als_inf.lux1_ch1_coef = chip->params.d_factor * pls[0].ch1_coef;
	chip->als_inf.lux2_ch0_coef = chip->params.d_factor * pls[1].ch0_coef;
	chip->als_inf.lux2_ch1_coef = chip->params.d_factor * pls[1].ch1_coef;
}

static void tmd2725_calc_cpl(struct tmd2725_chip *chip)
{
	u32 cpl;
	u32 sat;
	u8 atime;

	atime = chip->shadow[TMD2725_REG_ATIME];

	// max(cpl) == 64x * (2816 * 256) =  46,137,344 (0x02C00000)
	cpl = atime;
	cpl *= INTEGRATION_CYCLE;
	cpl *= als_gains[(chip->shadow[TMD2725_REG_CFG1] & TMD2725_MASK_AGAIN)];

	sat = min_t(u32, TMD2725_MAX_ALS_VALUE, (u32) atime << 10);
	sat = sat * 8 / 10;

	chip->als_inf.cpl = cpl;
	chip->als_inf.saturation = sat;
}

static int tmd2725_configure_als_mode(struct tmd2725_chip *chip, u8 state)
{
	//struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;

	if (state) // Turning on ALS
	{
		mutex_lock(&chip->als_lock);
		tmd2725_calc_lux_coef(chip);

		chip->shadow[TMD2725_REG_ATIME] = chip->params.als_time;
		tmd2725_calc_cpl(chip);

		/* set PERS.apers to 2 consecutive ALS values out of range */
		chip->shadow[TMD2725_REG_PERS] &= (~TMD2725_MASK_APERS);
		chip->shadow[TMD2725_REG_PERS] |= 0x02;
		tmd2725_flush_als_regs(chip);

		//ams_i2c_modify(client, sh, TMD2725_REG_INTENAB,
		//		TMD2725_AIEN, TMD2725_AIEN);
		if (chip->prox_enabled)
		    ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
			    TMD2725_WEN | TMD2725_AEN | TMD2725_PEN | TMD2725_PON,
			    TMD2725_AEN | TMD2725_PEN | TMD2725_PON);
		else
		    ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
			    TMD2725_AEN | TMD2725_PON,
			    TMD2725_AEN | TMD2725_PON);
		mutex_unlock(&chip->als_lock);
	}
	else  // Turning off ALS
	{
		// Disable ALS, Wait and ALS Interrupt
		mutex_lock(&chip->als_lock);
		//ams_i2c_modify(client, sh, TMD2725_REG_INTENAB,
				//TMD2725_AIEN, 0);
		if (chip->prox_enabled)
		    ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
				TMD2725_WEN |TMD2725_AEN, TMD2725_WEN);
		else
		    ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
				TMD2725_AEN, 0);
		// If nothing else is enabled set PON = 0;
		if (!(sh[TMD2725_REG_ENABLE] & TMD2725_EN_ALL))
			ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
			TMD2725_PON, 0);
		mutex_unlock(&chip->als_lock);
	}

	return 0;
}
static int tmd2725_als_set_enable(struct tmd2725_chip *chip, u8 enable, bool enable_poll)
{
	int ret;
	if (enable) {
		if (!chip->pdata->power_always_on) {
			if (chip->pdata->power_state == POWER_OFF)
					sensor_regulator_power_on(chip, true);
			chip->pdata->power_state |= POWER_ALS_ON;
		}
		ret = tmd2725_configure_als_mode(chip, 1);
		if (ret < 0) {
			SENSOR_LOG_ERROR("als turn %s failed\n", enable?"on":"off");
			return ret;
		}
		if (enable_poll) {
			mutex_lock(&chip->als_lock);
			chip->als_enabled = 1;
			mutex_unlock(&chip->als_lock);
			schedule_delayed_work(&chip->als_work, msecs_to_jiffies(200));
		}
	} else {
		if (enable_poll) {
			cancel_delayed_work_sync(&chip->als_work);
			mutex_lock(&chip->als_lock);
			chip->als_enabled = 0;
			mutex_unlock(&chip->als_lock);
		}
		ret = tmd2725_configure_als_mode(chip, 0);
		if (ret < 0) {
			SENSOR_LOG_ERROR("als turn %s failed\n", enable?"on":"off");
			return ret;
		}

		if (!chip->pdata->power_always_on) {
			chip->pdata->power_state &= ~POWER_ALS_ON;
			if (!chip->prox_enabled && !chip->als_enabled && chip->pdata->power_state == POWER_OFF)
				sensor_regulator_power_on(chip, false);
		}
	}
	SENSOR_LOG_INFO("als turn %s\n", enable?"on":"off");
	return 0;
}
static int tmd2725_set_als_gain(struct tmd2725_chip *chip, int gain)
{
	int rc;
	u8 ctrl_reg;

	switch (gain) {
	case 1:
		ctrl_reg = AGAIN_1;
		break;
	case 4:
		ctrl_reg = AGAIN_4;
		break;
	case 16:
		ctrl_reg = AGAIN_16;
		break;
	case 64:
		ctrl_reg = AGAIN_64;
		break;
	default:
		SENSOR_LOG_ERROR("wrong als gain %d\n", gain);
		return -EINVAL;
	}

	// Turn off ALS, so that new ALS gain value will take effect at start of
	// new integration cycle.
	// New ALS gain value will then be used in next lux calculation.
	// ams_i2c_read(chip->client, TMD2725_REG_ENABLE, &saved_enable);
	// ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_ENABLE, 0);
	rc = ams_i2c_modify(chip, chip->shadow, TMD2725_REG_CFG1,
			TMD2725_MASK_AGAIN, ctrl_reg);
	//ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_ENABLE, saved_enable);

	if (rc >= 0) {
		chip->params.als_gain = chip->shadow[TMD2725_REG_CFG1];
		SENSOR_LOG_INFO("new als gain %d\n", ctrl_reg);
	}

	return rc;
}

static void tmd2725_inc_gain(struct tmd2725_chip *chip)
{
	int rc;
	u8 gain = (chip->shadow[TMD2725_REG_CFG1] & TMD2725_MASK_AGAIN);

	if (gain > GAIN16)
		return;
	else if (gain < GAIN4)
		gain = als_gains[GAIN4];
	else if (gain < GAIN16)
		gain = als_gains[GAIN16];
	else
		gain = als_gains[GAIN64];

	rc = tmd2725_set_als_gain(chip, gain);
	if (rc == 0)
		tmd2725_calc_cpl(chip);
	//tmd2725_flush_als_regs(chip);
}

static void tmd2725_dec_gain(struct tmd2725_chip *chip)
{
	int rc;
	u8 gain = (chip->shadow[TMD2725_REG_CFG1] & 0x03);

	if (gain == GAIN1)
		return;
	else if (gain > GAIN16)
		gain = als_gains[GAIN16];
	else if (gain > GAIN4)
		gain = als_gains[GAIN4];
	else
		gain = als_gains[GAIN1];

	rc = tmd2725_set_als_gain(chip, gain);
	if (rc == 0)
		tmd2725_calc_cpl(chip);
	//tmd2725_flush_als_regs(chip);
}

static int tmd2725_get_lux(struct tmd2725_chip *chip)
{
	int ret;
	int64_t ch0;
	int64_t ch1;
	int64_t lux1_raw = 0;
	int64_t lux2_raw = 0;
	int64_t lux1;
	int64_t lux2;
	int lux;
	ret = tmd2725_read_als(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("read als fail\n");
		return -EINVAL;
	}
	ch0 = (int64_t) chip->als_inf.als_ch0;
	ch1 = (int64_t) chip->als_inf.als_ch1;
	/*
	**  Lux1 = 41*(Ch0-(0.26*Ch1)) / (Atime*Again)
	**  Lux2 = 41*((0.8*Ch0)-(0.27*Ch1)) / (Atime*Again)
	**  Lux = Max(Lux1,Lux2,0)
	**
	**  Lux1 = (((coef1*(ch0_coef1*1000))*Ch0)-((coef1*(ch1_coef1*1000))*Ch1)) / (Atime*1000)*Again)
	**  Lux2 = (((coef2*(ch0_coef2*1000))*Ch0)-((coef2*(ch1_coef2*1000))*Ch1)) / (Atime*1000)*Again)
	**
	**  where for default NO GLASS:
	**     coef1 = 41
	**     ch0_coef1 = 1
	**     ch1_coef1 = .26
	**     coef2 = 41
	**     ch0_coef2 = .8
	**     ch1_coef2 = .27
	*/
	lux1_raw = ((int64_t)(chip->als_inf.lux1_ch0_coef * ch0) -
		        (int64_t)(chip->als_inf.lux1_ch1_coef * ch1));
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level,"lux1_raw = %lld\n",lux1_raw);

	lux2_raw = ((int64_t)(chip->als_inf.lux2_ch0_coef * ch0) -
		        (int64_t)(chip->als_inf.lux2_ch1_coef * ch1));
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level,"lux2_raw = %lld\n",lux2_raw);

	if (lux1_raw < 0 && lux2_raw < 0) {
		SENSOR_LOG_INFO("lux1_raw&&lux2_raw < 0 use prev.\n");
		return chip->als_inf.prev_lux; // use previous value
	}

	lux1_raw = (lux1_raw > 0) ? lux1_raw : 0;
	lux2_raw = (lux2_raw > 0) ? lux2_raw : 0;
	lux1 = lux1_raw / chip->als_inf.cpl;
	lux2 = lux2_raw / chip->als_inf.cpl;
	lux = max(lux1, lux2);
	lux = min(TMD2725_MAX_LUX, max(0, lux));
	if (chip->pdata->debug_level) {
		SENSOR_LOG_INFO("lux:%d [ch0:%lld, ch1:%lld, lux1:%lld, lux2:%lld]\n",
			lux, ch0, ch1, lux1, lux2);
		SENSOR_LOG_INFO("d_factor:%lld seg[0][ch0_coef:%lld ch1_coef:%lld] seg[1][ch0_coef:%lld ch1_coef:%lld]\n",
			chip->params.d_factor,
			chip->params.lux_segment[0].ch0_coef,
			chip->params.lux_segment[0].ch1_coef,
			chip->params.lux_segment[1].ch0_coef,
			chip->params.lux_segment[1].ch1_coef);
		SENSOR_LOG_INFO("cpl:%d [ATIME:%d, ((gain:%d*CYCLE:%d) / 1000)] (sat:%d)\n",
			chip->als_inf.cpl,
			chip->shadow[TMD2725_REG_ATIME],
			als_gains[(chip->params.als_gain & TMD2725_MASK_AGAIN)],
			INTEGRATION_CYCLE,
			chip->als_inf.saturation);
	}

	if (lux < 0) {
		SENSOR_LOG_INFO("lux < 0 use prev.\n");
		return chip->als_inf.prev_lux; // use previous value
	}

	chip->als_inf.lux = lux;
	chip->als_inf.prev_lux = lux;

	if (!chip->als_gain_auto) {
		if (ch0 <= TMD2725_MIN_ALS_VALUE) {
			SENSOR_LOG_INFO("darkness (%lld <= %d)\n",
				ch0, TMD2725_MIN_ALS_VALUE);
		} else if (ch0 >= chip->als_inf.saturation) {
			SENSOR_LOG_INFO("saturation (%lld >= %d\n",
				ch0, chip->als_inf.saturation);
		}
	} else {

		if (ch0 < 100) {
			SENSOR_LOG_INFO("AUTOGAIN INC\n");
			tmd2725_inc_gain(chip);
		} else if (ch0 >= chip->als_inf.saturation || ch1 >= chip->als_inf.saturation) {
			SENSOR_LOG_INFO("AUTOGAIN DEC\n");
			tmd2725_dec_gain(chip);
		}
	}

	return 0;
}

static int tmd2725_update_als(struct tmd2725_chip *chip)
{
	int rc;
	rc = tmd2725_get_lux(chip);
	if (rc < 0) {
		return -EINVAL;
	}
	if (chip->als_cal_data.flag) {
			chip->als_inf.cal_lux = chip->als_inf.prev_lux  * SCALE_FACTOR(chip->als_cal_data.base.lux,
			chip->als_cal_data.cur.lux);
			if (chip->pdata->debug_level)
				SENSOR_LOG_INFO("cal_lux:%d lux:%d [base:cur]=[%d:%d]\n",
					chip->als_inf.cal_lux + 1,
					chip->als_inf.prev_lux + 1,
					chip->als_cal_data.base.lux,
					chip->als_cal_data.cur.lux);
	}
	return 0;
}
static void tmd2725_report_als(struct tmd2725_chip *chip)
{
	if (chip->als_cal_data.flag) {
		SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "cal_lux:%d\n", chip->als_inf.cal_lux + 1);
		input_report_rel(chip->a_idev, REL_X, chip->als_inf.cal_lux + 1);
	} else {
		SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "lux:%d\n", chip->als_inf.prev_lux + 1);
		input_report_rel(chip->a_idev, REL_X, chip->als_inf.prev_lux + 1);
	}
	input_sync(chip->a_idev);
}

static void tmd2725_als_report_work(struct work_struct *work)
{
	struct tmd2725_chip *chip = container_of((struct delayed_work *)work, struct tmd2725_chip, als_work);
	tmd2725_update_als(chip);
	tmd2725_report_als(chip);
	schedule_delayed_work(&chip->als_work, msecs_to_jiffies(chip->als_poll_delay));
}
static void tmd2725_config_tp_parameter(struct tmd2725_chip *chip)
{
	int i = 0;
	for (i = 0;i < MODULE_MANUFACTURE_NUMBER; i++){
		if (chip->params.panel_id == tp_module_parameter[i].tp_module_id){
			chip->params.d_factor= tp_module_parameter[i].tp_als_param.d_factor;
			chip->params.which_seg= tp_module_parameter[i].tp_als_param.which_seg;
			chip->params.lux_segment[0].ch0_coef =
				tp_module_parameter[i].tp_als_param.seg0_ch0_coef;
			chip->params.lux_segment[0].ch1_coef =
				tp_module_parameter[i].tp_als_param.seg0_ch1_coef;
			chip->params.lux_segment[1].ch0_coef =
				tp_module_parameter[i].tp_als_param.seg1_ch0_coef;
			chip->params.lux_segment[1].ch1_coef =
				tp_module_parameter[i].tp_als_param.seg1_ch1_coef;
		}
	}
}
static int tmd2725_als_calibrate_work(struct tmd2725_chip *chip, const char *cal_data)
{
	int err = 0;

	if (unlikely(IS_ERR_OR_NULL(cal_data))) {
		SENSOR_LOG_ERROR("NULL\n");
		return -1;
	}

	/*copy mem directly instead of parse string*/
	memcpy(&chip->als_cal_data.base, cal_data, sizeof(chip->als_cal_data.base));
	memcpy(&chip->als_cal_data.cur, cal_data, sizeof(chip->als_cal_data.cur));

	if (!chip->als_enabled) {
		err = tmd2725_als_set_enable(chip, 1, false);
		if (err < 0) {
			SENSOR_LOG_ERROR("enable failed.\n");
			goto als_cal_exit;
		}
	}
	msleep(2 * chip->als_poll_delay);
	err = tmd2725_get_lux(chip);
	if (err < 0) {
		SENSOR_LOG_ERROR("get mean lux value error\n");
		goto als_cal_exit;
	}
	chip->als_cal_data.cur.lux = chip->als_inf.prev_lux;
	chip->als_cal_data.cur.lux = (chip->als_cal_data.cur.lux > 0) ? chip->als_cal_data.cur.lux : 1;

	SENSOR_LOG_INFO("als_cal_data.base.lux = %d\n", chip->als_cal_data.base.lux);
	SENSOR_LOG_INFO("als_cal_data.cur.lux = %d\n", chip->als_cal_data.cur.lux);
	if (chip->als_cal_data.base.lux / chip->als_cal_data.cur.lux > 6) {
		err = -EINVAL;
		chip->als_cal_data.flag = 0;
		SENSOR_LOG_ERROR("calibrate param invalid; data->rgb_cal_data.flag=%d\n",
				chip->als_cal_data.flag);
		goto als_cal_exit;
	}

	chip->als_cal_data.flag = (chip->als_cal_data.base.lux > 0) ? 1 : 0;

	if (chip->als_cal_data.flag) {
		mutex_lock(&chip->als_lock);
		chip->als_inf.cal_lux= chip->als_inf.prev_lux * SCALE_FACTOR(chip->als_cal_data.base.lux,
			chip->als_cal_data.cur.lux);
		mutex_unlock(&chip->als_lock);
	} else {
		tmd2725_als_parameter_init(chip);
	}

	SENSOR_LOG_INFO("rgb_cal_data.flag = %d\n", chip->als_cal_data.flag);

	err = sensor_write_file(ALS_CAL_PATH,
								(const char *)&(chip->als_cal_data),
								sizeof(struct tmd2725_als_cal_data));
	if (err < 0) {
		SENSOR_LOG_ERROR("save rgb cal parameters failed\n");
		goto als_cal_exit;
	}

als_cal_exit:
	err = tmd2725_als_set_enable(chip, chip->als_enabled, false);
	if (err < 0) {
		SENSOR_LOG_ERROR("disable failed.\n");
	}
	return err;

}
/*****************/
/* ABI Functions */
/*****************/
static ssize_t tmd2725_als_flush_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	int count;
	AMS_MUTEX_LOCK(&chip->als_lock);

	tmd2725_report_als(chip);
	if (chip->als_cal_data.flag) {
		count = sprintf(buf, "%d", chip->als_inf.cal_lux);
	} else {
		count = sprintf(buf, "%d",  chip->als_inf.prev_lux);
	}

	AMS_MUTEX_UNLOCK(&chip->als_lock);
	return count;
}
static ssize_t tmd2725_als_lux_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	int count, ret;

	if (!chip->als_enabled) {
		SENSOR_LOG_INFO("disabled, need enable and waiting for updating data\n");
		tmd2725_als_set_enable(chip, 1, false);
		msleep(chip->als_poll_delay);
		ret = tmd2725_update_als(chip);
		if (ret < 0)
			return ret;
		tmd2725_als_set_enable(chip, chip->als_enabled, false);
	} else {
		SENSOR_LOG_INFO("enabled and just waiting for updating data\n");
		msleep(chip->als_poll_delay);
	}
	if (chip->als_cal_data.flag) {
		count = sprintf(buf, "%d", chip->als_inf.cal_lux);
	} else {
		count = sprintf(buf, "%d",  chip->als_inf.prev_lux);
	}
	SENSOR_LOG_INFO("cal_lux = %d ,als_lux =%d\n",chip->als_inf.cal_lux, chip->als_inf.prev_lux);
	return count;
}

static ssize_t tmd2725_lux_table_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	int k;

	AMS_MUTEX_LOCK(&chip->als_lock);

	k = snprintf(buf, PAGE_SIZE, "%lld,%lld,%lld,%lld,%lld\n",
		chip->params.d_factor,
		chip->params.lux_segment[0].ch0_coef,
		chip->params.lux_segment[0].ch1_coef,
		chip->params.lux_segment[1].ch0_coef,
		chip->params.lux_segment[1].ch1_coef);

	AMS_MUTEX_UNLOCK(&chip->als_lock);

	return k;
}

static ssize_t tmd2725_lux_table_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	int64_t d_factor, ch0_coef1, ch1_coef1, ch0_coef2, ch1_coef2;

	if (5 != sscanf(buf, "%10lld,%10lld,%10lld,%10lld,%10lld", &d_factor,
			&ch0_coef1, &ch1_coef1, &ch0_coef2, &ch1_coef2));
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->als_lock);

	chip->params.d_factor = d_factor;
	chip->params.lux_segment[0].ch0_coef = ch0_coef1;
	chip->params.lux_segment[0].ch1_coef = ch1_coef1;
	chip->params.lux_segment[1].ch0_coef = ch0_coef2;
	chip->params.lux_segment[1].ch1_coef = ch1_coef2;

	tmd2725_calc_lux_coef(chip);

	AMS_MUTEX_UNLOCK(&chip->als_lock);
	return size;
}

static ssize_t tmd2725_als_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_enabled);
}

static ssize_t tmd2725_als_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		tmd2725_als_set_enable(chip, 1, true);
	else
		tmd2725_als_set_enable(chip, 0, true);

	return size;
}

static ssize_t tmd2725_auto_gain_enable_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%s\n",
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmd2725_auto_gain_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	bool value;

	if (strtobool(buf, &value))
		return -EINVAL;

	if (value)
		chip->als_gain_auto = true;
	else
		chip->als_gain_auto = false;

	return size;
}

static ssize_t tmd2725_als_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d (%s)\n",
			als_gains[(chip->params.als_gain & TMD2725_MASK_AGAIN)],
			chip->als_gain_auto ? "auto" : "manual");
}

static ssize_t tmd2725_als_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gain;
	int i = 0;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &gain);

	if (rc)
		return -EINVAL;
	if (gain != 0 && gain != 1 && gain != 4 && gain != 16 &&
			gain != 60 && gain != 64)
		return -EINVAL;

	while (i < sizeof(als_gains)) {
		if (gain == als_gains[i])
			break;
		i++;
	}

	if (i > 3) {
		SENSOR_LOG_ERROR("wrong als gain %d\n",
				(int)gain);
		return -EINVAL;
	}

	AMS_MUTEX_LOCK(&chip->als_lock);

	if (gain) {
		chip->als_gain_auto = false;
		rc = tmd2725_set_als_gain(chip, als_gains[i]);
		if (!rc)
			tmd2725_calc_cpl(chip);
	} else {
		chip->als_gain_auto = true;
	}
	tmd2725_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->als_lock);

	return rc ? -EIO : size;
}

static ssize_t tmd2725_als_cpl_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.cpl);
}

static ssize_t tmd2725_als_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			(((chip->shadow[TMD2725_REG_PERS]) & TMD2725_MASK_APERS)));
}

static ssize_t tmd2725_als_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	long persist;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &persist);
	if (rc)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->als_lock);
	chip->shadow[TMD2725_REG_PERS] &= ~TMD2725_MASK_APERS;
	chip->shadow[TMD2725_REG_PERS] |= ((u8)persist & TMD2725_MASK_APERS);

	tmd2725_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->als_lock);
	return size;
}

static ssize_t tmd2725_als_itime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	int t;

	t = chip->shadow[TMD2725_REG_ATIME];
	t *= INTEGRATION_CYCLE;
	return snprintf(buf, PAGE_SIZE, "%dms (%dus)\n", t / 1000, t);
}

static ssize_t tmd2725_als_itime_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	long itime;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &itime);
	if (rc)
		return -EINVAL;
	itime *= 1000;
	itime /= INTEGRATION_CYCLE;

	AMS_MUTEX_LOCK(&chip->als_lock);

	chip->shadow[TMD2725_REG_ATIME] = (u8) itime;
	chip->params.als_time = chip->shadow[TMD2725_REG_ATIME];
	tmd2725_calc_cpl(chip);
	tmd2725_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->als_lock);

	return size;
}

static ssize_t tmd2725_als_wtime_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int t;
	u8 wlongcurr;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	AMS_MUTEX_LOCK(&chip->als_lock);

	t = chip->shadow[TMD2725_REG_WTIME];

	wlongcurr = chip->shadow[TMD2725_REG_CFG0] & TMD2725_MASK_WLONG;
	if (wlongcurr)
		t *= 12;

	t *= INTEGRATION_CYCLE;
	t /= 1000;

	AMS_MUTEX_UNLOCK(&chip->als_lock);

	return snprintf(buf, PAGE_SIZE, "%d (in ms)\n", t);
}

static ssize_t tmd2725_als_wtime_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	unsigned long wtime;
	int wlong;
	int rc;

	rc = kstrtoul(buf, 10, &wtime);
	if (rc)
		return -EINVAL;

	wtime *= 1000;
	if (wtime > (256 * INTEGRATION_CYCLE))
	{
		wlong = 1;
		wtime /= 12;
	}
	else
		wlong = 0;
	wtime /= INTEGRATION_CYCLE;

	AMS_MUTEX_LOCK(&chip->als_lock);

	chip->shadow[TMD2725_REG_WTIME] = (u8) wtime;
	if (wlong)
		chip->shadow[TMD2725_REG_CFG0] |= TMD2725_MASK_WLONG;
	else
		chip->shadow[TMD2725_REG_CFG0] &= ~TMD2725_MASK_WLONG;

	tmd2725_flush_als_regs(chip);

	AMS_MUTEX_UNLOCK(&chip->als_lock);
	return size;
}


static ssize_t tmd2725_als_deltaP_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE,
			"%d (in %%)\n", chip->params.als_deltaP);
}

static ssize_t tmd2725_als_deltaP_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long deltaP;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &deltaP);
	if (rc || deltaP > 100)
		return -EINVAL;
	AMS_MUTEX_LOCK(&chip->als_lock);
	chip->params.als_deltaP = deltaP;
	AMS_MUTEX_UNLOCK(&chip->als_lock);
	return size;
}

static ssize_t tmd2725_als_ch0_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	tmd2725_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.als_ch0);
}

static ssize_t tmd2725_als_ch1_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	tmd2725_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->als_inf.als_ch1);
}


#ifdef LUX_DBG
static ssize_t tmd2725_als_adc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	tmd2725_get_lux(chip);

	return snprintf(buf, PAGE_SIZE, "LUX: %d CH0: %d CH1:%d\n",
		chip->als_inf.lux,chip->als_inf.als_ch0, chip->als_inf.als_ch1);
}

static ssize_t tmd2725_als_adc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	u32 ch0, ch1;

	if (2 != sscanf(buf, "%10d,%10d", &ch0, &ch1))
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->als_lock);

	chip->als_inf.als_ch0 = ch0;
	chip->als_inf.als_ch1 = ch1;

	AMS_MUTEX_UNLOCK(&chip->als_lock);
	return size;
}
#endif // #ifdef LUX_DBG
static ssize_t tmd2725_als_delay_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->als_poll_delay);
}

static ssize_t tmd2725_als_delay_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoint(buf, 0, &val);
	if (val < 500)
		val = 500;
	AMS_MUTEX_LOCK(&chip->als_lock);
	chip->als_poll_delay = val;
	AMS_MUTEX_UNLOCK(&chip->als_lock);

	return size;
}
static ssize_t tmd2725_als_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	chip->pdata->debug_level = !chip->pdata->debug_level;
	return sprintf(buf, "%d\n", chip->pdata->debug_level);
}

static ssize_t tmd2725_als_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	AMS_MUTEX_LOCK(&chip->als_lock);
	chip->pdata->debug_level = val;
	AMS_MUTEX_UNLOCK(&chip->als_lock);
	return size;
}
static ssize_t tmd2725_als_calibrate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->als_cal_data.flag);
}

static ssize_t tmd2725_als_calibrate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err;
	int val = 1;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	if (IS_ERR_OR_NULL(buf)) {
		SENSOR_LOG_ERROR("NULL.\n");
		return -EINVAL;
	}

	err = kstrtoint(buf, 0, &val);
	if (err < 0) {
		SENSOR_LOG_ERROR("kstrtoint failed\n");
		return err;
	}

	err = tmd2725_als_calibrate_work(chip, (const char *)&val);
	if (err < 0) {
		SENSOR_LOG_ERROR("als calibrate work failed.\n");
	}
	return size;
}
static ssize_t tmd2725_als_fac_calibrate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->als_cal_data.flag);
}

static ssize_t tmd2725_als_fac_calibrate_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	if (IS_ERR_OR_NULL(buf)) {
		SENSOR_LOG_ERROR("NULL.\n");
		return -EINVAL;
	}

	err = tmd2725_als_calibrate_work(chip, buf);
	if (err < 0) {
		SENSOR_LOG_ERROR("als calibrate work failed.\n");
	}
	return size;
}

static ssize_t tmd2725_als_chip_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DEVICE_CHIP_NAME);
}

static ssize_t tmd2725_als_dev_init_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t tmd2725_als_dev_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int err;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	tmd2725_config_tp_parameter(chip);

	SENSOR_LOG_INFO("config seg paramter is [%lld %lld][%lld %lld]\n",
					chip->params.lux_segment[0].ch0_coef,
					chip->params.lux_segment[0].ch1_coef,
					chip->params.lux_segment[1].ch0_coef,
					chip->params.lux_segment[1].ch1_coef);

	err = sensor_read_file(ALS_CAL_PATH,
							(char *)&(chip->als_cal_data),
							sizeof(struct tmd2725_als_cal_data));
	if (err < 0) {
		SENSOR_LOG_ERROR("read factory cal parameters failed\n");
	}
	if (chip->als_cal_data.cur.lux == 0) {
		tmd2725_als_parameter_init(chip);
	}
	return size;
}
struct device_attribute tmd2725_als_attrs[] = {
	__ATTR(enable,        	  0644, tmd2725_als_enable_show,       tmd2725_als_enable_store),
	__ATTR(delay,         	  0644, tmd2725_als_delay_show,        tmd2725_als_delay_store),
	__ATTR(debug,         	  0644, tmd2725_als_debug_show,        tmd2725_als_debug_store),
	__ATTR(flush,   	      0444, tmd2725_als_flush_show, 	   	   NULL),
	__ATTR(light_value,   	  0444, tmd2725_als_lux_show, 	   	   NULL),
	__ATTR(dev_init,      	  0664, tmd2725_als_dev_init_show,     tmd2725_als_dev_init_store),
	__ATTR(chip_name,     	  0444, tmd2725_als_chip_id_show, 	   NULL),
	__ATTR(calibrate,     	  0644, tmd2725_als_calibrate_show,    tmd2725_als_calibrate_store),
	__ATTR(fac_calibrate, 	  0644, tmd2725_als_fac_calibrate_show,tmd2725_als_fac_calibrate_store),
	__ATTR(als_Itime,     	  0644, tmd2725_als_itime_show,        tmd2725_als_itime_store),
	__ATTR(als_Wtime,     	  0644, tmd2725_als_wtime_show,        tmd2725_als_wtime_store),
	__ATTR(als_gain,          0644, tmd2725_als_gain_show,         tmd2725_als_gain_store),
	__ATTR(als_cpl,           0444, tmd2725_als_cpl_show,          NULL),
	__ATTR(als_thresh_deltaP, 0644, tmd2725_als_deltaP_show,       tmd2725_als_deltaP_store),
	__ATTR(als_auto_gain,     0644, tmd2725_auto_gain_enable_show, tmd2725_auto_gain_enable_store),
	__ATTR(als_lux_table,     0644, tmd2725_lux_table_show,        tmd2725_lux_table_store),
	__ATTR(als_persist,       0644, tmd2725_als_persist_show,      tmd2725_als_persist_store),
	__ATTR(als_ch0,           0444, tmd2725_als_ch0_show,          NULL),
	__ATTR(als_ch1,           0444, tmd2725_als_ch1_show,          NULL),
#ifdef LUX_DBG
	__ATTR(als_adc,           0644, tmd2725_als_adc_show,          tmd2725_als_adc_store),
#endif
};
static int tmd2725_als_parse_dt(struct device *dev) {

	struct device_node *np = dev->of_node;

	unsigned int tp_moudle_count = 0;
	int rc = 0, i, array_len, retval;
	long *ptr;
	int index;
	const char *raw_data0_dts = NULL;

	/* ps tuning data*/
	rc = of_property_read_u32(np, "tmd,tp_moudle_count", &tp_moudle_count);
	SENSOR_LOG_INFO("tp_module_count is %d\n", tp_moudle_count);

	for (i=0; i < tp_moudle_count; i++){
		array_len = of_property_count_strings(np, dts_array_name[i]);
		if (array_len != PARSE_DTSI_NUMBER) {
			SENSOR_LOG_ERROR("tmd2725,length invaild or dts number is larger than:%d\n",array_len);
			return -array_len;
		}
		SENSOR_LOG_INFO("read lux cal parameter count from dtsi  is %d\n", array_len);

		ptr = (long *)&tp_module_parameter[i];

		for(index = 0; index < array_len; index++){
			retval = of_property_read_string_index(np, dts_array_name[i], index, &raw_data0_dts);
			if (retval) {
				SENSOR_LOG_ERROR("read index = %d,raw_data0_dts = %s,retval = %d error,\n",index, raw_data0_dts, retval);
				return -retval;
			}
			ptr[index] = simple_strtoul(raw_data0_dts, NULL, 10);
			SENSOR_LOG_DEBUG("lux cal parameter from dtsi  is %ld\n",ptr[index]);
		}
	}

	return 0;
}
static void tmd2725_als_parameter_init(struct tmd2725_chip *chip)
{
	chip->als_cal_data.base.lux = 1;
	chip->als_cal_data.cur.lux = 1;
	chip->als_cal_data.flag = 0;
}
static int tmd2725_als_input_device_init(struct tmd2725_chip *chip)
{
	int ret;
	if (IS_ERR_OR_NULL(chip))
		return -ENODEV;
	chip->a_idev = input_allocate_device();
	if (!chip->a_idev) {
		SENSOR_LOG_ERROR("no memory for input_dev '%s'\n",
				chip->pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}

	chip->a_idev->name = chip->pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_REL, chip->a_idev->evbit);
	set_bit(REL_X, chip->a_idev->relbit);

	dev_set_drvdata(&chip->a_idev->dev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		SENSOR_LOG_ERROR("cant register input '%s'\n",
				chip->pdata->als_name);
		ret = -ENODEV;
		goto input_a_register_failed;
	}
	return 0;
input_a_register_failed:
	input_free_device(chip->a_idev);
input_a_alloc_failed:
	return ret;
}


int tmd2725_als_device_register(struct tmd2725_chip *chip, struct i2c_driver *driver)
{
	int ret;
	mutex_init(&chip->als_lock);
	INIT_DELAYED_WORK(&chip->als_work, tmd2725_als_report_work);
	chip->als_poll_delay = 1000;
	tmd2725_als_parameter_init(chip);

	ret = tmd2725_als_parse_dt(&chip->client->dev);
	if (ret < 0) {
		SENSOR_LOG_ERROR("parse dts error\n");
		goto exit;
	}
	ret = tmd2725_als_input_device_init(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("intput device init fail\n");
		goto exit;
	}

	als_class = class_create(THIS_MODULE, DEV_ALS_NAME);
	alloc_chrdev_region(&tmd2725_als_dev_t, 0, 1, DEV_ALS_NAME);
	chip->als_dev = device_create(als_class, 0, tmd2725_als_dev_t, driver, DEV_ALS_NAME);
	if (IS_ERR_OR_NULL(chip->als_dev)) {
		SENSOR_LOG_ERROR("als device create fail\n");
		ret = -PTR_ERR(chip->als_dev);
		goto exit_input_dev;
	}

	ret = sensor_create_sysfs_interfaces(chip->als_dev, tmd2725_als_attrs, ARRAY_SIZE(tmd2725_als_attrs));
	if (ret < 0) {
		goto exit_remove_device;
	}
	dev_set_drvdata(chip->als_dev, chip);
	return 0;
exit_remove_device:
	device_destroy(als_class, tmd2725_als_dev_t);
	class_destroy(als_class);
exit_input_dev:
	if (chip->a_idev) {
		input_unregister_device(chip->a_idev);
		input_free_device(chip->a_idev);
	}
exit:
	mutex_destroy(&chip->als_lock);
	return ret;
}
void tmd2725_als_device_unregister(struct tmd2725_chip *chip)
{
	input_unregister_device(chip->a_idev);
	input_free_device(chip->a_idev);
	sensor_remove_sysfs_interfaces(chip->als_dev, tmd2725_als_attrs, ARRAY_SIZE(tmd2725_als_attrs));
	mutex_destroy(&chip->als_lock);
}
