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

#include "ams_tmd2725_prox.h"
#include "ams_i2c.h"
#include "ams_common.h"

#undef LOG_TAG
#define LOG_TAG "TMD2725-PS"

static void tmd2725_read_prox(struct tmd2725_chip *chip);
static void tmd2725_get_prox(struct tmd2725_chip *chip);
static void tmd2725_report_prox(struct tmd2725_chip *chip);
//static void tmd2725_set_prox_mode(struct tmd2725_chip *chip);
static void tmd2725_init_prox_mode(struct tmd2725_chip *chip, bool first);
static int tmd2725_configure_prox_mode(struct tmd2725_chip *chip, u8 state);
static void tmd2725_prox_thread(struct work_struct *work);
static void tmd2725_prox_irq_work(struct work_struct *work);
static void tmd2725_schedule_prox_work(struct tmd2725_chip *chip, enum tmd2725_prox_state prox_state);
static int tmd2725_offset_calibration(struct tmd2725_chip *chip);
static int tmd2725_irq_handler(struct tmd2725_chip *chip);

static dev_t tmd2725_ps_dev_t;
static struct class *ps_class;

static void tmd2725_do_prox_state(struct tmd2725_chip *chip)
{
	switch (chip->prox_state) {
	case PROX_STATE_INIT:
		tmd2725_init_prox_mode(chip, false);
		break;
	case PROX_STATE_CALIB:
		tmd2725_offset_calibration(chip);
		break;
	case PROX_STATE_WAIT_AND_CALIB:
		// TODO: change trigger from wait to looking for a certain
		// number of readings that are stable (delta <5 counts)
		msleep(100);

		tmd2725_offset_calibration(chip);
		tmd2725_init_prox_mode(chip, false);
		break;
	default:
		break;
	}

	chip->prox_state = PROX_STATE_NONE;
}

static void tmd2725_prox_thread(struct work_struct *work)
{
	struct tmd2725_chip *chip
		= container_of(work, struct tmd2725_chip, ps_work);

	tmd2725_do_prox_state(chip);
}

static void tmd2725_prox_irq_work(struct work_struct *work)
{
	struct tmd2725_chip *chip
		= container_of(work, struct tmd2725_chip, ps_irq_work);
	tmd2725_irq_handler(chip);
}
static void tmd2725_do_prox_offset_cal(struct tmd2725_chip *chip, enum tmd2725_prox_state prox_state)
{
	chip->prox_state = prox_state;
	tmd2725_offset_calibration(chip);
	chip->prox_state = PROX_STATE_NONE;
}
static void tmd2725_schedule_prox_work(struct tmd2725_chip *chip, enum tmd2725_prox_state prox_state)
{
	chip->prox_state = prox_state;
	if(!schedule_work(&chip->ps_work)) {
		SENSOR_LOG_ERROR("schedule work fail\n");
		return;
	}
	return;
}

/*
static void tmd2725_do_prox_work(struct tmd2725_chip *chip, enum tmd2725_prox_state prox_state)
{
	chip->prox_state = prox_state;
	tmd2725_do_prox_state(chip);
}
*/
/**************************/
/* General Prox Functions */
/**************************/

static void tmd2725_read_prox(struct tmd2725_chip *chip)
{
	ams_i2c_blk_read(chip->client, TMD2725_REG_PDATA, &chip->shadow[TMD2725_REG_PDATA], 1);
	chip->prx_inf.raw = chip->shadow[TMD2725_REG_PDATA];
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "chip->prx_inf.raw = %d\n",chip->prx_inf.raw);
}

static void tmd2725_get_prox(struct tmd2725_chip *chip)
{
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;
	bool update_thresholds = false;
	bool update_offset = false;
	if (chip->pdata->debug_level) {
		SENSOR_LOG_INFO("prox_thres_far:%d prox_thres_near:%d prox_thres_oil_far:%d prox_thres_oil_near:%d\n",
			chip->params.prox_thres_far,
			chip->params.prox_thres_near,
			chip->params.prox_thres_oil_far,
			chip->params.prox_thres_oil_near);
		SENSOR_LOG_INFO("th_min:%d th_max:%d raw:%d state:%d detected:%s\n",
			chip->params.prox_th_min,
			chip->params.prox_th_max,
			chip->prx_inf.raw,
			chip->prox_state,
			chip->prx_inf.detected == PROX_NEAR ? "NEAR" : "FAR");
	}

	if (chip->prox_state != PROX_STATE_NONE) {
		SENSOR_LOG_INFO("prox state is %d, calibration is pending.\n",(int) chip->prox_state);
		return;
	}
	/* state machine */
	if (chip->prx_inf.detected == PROX_NONE) {
		if (chip->prx_inf.raw < chip->params.prox_th_min) {
		    chip->prx_inf.detected = PROX_FAR;
		    tmd2725_report_prox(chip);
		    chip->params.prox_th_min = 0;
		    chip->params.prox_th_max = chip->params.prox_thres_near;
		    update_thresholds = true;
		} else if (chip->prx_inf.raw > chip->params.prox_th_max){
		    chip->prx_inf.detected = PROX_NEAR;
		    tmd2725_report_prox(chip);
		    chip->params.prox_th_min = chip->params.prox_thres_far;
		    chip->params.prox_th_max = chip->params.prox_thres_oil_near;
		    update_thresholds = true;
		}

	}
	if (chip->prx_inf.detected == PROX_FAR) {
		if (chip->prx_inf.raw > chip->params.prox_th_max) {
			SENSOR_LOG_INFO("prox detect\n");
			if (chip->prox_state == PROX_STATE_NONE) {
				chip->prx_inf.detected = PROX_NEAR;
				chip->params.prox_oil_state = OIL_NONE;
				SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!FAR-->NEAR\n");
				tmd2725_report_prox(chip);
				chip->params.prox_th_min = chip->params.prox_thres_far;
				chip->params.prox_th_max = chip->params.prox_thres_oil_near;
				update_thresholds = true;
			}
		}
		if (chip->prx_inf.raw < chip->params.prox_th_min) {
			if (chip->prox_state == PROX_STATE_NONE) {
				SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!FAR-->FAR\n");
				chip->prx_inf.detected = PROX_FAR;
				tmd2725_report_prox(chip);
				chip->params.prox_th_min = 0;
				chip->params.prox_th_max = chip->params.prox_thres_near;
				update_thresholds = true;
				if (chip->params.prox_oil_state) {
					SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!OIL FAR-->FAR\n");
				    chip->params.prox_oil_state = OIL_NONE;
				    tmd2725_schedule_prox_work(chip, PROX_STATE_CALIB);
				}
			}
		}

	} else if (chip->prx_inf.detected == PROX_NEAR){
		if (chip->prx_inf.raw < chip->params.prox_th_min) {
			SENSOR_LOG_INFO("prox release\n");
			if (chip->prox_state == PROX_STATE_NONE) {
				chip->prx_inf.detected = PROX_FAR;
				tmd2725_report_prox(chip);
				if (chip->params.prox_th_max != PS_DATA_MAX) {
					//chip->params.poffset = min(chip->params.poffset + 10, 255);
					SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!NEAR-->FAR\n");
					update_thresholds = true;
					chip->params.prox_th_min = 0;
					chip->params.prox_th_max = chip->params.prox_thres_near;
				} else {
					update_thresholds = true;
					SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!OIL NEAR-->FAR\n");
					chip->params.prox_th_min = chip->params.prox_thres_near - 10;
					chip->params.prox_th_max = chip->params.prox_thres_oil_far + 10;
				}
			}
		} else if (chip->prx_inf.raw > chip->params.prox_th_max) {
			if (chip->prox_state == PROX_STATE_NONE) {
				chip->prx_inf.detected = PROX_NEAR;
				chip->params.prox_oil_state = OIL_STATE;
				SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "!!OIL NEAR->NEAR\n");
				tmd2725_report_prox(chip);
				chip->params.prox_th_min = chip->params.prox_thres_oil_far;
				chip->params.prox_th_max = PS_DATA_MAX;
				update_thresholds = true;
			}
		}
	}

	if (update_thresholds) {
		SENSOR_LOG_INFO("update_thresholds: min:%d max:%d\n", chip->params.prox_th_min, chip->params.prox_th_max);
		ams_i2c_write(client, sh, TMD2725_REG_PILT,
				chip->params.prox_th_min & 0xff);
		ams_i2c_write(client, sh, TMD2725_REG_PIHT,
				chip->params.prox_th_max & 0xff);
	}
	if (update_offset) {
		SENSOR_LOG_INFO("update_offset: %d\n", chip->params.poffset);
		if (chip->params.poffset <= 0) {
			ams_i2c_write(client, sh, TMD2725_REG_POFFSET_L,
				chip->params.poffset * (-1));
		} else {
			ams_i2c_write(client, sh, TMD2725_REG_POFFSET_L,
				chip->params.poffset );
		}
		ams_i2c_write(client, sh, TMD2725_REG_POFFSET_H,
				chip->params.poffset < 0 ? 1 : 0);
	}
}

static void tmd2725_report_prox(struct tmd2725_chip *chip)
{
	if (chip->p_idev) {
		if (chip->prx_inf.detected != chip->prx_inf.last_detected) {
			input_report_rel(chip->p_idev, REL_RZ, chip->prx_inf.detected);
			input_sync(chip->p_idev);
			SENSOR_LOG_INFO("prox status: %s(%d)\n",
				chip->prx_inf.detected == PROX_NEAR ? "NEAR" : "FAR",
				chip->prx_inf.detected);
			chip->prx_inf.last_detected = chip->prx_inf.detected;
		}
	}
}
/*
static void tmd2725_set_prox_mode(struct tmd2725_chip *chip)
{
	u8 *sh = chip->shadow;
	u8 rPGCFG0;
	u8 rPGCFG1;
	u16 lux = chip->als_inf.lux;

	if (lux <= INDOOR_LUX_TRIGGER) // Indoor light conditions
	{
		chip->amsIndoorMode = true;
		rPGCFG0 = PG_PULSE_16US | 4;       // Pulse len and # pulses
		rPGCFG1 = PGAIN_4 | PDRIVE_MA(75); // Gain and drive current
	}
	if (lux >= OUTDOOR_LUX_TRIGGER) // Outdoor (bright sun) light conditions
	{
		chip->amsIndoorMode = false;
		rPGCFG0 = PG_PULSE_4US | 22;       // Pulse len and # pulses
		rPGCFG1 = PGAIN_2 | PDRIVE_MA(75); // Gain and drive current
	}

	// If a change was made then push it to the device
	if (rPGCFG0 != sh[TMD2725_REG_PGCFG0])
		ams_i2c_write(chip->client, sh, TMD2725_REG_PGCFG0, rPGCFG0);
	if (rPGCFG1 != sh[TMD2725_REG_PGCFG1])
		ams_i2c_write(chip->client, sh, TMD2725_REG_PGCFG1, rPGCFG1);
}
*/

static int tmd2725_offset_calibration(struct tmd2725_chip *chip)
{
	u8 *sh = chip->shadow;
	int calwait = 0;
	int ret;

	SENSOR_LOG_INFO("enter\n");
	/* turn on power, disable prox */
	ams_i2c_write(chip->client, sh, TMD2725_REG_ENABLE, 0);
	ams_i2c_write(chip->client, sh, TMD2725_REG_ENABLE, TMD2725_PON);
	mdelay(5);
	/* enable calib intr, disable prox intr */
	ams_i2c_write(chip->client, sh, TMD2725_REG_INTENAB, TMD2725_CIEN);
	/*
	** Prox Offset calibration
	**   binsrch_target (7 counts)
	**   prox averaging (2 reading window)
	**   prox_auto_offset_adjust
	*/
	ams_i2c_write(chip->client, sh, TMD2725_REG_CALIBCFG,
			(0x05 << TMD2725_SHIFT_BINSRCH_TARGET) |
				(1 << TMD2725_SHIFT_PROX_DATA_AVG) |
				(1 << TMD2725_SHIFT_PROX_AUTO_OFFSET_ADJUST));

	/* trigger calibration sequence */
	SENSOR_LOG_INFO("offset calibration started.\n");
	chip->amsCalComplete = false;

	ams_i2c_modify(chip, sh, TMD2725_REG_CALIB,
			TMD2725_MASK_START_OFFSET_CALIB,
			0x01 << TMD2725_SHIFT_START_OFFSET_CALIB);

	/* amsCalComplete set true in IRQ Handler */
	for (calwait = 0; (!chip->amsCalComplete) && (calwait < 5); calwait++) {
		// TODO: change to signal and eliminate amsCalComplete & sleep
		msleep(10);
	}

	if (calwait < 5) {
		ret = 0;
	} else {
		SENSOR_LOG_ERROR("No Calibration IRQ, exiting spin loop\n");
		ret = -1;
	}

	// get updated prox offset
	ams_i2c_blk_read(chip->client, TMD2725_REG_POFFSET_L,
			&chip->shadow[TMD2725_REG_POFFSET_L], 2);
	chip->params.poffset = chip->shadow[TMD2725_REG_POFFSET_L];
	if (chip->shadow[TMD2725_REG_POFFSET_H] & TMD2725_MASK_POFFSET_H) {
		chip->params.poffset *= -1;
	}

	/* enablecalib intr prox intr, disable calib intr */
	//ams_i2c_write(chip->client, sh, TMD2725_REG_ENABLE, 0);
	if (chip->als_enabled)
	    ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
			TMD2725_AEN |TMD2725_PEN | TMD2725_PON,
			TMD2725_AEN |TMD2725_PEN | TMD2725_PON);
	else
	    ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
			TMD2725_WEN |TMD2725_PEN | TMD2725_PON,
			TMD2725_WEN |TMD2725_PEN | TMD2725_PON);

	ams_i2c_write(chip->client, sh, TMD2725_REG_INTENAB, TMD2725_PIEN);
	SENSOR_LOG_INFO("Optical Crosstalk calibration complete.\n");

	return ret;
}

static int tmd2725_mean_prox_calc(struct tmd2725_chip *chip)
{
	int mean = 0, i;
	int tmp[PS_AVG_TIME] = { 0 };
	for(i = 0; i < PS_AVG_TIME; i++)
	{
		tmd2725_read_prox(chip);
		tmp[i] += chip->prx_inf.raw;
		msleep(30);
	}
	sensor_quick_sort(tmp, PS_AVG_TIME, 0, PS_AVG_TIME - 1);
	for (i = PS_AVG_TIME / 4; i < 3 * PS_AVG_TIME / 4; i++)
		mean += tmp[i];
	mean = 2 * mean / PS_AVG_TIME;
	return mean;
}

static void tmd2725_init_prox_mode(struct tmd2725_chip *chip, bool first)
{
	chip->params.prox_th_min = 0;
	chip->params.prox_th_max = chip->params.prox_thres_near;
	if (chip->prx_inf.prox_raw_debug || first){
		chip->params.prox_th_min = PS_DEFAULT_THRES;
		chip->params.prox_th_max = PS_DEFAULT_THRES;
	}
	SENSOR_LOG_INFO("update_thresholds: min:%d max:%d\n", chip->params.prox_th_min, chip->params.prox_th_max);
	ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_PILT,
			chip->params.prox_th_min);
	ams_i2c_write(chip->client, chip->shadow, TMD2725_REG_PIHT,
			chip->params.prox_th_max);
}

static int tmd2725_configure_prox_mode(struct tmd2725_chip *chip, u8 state)
{
	extern void tmd2725_reg_log(struct tmd2725_chip *chip);
	struct i2c_client *client = chip->client;
	u8 *sh = chip->shadow;

	if (state) // Turning on prox
	{
		// Configure default proximity settings
		tmd2725_init_prox_mode(chip, true);

		//tmd2725_offset_calibration(chip);

		ams_i2c_modify(chip, sh, TMD2725_REG_PERS,
				TMD2725_MASK_PROX_PERS,
				chip->params.persist & TMD2725_MASK_PROX_PERS);
		ams_i2c_write(client, sh, TMD2725_REG_PGCFG0,
				(chip->params.prox_pulse_cnt -1) |
				chip->params.prox_pulse_len);
		ams_i2c_write(client, sh, TMD2725_REG_PGCFG1,
				chip->params.prox_gain |
				chip->params.prox_drive);

		ams_i2c_write(client, sh, TMD2725_REG_PTIME, P_TIME_US(2816));

		// Enable Proximity and Proximity Interrupt
		if (chip->als_enabled)
		    ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
			    TMD2725_AEN |TMD2725_PEN | TMD2725_PON,
			    TMD2725_AEN |TMD2725_PEN | TMD2725_PON);
		else
		    ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
			    TMD2725_WEN |TMD2725_PEN | TMD2725_PON,
			    TMD2725_WEN |TMD2725_PEN | TMD2725_PON);

		ams_i2c_write(client, sh, TMD2725_REG_INTENAB,
				TMD2725_PIEN);

		chip->prox_enabled = true;
		chip->amsFirstProx = true;
		chip->amsIndoorMode = true;
	}
	else // Turning off prox
	{
		ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
				TMD2725_PEN|TMD2725_WEN , 0);

		ams_i2c_write(client, sh, TMD2725_REG_INTENAB, 0);

		// If nothing else is enabled set PON = 0
		if (!(sh[TMD2725_REG_ENABLE] & TMD2725_EN_ALL))
			ams_i2c_modify(chip, sh, TMD2725_REG_ENABLE,
					TMD2725_PON, 0x00);

		chip->prox_enabled = false;
		chip->amsFirstProx = true;
		chip->prx_inf.last_detected = PROX_NONE;
		chip->prx_inf.detected = PROX_NONE;
	}

	return(0);
}
static int tmd2725_prox_report_first_event(struct tmd2725_chip *chip)
{
	bool update_thresholds = false;
	u8 *sh = chip->shadow;
	struct i2c_client *client = chip->client;
	/* enabled and in first irq state, we need report distance when enable */
	if (chip->prx_inf.detected == PROX_NONE) {
		mdelay(20);
		tmd2725_read_prox(chip);
		if (chip->prx_inf.raw < chip->params.prox_th_min) {
		    chip->prx_inf.detected = PROX_FAR;
		    tmd2725_report_prox(chip);
		    chip->params.prox_th_min = 0;
		    chip->params.prox_th_max = chip->params.prox_thres_near;
		    update_thresholds = true;
		} else if (chip->prx_inf.raw > chip->params.prox_th_max){
		    chip->prx_inf.detected = PROX_NEAR;
		    tmd2725_report_prox(chip);
		    chip->params.prox_th_min = chip->params.prox_thres_far;
		    chip->params.prox_th_max = chip->params.prox_thres_oil_near;
		    update_thresholds = true;
		}

	}
	if (update_thresholds) {
		SENSOR_LOG_INFO("update_thresholds: min:%d max:%d\n", chip->params.prox_th_min, chip->params.prox_th_max);
		ams_i2c_write(client, sh, TMD2725_REG_PILT,
				chip->params.prox_th_min & 0xff);
		ams_i2c_write(client, sh, TMD2725_REG_PIHT,
				chip->params.prox_th_max & 0xff);
	}
	return 0;
}
static int tmd2725_prox_set_enable(struct tmd2725_chip *chip, u8 enable)
{
	int ret;
	SENSOR_LOG_INFO("enter\n");
	if (!chip->pdata->power_always_on) {
		if (enable) {
			if (chip->pdata->power_state == POWER_OFF)
			    sensor_regulator_power_on(chip, true);
			chip->pdata->power_state |= POWER_PS_ON;
		} else {
			chip->pdata->power_state &= ~POWER_PS_ON;
			if (!chip->prox_enabled && !chip->als_enabled && chip->pdata->power_state == POWER_OFF)
			    sensor_regulator_power_on(chip, false);
		}
	}

	ret = tmd2725_configure_prox_mode(chip, enable);
	if (ret < 0) {
		SENSOR_LOG_ERROR("prox turn %s failed\n", enable?"on":"off");
		return ret;
	}

	sensor_irq_enable(chip, enable, true);

	if (enable) {
		tmd2725_prox_report_first_event(chip);
		if (chip->prx_inf.raw <= PS_OFFSET_CAL_THRESH)
		    tmd2725_do_prox_offset_cal(chip, PROX_STATE_CALIB);
	}

	SENSOR_LOG_INFO("prox turn %s\n", enable?"on":"off");
	return 0;
}
static int tmd2725_prox_thres_calibrate(struct tmd2725_chip *chip)
{
	int ret;
	u8 prox_buf[2];
	SENSOR_LOG_INFO("enter\n");
	chip->params.prox_thres_near = tmd2725_mean_prox_calc(chip);
	if (chip->params.prox_thres_near < PS_THRESH_DATA_MIN ||
		chip->params.prox_thres_near > PS_THRESH_DATA_MAX) {
		SENSOR_LOG_ERROR("thres calibration failed\n");
		return -ERR_THRES_CAL;
	}
	chip->params.prox_thres_far = chip->params.prox_thres_near / 2 + 10;
	prox_buf[0] = chip->params.prox_thres_near;
	prox_buf[1] = chip->params.prox_thres_far;
	SENSOR_LOG_INFO("chip->params.prox_thres_near = %d\n",chip->params.prox_thres_near);
	ret = sensor_write_file(PS_CAL_FILE_PATH, prox_buf, sizeof(prox_buf));
	if (ret < 0) {
		SENSOR_LOG_ERROR("write file fail\n");
		return ret;
	}
	return chip->params.prox_thres_near;
}
/*****************/
/* ABI Functions */
/*****************/

static ssize_t tmd2725_device_prox_raw(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	tmd2725_read_prox(chip);

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.raw);
}

static ssize_t tmd2725_device_prox_detected(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	u8 val;

	// if prox intr enabled, just grab the flag that has been set
	ams_i2c_read(chip->client, TMD2725_REG_INTENAB, &val);
	if (!(val & TMD2725_PIEN)) {
		tmd2725_read_prox(chip);
		tmd2725_get_prox(chip);
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prx_inf.detected);
}

static ssize_t tmd2725_prox_gain_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", 1 << chip->params.prox_gain);
}

static ssize_t tmd2725_prox_gain_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long gain;
	u8 regVal = 0;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &gain);

	if (rc)
		return -EINVAL;

	switch(gain)
	{
	case 0:
		regVal = 0;
		break;
	case 1:
		regVal = PGAIN_1;
		break;
	case 2:
		regVal = PGAIN_2;
		break;
	case 4:
		regVal = PGAIN_4;
		break;
	case 8:
		regVal = PGAIN_8;
		break;
	default:
		return -EINVAL;
	}

	AMS_MUTEX_LOCK(&chip->ps_lock);
	rc = ams_i2c_modify(chip, chip->shadow, TMD2725_REG_PGCFG1,
			TMD2725_MASK_PGAIN, regVal);
	chip->params.prox_gain = chip->shadow[TMD2725_REG_PGCFG1] & TMD2725_MASK_PGAIN;
	AMS_MUTEX_UNLOCK(&chip->ps_lock);

	return size;
}

static ssize_t tmd2725_prox_offset_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	u8 rPoffsetl = 0;
	u8 rPoffseth = 0;
	int prxofs = 0;

	// must read it from chip to get calibration result
	ams_i2c_read(chip->client, TMD2725_REG_POFFSET_L, &rPoffsetl);
	ams_i2c_read(chip->client, TMD2725_REG_POFFSET_H, &rPoffseth);
	prxofs = (rPoffseth & 0x01) ? -((int)rPoffsetl) : ((int)rPoffsetl);

	return snprintf(buf, PAGE_SIZE, "%d\n", prxofs);
}

static ssize_t tmd2725_prox_persist_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
			((chip->params.persist & 0xf0) >> 4));
}

static ssize_t tmd2725_prox_persist_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	unsigned long persist;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoul(buf, 10, &persist);
	if (rc)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->ps_lock);
	ams_i2c_modify(chip, chip->shadow, TMD2725_REG_PERS, TMD2725_MASK_PROX_PERS, persist);
	chip->params.persist = chip->shadow[TMD2725_REG_PERS];
	AMS_MUTEX_UNLOCK(&chip->ps_lock);

	return size;
}

static ssize_t tmd2725_prox_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", chip->prox_enabled);
}

static ssize_t tmd2725_prox_enable_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int enable;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	rc = kstrtoint(buf, 0, &enable);
	if (rc)
		return -EINVAL;

	AMS_MUTEX_LOCK(&chip->ps_lock);

	switch(enable)
	{
		case 0:
			// Disable prox
			tmd2725_prox_set_enable(chip, 0);
			break;

		case 1:
			// Enable prox
			tmd2725_prox_set_enable(chip, 1);
			break;
		default:
			AMS_MUTEX_UNLOCK(&chip->ps_lock);
			return -EINVAL;
	}

	AMS_MUTEX_UNLOCK(&chip->ps_lock);
	return size;
}

static ssize_t tmd2725_prox_regs_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);

	u8 rEnable = 0;
	u8 rPtime = 0;
	u8 rPiltl = 0;
	u8 rPihtl = 0;
	u8 rPers = 0;
	u8 rPgcfg0 = 0;
	u8 rPgcfg1 = 0;
	u8 rCfg1 = 0;
	u8 rStatus = 0;
	u8 rCfg2 = 0;
	u8 rIntenab = 0;
	u8 rPdata =0;

	ams_i2c_read(chip->client, TMD2725_REG_ENABLE, &rEnable);
	ams_i2c_read(chip->client, TMD2725_REG_PTIME, &rPtime);
	ams_i2c_read(chip->client, TMD2725_REG_PILT, &rPiltl);
	ams_i2c_read(chip->client, TMD2725_REG_PIHT, &rPihtl);
	ams_i2c_read(chip->client, TMD2725_REG_PERS, &rPers);
	ams_i2c_read(chip->client, TMD2725_REG_PGCFG0, &rPgcfg0);
	ams_i2c_read(chip->client, TMD2725_REG_PGCFG1, &rPgcfg1);
	ams_i2c_read(chip->client, TMD2725_REG_CFG1, &rCfg1);
	ams_i2c_read(chip->client, TMD2725_REG_STATUS, &rStatus);
	ams_i2c_read(chip->client, TMD2725_REG_CFG2, &rCfg2);
	ams_i2c_read(chip->client, TMD2725_REG_INTENAB, &rIntenab);
	ams_i2c_read(chip->client, TMD2725_REG_PDATA, &rPdata);

	return snprintf(buf, PAGE_SIZE,
		"ENABLE =   %2x\nPTIME  =   %2x\nPILT   =   %2x\nPIHT   =   %2x\n"
		"PERS   =   %2x\nPGCFG0 =   %2x\nPGCFG1 =   %2x\nCFG1   =   %2x\n"
		"CFG2   =   %2x\nSTATUS =   %2x\nINTENAB=   %2x\nPdata=  %2x\n"
		"%s\n%s settings\n",
			rEnable,
			rPtime,
			rPiltl,
			rPihtl,
			rPers,
			rPgcfg0,
			rPgcfg1,
			rCfg1,
			rCfg2,
			rStatus,
			rIntenab,
			rPdata,
			chip->prx_inf.detected == PROX_NEAR ? "Prox Detect" : "Prox Release",
			chip->amsIndoorMode ? "Indoor" : "Outdoor");
}


static ssize_t tmd2725_prox_regs_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	u8 *sh = chip->shadow;
	int addr, cmd, ret;

	if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		SENSOR_LOG_ERROR("invalid format: '%s'\n", buf);
		return count;
	}
	ret = ams_i2c_write(chip->client, sh, addr, cmd);
	if(ret < 0)
		SENSOR_LOG_ERROR("yulhan store_reg fail");

	return count;
}



static ssize_t tmd2725_prox_chip_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DEVICE_CHIP_NAME);
}
static ssize_t tmd2725_prox_flush_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	input_report_rel(chip->p_idev, REL_RZ, chip->prx_inf.detected);
	input_sync(chip->p_idev);
	return sprintf(buf, "%s\n", DEVICE_CHIP_NAME);
}

static ssize_t tmd2725_prox_dev_init_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 1;
}

static ssize_t tmd2725_prox_dev_init_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val = 0;
	int rc;
	u8 data[2];
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (val) {
		rc = sensor_read_file(PS_CAL_FILE_PATH, data, sizeof(data));
		if (rc < 0) {
			SENSOR_LOG_ERROR("read file error\n");
			return -1;
		}
		if (data[0] >= PS_THRESH_DATA_MIN && data[0] > data[1]) {
			chip->params.prox_thres_near = data[0];
			chip->params.prox_thres_far = data[1];
		}
		SENSOR_LOG_ERROR("chip->params.prox_thres_near = %d\n",
		            chip->params.prox_thres_near);
		SENSOR_LOG_ERROR("chip->params.prox_thres_far = %d\n",
		            chip->params.prox_thres_far);
	}
	return size;
}
static ssize_t tmd2725_prox_uncover_data_min_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_UNCOVER_DATA_MIN);
}
static ssize_t tmd2725_prox_uncover_data_max_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_UNCOVER_DATA_MAX);
}
static ssize_t tmd2725_prox_min_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_THRESH_DATA_MIN);
}
static ssize_t tmd2725_prox_max_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_THRESH_DATA_MAX);
}
static ssize_t tmd2725_prox_max_data_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", PS_DATA_MAX);
}
static ssize_t tmd2725_prox_thres_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", tmd2725_prox_thres_calibrate(chip));
}
static ssize_t tmd2725_prox_thres_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	if (rc < 0)
		return rc;
	if (val) {
		rc = tmd2725_prox_thres_calibrate(chip);
	}
	return rc;
}
static ssize_t tmd2725_prox_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	chip->pdata->debug_level = !chip->pdata->debug_level;
	return sprintf(buf, "%d\n", chip->pdata->debug_level);
}
static ssize_t tmd2725_prox_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	chip->pdata->debug_level = val;
	return size;
}
static ssize_t tmd2725_prox_raw_debug_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", chip->prx_inf.prox_raw_debug);
}
static ssize_t tmd2725_prox_raw_debug_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	rc = kstrtoint(buf, 0, &val);
	SENSOR_LOG_INFO("val = %d\n", val);
	chip->prx_inf.prox_raw_debug = val & 0xff;
	if (chip->prx_inf.prox_raw_debug) {
	    tmd2725_init_prox_mode(chip, false);
	}
	return size;
}
static ssize_t tmd2725_prox_uncover_cal_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	SENSOR_LOG_INFO("enter\n");
	chip->params.prox_raw = tmd2725_mean_prox_calc(chip);
	SENSOR_LOG_ERROR("chip->params.prox_raw=%d", chip->params.prox_raw);
	return sprintf(buf, "%d\n", chip->params.prox_raw);
}
static ssize_t tmd2725_prox_uncover_cal_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	int val;
	int rc;
	struct tmd2725_chip *chip = dev_get_drvdata(dev);
	SENSOR_LOG_INFO("enter\n");
	rc = kstrtoint(buf, 0, &val);
	if (val) {
		/*@ not use, uncover_cal_show will called */
		rc = tmd2725_offset_calibration(chip);
		if (rc < 0) {
			SENSOR_LOG_ERROR("*#777# calibrate fail\n");
			return size;
		}
	}
	return size;
}

static struct device_attribute tmd2725_prox_attrs[] = {
	__ATTR(enable, 0644, tmd2725_prox_enable_show, tmd2725_prox_enable_store),
	__ATTR(chip_name, 0440, tmd2725_prox_chip_id_show, NULL),
	__ATTR(debug, 0644, tmd2725_prox_debug_show,  tmd2725_prox_debug_store),
	__ATTR(prox_value, 0440, tmd2725_prox_flush_show, NULL),
	__ATTR(prox_debug, 0644, tmd2725_prox_raw_debug_show,  tmd2725_prox_raw_debug_store),
	__ATTR(prox_offset_cal, 0644, tmd2725_prox_uncover_cal_show, tmd2725_prox_uncover_cal_store),
	__ATTR(prox_thres, 0644, tmd2725_prox_thres_show, tmd2725_prox_thres_store),
	__ATTR(prox_thres_max,0440, tmd2725_prox_max_thres_show, NULL),
	__ATTR(prox_thres_min, 0440, tmd2725_prox_min_thres_show, NULL),
	__ATTR(prox_init, 0644, tmd2725_prox_dev_init_show, tmd2725_prox_dev_init_store),
	__ATTR(prox_data_max, 0440, tmd2725_prox_max_data_show, NULL),
	__ATTR(prox_uncover_min, 0440, tmd2725_prox_uncover_data_min_show, NULL),
	__ATTR(prox_uncover_max, 0440, tmd2725_prox_uncover_data_max_show, NULL),
	__ATTR(prox_raw, 0440, tmd2725_device_prox_raw, NULL),
	__ATTR(prox_detect, 0440, tmd2725_device_prox_detected, NULL),
	__ATTR(prox_gain, 0644, tmd2725_prox_gain_show, tmd2725_prox_gain_store),
	__ATTR(prox_offset, 0440, tmd2725_prox_offset_show,NULL),
	__ATTR(prox_persist, 0644, tmd2725_prox_persist_show, tmd2725_prox_persist_store),
	__ATTR(prox_regs, 0640, tmd2725_prox_regs_show, tmd2725_prox_regs_store),
};
static int tmd2725_irq_handler_locked(struct tmd2725_chip *chip, u8 status)
{
	int ret = 1;

	/* Calibration IRQ */
	if (status & TMD2725_ST_CAL_IRQ) {
	    SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "TMD2725_ST_CAL_IRQ\n");
	    chip->amsCalComplete = true;
	    /*
	    ** Calibration has completed, no need for more
	    **  calibration interrupts. These events are one-shots.
	    **  next calibration start will re-enable.
	    */
	    ams_i2c_modify(chip, chip->shadow,
	        TMD2725_REG_INTENAB, TMD2725_CIEN, 0);
	}
	/* Proximity IRQ*/
	if (status & TMD2725_ST_PRX_IRQ) {
	    SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "TMD2725_ST_PRX_IRQ\n");
	    /* Read Prox */
	    tmd2725_read_prox(chip);

	    /* Open debug mode, report raw data */
	    if (chip->prx_inf.prox_raw_debug) {
			    input_report_rel(chip->p_idev, REL_MISC, chip->prx_inf.raw);
			    input_sync(chip->p_idev);
			    return ret;
	    }
	    /* determine detect/release, report results */
	    tmd2725_get_prox(chip);
	}
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "exit\n");
	return ret;
}

static int tmd2725_irq_handler(struct tmd2725_chip *chip)
{
	u8 status = 0;
	int ret;
	int retry_times;
	int resched_times = 10;
	u8 *sh = chip->shadow;
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "enter\n");
	if (chip->wakeup_from_suspend) {
	    SENSOR_LOG_INFO("wakeup from suspend\n");
	    mdelay(30);
	    chip->wakeup_from_suspend = false;
	}
resched:
	retry_times = 3;
	do {
	    ret = ams_i2c_read(chip->client, TMD2725_REG_STATUS, &chip->shadow[TMD2725_REG_STATUS]);
	    if (ret >= 0) {
	        break;
	    }
	    resched_times--;
	    mdelay(10);
	    SENSOR_LOG_ERROR("i2c read fail\n");
	} while (--retry_times);

	if (resched_times == 0) {
	    SENSOR_LOG_ERROR("exit irq handler\n");
	    return 1;
	}
	status = chip->shadow[TMD2725_REG_STATUS];
	if (status != 0) {
	    /* Clear the interrupts we'll process */
	    ams_i2c_write(chip->client, sh, TMD2725_REG_STATUS, status);
	    tmd2725_irq_handler_locked(chip, status);
	}

	if (!gpio_get_value(chip->irq_gpio)) {
	    SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "gpio val = %d\n", gpio_get_value(chip->irq_gpio));
	    goto resched;
	}
	/* we handled the interrupt */
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "exit\n");
	return 1;
}

static irqreturn_t tmd2725_irq(int irq, void *handle)
{
	struct tmd2725_chip *chip = handle;
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "enter\n");
	wake_lock_timeout(&chip->ps_wlock, msecs_to_jiffies(100));
	tmd2725_irq_handler(chip);
	SENSOR_LOG_DEBUG_IF(chip->pdata->debug_level, "exit\n");
	return IRQ_HANDLED;
}
static int tmd2725_ps_input_device_init(struct tmd2725_chip *chip)
{
	int ret;
	if (IS_ERR_OR_NULL(chip))
		return -ENODEV;

	chip->p_idev = input_allocate_device();
	if (!chip->p_idev) {
		SENSOR_LOG_ERROR("no memory for input_dev '%s'\n", chip->pdata->prox_name);
		ret = -ENODEV;
		goto input_p_alloc_failed;
	}
	chip->p_idev->name = chip->pdata->prox_name;
	chip->p_idev->id.bustype = BUS_I2C;
	set_bit(EV_REL, chip->p_idev->evbit);
	set_bit(REL_RZ,  chip->p_idev->relbit);
	set_bit(REL_MISC,  chip->p_idev->relbit);

	dev_set_drvdata(&chip->p_idev->dev, chip);
	ret = input_register_device(chip->p_idev);
	if (ret) {
		SENSOR_LOG_ERROR("cant register input '%s'\n", chip->pdata->prox_name);
		goto input_p_register_failed;
	}
	return 0;
input_p_register_failed:
	input_free_device(chip->p_idev);
input_p_alloc_failed:
	return ret;
}

static int tmd2725_prox_irq_init(struct tmd2725_chip *chip)
{
	int ret;
	if (IS_ERR_OR_NULL(chip)) {
		SENSOR_LOG_ERROR("null exception\n");
		return -EINVAL;
	}
	chip->wake_irq = true;
	if (gpio_is_valid(chip->irq_gpio)) {
		/* configure pa22 irq gpio */
		SENSOR_LOG_INFO("gpio value is %d \n", gpio_get_value(chip->irq_gpio));
		ret = gpio_request_one(chip->irq_gpio,
				GPIOF_DIR_IN,
				"tmd2725_irq_gpio");
		if (ret) {
			SENSOR_LOG_ERROR("unable to request gpio %d\n",
				chip->irq_gpio);
			ret = -ENODEV;
			goto exit;
		}
		chip->irq = chip->client->irq =
			gpio_to_irq(chip->irq_gpio);
	} else {
		SENSOR_LOG_ERROR("irq gpio not provided\n");
	}
	SENSOR_LOG_INFO("init [gpio: %d][irq: %d] success\n", chip->irq_gpio, chip->client->irq);

	if (chip->wake_irq)
		irq_set_irq_wake(chip->client->irq, 1);

	ret = request_threaded_irq(chip->client->irq, NULL, &tmd2725_irq,
			IRQF_TRIGGER_LOW | IRQF_ONESHOT,
			dev_name(chip->ps_dev), chip);
	if (ret) {
		SENSOR_LOG_ERROR("Failed to request irq %d\n", chip->client->irq);
		ret = -ENODEV;
		goto exit_request_irq;
	}
	sensor_irq_enable(chip, false, false);
	SENSOR_LOG_INFO("request threaded irq %d success\n", chip->client->irq);
	return 0;
exit_request_irq:
	if (chip->irq_gpio)
		gpio_free(chip->irq_gpio);
exit:
	return ret;
}
static void tmd2725_ps_parameters_init(struct tmd2725_chip *chip)
{
	mutex_init(&chip->ps_lock);
	INIT_WORK(&chip->ps_work, tmd2725_prox_thread);
	INIT_WORK(&chip->ps_irq_work, tmd2725_prox_irq_work);
	chip->prx_inf.last_detected = PROX_NONE;
	chip->prx_inf.detected = PROX_NONE;
	chip->irq_enabled = true;
	chip->wakeup_from_suspend = false;
	chip->pdata->debug_level = 0;
	chip->params.prox_thres_near = PS_THRES_NEAR;
	chip->params.prox_thres_far = PS_THRES_FAR;
	chip->params.prox_thres_oil_near= PS_THRES_OIL_NEAR;
	chip->params.prox_thres_oil_far= PS_THRES_OIL_FAR;
}
int tmd2725_ps_device_register(struct tmd2725_chip *chip, struct i2c_driver *driver)
{
	int ret;
	tmd2725_ps_parameters_init(chip);
	ret = tmd2725_ps_input_device_init(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("intput device init fail\n");
		goto exit;
	}
	/* create workqueue */
	chip->ps_workqueue = create_singlethread_workqueue("tmd2725_irq_workqueue");
	if (IS_ERR_OR_NULL(chip->ps_workqueue)) {
		ret = -ENOMEM;
		SENSOR_LOG_ERROR( "cannot create work taos_work_queue, ret = %d",ret);
		goto exit_input_dev;
	}
	/* create sysfs */
	ps_class = class_create(THIS_MODULE, DEV_PS_NAME);
	alloc_chrdev_region(&tmd2725_ps_dev_t, 0, 1, DEV_PS_NAME);
	chip->ps_dev = device_create(ps_class, 0, tmd2725_ps_dev_t, driver, DEV_PS_NAME);
	if (IS_ERR_OR_NULL(chip->ps_dev)) {
		SENSOR_LOG_ERROR("ps device create fail\n");
		ret = -PTR_ERR(chip->ps_dev);
		goto exit_clear_workqueue;
	}
	ret = sensor_create_sysfs_interfaces(chip->ps_dev,
			tmd2725_prox_attrs, ARRAY_SIZE(tmd2725_prox_attrs));
	if (ret < 0) {
		goto exit_remove_device;
	}

	dev_set_drvdata(chip->ps_dev, chip);
	ret = tmd2725_prox_irq_init(chip);
	if (ret < 0) {
		SENSOR_LOG_ERROR("irq init fail\n");
		goto exit_remove_sys_interfaces;
	}
	return 0;
exit_remove_sys_interfaces:
	sensor_remove_sysfs_interfaces(chip->ps_dev,
			tmd2725_prox_attrs, ARRAY_SIZE(tmd2725_prox_attrs));
exit_remove_device:
	device_destroy(ps_class, tmd2725_ps_dev_t);
	class_destroy(ps_class);
exit_clear_workqueue:
	destroy_workqueue(chip->ps_workqueue);
exit_input_dev:
	if (chip->p_idev){
		input_unregister_device(chip->p_idev);
		input_free_device(chip->p_idev);
	}
exit:
	return ret;
}

void tmd2725_ps_device_unregister(struct tmd2725_chip *chip)
{
	input_unregister_device(chip->p_idev);
	input_free_device(chip->p_idev);
	free_irq(chip->client->irq, chip->client);
	sensor_remove_sysfs_interfaces(chip->ps_dev,tmd2725_prox_attrs, ARRAY_SIZE(tmd2725_prox_attrs));
	wake_lock_destroy(&chip->ps_wlock);
	mutex_destroy(&chip->ps_lock);
	free_irq(chip->irq, chip->client);
}
