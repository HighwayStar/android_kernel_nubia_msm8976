/*
** =============================================================================
** Copyright (c) 2016  Texas Instruments Inc.
**
** This program is free software; you can redistribute it and/or modify it under
** the terms of the GNU General Public License as published by the Free Software 
** Foundation; version 2.
**
** This program is distributed in the hope that it will be useful, but WITHOUT
** ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
** FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License along with
** this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
** Street, Fifth Floor, Boston, MA 02110-1301, USA.
**
** File:
**     tas2555-core.c
**
** Description:
**     TAS2555 common functions for Android Linux
**
** =============================================================================
*/

#define DEBUG
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/crc8.h>
#include <asm/uaccess.h>

#include "tas2555.h"
#include "tas2555-core.h"

#define PPC_WITH_DRIVER_VERSION		0x010bc000
#define PPC_WITH_CHECKSUM			0x010c8400
#define PPC_DRIVER_VERSION			0x00000200

#define TAS2555_CAL_NAME    "/persist/tas2555_cal.bin"

//set default PLL CLKIN to GPI2 (MCLK) = 0x00
#define TAS2555_DEFAULT_PLL_CLKIN 0x00

static int tas2555_load_calibration(struct tas2555_priv *pTAS2555,
	char *pFileName);
static int tas2555_load_data(struct tas2555_priv *pTAS2555, struct TData *pData,
	unsigned int nType);
static int tas2555_load_block(struct tas2555_priv *pTAS2555, struct TBlock *pBlock);
static int tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame);
	
#define TAS2555_UDELAY 0xFFFFFFFE

#define FW_ERR_HEADER -1
#define FW_ERR_SIZE -2

#define TAS2555_BLOCK_PLL			0x00
#define TAS2555_BLOCK_BASE_MAIN		0x01
#define TAS2555_BLOCK_CONF_COEFF	0x03
#define TAS2555_BLOCK_CONF_PRE		0x04
#define TAS2555_BLOCK_CONF_POST		0x05
#define TAS2555_BLOCK_CONF_POST_POWER	0x06
#define TAS2555_BLOCK_CONF_CAL		0x0A

static unsigned int p_tas2555_default_data[] = {
	TAS2555_SAR_ADC2_REG, 0x05,	/* enable SAR ADC */
	TAS2555_CLK_ERR_CTRL2, 0x39,	//enable clock error detection on PLL
	TAS2555_CLK_ERR_CTRL3, 0x11,	//enable clock error detection on PLL
	TAS2555_SAFE_GUARD_REG, TAS2555_SAFE_GUARD_PATTERN,	//safe guard
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_irq_config[] = {
	TAS2555_CLK_HALT_REG, 0x71,
	TAS2555_INT_GEN1_REG, 0x11,	/* enable spk OC and OV */
	TAS2555_INT_GEN2_REG, 0x11,	/* enable clk err1 and die OT */
	TAS2555_INT_GEN3_REG, 0x11,	/* enable clk err2 and brownout */
	TAS2555_INT_GEN4_REG, 0x01,	/* disable SAR, enable clk halt */
	TAS2555_GPIO4_PIN_REG, 0x07,	/*set GPIO4 as int1, default */
	TAS2555_INT_MODE_REG, 0x80,	/* active high until INT_STICKY_1 and INT_STICKY_2 are read to be cleared. */
	0xFFFFFFFF, 0xFFFFFFFF
};

#define TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX 3
static unsigned int p_tas2555_startup_data[] = {
	TAS2555_CLK_ERR_CTRL1, 0x00,	//disable clock error detection on PLL
	TAS2555_PLL_CLKIN_REG, TAS2555_DEFAULT_PLL_CLKIN,
	TAS2555_POWER_CTRL2_REG, 0xA0,	//Class-D, Boost power up
	TAS2555_POWER_CTRL2_REG, 0xA3,	//Class-D, Boost, IV sense power up
	TAS2555_POWER_CTRL1_REG, 0xF8,	//PLL, DSP, clock dividers power up
	TAS2555_UDELAY, 2000,		//delay
	TAS2555_POWER_CTRL1_REG, 0x78,	//toggle DSP 
	TAS2555_UDELAY, 2000,		//delay 2ms
	TAS2555_POWER_CTRL1_REG, 0xf8,
	TAS2555_UDELAY, 2000,		//delay 2ms
	TAS2555_CLK_ERR_CTRL1, 0x03,	//enable clock error detection on PLL
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_unmute_data[] = {
	TAS2555_MUTE_REG, 0x00,		//unmute
	TAS2555_SOFT_MUTE_REG, 0x00,	//soft unmute
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_shutdown_data[] = {
	TAS2555_CLK_ERR_CTRL1, 0x00,	//disable clock error detection
	TAS2555_SOFT_MUTE_REG, 0x01,	//soft mute
	TAS2555_UDELAY, 10000,		//delay 10ms
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 2000,		//delay 2ms
	TAS2555_POWER_CTRL2_REG, 0x00,	//Class-D, Boost power down
	TAS2555_POWER_CTRL1_REG, 0x00,	//all power down
	0xFFFFFFFF, 0xFFFFFFFF
};

static unsigned int p_tas2555_mute_DSP_down_data[] = {
	TAS2555_CLK_ERR_CTRL1, 0x00,	//disable clock error detection
	TAS2555_MUTE_REG, 0x03,		//mute
	TAS2555_PLL_CLKIN_REG, 0x0F,	//PLL clock input = osc
	TAS2555_POWER_CTRL1_REG, 0x60,	//DSP power down
	TAS2555_UDELAY, 0xFF,		//delay
	0xFFFFFFFF, 0xFFFFFFFF
};

#if 0
/*
 *	<Gain reduction to be applied> = 
	dec2hex(10^(gain reduction in dB/20) * 2^31-1)
*/
static unsigned int p_tas2555_digital_gain_data[] = {
0x7FFFFFFF,        // 0	    dB
0x721482BE,        // -1	dB
0x65AC8C2E,        // -2	dB
0x5A9DF7AA,        // -3	dB
0x50C335D2,        // -4	dB
0x47FACCEF,        // -5	dB
0x4026E73B,        // -6	dB
0x392CED8C,        // -7	dB
0x32F52CFD,        // -8	dB
0x2D6A866E,        // -9	dB
0x287A26C3,        // -10	dB
};


static int tas2555_digital_gain(struct tas2555_priv *pTAS2555, int gain)
{
	int ret = -1;
	int value = 0;
	unsigned char Buf[4];
	
	if(gain < ARRAY_LEN(p_tas2555_digital_gain_data)){
		value = p_tas2555_digital_gain_data[gain];
		Buf[0] = (unsigned char)((value&0xff000000)>>24);
		Buf[1] = (unsigned char)((value&0x00ff0000)>>16);
		Buf[2] = (unsigned char)((value&0x0000ff00)>>8);
		Buf[3] = (unsigned char)((value&0x000000ff));
		
		ret = pTAS2555->bulk_write(pTAS2555, 
			TAS2555_DIGITAL_GAIN_REG,
			Buf, 4);
			
		Buf[0] = 0;
		Buf[1] = 0;
		Buf[2] = 0;
		Buf[3] = 1;	
		ret = pTAS2555->bulk_write(pTAS2555, 
			TAS2555_COEFFICENT_UPDATE_REG, Buf, 4);
	}
	
	return ret;
}
#endif

static int tas2555_dev_load_data(struct tas2555_priv *pTAS2555,
	unsigned int *pData)
{
	int ret = 0;
	unsigned int n = 0;
	unsigned int nRegister;
	unsigned int nData;

	do {
		nRegister = pData[n * 2];
		nData = pData[n * 2 + 1];
		if (nRegister == TAS2555_UDELAY)
			udelay(nData);
		else if (nRegister != 0xFFFFFFFF) {
			ret = pTAS2555->write(pTAS2555, nRegister, nData);
			if(ret < 0) {
				break;
			}
		}
		n++;
	} while (nRegister != 0xFFFFFFFF);

	return ret;
}

void failsafe(struct tas2555_priv *pTAS2555)
{
	dev_err(pTAS2555->dev, "%s\n", __func__);
	pTAS2555->mnErrorCode |= TAS2555_ERROR_FAILSAFE;
	dev_dbg(pTAS2555->dev, "Enable: load shutdown sequence\n");
	tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);
	if (pTAS2555->mpFirmware != NULL)
		tas2555_clear_firmware(pTAS2555->mpFirmware);
	pTAS2555->mbPowerUp = false;
	pTAS2555->hw_reset(pTAS2555);
	pTAS2555->write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);
	udelay(1000);
	pTAS2555->write(pTAS2555, TAS2555_SPK_CTRL_REG, 0x04);
}

#define Q_FACTOR 0x08000000

static bool chkReDeltaBoundary(struct tas2555_priv *pTAS2555, 
	unsigned int ReOrginal, unsigned int ReDelta, unsigned int Re, unsigned char scale, unsigned int *pActRe)
{
	bool nResult = false;
	unsigned int ReHigh_calc, ReLow_calc, Re_calc = 0;
	static unsigned int counter = 3;

	if (ReOrginal < Q_FACTOR)
		goto end;

	if (ReDelta == 0) {
		dev_err(pTAS2555->dev, "need to set ReDelta!");
		goto end;
	}

	switch (scale) {
	case 0:
	/* 8Ohm speaker */
		Re_calc = Re;
		*pActRe = Re;
		ReHigh_calc = ReOrginal + ReDelta;
		ReLow_calc = ReOrginal - ReDelta;
		break;

	case 1:
	/* 6Ohm speaker */
		ReDelta = (ReDelta / 3) * 4;
		ReHigh_calc = ReOrginal + ReDelta;
		ReLow_calc = ReOrginal - ReDelta;
		Re_calc = Re;
		*pActRe = (Re_calc / 4) * 3;
		break;

	case 2:
	/* 4Ohm speaker */
		ReDelta = ReDelta * 2;
		ReHigh_calc = ReOrginal + ReDelta;
		ReLow_calc = ReOrginal - ReDelta;
		Re_calc = Re;
		*pActRe = Re_calc / 2;
		break;
	}

	if (Re_calc > 0) {
		pTAS2555->mnReLastKnown = *pActRe;

		if (Re_calc > ReHigh_calc) {
			counter--;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_RETOOHIGH;
		} else if (Re_calc < ReLow_calc) {
			counter--;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_RETOOLOW;
		} else
			counter = 3;

		if (counter == 0) {
			dev_err(pTAS2555->dev, "will failsafe, OrigRe=%d, Delta_Re=%d, Re=%d, scale=%d\n",
				ReOrginal, ReDelta, Re, scale);
			nResult = true;
		}
	}

end:
	return nResult;
}

static int tas2555_get_ReCoefficient(struct tas2555_priv *pTAS2555, unsigned int *pRe)
{
	int ret = 0;
	unsigned char Buf[4];

	/* wait DSP to update the XRAM */
	udelay(2000);

	ret = pTAS2555->bulk_read(pTAS2555, TAS2555_COEFFICENT_RE_REG, Buf, 4);
	if (ret < 0) {
		goto err;
	}

	*pRe = ((unsigned int)Buf[0] << 24) | ((unsigned int)Buf[1] << 16) 
			| ((unsigned int)Buf[2] << 8) | Buf[3];

err:

	return ret;
}

int tas2555_get_Re(struct tas2555_priv *pTAS2555, unsigned int *pRe)
{
	int ret = 0;
	unsigned int nRe = 0, nValue;

	if (pTAS2555->mbPowerUp) {
		ret = tas2555_get_ReCoefficient(pTAS2555, &nRe);
		if (ret < 0)
			goto err;

		ret = pTAS2555->read(pTAS2555, TAS2555_CHANNEL_CTRL_REG, &nValue);
		if (ret < 0) {
			goto err;
		}

		nValue = (nValue & 0x06) >> 1;
		if (chkReDeltaBoundary(pTAS2555, pTAS2555->mnReOrignal, pTAS2555->mnReDelta, nRe, nValue, pRe))
			failsafe(pTAS2555);
	} else {
		*pRe = pTAS2555->mnReLastKnown;
	}

err:

	return ret;
}

int tas2555_get_errcode(struct tas2555_priv *pTAS2555, unsigned int *pErrCode)
{
	unsigned int errcode = 0;
	unsigned int nValue = 0;
	int nResult = 0;

	nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &nValue);
	if (nResult >= 0) {
		nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &nValue);

		if (nValue & 0x04) {
			errcode |= TAS2555_ERROR_CLKPRESENT;
			nResult = pTAS2555->write(pTAS2555, TAS2555_CLK_ERR_CTRL1, 0x00);
		}
		if (nValue & 0x08)
			errcode |= TAS2555_ERROR_BROWNOUT;
		if (nValue & 0x10)
			errcode |= TAS2555_ERROR_OVERTMP;
		if (nValue & 0x40)
			errcode |= TAS2555_ERROR_UNDERVOLTAGET;
		if (nValue & 0x80)
			errcode |= TAS2555_ERROR_OVERCURRENT;
		if (nValue & 0xdc) 
			nResult = tas2555_enable(pTAS2555, false);
	}

	*pErrCode = errcode | pTAS2555->mnErrorCode;
	pTAS2555->mnErrorCode = 0;
	return nResult;
}

/*
* die temperature calculation:
* 	deltaT = (nT1 - nT2 ) / 2^10
* 	DieTemp = (deltaT - 0.3459) / 0.001
*/
int tas2555_get_die_delta_temperature(struct tas2555_priv *pTAS2555, int *pDeltaT)
{
	unsigned char nBuf[4];
	int nResult = 0;
	unsigned int nT1 = 0, nT2 = 0;

	nResult = pTAS2555->bulk_read(pTAS2555, TAS2555_SAR_T1MSB_REG, nBuf, 4);
	if (nResult >= 0) {
		nT1 = ((unsigned int)nBuf[0] << 2) | (((unsigned int)nBuf[1] & 0xc0) >> 6);
		nT2 = ((unsigned int)nBuf[2] << 2) | (((unsigned int)nBuf[3] & 0xc0) >> 6);
		*pDeltaT = nT1 - nT2;
	}

	return nResult;
}

int tas2555_configIRQ(struct tas2555_priv *pTAS2555)
{
	return tas2555_dev_load_data(pTAS2555, p_tas2555_irq_config);
}

int tas2555_load_platdata(struct tas2555_priv *pTAS2555)
{
	int nResult = 0;

	if (gpio_is_valid(pTAS2555->mnGpioINT)) {
		nResult = tas2555_configIRQ(pTAS2555);
		if (nResult >= 0)
			nResult = pTAS2555->enableIRQ(pTAS2555, false, true);
	}

	return nResult;
}

int tas2555_load_default(struct tas2555_priv *pTAS2555)
{
	int nResult = 0;

	nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_default_data);
	if (nResult >= 0)
		nResult = tas2555_load_platdata(pTAS2555);

	return nResult;
}

int tas2555_enable(struct tas2555_priv *pTAS2555, bool bEnable)
{
	int nResult = 0;
	unsigned int nValue = 0;
	struct TConfiguration *pConfiguration;

	dev_dbg(pTAS2555->dev, "Enable: %d\n", bEnable);

	if ((pTAS2555->mpFirmware->mnPrograms == 0)
		||(pTAS2555->mpFirmware->mnConfigurations == 0)) {
			pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;
			goto end;
		}

	if (bEnable) {
		if (!pTAS2555->mbPowerUp) {
			nResult = pTAS2555->read(pTAS2555, TAS2555_DSP_MODE_SELECT_REG, &nValue);
			if (nResult < 0)
				goto end;
			if (pTAS2555->mnCurrentProgram == 0) {
				/* smart-amp mode */
				if ((nValue & 0x03) != 0) {
					/* unexpected error happens */
					nResult = -1;
					goto end;
				}
			}

			nResult = pTAS2555->read(pTAS2555, TAS2555_SAFE_GUARD_REG, &nValue);
			if ((nValue & 0xff) != TAS2555_SAFE_GUARD_PATTERN) {
					/* failed to pass safe guard check */
					nResult = -1;
					goto end;
			}

			if (!pTAS2555->mbCalibrationLoaded) {
				tas2555_load_calibration(pTAS2555, TAS2555_CAL_NAME);
				pTAS2555->mbCalibrationLoaded = true;
			}

			dev_dbg(pTAS2555->dev, "Enable: load startup sequence\n");
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
			if (nResult < 0)
				goto end;
			if (pTAS2555->mpFirmware->mpConfigurations) {
				pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
				nResult = tas2555_load_data(pTAS2555, &(pConfiguration->mData), TAS2555_BLOCK_CONF_POST_POWER);
				if (nResult < 0)
					goto end;
				if (pTAS2555->mbLoadConfigurationPostPowerUp) {
					dev_dbg(pTAS2555->dev, "Enable: load configuration: %s, %s\n",
						pConfiguration->mpName, pConfiguration->mpDescription);
					nResult = tas2555_load_data(pTAS2555, &(pConfiguration->mData), TAS2555_BLOCK_CONF_COEFF);
					if (nResult < 0)
						goto end;
					pTAS2555->mbLoadConfigurationPostPowerUp = false;
					if (pTAS2555->mpCalFirmware->mnCalibrations) {
						dev_dbg(pTAS2555->dev, "Enable: load calibration\n");
						nResult = tas2555_load_block(pTAS2555,
									&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));
						if (nResult < 0)
							goto end;
						nResult = tas2555_get_ReCoefficient(pTAS2555, &pTAS2555->mnReOrignal);
						if (nResult < 0)
							goto end;
						pTAS2555->mbLoadCalibrationPostPowerUp = false;
					} else if (pTAS2555->mnReOrignal == 0) {
						nResult = tas2555_get_ReCoefficient(pTAS2555, &pTAS2555->mnReOrignal);
						if (nResult < 0)
							goto end;
					}
				} else {
					if (pTAS2555->mpCalFirmware->mnCalibrations) {
						if (pTAS2555->mbLoadCalibrationPostPowerUp) {
							dev_dbg(pTAS2555->dev, "Enable: load calibration\n");
							nResult = tas2555_load_block(pTAS2555,
								&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));
							if (nResult < 0)
								goto end;
							nResult = tas2555_get_ReCoefficient(pTAS2555, &pTAS2555->mnReOrignal);
							if (nResult < 0)
								goto end;
							pTAS2555->mbLoadCalibrationPostPowerUp = false;
						}
					}
				}
			}

			dev_dbg(pTAS2555->dev, "Enable: load unmute sequence\n");
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
			if (nResult < 0)
				goto end;
			pTAS2555->mbPowerUp = true;
		}
	} else {
		if (pTAS2555->mbPowerUp) {
			dev_dbg(pTAS2555->dev, "Enable: load shutdown sequence\n");
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);
			pTAS2555->mbPowerUp = false;
		}
	}

end:
	if (nResult < 0)
		failsafe(pTAS2555);

	return nResult;
}

int tas2555_set_sampling_rate(struct tas2555_priv *pTAS2555, unsigned int nSamplingRate)
{
	struct TConfiguration *pConfiguration;
	unsigned int nConfiguration;

	dev_dbg(pTAS2555->dev, "tas2555_setup_clocks: nSamplingRate = %d [Hz]\n",
		nSamplingRate);

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		return -EINVAL;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	if (pConfiguration->mnSamplingRate == nSamplingRate) {
		dev_info(pTAS2555->dev, "Sampling rate for current configuration matches: %d\n",
			nSamplingRate);
		return 0;
	}

	for (nConfiguration = 0;
		nConfiguration < pTAS2555->mpFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration =
			&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
		if ((pConfiguration->mnSamplingRate == nSamplingRate)
			&&(pConfiguration->mnProgram == pTAS2555->mnCurrentProgram)){
			dev_info(pTAS2555->dev,
				"Found configuration: %s, with compatible sampling rate %d\n",
				pConfiguration->mpName, nSamplingRate);
			return tas2555_load_configuration(pTAS2555, nConfiguration, false);
		}
	}

	dev_err(pTAS2555->dev, "Cannot find a configuration that supports sampling rate: %d\n",
		nSamplingRate);

	return -EINVAL;
}

static void fw_print_header(struct tas2555_priv *pTAS2555, struct TFirmware * pFirmware)
{
	dev_info(pTAS2555->dev, "FW Size        = %d", pFirmware->mnFWSize);
	dev_info(pTAS2555->dev, "Checksum       = 0x%04X", pFirmware->mnChecksum);
	dev_info(pTAS2555->dev, "PPC Version    = 0x%04X", pFirmware->mnPPCVersion);
	dev_info(pTAS2555->dev, "FW  Version    = 0x%04X", pFirmware->mnFWVersion);
	if(pFirmware->mnPPCVersion >= PPC_WITH_DRIVER_VERSION)
		dev_info(pTAS2555->dev, "Driver Version = 0x%04X", pFirmware->mnDriverVersion);
	dev_info(pTAS2555->dev, "Timestamp      = %d", pFirmware->mnTimeStamp);
	dev_info(pTAS2555->dev, "DDC Name       = %s", pFirmware->mpDDCName);
	dev_info(pTAS2555->dev, "Description    = %s", pFirmware->mpDescription);
}

inline unsigned int fw_convert_number(unsigned char *pData)
{
	return pData[3] + (pData[2] << 8) + (pData[1] << 16) + (pData[0] << 24);
}

static int fw_parse_header(struct tas2555_priv *pTAS2555, 
	struct TFirmware * pFirmware, unsigned char *pData,
	unsigned int nSize)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned char pMagicNumber[] = { 0x35, 0x35, 0x35, 0x32 };
	if (nSize < 102) {
		dev_err(pTAS2555->dev, "Firmware: Header too short");
		return -1;
	}

	if (memcmp(pData, pMagicNumber, 4)) {
		dev_err(pTAS2555->dev, "Firmware: Magic number doesn't match");
		return -1;
	}

	pData += 4;

	pFirmware->mnFWSize = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnChecksum = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnPPCVersion = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnFWVersion = fw_convert_number(pData);
	pData += 4;

	if (pFirmware->mnPPCVersion >= PPC_WITH_DRIVER_VERSION) {
		pFirmware->mnDriverVersion = fw_convert_number(pData);
		pData += 4;
	}

	pFirmware->mnTimeStamp = fw_convert_number(pData);
	pData += 4;

	memcpy(pFirmware->mpDDCName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pFirmware->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	if ((pData - pDataStart) >= nSize) {
		dev_err(pTAS2555->dev, "Firmware: Header too short after DDC description");
		return -1;
	}

	pFirmware->mnDeviceFamily = fw_convert_number(pData);
	pData += 4;

	pFirmware->mnDevice = fw_convert_number(pData);
	pData += 4;

	fw_print_header(pTAS2555, pFirmware);

	return pData - pDataStart;
}

static int fw_parse_block_data(struct tas2555_priv *pTAS2555, struct TFirmware *pFirmware,
	struct TBlock * pBlock, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;

	pBlock->mnType = fw_convert_number(pData);
	pData += 4;

	if (pFirmware->mnDriverVersion >= PPC_DRIVER_VERSION) {
		pBlock->mbPChkSumPresent = pData[0];
		pData++;

		pBlock->mnPChkSum = pData[0];
		pData++;

		pBlock->mbYChkSumPresent = pData[0];
		pData++;

		pBlock->mnYChkSum = pData[0];
		pData++;
	} else {
		pBlock->mbPChkSumPresent = 0;
		pBlock->mbYChkSumPresent = 0;
	}

	pBlock->mnCommands = fw_convert_number(pData);
	pData += 4;

	n = pBlock->mnCommands * 4;
	pBlock->mpData = kmemdup(pData, n, GFP_KERNEL);
	pData += n;

	return pData - pDataStart;
}

static int fw_parse_data(struct tas2555_priv *pTAS2555, struct TFirmware *pFirmware,
	struct TData *pImageData, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int nBlock;
	unsigned int n;

	memcpy(pImageData->mpName, pData, 64);
	pData += 64;

	n = strlen(pData);
	pImageData->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
	pData += n + 1;

	pImageData->mnBlocks = (pData[0] << 8) + pData[1];
	pData += 2;

	pImageData->mpBlocks =
		kmalloc(sizeof(struct TBlock) * pImageData->mnBlocks, GFP_KERNEL);

	for (nBlock = 0; nBlock < pImageData->mnBlocks; nBlock++) {
		n = fw_parse_block_data(pTAS2555, pFirmware, &(pImageData->mpBlocks[nBlock]), pData);
		pData += n;
	}

	return pData - pDataStart;
}

static int fw_parse_pll_data(struct tas2555_priv *pTAS2555,
	struct TFirmware *pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nPLL;
	struct TPLL *pPLL;

	pFirmware->mnPLLs = (pData[0] << 8) + pData[1];
	pData += 2;

	if (pFirmware->mnPLLs == 0)
		goto end;

	pFirmware->mpPLLs = kmalloc(sizeof(struct TPLL) * pFirmware->mnPLLs, GFP_KERNEL);
	for (nPLL = 0; nPLL < pFirmware->mnPLLs; nPLL++) {
		pPLL = &(pFirmware->mpPLLs[nPLL]);

		memcpy(pPLL->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pPLL->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_block_data(pTAS2555, pFirmware, &(pPLL->mBlock), pData);
		pData += n;
	}

end:

	return pData - pDataStart;
}

static int fw_parse_program_data(struct tas2555_priv *pTAS2555,
	struct TFirmware *pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nProgram;
	struct TProgram *pProgram;

	pFirmware->mnPrograms = (pData[0] << 8) + pData[1];
	pData += 2;

	if (pFirmware->mnPrograms == 0)
		goto end;

	pFirmware->mpPrograms =
		kmalloc(sizeof(struct TProgram) * pFirmware->mnPrograms, GFP_KERNEL);
	for (nProgram = 0; nProgram < pFirmware->mnPrograms; nProgram++) {
		pProgram = &(pFirmware->mpPrograms[nProgram]);
		memcpy(pProgram->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pProgram->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		n = fw_parse_data(pTAS2555, pFirmware, &(pProgram->mData), pData);
		pData += n;
	}

end:

	return pData - pDataStart;
}

static int fw_parse_configuration_data(struct tas2555_priv *pTAS2555,
	struct TFirmware *pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nConfiguration;
	struct TConfiguration *pConfiguration;

	pFirmware->mnConfigurations = (pData[0] << 8) + pData[1];
	pData += 2;

	if (pFirmware->mnConfigurations == 0)
		goto end;

	pFirmware->mpConfigurations =
		kmalloc(sizeof(struct TConfiguration) * pFirmware->mnConfigurations,
		GFP_KERNEL);
	for (nConfiguration = 0; nConfiguration < pFirmware->mnConfigurations;
		nConfiguration++) {
		pConfiguration = &(pFirmware->mpConfigurations[nConfiguration]);
		memcpy(pConfiguration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pConfiguration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pConfiguration->mnProgram = pData[0];
		pData++;

		pConfiguration->mnPLL = pData[0];
		pData++;

		pConfiguration->mnSamplingRate = fw_convert_number(pData);
		pData += 4;

		n = fw_parse_data(pTAS2555, pFirmware, &(pConfiguration->mData), pData);
		pData += n;
	}

end:

	return pData - pDataStart;
}

int fw_parse_calibration_data(struct tas2555_priv *pTAS2555,
	struct TFirmware *pFirmware, unsigned char *pData)
{
	unsigned char *pDataStart = pData;
	unsigned int n;
	unsigned int nCalibration;
	struct TCalibration *pCalibration;

	pFirmware->mnCalibrations = (pData[0] << 8) + pData[1];
	pData += 2;

	if (pFirmware->mnCalibrations == 0)
		goto end;

	pFirmware->mpCalibrations =
		kmalloc(sizeof(struct TCalibration) * pFirmware->mnCalibrations, GFP_KERNEL);
	for (nCalibration = 0;
		nCalibration < pFirmware->mnCalibrations;
		nCalibration++) {
		pCalibration = &(pFirmware->mpCalibrations[nCalibration]);
		memcpy(pCalibration->mpName, pData, 64);
		pData += 64;

		n = strlen(pData);
		pCalibration->mpDescription = kmemdup(pData, n + 1, GFP_KERNEL);
		pData += n + 1;

		pCalibration->mnProgram = pData[0];
		pData++;

		pCalibration->mnConfiguration = pData[0];
		pData++;

		n = fw_parse_block_data(pTAS2555, pFirmware, &(pCalibration->mBlock), pData);
		pData += n;
	}

end:

	return pData - pDataStart;
}

static int fw_parse(struct tas2555_priv *pTAS2555,
	struct TFirmware * pFirmware,
	unsigned char *pData,
	unsigned int nSize)
{
	int nPosition = 0;

	nPosition = fw_parse_header(pTAS2555, pFirmware, pData, nSize);
	if (nPosition < 0) {
		dev_err(pTAS2555->dev, "Firmware: Wrong Header");
		return FW_ERR_HEADER;
	}

	if (nPosition >= nSize) {
		dev_err(pTAS2555->dev, "Firmware: Too short");
		return FW_ERR_SIZE;
	}

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_pll_data(pTAS2555, pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_program_data(pTAS2555, pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	nPosition = fw_parse_configuration_data(pTAS2555, pFirmware, pData);

	pData += nPosition;
	nSize -= nPosition;
	nPosition = 0;

	if (nSize > 64)
		nPosition = fw_parse_calibration_data(pTAS2555, pFirmware, pData);

	return 0;
}

static const unsigned char crc8_lookup_table[CRC8_TABLE_SIZE] = {
0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C, 
0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5, 
0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43, 
0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA, 
0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62, 
0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB, 
0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D, 
0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4, 
0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20, 
0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89, 
0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F, 
0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96, 
0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E, 
0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7, 
0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01, 
0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8 
};

static int isYRAM(struct tas2555_priv *pTAS2555, struct TYCRC *pCRCData, 
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned char len)
{
	int result = -1;

	if (nBook == TAS2555_YRAM_BOOK) {
		if (nPage == TAS2555_YRAM1_PAGE) {
			if ((nReg >= TAS2555_YRAM1_START_REG)
				&&(nReg <= TAS2555_YRAM1_END_REG)) {
				if ((nReg + len -1) <= TAS2555_YRAM1_END_REG) {
					result = 1;
					if (pCRCData != NULL) {
						pCRCData->mnOffset = nReg;
						pCRCData->mnLen = len;
					}
				} else {
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
					dev_err(pTAS2555->dev, "nReg 0x%x error, len %d\n", nReg, len);
				}
			} else if (nReg > TAS2555_YRAM1_END_REG) {
				pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				dev_err(pTAS2555->dev, "nReg 0x%x error\n", nReg);
			} else if (len > 1) {
				if ((nReg + (len-1))> TAS2555_YRAM1_END_REG) {
					dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				} else if ((nReg + (len-1)) >= TAS2555_YRAM1_START_REG) {
					result = 1;
					if (pCRCData != NULL) {
						pCRCData->mnOffset = TAS2555_YRAM1_START_REG;
						pCRCData->mnLen = nReg + len - TAS2555_YRAM1_START_REG;
					}
				} else 
					result = 0;
			} else 
				result = 0;
		} else if ((nPage >= TAS2555_YRAM2_START_PAGE)
			&& (nPage <= TAS2555_YRAM2_END_PAGE)) {
			if (nReg > TAS2555_YRAM2_END_REG) {
				dev_err(pTAS2555->dev, "nReg 0x%x error\n", nReg);
				pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
			} else if ((nReg >= TAS2555_YRAM2_START_REG)
				&& (nReg <= TAS2555_YRAM2_END_REG)) {
				if ((nReg + len -1) <= TAS2555_YRAM2_END_REG) {
					result = 1;
					if (pCRCData != NULL) {
						pCRCData->mnOffset = nReg;
						pCRCData->mnLen = len;
					}
				} else {
					dev_err(pTAS2555->dev, "nReg 0x%x error, len %d\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				}
			} else if (len > 1) {
				if ((nReg + (len-1)) > TAS2555_YRAM2_END_REG) {
					dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				} else if ((nReg + (len-1)) >= TAS2555_YRAM2_START_REG) {
					result = 1;
					if (pCRCData != NULL) {
						pCRCData->mnOffset = TAS2555_YRAM2_START_REG;
						pCRCData->mnLen = nReg + len - TAS2555_YRAM1_START_REG;
					}
				} else
					result = 0;
			}else
				result = 0;
		} else if (nPage == TAS2555_YRAM3_PAGE) {
			if (nReg > TAS2555_YRAM2_END_REG) {
				dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
				pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
			} else if (nReg > TAS2555_YRAM3_END_REG)
				result = 0;
			else if (nReg >= TAS2555_YRAM3_START_REG) {
				if ((nReg + len -1) <= TAS2555_YRAM3_END_REG) {
					if (pCRCData != NULL) {
						pCRCData->mnOffset = nReg;
						pCRCData->mnLen = len;
					}
					result = 1;
				} else if ((nReg + len -1) <= TAS2555_YRAM2_END_REG) {
					if (pCRCData != NULL) {
						pCRCData->mnOffset = nReg;
						pCRCData->mnLen = TAS2555_YRAM3_END_REG - nReg + 1;
					}
					result = 1;
				} else {
					dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				}
			} else if (len > 1) {
				if ((nReg + (len-1)) > TAS2555_YRAM2_END_REG) {
					dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
				} else if ((nReg + (len-1)) >= TAS2555_YRAM3_START_REG) {
					if ((nReg + len -1) <= TAS2555_YRAM3_END_REG) {
						if (pCRCData != NULL) {
							pCRCData->mnOffset = TAS2555_YRAM3_START_REG;
							pCRCData->mnLen = nReg + len - TAS2555_YRAM3_START_REG;
						}
						result = 1;
					} else if ((nReg + len -1) <= TAS2555_YRAM2_END_REG) {
						if (pCRCData != NULL) {
							pCRCData->mnOffset = TAS2555_YRAM3_START_REG;
							pCRCData->mnLen = TAS2555_YRAM3_END_REG - TAS2555_YRAM3_START_REG + 1;
						}
						result = 1;
					} else {
						pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
						dev_err(pTAS2555->dev, "nReg 0x%x, len %d error\n", nReg, len);
					}
				} else 
					result = 0;
			} else 
				result = 0;
		} else 
			result = 0;
	} else
		result = 0;

	return result;
}

/*
 * crc8 - calculate a crc8 over the given input data.
 *
 * table: crc table used for calculation.
 * pdata: pointer to data buffer.
 * nbytes: number of bytes in data buffer.
 * crc:	previous returned crc8 value.
 */
static u8 ti_crc8(const u8 table[CRC8_TABLE_SIZE], u8 *pdata, size_t nbytes, u8 crc)
{
	/* loop over the buffer data */
	while (nbytes-- > 0)
		crc = table[(crc ^ *pdata++) & 0xff];

	return crc;
}

static int doSingleRegCheckSum(struct tas2555_priv *pTAS2555,
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned char nValue)
{
	int nResult = -1;
	unsigned int nData = 0;
	unsigned char nRegVal = 0;

	nResult = isYRAM(pTAS2555, NULL, nBook, nPage, nReg, 1);
	if (nResult < 0) {
		dev_err(pTAS2555->dev, "firmware error\n");
		goto end;
	} else if (nResult == 1) {
		nResult = pTAS2555->read(pTAS2555, TAS2555_REG(nBook, nPage, nReg), &nData);
		if (nResult < 0) {
			goto end;
		}
		nRegVal = (unsigned char)nData;
		if(nValue != nRegVal){
			dev_err(pTAS2555->dev, 
				"error (line %d),B[0x%x]P[0x%x]R[0x%x] W[0x%x], R[0x%x]\n", 
				__LINE__, nBook, nPage, nReg, nValue, nRegVal);
			nResult = -EAGAIN;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_VALUENOTMATCH;
			goto end;
		}

		nResult = ti_crc8(crc8_lookup_table, &nRegVal, 1, 0);
	}

end:

	return nResult;
}

static int doMultiRegCheckSum(struct tas2555_priv *pTAS2555,
	unsigned char nBook, unsigned char nPage, unsigned char nReg, unsigned int len)
{
	int nResult = -1, i;
	unsigned char nCRCChkSum = 0;
	unsigned char nBuf[127];
	struct TYCRC TCRCData;

	if ((nReg + len-1) > 127) {
		dev_err(pTAS2555->dev, "firmware error\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
		goto err;
	}

	nResult = isYRAM(pTAS2555, &TCRCData, nBook, nPage, nReg, len);
	if (nResult < 0) {
		dev_err(pTAS2555->dev, "firmware error\n");
		goto err;
	} else if (nResult == 1) {
		if (len == 1) {
			/* here shouldn't happen */
			dev_err(pTAS2555->dev, "firmware error\n");
			goto err;
		} else {
			nResult = pTAS2555->bulk_read(pTAS2555, 
				TAS2555_REG(nBook, nPage, TCRCData.mnOffset), nBuf, TCRCData.mnLen);  
			if(nResult < 0) {
				goto err;
			}
			for (i=0; i < TCRCData.mnLen; i++) {
				nCRCChkSum += ti_crc8(crc8_lookup_table, &nBuf[i], 1, 0);
			}
			nResult = nCRCChkSum;
		}
	}

err:

	return nResult;
}

static int tas2555_load_block(struct tas2555_priv *pTAS2555, struct TBlock * pBlock)
{
	int nResult = 0;
	int nRetry = 6;
	unsigned int nCommand = 0;
	unsigned char nBook;
	unsigned char nPage;
	unsigned char nOffset;
	unsigned char nData;
	unsigned int nLength;
	unsigned int nValue = 0;
	unsigned char nCRCChkSum = 0;
	unsigned char *pData = pBlock->mpData;

	dev_dbg(pTAS2555->dev, "TAS2555 load block: Type = %d, commands = %d\n",
		pBlock->mnType, pBlock->mnCommands);

start:
	if (pBlock->mbPChkSumPresent) {
		nResult = pTAS2555->write(pTAS2555, TAS2555_CRC_RESET_REG, 1);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "I2C err\n");
			goto end;
		}
		pTAS2555->mnErrorCode &= ~TAS2555_ERROR_PCHKSUM;
	}

	if (pBlock->mbYChkSumPresent) {
		nCRCChkSum = 0;
		pTAS2555->mnErrorCode &= ~TAS2555_ERROR_YCHKSUM;
	}

	nCommand = 0;

	while (nCommand < pBlock->mnCommands) {
		pData = pBlock->mpData + nCommand * 4;
		nBook = pData[0];
		nPage = pData[1];
		nOffset = pData[2];
		nData = pData[3];

		nCommand++;

		if (nOffset <= 0x7F){
			nResult = pTAS2555->write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset), nData);
			if (nResult < 0) {
				goto end;
			}
			if (pBlock->mbYChkSumPresent) {
				nResult = doSingleRegCheckSum(pTAS2555, nBook, nPage, nOffset, nData);
				if (nResult < 0)
					goto check;
				nCRCChkSum += (unsigned char)nResult;
			}
		} else if (nOffset == 0x81) {
			unsigned int nSleep = (nBook << 8) + nPage;
			msleep(nSleep);
		} else if (nOffset == 0x85) {
			pData += 4;
			nLength = (nBook << 8) + nPage;
			nBook = pData[0];
			nPage = pData[1];
			nOffset = pData[2];
			if (nLength > 1) {
				nResult = pTAS2555->bulk_write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset), pData + 3, nLength);
				if (nResult < 0) {
					goto end;
				}
				if (pBlock->mbYChkSumPresent) {
					nResult = doMultiRegCheckSum(pTAS2555, nBook, nPage, nOffset, nLength);
					if (nResult < 0)
						goto check;
					nCRCChkSum += (unsigned char)nResult;
				}
			} else {
				nResult = pTAS2555->write(pTAS2555, TAS2555_REG(nBook, nPage, nOffset), pData[3]);
				if (nResult < 0) {
					goto end;
				}
				if (pBlock->mbYChkSumPresent) {
					nResult = doSingleRegCheckSum(pTAS2555, nBook, nPage, nOffset, pData[3]);
					if (nResult < 0)
						goto check;
					nCRCChkSum += (unsigned char)nResult;
				}
			}
			nCommand++;
			if (nLength >= 2)
				nCommand += ((nLength - 2) / 4) + 1;
		}
	}

	if (pBlock->mbPChkSumPresent) {
		pTAS2555->read(pTAS2555, TAS2555_CRC_CHECKSUM_REG, &nValue);
		if ((nValue&0xff) != pBlock->mnPChkSum) {
			dev_err(pTAS2555->dev, "Block PChkSum Error: FW = 0x%x, Reg = 0x%x\n",
				pBlock->mnPChkSum, (nValue&0xff));
			nResult = -EAGAIN;
			pTAS2555->mnErrorCode |= TAS2555_ERROR_PCHKSUM;
			goto check;
		} else {
			nResult = 0;
			dev_dbg(pTAS2555->dev, "Block[0x%x] PChkSum match\n", pBlock->mnType);
		}
	}
	
	if (pBlock->mbYChkSumPresent) {
		if (nCRCChkSum != pBlock->mnYChkSum) {
			dev_err(pTAS2555->dev, "Block YChkSum Error: FW = 0x%x, YCRC = 0x%x\n",
			pBlock->mnYChkSum, nCRCChkSum);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_YCHKSUM;
			nResult = -EAGAIN;
			goto check;
		} else {
			nResult = 0;
			dev_dbg(pTAS2555->dev, "Block[0x%x] YChkSum match\n", pBlock->mnType);
		}
	}

check:
	if (nResult == -EAGAIN) {
		nRetry--;
		if (nRetry > 0)
			goto start;
	}

end:
	if (nResult < 0) {
		dev_err(pTAS2555->dev, "Block (%d) load error\n",
				pBlock->mnType);
	}
	return nResult;
}

static int tas2555_load_data(struct tas2555_priv *pTAS2555, struct TData * pData,
	unsigned int nType)
{
	unsigned int nBlock;
	int nResult = 0;
	struct TBlock *pBlock;

	dev_dbg(pTAS2555->dev,
		"TAS2555 load data: %s, Blocks = %d, Block Type = %d\n", pData->mpName,
		pData->mnBlocks, nType);

	for (nBlock = 0; nBlock < pData->mnBlocks; nBlock++) {
		pBlock = &(pData->mpBlocks[nBlock]);
		if (pBlock->mnType == nType) {
			nResult = tas2555_load_block(pTAS2555, pBlock);
			if (nResult < 0)
				break;
		}
	}

	return nResult;
}

static int tas2555_load_configuration(struct tas2555_priv *pTAS2555,
	unsigned int nConfiguration, bool bLoadSame)
{
	int nResult = 0;
	struct TConfiguration *pCurrentConfiguration;
	struct TConfiguration *pNewConfiguration;
	struct TPLL *pNewPLL;

	dev_dbg(pTAS2555->dev, "tas2555_load_configuration: %d\n", nConfiguration);

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;
		return -1;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	if ((nConfiguration == pTAS2555->mnCurrentConfiguration) && (!bLoadSame)) {
		dev_info(pTAS2555->dev, "Configuration %d is already loaded\n",
			nConfiguration);
		return 0;
	}

	pCurrentConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration]);
	pNewConfiguration =
		&(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);

	if (pNewConfiguration->mnProgram != pCurrentConfiguration->mnProgram) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s doesn't share the same program as current %d\n",
			nConfiguration, pNewConfiguration->mpName, pCurrentConfiguration->mnProgram);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	if (pNewConfiguration->mnPLL >= pTAS2555->mpFirmware->mnPLLs) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s doesn't have a valid PLL index %d\n",
			nConfiguration, pNewConfiguration->mpName, pNewConfiguration->mnPLL);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
		return -1;
	}

	pNewPLL = &(pTAS2555->mpFirmware->mpPLLs[pNewConfiguration->mnPLL]);

	if (pTAS2555->mbPowerUp) {
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up -> mute and power down DSP before loading new configuration\n");
			//tas2555_dev_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_shutdown_data);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			nResult = tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			if (nResult < 0)
				goto end;
			pTAS2555->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev, "TAS2555: power up TAS2555\n");
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, post power up block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_POST_POWER);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev,
				"TAS2555: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev, "TAS2555: unmute TAS2555\n");
			nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
			if (nResult < 0)
				goto end;
		} else {
			dev_dbg(pTAS2555->dev,
				"TAS2555 is powered up, no change in PLL: load new configuration: %s, coeff block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_COEFF);
			if (nResult < 0)
				goto end;
		}

		if (pTAS2555->mpCalFirmware->mnCalibrations) {
				dev_dbg(pTAS2555->dev, "Enable: load calibration\n");
				nResult = tas2555_load_block(pTAS2555, 
					&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));
				pTAS2555->mbLoadCalibrationPostPowerUp = false;
				if (nResult < 0)
					goto end;
		}

		pTAS2555->mbLoadConfigurationPostPowerUp = false;
	} else {
		dev_dbg(pTAS2555->dev,
			"TAS2555 was powered down -> set flag to load configuration data when OS powers up the TAS2555 the next time\n");
		if (pNewConfiguration->mnPLL != pCurrentConfiguration->mnPLL) {
			dev_dbg(pTAS2555->dev,
				"load post block from current configuration: %s, before loading new configuration: %s\n",
				pCurrentConfiguration->mpName, pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pCurrentConfiguration->mData),
				TAS2555_BLOCK_CONF_POST);
			if (nResult < 0)
				goto end;
			dev_dbg(pTAS2555->dev, "TAS2555: load new PLL: %s, block data\n",
				pNewPLL->mpName);
			nResult = tas2555_load_block(pTAS2555, &(pNewPLL->mBlock));
			if (nResult < 0)
				goto end;
			pTAS2555->mnCurrentSampleRate = pNewConfiguration->mnSamplingRate;
			dev_dbg(pTAS2555->dev,
				"load new configuration: %s, pre block data\n",
				pNewConfiguration->mpName);
			nResult = tas2555_load_data(pTAS2555, &(pNewConfiguration->mData),
				TAS2555_BLOCK_CONF_PRE);
			if (nResult < 0)
				goto end;
		}

		pTAS2555->mbLoadConfigurationPostPowerUp = true;
	}

	pTAS2555->mnCurrentConfiguration = nConfiguration;

end:

	if (nResult < 0)
		failsafe(pTAS2555);

	return nResult;
}

int tas2555_set_config(struct tas2555_priv *pTAS2555, int config)
{
	struct TConfiguration *pConfiguration;
	struct TProgram *pProgram;
	unsigned int nProgram = pTAS2555->mnCurrentProgram;
	unsigned int nConfiguration = config;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;
		return -1;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, "Configuration %d doesn't exist\n",
			nConfiguration);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
	pProgram = &(pTAS2555->mpFirmware->mpPrograms[nProgram]);

	if (nProgram != pConfiguration->mnProgram) {
		dev_err(pTAS2555->dev,
			"Configuration %d, %s with Program %d isn't compatible with existing Program %d, %s\n",
			nConfiguration, pConfiguration->mpName, pConfiguration->mnProgram,
			nProgram, pProgram->mpName);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	tas2555_load_configuration(pTAS2555, nConfiguration, false);

	return 0;
}

void tas2555_clear_firmware(struct TFirmware *pFirmware)
{
	unsigned int n, nn;

	if (!pFirmware)
		return;
	if (pFirmware->mpDescription)
		kfree(pFirmware->mpDescription);

	if (pFirmware->mnPLLs) {
		for (n = 0; n < pFirmware->mnPLLs; n++) {
			kfree(pFirmware->mpPLLs[n].mpDescription);
			kfree(pFirmware->mpPLLs[n].mBlock.mpData);
		}
		kfree(pFirmware->mpPLLs);
	}

	if (pFirmware->mnPrograms) {
		for (n = 0; n < pFirmware->mnPrograms; n++) {
			kfree(pFirmware->mpPrograms[n].mpDescription);
			kfree(pFirmware->mpPrograms[n].mData.mpDescription);
			for (nn = 0; nn < pFirmware->mpPrograms[n].mData.mnBlocks; nn++)
				kfree(pFirmware->mpPrograms[n].mData.mpBlocks[nn].mpData);
			kfree(pFirmware->mpPrograms[n].mData.mpBlocks);
		}
		kfree(pFirmware->mpPrograms);
	}

	if (pFirmware->mnConfigurations) {
		for (n = 0; n < pFirmware->mnConfigurations; n++) {
			kfree(pFirmware->mpConfigurations[n].mpDescription);
			kfree(pFirmware->mpConfigurations[n].mData.mpDescription);
			for (nn = 0; nn < pFirmware->mpConfigurations[n].mData.mnBlocks; nn++)
				kfree(pFirmware->mpConfigurations[n].mData.mpBlocks[nn].mpData);
			kfree(pFirmware->mpConfigurations[n].mData.mpBlocks);
		}
		kfree(pFirmware->mpConfigurations);
	}

	if (pFirmware->mnCalibrations) {
		for (n = 0; n < pFirmware->mnCalibrations; n++) {
			kfree(pFirmware->mpCalibrations[n].mpDescription);
			kfree(pFirmware->mpCalibrations[n].mBlock.mpData);
		}
		kfree(pFirmware->mpCalibrations);
	}

	memset(pFirmware, 0x00, sizeof(struct TFirmware));
}

static int tas2555_load_calibration(struct tas2555_priv *pTAS2555,
	char *pFileName)
{
	int nResult;
	int nFile;
	mm_segment_t fs;
	unsigned char pBuffer[512];
	int nSize = 0;

	dev_dbg(pTAS2555->dev, "%s:\n", __func__);

	fs = get_fs();
	set_fs(KERNEL_DS);
	nFile = sys_open(pFileName, O_RDONLY, 0);

	dev_info(pTAS2555->dev, "TAS2555 calibration file = %s, handle = %d\n",
		pFileName, nFile);

	if (nFile >= 0) {
		nSize = sys_read(nFile, pBuffer, 512);
		sys_close(nFile);
	} else {
		dev_err(pTAS2555->dev, "TAS2555 cannot open calibration file: %s\n",
			pFileName);
	}

	set_fs(fs);

	if (!nSize)
		return -1;

	tas2555_clear_firmware(pTAS2555->mpCalFirmware);
		
	dev_info(pTAS2555->dev, "TAS2555 calibration file size = %d\n", nSize);
	nResult = fw_parse(pTAS2555, pTAS2555->mpCalFirmware, pBuffer, nSize);

	if (nResult) {
		dev_err(pTAS2555->dev, "TAS2555 calibration file is corrupt\n");
		return -1;
	}

	dev_info(pTAS2555->dev, "TAS2555 calibration: %d calibrations\n",
		pTAS2555->mpCalFirmware->mnCalibrations);
		
	return 0;
}

void tas2555_fw_ready(const struct firmware *pFW, void *pContext)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *) pContext;
	int nResult;
	unsigned int nProgram = 0;
	int nConfiguration = 0;
	unsigned int nSampleRate = 0;

	dev_info(pTAS2555->dev, "%s:\n", __func__);

	if (unlikely(!pFW) || unlikely(!pFW->data)) {
		dev_err(pTAS2555->dev, "%s firmware is not loaded\n", TAS2555_FW_NAME);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;
		return;
	}

	nProgram = pTAS2555->mnCurrentProgram;
	nSampleRate = pTAS2555->mnCurrentSampleRate;
	nConfiguration = pTAS2555->mnCurrentConfiguration;

	if (pTAS2555->mpFirmware->mpConfigurations) {
		dev_dbg(pTAS2555->dev, "clear current firmware\n");
		tas2555_clear_firmware(pTAS2555->mpFirmware);
	}
	
	nResult = fw_parse(pTAS2555, pTAS2555->mpFirmware, 
		(unsigned char *) (pFW->data), pFW->size);

	release_firmware(pFW);

	if (nResult) {
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
		dev_err(pTAS2555->dev, "firmware is corrupt\n");
		return;
	}

	if (!pTAS2555->mpFirmware->mnPrograms) {
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
		dev_err(pTAS2555->dev, "firmware contains no programs\n");
		return;
	}

	if (!pTAS2555->mpFirmware->mnConfigurations) {
		dev_err(pTAS2555->dev, 
			"firmware contains no configurations\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
		return;
	}

	if (nProgram >= pTAS2555->mpFirmware->mnPrograms) {
		dev_info(pTAS2555->dev, 
			"no previous program, set to default\n");
		nProgram = 0;
	}

	if (nConfiguration >= pTAS2555->mpFirmware->mnConfigurations) {
		dev_info(pTAS2555->dev, 
			"no previous program, set to default\n");
		nConfiguration = -1;
	}

	if (nConfiguration >= 0) {
		if (pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnProgram != nProgram)
			nConfiguration = -1;
	}

	if (nConfiguration >= 0) {
		if (pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnSamplingRate != nSampleRate)
			nConfiguration = -1;
	}

	tas2555_set_program(pTAS2555, nProgram, nConfiguration);
}

int tas2555_set_program(struct tas2555_priv *pTAS2555, unsigned int nProgram, int nConfig)
{
	struct TPLL *pPLL;
	struct TConfiguration *pConfiguration;
	unsigned int nConfiguration = 0;
	unsigned int nSampleRate = 0;
	unsigned int Value = 0;
	bool bFound = false;
	int nResult = -1;

	if ((!pTAS2555->mpFirmware->mpPrograms) ||
		(!pTAS2555->mpFirmware->mpConfigurations)) {
		dev_err(pTAS2555->dev, "Firmware not loaded\n");
		pTAS2555->mnErrorCode |= TAS2555_ERROR_FWNOTLOAD;
		return -1;
	}
	
	if (nProgram >= pTAS2555->mpFirmware->mnPrograms) {
		dev_err(pTAS2555->dev, "TAS2555: Program %d doesn't exist\n",
			nConfiguration);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	if (nConfig < 0) {
		nConfiguration = 0;
		nSampleRate = pTAS2555->mnCurrentSampleRate;

		while (!bFound 
			&& (nConfiguration < pTAS2555->mpFirmware->mnConfigurations)) {
			if (pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnProgram 
				== nProgram){
				if (nSampleRate == 0){
					bFound = true;
					dev_info(pTAS2555->dev, "find default configuration %d\n", nConfiguration);
				} else if (nSampleRate 
					== pTAS2555->mpFirmware->mpConfigurations[nConfiguration].mnSamplingRate){
					bFound = true;
					dev_info(pTAS2555->dev, "find matching configuration %d\n", nConfiguration);
				} else {
					nConfiguration++;
				}
			} else {
				nConfiguration++;
			}
		}
		if (!bFound) {
			dev_err(pTAS2555->dev, 
				"Program %d, no valid configuration found for sample rate %d, ignore\n",
				nProgram, nSampleRate);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_FWFORMAT;
			return -1;
		}
	} else
		nConfiguration = nConfig;

	pTAS2555->mnCurrentProgram = nProgram;

	if (pTAS2555->mbPowerUp) {
		nResult = pTAS2555->enableIRQ(pTAS2555, false, true);
		if (nResult < 0)
			goto end;
		nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_mute_DSP_down_data);
		if (nResult < 0)
			goto end;
	}

	nResult = pTAS2555->write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);
	if (nResult < 0) {
		goto end;
	}

	msleep(1);
	nResult = tas2555_load_default(pTAS2555);
	if (nResult < 0)
		goto end;

	dev_info(pTAS2555->dev, "load program %d\n", nProgram);
	nResult = tas2555_load_data(pTAS2555, &(pTAS2555->mpFirmware->mpPrograms[nProgram].mData), TAS2555_BLOCK_BASE_MAIN);
	if (nResult < 0)
		goto end;

	pTAS2555->mnCurrentConfiguration = nConfiguration;

	pConfiguration = &(pTAS2555->mpFirmware->mpConfigurations[nConfiguration]);
	pPLL = &(pTAS2555->mpFirmware->mpPLLs[pConfiguration->mnPLL]);
	dev_dbg(pTAS2555->dev, "TAS2555 load PLL: %s block for Configuration %s\n", pPLL->mpName, pConfiguration->mpName);
	
	nResult = tas2555_load_block(pTAS2555, &(pPLL->mBlock));
	if (nResult < 0)
		goto end;
	pTAS2555->mnCurrentSampleRate = pConfiguration->mnSamplingRate;
	dev_dbg(pTAS2555->dev,
		"load configuration %s conefficient pre block\n",
		pConfiguration->mpName);
	nResult = tas2555_load_data(pTAS2555, &(pConfiguration->mData), TAS2555_BLOCK_CONF_PRE);
	if(nResult < 0)
		goto end;
	nResult = pTAS2555->read(pTAS2555, TAS2555_PLL_CLKIN_REG, &Value);
	if(nResult < 0) {
		goto end;
	}
	dev_info(pTAS2555->dev, "TAS2555 PLL_CLKIN = 0x%02X\n", Value);
	p_tas2555_startup_data[TAS2555_STARTUP_DATA_PLL_CLKIN_INDEX] = Value;

	if (pTAS2555->mbPowerUp){
		dev_dbg(pTAS2555->dev, "device powered up, load startup\n");
		nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_startup_data);
		if(nResult < 0)
			goto end;
		dev_dbg(pTAS2555->dev, 
			"device powered up, load configuration %s post power block\n",
			pConfiguration->mpName);
		nResult = tas2555_load_data(pTAS2555, &(pConfiguration->mData),
			TAS2555_BLOCK_CONF_POST_POWER);
		if(nResult < 0)
			goto end;
	}
	
	nResult = tas2555_load_configuration(pTAS2555, nConfiguration, true);
	if (nResult < 0)
		goto end;

	if (pTAS2555->mbPowerUp){
		dev_dbg(pTAS2555->dev,
			"device powered up, load unmute\n");
		nResult = tas2555_dev_load_data(pTAS2555, p_tas2555_unmute_data);
		if (nResult < 0)
			goto end;
	}

end:

	if (nResult < 0)
		failsafe(pTAS2555);

	return nResult;
}

int tas2555_set_calibration(struct tas2555_priv *pTAS2555,
	int nCalibration)
{
	int nResult = 0;
	if ((!pTAS2555->mpFirmware->mpPrograms) || (!pTAS2555->mpFirmware->mpConfigurations)) 
	{
		dev_err(pTAS2555->dev, "Firmware not loaded\n\r");
		return -1;
	}

	if (nCalibration == 0x00FF)
	{
		dev_info(pTAS2555->dev, "load new calibration file %s\n", TAS2555_CAL_NAME); 	
		tas2555_load_calibration(pTAS2555, TAS2555_CAL_NAME);
		nCalibration = 0;
	}

	if (nCalibration >= pTAS2555->mpFirmware->mnCalibrations) {
		dev_err(pTAS2555->dev,
			"Calibration %d doesn't exist\n", nCalibration);
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
		return -1;
	}

	pTAS2555->mnCurrentCalibration = nCalibration;
	if(pTAS2555->mbPowerUp){
		nResult = tas2555_load_block(pTAS2555, 
			&(pTAS2555->mpCalFirmware->mpCalibrations[pTAS2555->mnCurrentCalibration].mBlock));
		if (nResult >= 0) {
			pTAS2555->mbLoadCalibrationPostPowerUp = false; 
			nResult = tas2555_get_ReCoefficient(pTAS2555, &pTAS2555->mnReOrignal);
		}
	}else{
		pTAS2555->mbLoadCalibrationPostPowerUp = true; 
	}

	return nResult;
}

int tas2555_parse_dt(struct device *dev, struct tas2555_priv *pTAS2555)
{
	struct device_node *np = dev->of_node;
	int ret = 0;

	pTAS2555->mnResetGPIO = of_get_named_gpio(np, "ti,reset-gpio", 0);
	if (pTAS2555->mnResetGPIO < 0) {
		dev_err(pTAS2555->dev, "Looking up %s property in node %s failed %d\n",
			"ti,reset-gpio", np->full_name, pTAS2555->mnResetGPIO);
		ret = -EINVAL;
		pTAS2555->mnErrorCode |= TAS2555_ERROR_INVALIDPARAM;
	} else
		dev_dbg(pTAS2555->dev, "ti,reset-gpio=%d\n", pTAS2555->mnResetGPIO);

	if (ret >= 0) {
		pTAS2555->mnGpioINT = of_get_named_gpio(np, "ti,irq-gpio", 0);
		if (pTAS2555->mnGpioINT < 0)
			dev_err(pTAS2555->dev, "Looking up %s property in node %s failed %d\n",
				"ti,irq-gpio", np->full_name, pTAS2555->mnGpioINT);
		else
			dev_dbg(pTAS2555->dev, "ti,irq-gpio=%d\n", pTAS2555->mnGpioINT);
	}

	return ret;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2555 common functions for Android Linux");
MODULE_LICENSE("GPLv2");
