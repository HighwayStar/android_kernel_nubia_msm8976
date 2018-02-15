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
**     tas2555-codec.c
**
** Description:
**     ALSA SoC driver for Texas Instruments TAS2555 High Performance 4W Smart Amplifier
**
** =============================================================================
*/

#ifdef CONFIG_TAS2555_CODEC

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
#include <asm/uaccess.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "tas2555-core.h"
#include "tas2555-codec.h"

//#define KCONTROL_CODEC

static struct tas2555_register register_addr = { 0 };

#define TAS2555_REG_IS_VALID(book, page, reg) \
        ((book >= 0) && (book <= 255) &&\
        (page >= 0) && (page <= 255) &&\
        (reg >= 0) && (reg <= 127))

static unsigned int tas2555_codec_read(struct snd_soc_codec *pCodec,
	unsigned int nRegister)
{
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);
	int ret = 0;
	unsigned int Value = 0;

	ret = pTAS2555->read(pTAS2555, nRegister, &Value);
	if (ret < 0) {
		dev_err(pTAS2555->dev, "%s, %d, ERROR happen=%d\n", __FUNCTION__,
			__LINE__, ret);
		return 0;
	} else
		return Value;
}

static int tas2555_codec_write(struct snd_soc_codec *pCodec, unsigned int nRegister,
	unsigned int nValue)
{
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);
	int ret = 0;

	ret = pTAS2555->write(pTAS2555, nRegister, nValue);

	return ret;
}

static const struct snd_soc_dapm_widget tas2555_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("ASI1", "ASI1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASI2", "ASI2 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("ASIM", "ASIM Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("DAC", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUT_DRV("ClassD", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_SUPPLY("PLL", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("NDivider", SND_SOC_NOPM, 0, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("OUT")
};

static const struct snd_soc_dapm_route tas2555_audio_map[] = {
	{"DAC", NULL, "ASI1"},
	{"DAC", NULL, "ASI2"},
	{"DAC", NULL, "ASIM"},
	{"ClassD", NULL, "DAC"},
	{"OUT", NULL, "ClassD"},
	{"DAC", NULL, "PLL"},
	{"DAC", NULL, "NDivider"},
};

static int tas2555_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	
	dev_dbg(pTAS2555->dev, "%s\n", __func__);

	return 0;
}

static void tas2555_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "%s\n", __func__);
}

static int tas2555_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "%s\n", __func__);
	tas2555_enable(pTAS2555, !mute);

	return 0;
}

static int tas2555_set_dai_sysclk(struct snd_soc_dai *pDAI,
	int nClkID, unsigned int nFreqency, int nDir)
{
	struct snd_soc_codec *pCodec = pDAI->codec;
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);

	dev_dbg(pTAS2555->dev, "tas2555_set_dai_sysclk: freq = %u\n", nFreqency);

	return 0;
}

static int tas2555_hw_params(struct snd_pcm_substream *pSubstream,
	struct snd_pcm_hw_params *pParams, struct snd_soc_dai *pDAI)
{
	struct snd_soc_codec *pCodec = pDAI->codec;
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);

	dev_dbg(pCodec->dev, "%s\n", __func__);

	tas2555_set_sampling_rate(pTAS2555, params_rate(pParams));

	return 0;
}

static int tas2555_set_dai_fmt(struct snd_soc_dai *pDAI, unsigned int nFormat)
{
	struct snd_soc_codec *codec = pDAI->codec;
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "%s\n", __func__);

	return 0;
}

static int tas2555_prepare(struct snd_pcm_substream *pSubstream,
	struct snd_soc_dai *pDAI)
{
	struct snd_soc_codec *codec = pDAI->codec;
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "%s\n", __func__);

	return 0;
}

static int tas2555_set_bias_level(struct snd_soc_codec *pCodec,
	enum snd_soc_bias_level eLevel)
{
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);

	dev_dbg(pTAS2555->dev, "%s: %d\n", __func__, eLevel);

	return 0;
}

static int tas2555_codec_probe(struct snd_soc_codec *pCodec)
{
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(pCodec);

	dev_info(pTAS2555->dev, "%s\n", __func__);

	return 0;
}

static int tas2555_codec_remove(struct snd_soc_codec *pCodec)
{
	return 0;
}

static int tas2555_get_reg_addr(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pUcontrol->value.integer.value[0] = register_addr.book;
	pUcontrol->value.integer.value[1] = register_addr.page;
	pUcontrol->value.integer.value[2] = register_addr.reg;

	dev_dbg(pTAS2555->dev, "%s: Get address [%d, %d, %d]\n", __func__,
		register_addr.book, register_addr.page, register_addr.reg);

	return 0;
}

static int tas2555_put_reg_addr(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	register_addr.book = pUcontrol->value.integer.value[0];
	register_addr.page = pUcontrol->value.integer.value[1];
	register_addr.reg = pUcontrol->value.integer.value[2];

	dev_dbg(pTAS2555->dev, "%s: Set address [%d, %d, %d]\n", __func__,
		register_addr.book, register_addr.page, register_addr.reg);

	return 0;
}

static int tas2555_get_reg_value(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int reg;
	unsigned int nValue;

	if (TAS2555_REG_IS_VALID(register_addr.book,
			register_addr.page, register_addr.reg)) {
		reg = TAS2555_REG((unsigned int) register_addr.book,
			(unsigned int) register_addr.page,
			(unsigned int) register_addr.reg);
		pTAS2555->read(pTAS2555, reg, &nValue);
		pUcontrol->value.integer.value[0] = nValue;
	} else {
		dev_err(pTAS2555->dev, "%s: Invalid register address!\n", __func__);
		pUcontrol->value.integer.value[0] = 0xFFFF;
	}

	dev_dbg(pTAS2555->dev, "%s: Read [%d, %d, %d] = %ld\n", __func__,
		register_addr.book, register_addr.page, register_addr.reg,
		pUcontrol->value.integer.value[0]);

	return 0;
}

static int tas2555_put_reg_value(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pUcontrol)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int reg;

	if (TAS2555_REG_IS_VALID(register_addr.book,
			register_addr.page, register_addr.reg)) {
		reg = TAS2555_REG((unsigned int) register_addr.book,
			(unsigned int) register_addr.page,
			(unsigned int) register_addr.reg);
		pTAS2555->write(pTAS2555, reg, pUcontrol->value.integer.value[0]);
	} else {
		dev_err(pTAS2555->dev, "%s: Invalid register address!\n", __func__);
	}

	dev_dbg(pTAS2555->dev, "%s: Write [%d, %d, %d] = %ld\n", __func__,
		register_addr.book, register_addr.page, register_addr.reg,
		pUcontrol->value.integer.value[0]);

	return 0;
}

static int tas2555_power_ctrl_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnPowerCtrl;
	dev_dbg(pTAS2555->dev, "tas2555_power_ctrl_get = %d\n",
		pTAS2555->mnPowerCtrl);

	return 0;
}

static int tas2555_power_ctrl_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pTAS2555->mnPowerCtrl = pValue->value.integer.value[0];

	dev_dbg(pTAS2555->dev, "tas2555_power_ctrl_put = %d\n",
		pTAS2555->mnPowerCtrl);

	if (pTAS2555->mnPowerCtrl == 1)
		tas2555_enable(pTAS2555, true);
	if (pTAS2555->mnPowerCtrl == 0)
		tas2555_enable(pTAS2555, false);

	return 0;
}

static int tas2555_fs_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int nFS = 48000;

	if (pTAS2555->mpFirmware->mnConfigurations)
		nFS = pTAS2555->mpFirmware->mpConfigurations[pTAS2555->mnCurrentConfiguration].mnSamplingRate;

	pValue->value.integer.value[0] = nFS;

	dev_dbg(pTAS2555->dev, "tas2555_fs_get = %d\n", nFS);
	return 0;
}

static int tas2555_fs_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
	int nFS = pValue->value.integer.value[0];

	dev_info(pTAS2555->dev, "tas2555_fs_put = %d\n", nFS);

	ret = tas2555_set_sampling_rate(pTAS2555, nFS);

	return ret;
}

static int tas2555_nRe_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nRe = 0;
	int ret = 0;

	if ((pTAS2555->mpFirmware->mnConfigurations > 0) && pTAS2555->mbPowerUp) {
		ret = tas2555_get_Re(pTAS2555, &nRe);
		if (ret >= 0)
			pValue->value.integer.value[0] = nRe;
		else
			pValue->value.integer.value[0] = 0;
	}

	dev_dbg(pTAS2555->dev, "tas2555_nRe_get = %d\n", nRe);
	return 0;
}

static int tas2555_nRe_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "ignore tas2555_nRe_put\n");
	return 0;
}

static int tas2555_errcode_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int errCode = 0;
	int nResult;

	nResult = tas2555_get_errcode(pTAS2555, &errCode);
	if (nResult >= 0)
		pValue->value.integer.value[0] = errCode;

	dev_dbg(pTAS2555->dev, "tas2555_errcode_get = 0x%x\n", errCode);
	return 0;
}

static int tas2555_errcode_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "ignore tas2555_errcode_put\n");
	return 0;
}

static int tas2555_program_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnCurrentProgram;
	dev_dbg(pTAS2555->dev, "tas2555_program_get = %d\n",
		pTAS2555->mnCurrentProgram);
	return 0;
}

static int tas2555_program_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nProgram = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2555_set_program(pTAS2555, nProgram, -1);
	return ret;
}

static int tas2555_configuration_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnCurrentConfiguration;
	dev_dbg(pTAS2555->dev, "tas2555_configuration_get = %d\n",
		pTAS2555->mnCurrentConfiguration);
	return 0;
}

static int tas2555_configuration_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nConfiguration = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2555_set_config(pTAS2555, nConfiguration);
	return ret;
}

static int tas2555_calibration_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnCurrentCalibration;
	dev_info(pTAS2555->dev,
		"tas2555_calibration_get = %d\n",
		pTAS2555->mnCurrentCalibration);
	return 0;
}

static int tas2555_calibration_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int nCalibration = pValue->value.integer.value[0];
	int ret = 0;

	ret = tas2555_set_calibration(pTAS2555, nCalibration);

	return ret;
}

static int tas2555_nReDelta_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = pTAS2555->mnReDelta;

	dev_dbg(pTAS2555->dev, "tas2555_nReDelta_get = %d\n", pTAS2555->mnReDelta);
	return 0;
}

static int tas2555_nReDelta_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	unsigned int ReDelata = pValue->value.integer.value[0];

	pTAS2555->mnReDelta = ReDelata;

	return 0;
}

static int tas2555_fail_safe_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	pValue->value.integer.value[0] = ((pTAS2555->mnErrorCode & TAS2555_ERROR_FAILSAFE) != 0);
	dev_dbg(pTAS2555->dev, "tas2555_fail_safe_get = 0x%x\n",
		pTAS2555->mnErrorCode);

	return 0;
}

static int tas2555_fail_safe_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int nFailsafe = pValue->value.integer.value[0];

	dev_dbg(pTAS2555->dev, "tas2555_fail_safe_put = %d\n",
		nFailsafe);

	if (nFailsafe == 1)
		failsafe(pTAS2555);

	return 0;
}

static int tas2555_DieTemp_DeltaT_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int nResult = 0, nDeltaT;

	nResult = tas2555_get_die_delta_temperature(pTAS2555, &nDeltaT);
	if (nResult >= 0)
		pValue->value.integer.value[0] = nDeltaT & 0x3ff;

	dev_dbg(pTAS2555->dev, "tas2555_DieTemp_DeltaT_get = 0x%x\n",
		nDeltaT);

	return 0;
}

static int tas2555_DieTemp_DeltaT_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);

	dev_dbg(pTAS2555->dev, "tas2555_DieTemp_DeltaT_put = bypass\n");

	return 0;
}

static int tas2555_fw_load_get(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int nResult = 0;

	if ((pTAS2555->mpFirmware->mnConfigurations > 0)
		&& (pTAS2555->mpFirmware->mnPrograms > 0))
		nResult = 1;

	pValue->value.integer.value[0] = nResult;

	dev_dbg(pTAS2555->dev, "%s = 0x%x\n", __func__, nResult);

	return 0;
}

static int tas2555_fw_load_put(struct snd_kcontrol *pKcontrol,
	struct snd_ctl_elem_value *pValue)
{
#ifdef KCONTROL_CODEC
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(pKcontrol);
#else
	struct snd_soc_codec *codec = snd_kcontrol_chip(pKcontrol);
#endif
	struct tas2555_priv *pTAS2555 = snd_soc_codec_get_drvdata(codec);
	int nReload = pValue->value.integer.value[0];

	if (nReload)
		request_firmware_nowait(THIS_MODULE, 1, TAS2555_FW_NAME,
			pTAS2555->dev, GFP_KERNEL, pTAS2555, tas2555_fw_ready);

	dev_dbg(pTAS2555->dev, "%s, %d\n", __func__, nReload);

	return 0;
}

/*
 * DAC digital volumes. From 0 to 15 dB in 1 dB steps
 */
static DECLARE_TLV_DB_SCALE(dac_tlv, 0, 100, 0);

static const struct snd_kcontrol_new tas2555_snd_controls[] = {
	SOC_SINGLE_TLV("DAC Playback Volume", TAS2555_SPK_CTRL_REG, 3, 0x0f, 0,
		dac_tlv),
	SOC_SINGLE_MULTI_EXT("Reg Addr", 0, 0, INT_MAX, 0, 3, tas2555_get_reg_addr,
		tas2555_put_reg_addr),
	SOC_SINGLE_EXT("Reg Value", SND_SOC_NOPM, 0, 0xFFFF, 0,
		tas2555_get_reg_value, tas2555_put_reg_value),
	SOC_SINGLE_EXT("PowerCtrl", SND_SOC_NOPM, 0, 0x0001, 0,
		tas2555_power_ctrl_get, tas2555_power_ctrl_put),
	SOC_SINGLE_EXT("Program", SND_SOC_NOPM, 0, 0x00FF, 0, tas2555_program_get,
		tas2555_program_put),
	SOC_SINGLE_EXT("Configuration", SND_SOC_NOPM, 0, 0x00FF, 0,
		tas2555_configuration_get, tas2555_configuration_put),
	SOC_SINGLE_EXT("TAS_FWLoad", SND_SOC_NOPM, 0, 0x0001, 0,
		tas2555_fw_load_get, tas2555_fw_load_put),
	SOC_SINGLE_EXT("FS", SND_SOC_NOPM, 8000, 48000, 0,
		tas2555_fs_get, tas2555_fs_put),
	SOC_SINGLE_EXT("nRe_Delta", SND_SOC_NOPM, 0, 0x7fffffff, 0,
		tas2555_nReDelta_get, tas2555_nReDelta_put),
	SOC_SINGLE_EXT("nRe", SND_SOC_NOPM, 0, 0x7fffffff, 0,
		tas2555_nRe_get, tas2555_nRe_put),
	SOC_SINGLE_EXT("TAS_Status", SND_SOC_NOPM, 0, 0x7fffffff, 0,
		tas2555_errcode_get, tas2555_errcode_put),
	SOC_SINGLE_EXT("Calibration", SND_SOC_NOPM, 0, 0x00FF, 0,
		tas2555_calibration_get, tas2555_calibration_put),
	SOC_SINGLE_EXT("FailSafe", SND_SOC_NOPM, 0, 0x0001, 0,
		tas2555_fail_safe_get, tas2555_fail_safe_put),
	SOC_SINGLE_EXT("DieTempDeltaT", SND_SOC_NOPM, 0, 0x03ff, 0,
		tas2555_DieTemp_DeltaT_get, tas2555_DieTemp_DeltaT_put),
};

static struct snd_soc_codec_driver soc_codec_driver_tas2555 = {
	.probe = tas2555_codec_probe,
	.remove = tas2555_codec_remove,
	.read = tas2555_codec_read,
	.write = tas2555_codec_write,
	.set_bias_level = tas2555_set_bias_level,
	.idle_bias_off = true,
	//.ignore_pmdown_time = true,
	.controls = tas2555_snd_controls,
	.num_controls = ARRAY_SIZE(tas2555_snd_controls),
	.dapm_widgets = tas2555_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tas2555_dapm_widgets),
	.dapm_routes = tas2555_audio_map,
	.num_dapm_routes = ARRAY_SIZE(tas2555_audio_map),
};

static struct snd_soc_dai_ops tas2555_dai_ops = {
	.startup = tas2555_startup,
	.shutdown = tas2555_shutdown,
	.digital_mute = tas2555_mute,
	.hw_params = tas2555_hw_params,
	.prepare = tas2555_prepare,
	.set_sysclk = tas2555_set_dai_sysclk,
	.set_fmt = tas2555_set_dai_fmt,
};

#define TAS2555_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
             SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)
static struct snd_soc_dai_driver tas2555_dai_driver[] = {
	{
		.name = "tas2555 ASI1",
		.id = 0,
		.playback = {
				.stream_name = "ASI1 Playback",
				.channels_min = 2,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_8000_192000,
				.formats = TAS2555_FORMATS,
			},
		.ops = &tas2555_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tas2555 ASI2",
		.id = 1,
		.playback = {
				.stream_name = "ASI2 Playback",
				.channels_min = 2,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_8000_192000,
				.formats = TAS2555_FORMATS,
			},
		.ops = &tas2555_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "tas2555 ASIM",
		.id = 2,
		.playback = {
				.stream_name = "ASIM Playback",
				.channels_min = 2,
				.channels_max = 2,
				.rates = SNDRV_PCM_RATE_8000_192000,
				.formats = TAS2555_FORMATS,
			},
		.ops = &tas2555_dai_ops,
		.symmetric_rates = 1,
	},
};

int tas2555_register_codec(struct tas2555_priv *pTAS2555)
{
	int nResult = 0;

	dev_info(pTAS2555->dev, "%s, enter\n", __FUNCTION__);
	  
	nResult = snd_soc_register_codec(pTAS2555->dev, 
		&soc_codec_driver_tas2555,
		tas2555_dai_driver, ARRAY_SIZE(tas2555_dai_driver));

	return nResult;
}

int tas2555_deregister_codec(struct tas2555_priv *pTAS2555)
{
	snd_soc_unregister_codec(pTAS2555->dev);

	return 0;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2555 ALSA SOC Smart Amplifier driver");
MODULE_LICENSE("GPLv2");
#endif
