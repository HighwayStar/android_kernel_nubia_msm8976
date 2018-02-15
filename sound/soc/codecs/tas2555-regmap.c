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
**     tas2555-regmap.c
**
** Description:
**     I2C driver with regmap for Texas Instruments TAS2555 High Performance 4W Smart Amplifier
**
** =============================================================================
*/

#ifdef CONFIG_TAS2555_REGMAP

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
#include "tas2555.h"
#include "tas2555-core.h"
#include <linux/interrupt.h>
#define ENABLE_GPIO_RESET

#ifdef CONFIG_TAS2555_CODEC
#include "tas2555-codec.h"
#endif

#ifdef CONFIG_TAS2555_MISC
#include "tas2555-misc.h"
#endif

#define ENABLE_TILOAD			//only enable this for in-system tuning or debug, not for production systems
#ifdef ENABLE_TILOAD
#include "tiload.h"
#endif

static int tas2555_change_book_page(struct tas2555_priv *pTAS2555, unsigned char nBook,
	unsigned char nPage)
{
	int nResult;

	if ((pTAS2555->mnCurrentBook == nBook) 
		&& pTAS2555->mnCurrentPage == nPage){
		return 0;
	}

	if (pTAS2555->mnCurrentBook != nBook) {
		nResult = regmap_write(pTAS2555->mpRegmap, TAS2555_BOOKCTL_PAGE, 0);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
			goto end;
		}
		pTAS2555->mnCurrentPage = 0;
		nResult = regmap_write(pTAS2555->mpRegmap, TAS2555_BOOKCTL_REG, nBook);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
			goto end;
		}
		pTAS2555->mnCurrentBook = nBook;
		if (nPage != 0) {
			nResult = regmap_write(pTAS2555->mpRegmap, TAS2555_BOOKCTL_PAGE, nPage);
			if (nResult < 0) {
				dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);
				pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
				goto end;
			}
			pTAS2555->mnCurrentPage = nPage;
		}
	} else if (pTAS2555->mnCurrentPage != nPage) {
		nResult = regmap_write(pTAS2555->mpRegmap, TAS2555_BOOKCTL_PAGE, nPage);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
			goto end;
		}
		pTAS2555->mnCurrentPage = nPage;
	}
end:

	return nResult;
}

static int tas2555_dev_read(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, unsigned int *pValue)
{
	int nResult = 0;

	mutex_lock(&pTAS2555->dev_lock);

	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2555->dev_lock);
			return 0;	/* let only reads from TILoad pass. */
		}
		nRegister &= ~0x80000000;
	}

/*	
	dev_dbg(pTAS2555->dev, "%s: BOOK:PAGE:REG %u:%u:%u\n", __func__,
		TAS2555_BOOK_ID(nRegister), TAS2555_PAGE_ID(nRegister),
		TAS2555_PAGE_REG(nRegister));
*/
	nResult = tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
	if (nResult >= 0) {
		nResult = regmap_read(pTAS2555->mpRegmap, TAS2555_PAGE_REG(nRegister), pValue);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
		}
	}
	mutex_unlock(&pTAS2555->dev_lock);
	return nResult;
}

static int tas2555_dev_write(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, unsigned int nValue)
{
	int nResult = 0;

	mutex_lock(&pTAS2555->dev_lock);
	if ((nRegister == 0xAFFEAFFE) && (nValue == 0xBABEBABE)) {
		pTAS2555->mbTILoadActive = true;
		mutex_unlock(&pTAS2555->dev_lock);
		return 0;
	}

	if ((nRegister == 0xBABEBABE) && (nValue == 0xAFFEAFFE)) {
		pTAS2555->mbTILoadActive = false;
		mutex_unlock(&pTAS2555->dev_lock);
		return 0;
	}

	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2555->dev_lock);
			return 0;			// let only writes from TILoad pass.
		}
		nRegister &= ~0x80000000;
	}

	nResult = tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
//  dev_err(codec->dev, "%s: BOOK:PAGE:REG %u:%u:%u, VAL: 0x%02x\n",
//      __func__, TAS2555_BOOK_ID(nRegister), TAS2555_PAGE_ID(nRegister),
//      TAS2555_PAGE_REG(nRegister), value);
	if (nResult >= 0) {
		nResult = regmap_write(pTAS2555->mpRegmap, TAS2555_PAGE_REG(nRegister), nValue);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
		}
	}
	mutex_unlock(&pTAS2555->dev_lock);

	return nResult;
}

static int tas2555_dev_bulk_read(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, u8 * pData, unsigned int nLength)
{
	int nResult = 0;

	mutex_lock(&pTAS2555->dev_lock);
	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2555->dev_lock);
			return 0;			// let only writes from TILoad pass.
		}
		nRegister &= ~0x80000000;
	}

	nResult = tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister),
		TAS2555_PAGE_ID(nRegister));
	if (nResult >= 0) {
		nResult = regmap_bulk_read(pTAS2555->mpRegmap, TAS2555_PAGE_REG(nRegister), pData, nLength);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
		}
	}
	mutex_unlock(&pTAS2555->dev_lock);

	return nResult;
}

static int tas2555_dev_bulk_write(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, u8 * pData, unsigned int nLength)
{
	int nResult = 0;
	mutex_lock(&pTAS2555->dev_lock);
	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2555->dev_lock);
			return 0;			// let only writes from TILoad pass.
		}
		nRegister &= ~0x80000000;
	}

	nResult = tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister), TAS2555_PAGE_ID(nRegister));
	if (nResult >= 0) {
		nResult = regmap_bulk_write(pTAS2555->mpRegmap, TAS2555_PAGE_REG(nRegister), pData, nLength);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
		}
	}
	mutex_unlock(&pTAS2555->dev_lock);

	return nResult;
}

static int tas2555_dev_update_bits(struct tas2555_priv *pTAS2555,
	unsigned int nRegister, unsigned int nMask, unsigned int nValue)
{
	int nResult = 0;

	mutex_lock(&pTAS2555->dev_lock);

	if (pTAS2555->mbTILoadActive) {
		if (!(nRegister & 0x80000000)){
			mutex_unlock(&pTAS2555->dev_lock);
			return 0;			// let only writes from TILoad pass.
		}
		nRegister &= ~0x80000000;
	}
	
	nResult = tas2555_change_book_page(pTAS2555, TAS2555_BOOK_ID(nRegister), TAS2555_PAGE_ID(nRegister));
	if (nResult >= 0) {
		nResult = regmap_update_bits(pTAS2555->mpRegmap, TAS2555_PAGE_REG(nRegister), nMask, nValue);
		if (nResult < 0) {
			dev_err(pTAS2555->dev, "%s, I2C error\n", __func__);
			pTAS2555->mnErrorCode |= TAS2555_ERROR_I2CIO;
		}
	}
	mutex_unlock(&pTAS2555->dev_lock);
	return nResult;
}

static void tas2555_hw_reset(struct tas2555_priv *pTAS2555)
{
#ifdef ENABLE_GPIO_RESET
	if (gpio_is_valid(pTAS2555->mnResetGPIO)) {
		devm_gpio_request_one(pTAS2555->dev, pTAS2555->mnResetGPIO,
			GPIOF_OUT_INIT_LOW, "TAS2555_RST");
		msleep(10);
		gpio_set_value_cansleep(pTAS2555->mnResetGPIO, 1);
		udelay(1000);
	}
#endif
}

int tas2555_enableIRQ(struct tas2555_priv *pTAS2555, bool enable, bool clear)
{
	unsigned int nValue;
	int nResult = 0;

	if (enable) {
		if (clear) {
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &nValue);
			if (nResult < 0)
				goto end;
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &nValue);
		}

		if (!pTAS2555->mbIRQEnable) {
			if (pTAS2555->mnIRQ != 0)
				enable_irq(pTAS2555->mnIRQ);
			pTAS2555->mbIRQEnable = true;
		}
	} else {
		if (pTAS2555->mbIRQEnable) {
			if (pTAS2555->mnIRQ != 0)
				disable_irq_nosync(pTAS2555->mnIRQ);
			pTAS2555->mbIRQEnable = false;
		}

		if (clear) {
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_1, &nValue);
			if (nResult < 0)
				goto end;
			nResult = pTAS2555->read(pTAS2555, TAS2555_FLAGS_2, &nValue);
		}
	}

end:

	return nResult;
}

static void irq_work_routine(struct work_struct *work)
{
	int nResult = 0;
	unsigned int nDevInt1Status = 0, nDevInt2Status = 0;
	struct tas2555_priv *pTAS2555 =
		container_of(work, struct tas2555_priv, irq_work.work);

	if (!pTAS2555->mbPowerUp)
		return;

	nResult = tas2555_dev_read(pTAS2555, TAS2555_FLAGS_1, &nDevInt1Status);
	if (nResult < 0)
		dev_err(pTAS2555->dev, "I2C doesn't work\n");
	else
		nResult = tas2555_dev_read(pTAS2555, TAS2555_FLAGS_2, &nDevInt2Status);

	if ((nDevInt1Status & 0xdc) != 0) {
		/* in case of INT_OC, INT_UV, INT_OT, INT_BO, INT_CL, INT_CLK1, INT_CLK2 */
		dev_err(pTAS2555->dev, "critical error INT Status: 0x%x\n", nDevInt1Status);
		if (nDevInt1Status & 0x04) {
			pTAS2555->mnErrorCode |= TAS2555_ERROR_CLKPRESENT;
			nResult = pTAS2555->write(pTAS2555, TAS2555_CLK_ERR_CTRL1, 0x00);
		}
		if (nDevInt1Status & 0x08)
			pTAS2555->mnErrorCode |= TAS2555_ERROR_BROWNOUT;
		if (nDevInt1Status & 0x10)
			pTAS2555->mnErrorCode |= TAS2555_ERROR_OVERTMP;
		if (nDevInt1Status & 0x40)
			pTAS2555->mnErrorCode |= TAS2555_ERROR_UNDERVOLTAGET;
		if (nDevInt1Status & 0x80)
			pTAS2555->mnErrorCode |= TAS2555_ERROR_OVERCURRENT;
		goto program;
	} else
		dev_dbg(pTAS2555->dev, "%s, INT Status: 0x%x\n", __func__, nDevInt1Status);

	return;

program:
	/* hardware reset and reload */
	tas2555_hw_reset(pTAS2555);
	tas2555_set_program(pTAS2555, pTAS2555->mnCurrentProgram, pTAS2555->mnCurrentConfiguration);
}

static irqreturn_t tas2555_irq_handler(int irq, void *dev_id)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)dev_id;

	tas2555_enableIRQ(pTAS2555, false, false);
	/* get IRQ status after 100 ms */
	schedule_delayed_work(&pTAS2555->irq_work, msecs_to_jiffies(100));
	return IRQ_HANDLED;
}

static bool tas2555_volatile(struct device *pDev, unsigned int nRegister)
{
	return true;
}

static bool tas2555_writeable(struct device *pDev, unsigned int nRegister)
{
	return true;
}

static const struct regmap_config tas2555_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = tas2555_writeable,
	.volatile_reg = tas2555_volatile,
	.cache_type = REGCACHE_NONE,
	.max_register = 128,
};

static int tas2555_i2c_probe(struct i2c_client *pClient,
	const struct i2c_device_id *pID)
{
	struct tas2555_priv *pTAS2555;
	unsigned int n;
	int nResult;

	dev_info(&pClient->dev, "%s enter\n", __FUNCTION__);
	
	pTAS2555 = devm_kzalloc(&pClient->dev, sizeof(struct tas2555_priv), GFP_KERNEL);
	if (!pTAS2555)
		return -ENOMEM;

	pTAS2555->dev = &pClient->dev;
	i2c_set_clientdata(pClient, pTAS2555);
	dev_set_drvdata(&pClient->dev, pTAS2555);

	if (pClient->dev.of_node)
		tas2555_parse_dt(&pClient->dev, pTAS2555);

	tas2555_hw_reset(pTAS2555);

	pTAS2555->mpRegmap = devm_regmap_init_i2c(pClient, &tas2555_i2c_regmap);
	if (IS_ERR(pTAS2555->mpRegmap)) {
		nResult = PTR_ERR(pTAS2555->mpRegmap);
		dev_err(&pClient->dev, "Failed to allocate register map: %d\n",
			nResult);
		return nResult;
	}

	pTAS2555->read = tas2555_dev_read;
	pTAS2555->write = tas2555_dev_write;
	pTAS2555->bulk_read = tas2555_dev_bulk_read;
	pTAS2555->bulk_write = tas2555_dev_bulk_write;
	pTAS2555->update_bits = tas2555_dev_update_bits;
	pTAS2555->enableIRQ = tas2555_enableIRQ;
	pTAS2555->hw_reset = tas2555_hw_reset;
	pTAS2555->set_config = tas2555_set_config;
	pTAS2555->set_calibration = tas2555_set_calibration;
		
	mutex_init(&pTAS2555->dev_lock);
	
	/* Reset the chip */
	nResult = tas2555_dev_write(pTAS2555, TAS2555_SW_RESET_REG, 0x01);
	if(nResult < 0){
		dev_err(&pClient->dev, "I2C communication ERROR: %d\n",
			nResult);
		return nResult;
	}
	
	udelay(1000);

	pTAS2555->mpFirmware =
		devm_kzalloc(&pClient->dev, sizeof(struct TFirmware),
		GFP_KERNEL);
	if (!pTAS2555->mpFirmware)
		return -ENOMEM;

	pTAS2555->mpCalFirmware =
		devm_kzalloc(&pClient->dev, sizeof(struct TFirmware),
		GFP_KERNEL);
	if (!pTAS2555->mpCalFirmware)
		return -ENOMEM;

	if (gpio_is_valid(pTAS2555->mnGpioINT)) {
		nResult = gpio_request(pTAS2555->mnGpioINT, "TAS2555-IRQ");
		if (nResult < 0) {
			dev_err(pTAS2555->dev,
				"%s: GPIO %d request INT error\n",
				__func__, pTAS2555->mnGpioINT);
			goto fail;
		}

		gpio_direction_input(pTAS2555->mnGpioINT);
		pTAS2555->mnIRQ = gpio_to_irq(pTAS2555->mnGpioINT);
		dev_dbg(pTAS2555->dev, "irq = %d\n", pTAS2555->mnIRQ);
		nResult = request_threaded_irq(pTAS2555->mnIRQ, tas2555_irq_handler,
				NULL, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				pClient->name, pTAS2555);
		if (nResult < 0) {
			dev_err(pTAS2555->dev,
				"request_irq failed, %d\n", nResult);
			goto fail;
		}

		INIT_DELAYED_WORK(&pTAS2555->irq_work, irq_work_routine);
	}

	nResult = tas2555_dev_read(pTAS2555, TAS2555_REV_PGID_REG, &n);
	dev_info(&pClient->dev, "TAS2555 PGID: 0x%02x\n", n);

	pTAS2555->mbTILoadActive = false;

#ifdef CONFIG_TAS2555_CODEC	
	tas2555_register_codec(pTAS2555);
#endif

#ifdef CONFIG_TAS2555_MISC	
	mutex_init(&pTAS2555->file_lock);
	tas2555_register_misc(pTAS2555);
#endif

#ifdef ENABLE_TILOAD
	tiload_driver_init(pTAS2555);
#endif

	nResult = request_firmware_nowait(THIS_MODULE, 1, TAS2555_FW_NAME,
		pTAS2555->dev, GFP_KERNEL, pTAS2555, tas2555_fw_ready);

fail:

	return nResult;
}

static int tas2555_i2c_remove(struct i2c_client *pClient)
{
	struct tas2555_priv *pTAS2555 = i2c_get_clientdata(pClient);

	dev_info(pTAS2555->dev, "%s\n", __FUNCTION__);

#ifdef CONFIG_TAS2555_CODEC		
	tas2555_deregister_codec(pTAS2555);
#endif

#ifdef CONFIG_TAS2555_MISC		
	tas2555_deregister_misc(pTAS2555);
	mutex_destroy(&pTAS2555->file_lock);
#endif

	return 0;
}

static const struct i2c_device_id tas2555_i2c_id[] = {
	{"tas2555", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, tas2555_i2c_id);

#if defined(CONFIG_OF)
static const struct of_device_id tas2555_of_match[] = {
	{.compatible = "ti,tas2555"},
	{},
};

MODULE_DEVICE_TABLE(of, tas2555_of_match);
#endif

static struct i2c_driver tas2555_i2c_driver = {
	.driver = {
			.name = "tas2555",
			.owner = THIS_MODULE,
#if defined(CONFIG_OF)
			.of_match_table = of_match_ptr(tas2555_of_match),
#endif
		},
	.probe = tas2555_i2c_probe,
	.remove = tas2555_i2c_remove,
	.id_table = tas2555_i2c_id,
};

module_i2c_driver(tas2555_i2c_driver);

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("TAS2555 I2C Smart Amplifier driver");
MODULE_LICENSE("GPLv2");
#endif