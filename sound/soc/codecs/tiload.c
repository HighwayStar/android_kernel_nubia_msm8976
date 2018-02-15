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
**     tiload.c
**
** Description:
**     utility for TAS2555 Android in-system tuning
**
** =============================================================================
*/


#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>

#include "tiload.h"

/* enable debug prints in the driver */
#define DEBUG

static struct cdev *tiload_cdev;
static int tiload_major = 0;	/* Dynamic allocation of Mjr No. */
static int tiload_opened = 0;	/* Dynamic allocation of Mjr No. */
static struct tas2555_priv *g_TAS2555;
struct class *tiload_class;
static unsigned int magic_num = 0x00;

static char gPage = 0;
static char gBook = 0;
/******************************** Debug section *****************************/

/*
 *----------------------------------------------------------------------------
 * Function : tiload_open
 *
 * Purpose  : open method for tiload programming interface
 *----------------------------------------------------------------------------
 */
static int tiload_open(struct inode *in, struct file *filp)
{
	struct tas2555_priv *pTAS2555 = g_TAS2555;
	
	dev_info(pTAS2555->dev, "%s\n", __FUNCTION__);
	
	if (tiload_opened) {
			dev_info(pTAS2555->dev, "%s device is already opened\n", "tiload");
		return -1;
	}
	filp->private_data = (void*)pTAS2555;
	tiload_opened++;
	return 0;
}

/*
 *----------------------------------------------------------------------------
 * Function : tiload_release
 *
 * Purpose  : close method for tiload programming interface
 *----------------------------------------------------------------------------
 */
static int tiload_release(struct inode *in, struct file *filp)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)filp->private_data;
	
	dev_info(pTAS2555->dev, "%s\n", __FUNCTION__);
	filp->private_data = NULL;
	tiload_opened--;
	return 0;
}

#define MAX_LENGTH 128
/*
 *----------------------------------------------------------------------------
 * Function : tiload_read
 *
 * Purpose  : read from codec
 *----------------------------------------------------------------------------
 */
static ssize_t tiload_read(struct file *filp, char __user * buf,
	size_t count, loff_t * offset)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)filp->private_data;	
	static char rd_data[MAX_LENGTH + 1];
	unsigned int nCompositeRegister = 0, Value = 0;
	//unsigned int n;
	char reg_addr;
	size_t size;
	int ret = 0;
#ifdef DEBUG
	//int i;
#endif
//    struct i2c_client *i2c = g_codec->control_data;

	dev_dbg(pTAS2555->dev, "%s\n", __FUNCTION__);
	if (count > MAX_LENGTH) {
		dev_err(pTAS2555->dev, "Max %d bytes can be read\n", MAX_LENGTH);
		return -1;
	}

	/* copy register address from user space  */
	size = copy_from_user(&reg_addr, buf, 1);
	if (size != 0) {
		dev_err(pTAS2555->dev, "read: copy_from_user failure\n");
		return -1;
	}

	size = count;

	nCompositeRegister = BPR_REG(gBook, gPage, reg_addr);
	if (count == 1) {
		ret =
			pTAS2555->read(pTAS2555, 0x80000000 | nCompositeRegister, &Value);
		if (ret >= 0)
			rd_data[0] = (char) Value;
	} else if (count > 1) {
		ret =
			pTAS2555->bulk_read(pTAS2555, 0x80000000 | nCompositeRegister,
			rd_data, size);
	}
	if (ret < 0)
		dev_err(pTAS2555->dev, 
				"%s, %d, ret=%d, count=%zu error happen!\n", 
				__FUNCTION__, __LINE__, ret, count);

#ifdef DEBUG
	dev_dbg(pTAS2555->dev, 
		"read size = %d, reg_addr= 0x%x , count = %d\n",
		(int) size, reg_addr, (int) count);
//	for (i = 0; i < (int) size; i++) {
//		dev_dbg(pTAS2557->dev, "rd_data[%d]=%x\n", i, rd_data[i]);
//	}
#endif
	if (size != count) {
		dev_err(pTAS2555->dev, 
			"read %d registers from the codec\n", (int) size);
	}

	if (copy_to_user(buf, rd_data, size) != 0) {
		dev_err(pTAS2555->dev, "copy_to_user failed\n");
		return -1;
	}

	return size;
}

/*
 *----------------------------------------------------------------------------
 * Function : tiload_write
 *
 * Purpose  : write to codec
 *----------------------------------------------------------------------------
 */
static ssize_t tiload_write(struct file *filp, const char __user * buf,
	size_t count, loff_t * offset)
{
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)filp->private_data;		
	static char wr_data[MAX_LENGTH + 1];
	char *pData = wr_data;
	size_t size;
	unsigned int nCompositeRegister = 0;
//    u8 pg_no;
//    unsigned int n;   
	unsigned int nRegister;
	int ret = 0;
#ifdef DEBUG
	//int i;
#endif

	dev_info(pTAS2555->dev, "%s\n", __FUNCTION__);

	if (count > MAX_LENGTH) {
		dev_err(pTAS2555->dev,"Max %d bytes can be read\n", MAX_LENGTH);
		return -1;
	}
	
	/* copy buffer from user space  */
	size = copy_from_user(wr_data, buf, count);
	if (size != 0) {
		dev_err(pTAS2555->dev,
			"copy_from_user failure %d\n", (int) size);
		return -1;
	}
#ifdef DEBUG
	dev_dbg(pTAS2555->dev, "write size = %zu\n", count);
	//for (i = 0; i < (int) count; i++) {
	//	dev_info(pTAS2557->dev, "wr_data[%d]=%x\n", i, wr_data[i]);
	//}
#endif
	nRegister = wr_data[0];
	size = count;
	if ((nRegister == 127) && (gPage == 0)) {
		gBook = wr_data[1];
		return size;
	}

	if (nRegister == 0) {
		gPage = wr_data[1];
		pData++;
		count--;
	}
#if 1
	nCompositeRegister = BPR_REG(gBook, gPage, nRegister);
	if (count == 2) {
		ret =
			pTAS2555->write(pTAS2555, 0x80000000 | nCompositeRegister,
			pData[1]);
	} else if (count > 2) {
		ret =
			pTAS2555->bulk_write(pTAS2555, 0x80000000 | nCompositeRegister,
			&pData[1], count - 1);
	}
	if (ret < 0)
		dev_err(pTAS2555->dev,
			"%s, %d, ret=%d, count=%zu, ERROR Happen\n", 
			__FUNCTION__, __LINE__, ret, count);
#else
	for (n = 1; n < count; n++) {
		nCompositeRegister = BPR_REG(gBook, gPage, nRegister + n - 1);
		g_codec->driver->write(g_codec, 0x80000000 | nCompositeRegister,
			pData[n]);
	}
#endif

	return size;
}

static void tiload_route_IO(struct tas2555_priv *pTAS2555, 
	unsigned int bLock)
{
	if (bLock) {
		pTAS2555->write(pTAS2555, 0xAFFEAFFE, 0xBABEBABE);
	} else {
		pTAS2555->write(pTAS2555, 0xBABEBABE, 0xAFFEAFFE);
	}
}

static long tiload_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	long num = 0;
	struct tas2555_priv *pTAS2555 = (struct tas2555_priv *)filp->private_data;	
	void __user *argp = (void __user *) arg;
	int val;
	BPR bpr;

	dev_info(pTAS2555->dev, "%s, cmd=0x%x\n", __FUNCTION__, cmd);
//    if (_IOC_TYPE(cmd) != TILOAD_IOC_MAGIC)
//        return -ENOTTY;

	switch (cmd) {
	case TILOAD_IOMAGICNUM_GET:
		num = copy_to_user(argp, &magic_num, sizeof(int));
		break;
	case TILOAD_IOMAGICNUM_SET:
		num = copy_from_user(&magic_num, argp, sizeof(int));
		dev_info(pTAS2555->dev, "TILOAD_IOMAGICNUM_SET\n");
			tiload_route_IO(pTAS2555, magic_num);
		break;
	case TILOAD_BPR_READ:
		break;
	case TILOAD_BPR_WRITE:
		num = copy_from_user(&bpr, argp, sizeof(BPR));
		dev_dbg(pTAS2555->dev, 
			"TILOAD_BPR_WRITE: 0x%02X, 0x%02X, 0x%02X\n\r", bpr.nBook,
			bpr.nPage, bpr.nRegister);
		break;
	case TILOAD_IOCTL_SET_CHL:
		break;		
	case TILOAD_IOCTL_SET_CONFIG:
		num = copy_from_user(&val, argp, sizeof(val));
		pTAS2555->set_config(pTAS2555, val);
		break;
	case TILOAD_IOCTL_SET_CALIBRATION:
		num = copy_from_user(&val, argp, sizeof(val));
		pTAS2555->set_calibration(pTAS2555, val);
		break;				
	default:
		break;
	}
	return num;
}

/*********** File operations structure for tiload *************/
static struct file_operations tiload_fops = {
	.owner = THIS_MODULE,
	.open = tiload_open,
	.release = tiload_release,
	.read = tiload_read,
	.write = tiload_write,
	.unlocked_ioctl = tiload_ioctl,
};

/*
 *----------------------------------------------------------------------------
 * Function : tiload_driver_init
 *
 * Purpose  : Register a char driver for dynamic tiload programming
 *----------------------------------------------------------------------------
 */
int tiload_driver_init(struct tas2555_priv *pTAS2555)
{
	int result;

	dev_t dev = MKDEV(tiload_major, 0);

	dev_info(pTAS2555->dev, "%s\n", __FUNCTION__);
	g_TAS2555 = pTAS2555;

	result = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (result < 0) {
		dev_err(pTAS2555->dev,
			"cannot allocate major number %d\n", tiload_major);
		return result;
	}
	tiload_class = class_create(THIS_MODULE, DEVICE_NAME);
	tiload_major = MAJOR(dev);
	dev_info(pTAS2555->dev,
		"allocated Major Number: %d\n", tiload_major);

	tiload_cdev = cdev_alloc();
	cdev_init(tiload_cdev, &tiload_fops);
	tiload_cdev->owner = THIS_MODULE;
	tiload_cdev->ops = &tiload_fops;

	if (device_create(tiload_class, NULL, dev, NULL, "tiload_node") == NULL)
		dev_err(pTAS2555->dev,
			"Device creation failed\n");

	if (cdev_add(tiload_cdev, dev, 1) < 0) {
		dev_err(pTAS2555->dev,
				"tiload_driver: cdev_add failed \n");
		unregister_chrdev_region(dev, 1);
		tiload_cdev = NULL;
		return 1;
	}
	
	dev_info(pTAS2555->dev,
		"Registered TiLoad driver, Major number: %d \n", tiload_major);
		
	return 0;
}

MODULE_AUTHOR("Texas Instruments Inc.");
MODULE_DESCRIPTION("Utility for TAS2555 Android in-system tuning");
MODULE_LICENSE("GPLv2");
//#endif
