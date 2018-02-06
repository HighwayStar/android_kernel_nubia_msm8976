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
#include <linux/i2c.h>
#include <linux/delay.h>
#include "ams_i2c.h"

int ams_i2c_blk_read(struct i2c_client *client, u8 reg, u8 *val, int size)
{
	s32 ret;
	ret =  i2c_smbus_read_i2c_block_data(client, reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);

	return ret;
}

int ams_i2c_read(struct i2c_client *client, u8 reg, u8 *val)
{
	return ams_i2c_blk_read(client, reg, val, 1);
}

int ams_i2c_blk_read_direct(struct i2c_client *client, u8 reg, u8 *val, int size)
{
	s32 ret;

	ret =  i2c_smbus_read_i2c_block_data(client, reg, size, val);
	if (ret < 0)
		dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
				__func__, reg, size);

	return ret;
}

int ams_i2c_write_direct(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed to write register %x err= %d\n",
				__func__,reg, ret);
		}
	}

	return ret;
}

int ams_i2c_write(struct i2c_client *client, u8 *shadow, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		mdelay(3);
		ret = i2c_smbus_write_byte_data(client, reg, val);
		if (ret < 0) {
			dev_err(&client->dev, "%s: failed to write register %x err= %d\n",
				__func__, reg, ret);
		}
	}

	shadow[reg] = val;

	return ret;
}

int ams_i2c_reg_blk_write(struct i2c_client *client,	u8 reg, u8 *val, int size)
{
	s32 ret;

	ret =  i2c_smbus_write_i2c_block_data(client, reg, size, val);
	if (ret < 0) {
		dev_err(&client->dev, "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);
	}

	return ret;
}

int ams_i2c_ram_blk_write(struct i2c_client *client,	u8 reg, u8 *val, int size)
{
	s32 ret = 0;
	int i;
	int bsize = 0;
	int breg = reg;
	int validx = 0;
	int maxblocksize = 32;

	for(i = 0; i < size; i+=maxblocksize) {
	    if ((size - i) >= maxblocksize) {
	        bsize = maxblocksize;
	    }
	    else {
	        bsize = size -i;
	    }

	    ret =  i2c_smbus_write_i2c_block_data(client, breg, bsize, &val[validx]);
	    if (ret < 0) {
	        dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
			__func__, reg, size);
	    }
	    breg += bsize;
	    validx += bsize;
	}

	return ret;
}

int ams_i2c_ram_blk_read(struct i2c_client *client, u8 reg, u8 *val, int size)
{
	s32 ret = 0;
	int i;
	int bsize = 0;
	int breg = reg;
	int validx = 0;
	int maxblocksize = 32;

	for(i = 0; i < size; i+=maxblocksize) {
	      if ((size - i) >= maxblocksize) {
		  bsize = maxblocksize;
	      }
	      else {
		  bsize = size -i;
	      }

	      ret =  i2c_smbus_read_i2c_block_data(client, breg, bsize, &val[validx]);
	      if (ret < 0)
	   	    ret =  i2c_smbus_read_i2c_block_data(client, breg, bsize, &val[validx]);
	      if (ret < 0)
		    dev_err(&client->dev, "%s: failed 2X at address %x (%d bytes)\n",
				__func__, reg, size);

	      ret =  i2c_smbus_write_i2c_block_data(client, breg, bsize, &val[validx]);
	      if (ret < 0) {
		    dev_err(&client->dev, "%s: failed at address %x (%d bytes)\n",
			    __func__, reg, size);
	      }
	      breg += bsize;
	      validx += bsize;
	}

	return ret;
}

int ams_i2c_modify(struct tmd2725_chip *chip, u8 *shadow, u8 reg, u8 mask, u8 val)
{
	struct i2c_client *client = chip->client;
	int ret = 0;
	u8 temp = 0;
	mutex_lock(&chip->pdata->i2c_lock);
	ret = ams_i2c_read(client, reg, &temp);
	temp &= ~mask;
	temp |= val;
	ret = ams_i2c_write(client, shadow, reg, temp);
	shadow[reg] = temp;
	mutex_unlock(&chip->pdata->i2c_lock);
	return ret;
}


void ams_i2c_set_field(struct i2c_client *client, u8* shadow, u8 reg, u8 pos, u8 nbits, u8 val)
{
    u8 tmp = 0;
    u8 mask = (1 << nbits) - 1;

    ams_i2c_read(client, reg, &tmp);
    tmp &= ~(mask << pos);
    tmp |= (val << pos);
    ams_i2c_write(client, shadow, reg, tmp);
}


void ams_i2c_get_field(struct i2c_client *client, u8 reg, u8 pos, u8 nbits, u8* val)
{
    u8 tmp;
    u8 mask = (1 << nbits) - 1;

    ams_i2c_read(client, reg, &tmp);
    tmp &= mask << pos;
    *val = tmp >> pos;
}


