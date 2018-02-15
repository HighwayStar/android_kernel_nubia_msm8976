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
#ifndef __AMSI2C_H
#define __AMSI2C_H

#include "ams_tmd2725.h"
#include "ams_common.h"

extern int ams_i2c_blk_read(struct i2c_client *client, u8 reg, u8 *val, int size);
extern int ams_i2c_read(struct i2c_client *client, u8 reg, u8 *val);
extern int ams_i2c_blk_read_direct(struct i2c_client *client, u8 reg, u8 *val, int size);
extern int ams_i2c_write_direct(struct i2c_client *client, u8 reg, u8 val);
extern int ams_i2c_write(struct i2c_client *client, u8 *shadow, u8 reg, u8 val);
extern int ams_i2c_reg_blk_write(struct i2c_client *client,	u8 reg, u8 *val, int size);
extern int ams_i2c_ram_blk_write(struct i2c_client *client,	u8 reg, u8 *val, int size);
extern int ams_i2c_ram_blk_read(struct i2c_client *client, u8 reg, u8 *val, int size);
extern int ams_i2c_modify(struct tmd2725_chip *chip, u8 *shadow, u8 reg, u8 mask, u8 val);
extern void ams_i2c_set_field(struct i2c_client *client, u8* shadow, u8 reg, u8 pos, u8 nbits, u8 val);
extern void ams_i2c_get_field(struct i2c_client *client, u8 reg, u8 pos, u8 nbits, u8* val);


#endif /* __AMSI2C_H */

